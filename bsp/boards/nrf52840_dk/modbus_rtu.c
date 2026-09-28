#include "modbus_rtu.h"

/* ------------------------------------------------------------------------ */
/* 1. 硬件引脚物理定义                              */
/* ------------------------------------------------------------------------ */
#define UART_TX_PIN      27  // P0.27 -> 接 485 模块的 TTL_RXD
#define UART_RX_PIN      26  // P0.26 -> 接 485 模块的 TTL_TXD

/* ------------------------------------------------------------------------ */
/* 2. 静态全局私有变量                              */
/* ------------------------------------------------------------------------ */
modbus_bus_state_t g_bus_state = MODBUS_STATE_IDLE;

static uint8_t  g_rx_buffer[MODBUS_RX_BUF_SIZE];
static volatile uint16_t g_rx_cnt = 0;            // ISR 写、主循环读 -> 必须 volatile
static volatile uint32_t g_last_rx_timestamp = 0;
/* 诊断计数：供上层判断链路断在哪一环 */
static volatile uint32_t g_count_frame   = 0;     // CRC 与功能码都通过的合法帧
static volatile uint32_t g_count_timeout = 0;     // 发出去后等满超时、一个字节都没收到
static volatile uint32_t g_count_error   = 0;     // 长度/CRC/功能码异常（含 0x83 异常码）
static volatile uint32_t g_count_uarterr = 0;     // UARTE 硬件错误（帧错误/溢出）
static uint8_t  g_last_error_frame[MODBUS_RX_BUF_SIZE]; // 最后一个坏帧的现场
static volatile uint16_t g_last_error_len = 0;
/* 帧内字节间隔统计：用来判断"流是连续断开"还是"传感器自己停顿" */
static uint32_t g_frame_first_ts = 0;   // 本帧首字节时刻
static uint32_t g_frame_prev_ts  = 0;   // 本帧上一字节时刻
static uint16_t g_frame_maxgap   = 0;   // 本帧内最大字节间隔(ms)
static volatile uint16_t g_max_gap_all = 0; // 开机以来所有帧中的最大字节间隔(ms)
static volatile bool g_tx_complete_flag = false;

/* ⚡ 极其重要：驻留在 SRAM 中的单字节缓冲区，专供 EasyDMA 硬件每次挪移 1 字节使用 */
static uint8_t g_dma_rx_byte; 

/* 核心解耦件：存储当前通信周期挂载的专用解析回调函数 */
static modbus_parser_t g_current_parser = NULL;

/* ------------------------------------------------------------------------ */
/* 3. 寄存器级辅助工具函数                           */
/* ------------------------------------------------------------------------ */

/**
 * @brief 获取当前系统毫秒级时间戳
 * @note 直接读取硬件 RTC1 计数器。假设系统已启动 RTC1 且分频器预设为 0 (32768Hz)
 */
static uint32_t get_system_time_ms(void) {
    // RTC0 以 32768Hz 自由运行；必须用 64 位中间量：24 位计数器 ×1000 在 32 位下约 131 秒
    // 就会溢出，会让断帧/超时判定出现异常跳变（假断帧 / 假超时）。
    return (uint32_t)(((uint64_t)NRF_RTC0->COUNTER * 1000ULL) / 32768ULL);
}

/**
 * @brief 工业标准 Modbus CRC16 快速计算
 */
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ------------------------------------------------------------------------ */
/* 4. 纯寄存器底层硬件配置                           */
/* ------------------------------------------------------------------------ */
void modbus_init(void) {
    // 1. 配置 GPIO 引脚
    //    ⚠ nRF52840 的 PIN_CNF 复位值 = 0x00000002，即 INPUT = Disconnect（输入缓冲断开）。
    //    只写 DIRCLR 把方向设成输入是不够的：输入缓冲不 Connect，UARTE 一个字节都收不到。
    //    TX(P0.27)：输出、输入缓冲断开；RX(P0.26)：输入、输入缓冲 Connect。
    NRF_P0->PIN_CNF[UART_TX_PIN] = ((uint32_t)GPIO_PIN_CNF_DIR_Output      << GPIO_PIN_CNF_DIR_Pos)
                                 | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                 | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled   << GPIO_PIN_CNF_PULL_Pos)
                                 | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1      << GPIO_PIN_CNF_DRIVE_Pos)
                                 | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled  << GPIO_PIN_CNF_SENSE_Pos);
    NRF_P0->PIN_CNF[UART_RX_PIN] = ((uint32_t)GPIO_PIN_CNF_DIR_Input       << GPIO_PIN_CNF_DIR_Pos)
                                 | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect   << GPIO_PIN_CNF_INPUT_Pos)
                                 | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled   << GPIO_PIN_CNF_PULL_Pos)
                                 | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1      << GPIO_PIN_CNF_DRIVE_Pos)
                                 | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled  << GPIO_PIN_CNF_SENSE_Pos);

    // 2. 将引脚映射到 UARTE1 外设，并连通物理线路 (Connected = 0)
    NRF_UARTE1->PSEL.TXD = (UART_TX_PIN << UARTE_PSEL_TXD_PIN_Pos) | 
                           (UARTE_PSEL_TXD_CONNECT_Connected << UARTE_PSEL_TXD_CONNECT_Pos);
    NRF_UARTE1->PSEL.RXD = (UART_RX_PIN << UARTE_PSEL_RXD_PIN_Pos) | 
                           (UARTE_PSEL_RXD_CONNECT_Connected << UARTE_PSEL_RXD_CONNECT_Pos);

    // 3. 配置硬核波特率：4800 (对应官方计算出的内部寄存器密匙值)
    NRF_UARTE1->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud4800; 

    // 4. 配置帧格式：无奇偶校验 (Excluded), 1 位停止位 (One)，自动组成 8N1
    NRF_UARTE1->CONFIG = (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos) |
                         (UARTE_CONFIG_STOP_One        << UARTE_CONFIG_STOP_Pos);

    // 5. 开启中断允许掩码：仅监听 EasyDMA 接收完 (ENDRX) 与 发送完 (ENDTX) 事件
    NRF_UARTE1->INTENSET = UARTE_INTENSET_ENDRX_Msk | UARTE_INTENSET_ENDTX_Msk;

    // 5.5 启用前把历史事件清零，避免带着脏事件进状态机
    NRF_UARTE1->EVENTS_ENDRX = 0;
    NRF_UARTE1->EVENTS_ENDTX = 0;
    NRF_UARTE1->EVENTS_ERROR = 0;

    // 6. 激活 UARTE1 外设核心引擎
    NRF_UARTE1->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

    // 7. 预铺设 EasyDMA 接收双向指针
    NRF_UARTE1->RXD.PTR    = (uint32_t)&g_dma_rx_byte; // 锁定单字节变量地址
    NRF_UARTE1->RXD.MAXCNT = 1;                        // 💡 强行限流 MAXCNT=1，依靠单字节搬运实现无 IDLE 中断断帧

    // 7.5 ⚡ 关键：ENDRX -> STARTRX 硬件短路。
    //     UARTE 收满 MAXCNT 之后接收器会停；若靠 ISR 软件重装 STARTRX，
    //     从 ENDRX 到重装之间到达的字节会被静默丢掉（丢几个字节就足以让
    //     3.5T 静默判定提前收尾 -> 坏帧）。硬件短路把重装挪到事件产生瞬间。
    NRF_UARTE1->SHORTS = UARTE_SHORTS_ENDRX_STARTRX_Msk;

    // 8. 配置 Cortex-M4 内核 NVIC 中断控制器 (专为 OpenWSN 6TiSCH 进行时序避让)
    NVIC_ClearPendingIRQ(UARTE1_IRQn);
    NVIC_SetPriority(UARTE1_IRQn, 5);                  // 💡 设为低优先级 5，绝不抢占 6TiSCH 无线电(0级)时隙中断
    NVIC_EnableIRQ(UARTE1_IRQn);

    // 9. 踢出第一脚，启动硬件长线接收监听
    NRF_UARTE1->TASKS_STARTRX = 1;
}

/* ------------------------------------------------------------------------ */
/* 5. 核心硬件中断服务函数 (ISR)                       */
/* ------------------------------------------------------------------------ */
void UARTE1_IRQHandler(void) {
    // 📥 状况 A：EasyDMA 成功将 485 总线上的 1 个字符搬运进 SRAM 变量 g_dma_rx_byte
    if (NRF_UARTE1->EVENTS_ENDRX != 0) {
        NRF_UARTE1->EVENTS_ENDRX = 0; // ⚠️ 必须率先手动清零事件，否则死卡中断

        // 仅在状态机处于“等待接收”状态时，才把电平字节推入大缓冲区
        if (g_bus_state == MODBUS_STATE_RX_WAIT) {
            if (g_rx_cnt < MODBUS_RX_BUF_SIZE) {
                g_rx_buffer[g_rx_cnt++] = g_dma_rx_byte;
            }
            // ⚡【断帧核心】：只要总线蹦出一个新字节，就疯狂刷新最近接收时间戳
            uint32_t now_ms = get_system_time_ms();
            if (g_rx_cnt == 0) {
                g_frame_first_ts = now_ms;
                g_frame_maxgap   = 0;
            } else {
                uint32_t gap = now_ms - g_frame_prev_ts;
                if (gap > g_frame_maxgap) g_frame_maxgap = (uint16_t)gap;
            }
            g_frame_prev_ts      = now_ms;
            g_last_rx_timestamp  = now_ms;
        }

        // ⚠️【DMA重载】：EasyDMA 是一次性的，必须由软件在此处强行重新启动接收任务
        // 重装已由 SHORTS(=ENDRX_STARTRX) 硬件完成，不再软件重装
    }

    // 📤 状况 B：EasyDMA 成功将内存中的主机命令（如8字节）全部倾泻发射至 485 总线
    if (NRF_UARTE1->EVENTS_ENDTX != 0) {
        NRF_UARTE1->EVENTS_ENDTX = 0; // 手动清零

        // 升起完成标志旗帜，通知主循环状态机
        g_tx_complete_flag = true;
    }

    // 状况 C：UARTE 硬件报错（frame error / overrun ...）。不清会一直挂着；
    //         次数暴增通常意味着波特率不对或 485 侧信号有问题。
    if (NRF_UARTE1->EVENTS_ERROR != 0) {
        NRF_UARTE1->EVENTS_ERROR = 0;
        NRF_UARTE1->ERRORSRC     = 0xFFFFFFFF; // 写 1 清
        g_count_uarterr++;
    }
}

/* ------------------------------------------------------------------------ */
/* 6. 寄存器级后台异步发送                           */
/* ------------------------------------------------------------------------ */
static void nrf52840_uart_dma_tx(const uint8_t *buf, uint16_t len) {
    NRF_UARTE1->TXD.PTR    = (uint32_t)buf; // 直接将包含数据的 SRAM 空间首地址写入
    NRF_UARTE1->TXD.MAXCNT = len;
    
    // 扣动硬件发射扳机，EasyDMA 自动在后台工作，该任务指令耗时微秒级，瞬间返回
    NRF_UARTE1->TASKS_STARTTX = 1;
}

/* ------------------------------------------------------------------------ */
/* 7. 通用非阻塞读指令投放                           */
/* ------------------------------------------------------------------------ */
bool modbus_read_registers(uint8_t dev_addr, uint16_t start_reg, uint16_t reg_cnt, modbus_parser_t parse_callback) {
    if (g_bus_state != MODBUS_STATE_IDLE) {
        return false; // 总线正忙（或正在发、或正在等收），拒绝本次叠加请求
    }
    
    // 核心动作：动态挂载针对本次读取的专用回调解析器
    g_current_parser = parse_callback; 
    
    // 必须是 static 或者是全局空间，确保变量生命周期跨越整个后台 EasyDMA 发送期
    static uint8_t tx_frame[8]; 
    
    tx_frame[0] = dev_addr;
    tx_frame[1] = 0x03; // 功能码：读保持寄存器
    tx_frame[2] = (uint8_t)(start_reg >> 8);
    tx_frame[3] = (uint8_t)(start_reg & 0xFF);
    tx_frame[4] = (uint8_t)(reg_cnt >> 8);
    tx_frame[5] = (uint8_t)(reg_cnt & 0xFF);
    
    uint16_t crc = modbus_crc16(tx_frame, 6);
    tx_frame[6] = crc & 0xFF; // Modbus 标准：低字节在前
    tx_frame[7] = crc >> 8;
    
    g_rx_cnt = 0;
    g_tx_complete_flag = false;
    
    // 切换状态机进入发送状态
    g_bus_state = MODBUS_STATE_TX;
    
    // 投递给硬件底层发送
    nrf52840_uart_dma_tx(tx_frame, 8); 
    
    return true;
}

/* ------------------------------------------------------------------------ */
/* 8. 状态机非阻塞主轮询引擎                          */
/* ------------------------------------------------------------------------ */
void modbus_poll(void) {
    uint32_t now = get_system_time_ms();
    
    switch (g_bus_state) {
        case MODBUS_STATE_IDLE:
            break;
            
        case MODBUS_STATE_TX:
            // 等待来自 UARTE1 中断服务程序刷出的发送完毕信号
            if (g_tx_complete_flag) {
                g_last_rx_timestamp = now; // 重置断帧基准时间，开始进入漫长等待
                g_bus_state = MODBUS_STATE_RX_WAIT;
            }
            break;
            
        case MODBUS_STATE_RX_WAIT:
            // 【整帧收齐判定】
            // ★ 首选判据：MODBUS 应答自带"有效字节数"(buf[2])，收满 5+buf[2] 字节即整帧。
            //   实测本传感器会在帧中间停顿 >10ms（内部测量周期打断 UART 发送），
            //   只靠静默断帧会把它切成"正确前缀"的坏帧（实测约 5%）。
            if (g_rx_cnt >= 5 && g_rx_cnt >= (uint16_t)(5 + g_rx_buffer[2])) {
                g_bus_state = MODBUS_STATE_PARSE;
            }
            // 兜底：收不齐才退回静默断帧（异常帧 / 真丢字节）
            else if (g_rx_cnt > 0 && (now - g_last_rx_timestamp >= MODBUS_FRAME_GAP_MS)) {
                g_bus_state = MODBUS_STATE_PARSE;
            }
            // ⏳【总线超时异常保护】
            // 指令丢出去 200ms 了，传感器连一个字节的回话都没有（可能线断了或没上电）
            else if (g_rx_cnt == 0 && (now - g_last_rx_timestamp >= MODBUS_RX_TIMEOUT_MS)) {
                g_count_timeout++;
                g_current_parser = NULL; 
                g_bus_state = MODBUS_STATE_IDLE; // 复位总线
            }
            break;
            
        case MODBUS_STATE_PARSE:
        {
            bool frame_ok = false;
            // 基础合法性过滤：长度至少包含（地址+功能码+字节数+2字节CRC） = 5
            if (g_rx_cnt >= 5) {
                uint16_t calc_crc = modbus_crc16(g_rx_buffer, g_rx_cnt - 2);
                uint16_t recv_crc = g_rx_buffer[g_rx_cnt - 2] | (g_rx_buffer[g_rx_cnt - 1] << 8);

                // 校验完全通关，证明这包数据完美且无损
                if (calc_crc == recv_crc && g_rx_buffer[1] == 0x03) {
                    frame_ok = true;
                }
            }

            if (frame_ok) {
                g_count_frame++;
                if (g_frame_maxgap > g_max_gap_all) g_max_gap_all = g_frame_maxgap;
                // 直接把大缓冲区交给当前挂载的业务解析器，底层不关心具体数据含义
                if (g_current_parser != NULL) {
                    g_current_parser(g_rx_buffer, g_rx_cnt);
                }
            } else {
                // 坏帧：留一份现场，上层可以打出来看传感器到底回了什么
                //（例如 0x83 就是 Modbus 异常码：非法数据地址）
                g_count_error++;
                uint16_t n = (g_rx_cnt < MODBUS_RX_BUF_SIZE) ? (uint16_t)g_rx_cnt : (uint16_t)MODBUS_RX_BUF_SIZE;
                for (uint16_t k = 0; k < n; k++) {
                    g_last_error_frame[k] = g_rx_buffer[k];
                }
                g_last_error_len = n;
                if (g_frame_maxgap > g_max_gap_all) g_max_gap_all = g_frame_maxgap;
            }

            // 清空本次传输事务，释放总线控制权，迎接下一次应用层触发
            g_current_parser = NULL;
            g_bus_state = MODBUS_STATE_IDLE;
            break;
        }
    }
}

/* ------------------------------------------------------------------------ */
/* 9. 对外接口：给应用层做节流与链路诊断                      */
/* ------------------------------------------------------------------------ */

/** @brief 系统毫秒时间戳（32 位自然回绕，求差即可） */
uint32_t modbus_millis(void) {
    return get_system_time_ms();
}

/** @brief 累计解析成功的合法帧数 */
uint32_t modbus_get_frame_count(void) {
    return g_count_frame;
}

/** @brief 累计超时次数（发出去后一个字节都没收到） */
uint32_t modbus_get_timeout_count(void) {
    return g_count_timeout;
}

/** @brief 累计坏帧次数（长度/CRC/功能码异常） */
uint32_t modbus_get_error_count(void) {
    return g_count_error;
}

/** @brief 累计 UARTE 硬件错误次数（帧错误/溢出） */
uint32_t modbus_get_uart_error_count(void) {
    return g_count_uarterr;
}

/**
 * @brief 取出最后一个坏帧的现场字节（例如传感器回 0x83 异常码时很有用）
 * @return 实际拷贝的字节数
 */
uint16_t modbus_get_last_error(uint8_t *dst, uint16_t dst_size) {
    uint16_t n = g_last_error_len;
    if (n > dst_size) n = dst_size;
    for (uint16_t i = 0; i < n; i++) {
        dst[i] = g_last_error_frame[i];
    }
    return n;
}

/**
 * @brief 开机以来观测到的最大“帧内字节间隔”(ms)
 * @note  远大于正常字节时间(4800bps≈2.08ms) 就说明传感器会在帧中间停顿，
 *        这也是不能用 3.5T 静默当整帧判据的原因
 */
uint16_t modbus_get_max_frame_gap(void) {
    return g_max_gap_all;
}

/** @brief 最后一帧坏帧内的最大字节间隔(ms) */
uint16_t modbus_get_last_error_maxgap(void) {
    return g_frame_maxgap;
}
