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

/* ---- 失败诊断与自动恢复所需的附加状态 ------------------------------------ */
static volatile uint32_t g_count_tx_timeout = 0;  // ENDTX 丢失，TX 态被强制复位
static volatile uint32_t g_count_retry      = 0;  // 坏帧/超时后自动补发同一请求
static volatile uint32_t g_count_overflow   = 0;  // RX 缓冲溢出被丢弃的字节数
static volatile uint32_t g_count_reject     = 0;  // 参数非法被拒的请求数
static volatile uint8_t  g_last_exception   = 0;  // 最后一个 0x83 异常应答的异常码
static uint32_t g_tx_start_ts = 0;         // 进入 TX 态的时刻（发送兜底超时基准）
static uint8_t  g_retry_used  = 0;         // 本次事务已经自动重发过几次
static uint8_t  g_tx_frame[8];             // 当前事务的请求帧（重发时原样复用）

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
            } else {
                // 缓冲写满后继续来字节：静默丢弃会让 CRC 报错，但必须留下痕迹，
                // 否则事后无法区分"传感器少发了"和"我们缓冲不够"。
                g_count_overflow++;
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

/**
 * @brief Start one transmission: clear the previous transaction, arm the TX
 *        watchdog baseline, hand the frame to EasyDMA.
 * @note  First send and automatic retry both go through here, so both paths
 *        behave identically. The request frame lives in g_tx_frame, which is
 *        what makes an immediate re-send possible at all.
 */
static void modbus_start_tx(void) {
    g_rx_cnt           = 0;
    g_frame_maxgap     = 0;
    g_tx_complete_flag = false;
    g_tx_start_ts      = get_system_time_ms();
    g_bus_state        = MODBUS_STATE_TX;
    nrf52840_uart_dma_tx(g_tx_frame, 8);
}

/**
 * @brief Transaction failed: re-send once if the budget allows, else release.
 * @note  The sensor aborts ~3-5% of its own replies mid-frame (measured), so a
 *        bad frame costs a whole polling period if the only way back is the
 *        application loop. One immediate retry turns that into ~0.2% loss.
 */
static void modbus_finish_or_retry(void) {
    if (g_retry_used < MODBUS_MAX_RETRY) {
        g_retry_used++;
        g_count_retry++;
        g_bus_state = MODBUS_STATE_RETRY_WAIT;   // modbus_poll re-sends after the 3.5T guard
    } else {
        g_current_parser = NULL;
        g_bus_state = MODBUS_STATE_IDLE;
    }
}

/* ------------------------------------------------------------------------ */
/* 7. 通用非阻塞读指令投放                           */
/* ------------------------------------------------------------------------ */
bool modbus_read_registers(uint8_t dev_addr, uint16_t start_reg, uint16_t reg_cnt, modbus_parser_t parse_callback) {
    if (g_bus_state != MODBUS_STATE_IDLE) {
        return false;   // bus busy: still sending, or still waiting for a reply
    }
    // Parameter guards: an out-of-range reg_cnt can never fit in MODBUS_RX_BUF_SIZE,
    // and a NULL parser would silently discard the reply. Count both as rejects so
    // the application can tell "bus busy" from "caller bug".
    if (parse_callback == NULL || reg_cnt == 0 || reg_cnt > MODBUS_MAX_REG_CNT) {
        g_count_reject++;
        return false;
    }

    g_current_parser = parse_callback;   // mount the parser for this transaction

    g_tx_frame[0] = dev_addr;
    g_tx_frame[1] = 0x03;                // function code: read holding registers
    g_tx_frame[2] = (uint8_t)(start_reg >> 8);
    g_tx_frame[3] = (uint8_t)(start_reg & 0xFF);
    g_tx_frame[4] = (uint8_t)(reg_cnt >> 8);
    g_tx_frame[5] = (uint8_t)(reg_cnt & 0xFF);

    uint16_t crc = modbus_crc16(g_tx_frame, 6);
    g_tx_frame[6] = crc & 0xFF;          // Modbus standard: low byte first
    g_tx_frame[7] = crc >> 8;

    g_retry_used = 0;

    modbus_start_tx();

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
            if (g_tx_complete_flag) {
                g_last_rx_timestamp = now;   // start the reply timer here
                g_bus_state = MODBUS_STATE_RX_WAIT;
            } else if ((uint32_t)(now - g_tx_start_ts) >= MODBUS_TX_TIMEOUT_MS) {
                // ENDTX lost (reset glitch, or the IRQ was masked for a long
                // time). Without this watchdog the state machine parks in TX
                // forever: no data, no timeout counter, nothing to notice.
                g_count_tx_timeout++;
                modbus_finish_or_retry();
            }
            break;

        case MODBUS_STATE_RETRY_WAIT:
            // Modbus requires 3.5T of silence before the next request; never
            // go back-to-back on the wire.
            if ((uint32_t)(now - g_last_rx_timestamp) >= MODBUS_INTER_FRAME_GUARD_MS) {
                modbus_start_tx();
            }
            break;

        case MODBUS_STATE_RX_WAIT:
            // Primary rule: the reply carries its own byte count (buf[2]), so a
            // complete frame is 5 + buf[2] bytes. The sensor really does pause
            // ~175-210ms mid-frame, which would otherwise cut a valid reply
            // into a bad "correct prefix".
            if (g_rx_cnt >= 3) {
                // 0x83 exception replies carry no data block, they are 5 bytes.
                uint16_t expect = (g_rx_buffer[1] & 0x80)
                                 ? (uint16_t)5
                                 : (uint16_t)(5 + g_rx_buffer[2]);
                // A byte count we could never buffer means the header is garbage
                // (or the slave wants more than our buffer): fail fast instead of
                // stalling 400ms on the silence fallback.
                if (expect > MODBUS_RX_BUF_SIZE || g_rx_cnt >= expect) {
                    g_bus_state = MODBUS_STATE_PARSE;
                }
            }
            // Fallback: truncated frame, close it on bus silence
            else if (g_rx_cnt > 0 && (uint32_t)(now - g_last_rx_timestamp) >= MODBUS_FRAME_GAP_MS) {
                g_bus_state = MODBUS_STATE_PARSE;
            }
            // Nothing at all came back within the timeout
            else if (g_rx_cnt == 0 && (uint32_t)(now - g_last_rx_timestamp) >= MODBUS_RX_TIMEOUT_MS) {
                g_count_timeout++;
                modbus_finish_or_retry();
            }
            break;

        case MODBUS_STATE_PARSE:
        {
            bool frame_ok = false;
            if (g_rx_cnt >= 5) {
                uint16_t calc_crc = modbus_crc16(g_rx_buffer, g_rx_cnt - 2);
                uint16_t recv_crc = g_rx_buffer[g_rx_cnt - 2] | (g_rx_buffer[g_rx_cnt - 1] << 8);

                if (calc_crc == recv_crc) {
                    if (g_rx_buffer[1] == 0x03) {
                        frame_ok = true;
                        g_last_exception = 0;      // good frame clears a stale code
                    } else if (g_rx_buffer[1] & 0x80) {
                        // 0x83 + exception code: keep it structured (e.g. 0x02
                        // illegal data address tells the caller to split the read)
                        g_last_exception = g_rx_buffer[2];
                    }
                }
            }

            if (frame_ok) {
                g_count_frame++;
                if (g_frame_maxgap > g_max_gap_all) g_max_gap_all = g_frame_maxgap;
                if (g_current_parser != NULL) {
                    g_current_parser(g_rx_buffer, g_rx_cnt);
                }
                g_current_parser = NULL;
                g_bus_state = MODBUS_STATE_IDLE;      // good frame: never retry
            } else {
                // Bad frame: keep the scene for the application to dump
                g_count_error++;
                uint16_t n = (g_rx_cnt < MODBUS_RX_BUF_SIZE) ? (uint16_t)g_rx_cnt : (uint16_t)MODBUS_RX_BUF_SIZE;
                for (uint16_t k = 0; k < n; k++) {
                    g_last_error_frame[k] = g_rx_buffer[k];
                }
                g_last_error_len = n;
                if (g_frame_maxgap > g_max_gap_all) g_max_gap_all = g_frame_maxgap;
                modbus_finish_or_retry();
            }
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

/** @brief Cumulative send-timeouts (ENDTX lost, TX state force-reset) */
uint32_t modbus_get_tx_timeout_count(void) {
    return g_count_tx_timeout;
}

/** @brief Cumulative automatic re-sends after a bad frame / timeout */
uint32_t modbus_get_retry_count(void) {
    return g_count_retry;
}

/** @brief Cumulative RX bytes dropped because the buffer was full */
uint32_t modbus_get_overflow_count(void) {
    return g_count_overflow;
}

/** @brief Cumulative requests rejected for illegal parameters */
uint32_t modbus_get_reject_count(void) {
    return g_count_reject;
}

/**
 * @brief Last Modbus exception code seen (0 = none)
 * @note  Sticky: cleared only when a good frame arrives, so the application can
 *        poll it after the fact. Exception reply looks like: 0x83 code CRC.
 */
uint8_t modbus_get_last_exception(void) {
    return g_last_exception;
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
