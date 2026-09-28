/**
 * 01bsp_modbus.c -- RS-CFSFX-N01-3D 三维超声波风速风向变送器 (Modbus-RTU) 采集 demo
 *
 * 硬件：nRF52840-DK
 *   P0.27 (UARTE1 TX) -> TTL 转 485 模块 TTL_RXD
 *   P0.26 (UARTE1 RX) <- TTL 转 485 模块 TTL_TXD
 *   调试口：UART0 (P0.06/P0.08) = DK 板载 J-Link VCOM，115200 8N1
 *
 * 传感器：默认地址 0x01，默认 4800 bps，8N1，功能码 0x03
 *
 * ⚠ 历史坑：曾经把 uart_setCallbacks() 注释掉却调用 uart_send_string()，
 *    uart_writeByte() 后会在 while(!uart_tx_done) 上死等（cb_uartTxDone 永远不来），
 *    表现为"固件卡死、再也不出数据"。所以打印之前必须先注册回调，且发送等待带兜底。
 *
 * ⚠ 打印一律不用 %f：SEGGER RTL 的浮点 printf 支持不一定打开，定点数用整数运算自己打。
 */

#include "board.h"
#include "renke_wind_sensor.h"
#include "uart.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "stdint.h"

extern dev_calibration_param_t calibration_params;
extern dev_communication_param_t communication_params;
extern dev_wind_field_data_t wind_field_data;
extern dev_sensor_version_t sensor_version;
extern modbus_bus_state_t g_bus_state;

//=========================== 可调参数 ======================================

#define MODBUS_POLL_PERIOD_MS   200     // 两次 Modbus 请求最小间隔
#define REPORT_PERIOD_MS        1000    // 风场数据打印周期（约等于传感器更新率）
#define STATS_PERIOD_MS         2000    // 链路心跳打印周期
#define UART_TX_GUARD_LOOPS     2000000 // 单字节发送等待兜底上限

//=========================== UART 变量 =====================================

static char             s_tx_buf[512];
static uint16_t         s_tx_len = 0;
static volatile uint8_t uart_tx_done = 1;

//=========================== UART 回调 =====================================

void cb_uartTxDone(void) {
    uart_tx_done = 1;
}

uint8_t cb_uartRxCb(void) {
    // 调试口收到的字节直接丢弃（本 demo 只用它做输出）
    (void)uart_readByte();
    return 0;
}

//=========================== UART 辅助函数 =================================

// 阻塞式发送字符串，靠 cb_uartTxDone 置位推进；带兜底计数，任何情况都不许卡死
static void uart_send_string(const char *str) {
    while (*str) {
        uart_tx_done = 0;
        uart_writeByte((uint8_t)*str++);
        for (uint32_t guard = 0; uart_tx_done == 0 && guard < UART_TX_GUARD_LOOPS; guard++) {
            // 等 TXDRDY 中断
        }
    }
}

static void tx_reset(void) {
    s_tx_len = 0;
    s_tx_buf[0] = '\0';
}

// 攒格式化文本（不立即发送），攒完一次性发，避免把主循环切得太碎
static void tx_addf(const char *fmt, ...) {
    if (s_tx_len >= sizeof(s_tx_buf) - 1) return;

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(&s_tx_buf[s_tx_len], sizeof(s_tx_buf) - s_tx_len, fmt, ap);
    va_end(ap);

    if (n > 0) {
        s_tx_len += (uint16_t)n;
        if (s_tx_len > sizeof(s_tx_buf) - 1) {
            s_tx_len = sizeof(s_tx_buf) - 1;
        }
    }
}

static void tx_flush(void) {
    uart_send_string(s_tx_buf);
    tx_reset();
}

static void tx_add_hex(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        tx_addf(" %02X", buf[i]);
    }
}

// 打印定点小数：不依赖 libc 的 %f，只用整数除法逐位输出
static void tx_add_fixed(float v, unsigned decimals) {
    static const uint32_t pow10[4] = { 1u, 10u, 100u, 1000u };
    uint32_t scale, whole, frac, p;

    if (decimals > 3) decimals = 3;
    scale = pow10[decimals];

    bool  neg = (v < 0.0f);
    float av  = neg ? -v : v;

    if (av > 999999.0f) {
        tx_addf("OVF");
        return;
    }

    whole = (uint32_t)av;
    frac  = (uint32_t)((av - (float)whole) * (float)scale + 0.5f);
    if (frac >= scale) {   // 四舍五入进位
        whole++;
        frac -= scale;
    }

    if (neg && (whole != 0 || frac != 0)) {
        tx_addf("-");
    }
    tx_addf("%lu", (unsigned long)whole);

    if (decimals > 0) {
        p = scale / 10;
        tx_addf(".");
        while (p > 0) {   // 逐位输出，自动补前导零
            tx_addf("%lu", (unsigned long)((frac / p) % 10u));
            p /= 10;
        }
    }
}

//=========================== 打印业务数据 ==================================

static void report_wind_data(void)
{
    tx_reset();
    tx_addf("\r\n===== RS-CFSFX-N01-3D wind @%lu ms =====\r\n", (unsigned long)modbus_millis());

    tx_addf("vec  : ");
    tx_add_fixed(wind_field_data.vector_avg_wind_speed, 2);
    tx_addf(" m/s  ");
    tx_add_fixed(wind_field_data.vector_avg_hor_direction, 1);
    tx_addf(" deg  ");
    tx_add_fixed(wind_field_data.vector_avg_ver_direction, 1);
    tx_addf(" deg\r\n");

    tx_addf("sca  : ");
    tx_add_fixed(wind_field_data.scalar_avg_wind_speed, 2);
    tx_addf(" m/s  ");
    tx_add_fixed(wind_field_data.scalar_avg_hor_direction, 1);
    tx_addf(" deg  ");
    tx_add_fixed(wind_field_data.scalar_avg_ver_direction, 1);
    tx_addf(" deg\r\n");

    tx_addf("env  : level %u | ", (unsigned)wind_field_data.wind_level);
    tx_add_fixed(wind_field_data.acoustic_temperature, 2);
    tx_addf(" C | ");
    tx_add_fixed(wind_field_data.acoustic_velocity, 2);
    tx_addf(" m/s\r\n");

    tx_addf("rt   : ");
    tx_add_fixed(wind_field_data.realtime_wind_speed, 2);
    tx_addf(" m/s  ");
    tx_add_fixed(wind_field_data.realtime_hor_direction, 1);
    tx_addf(" deg  ");
    tx_add_fixed(wind_field_data.realtime_ver_direction, 1);
    tx_addf(" deg\r\n");

    tx_addf("xyz  : X ");
    tx_add_fixed(wind_field_data.x_axis_wind_speed, 2);
    tx_addf("  Y ");
    tx_add_fixed(wind_field_data.y_axis_wind_speed, 2);
    tx_addf("  Z ");
    tx_add_fixed(wind_field_data.z_axis_wind_speed, 2);
    tx_addf(" m/s\r\n");

    tx_addf("raw32:");
    tx_add_hex(wind_field_data.raw, WIND_FIELD_RAW_LEN);
    tx_addf("\r\n");
    tx_flush();
}

// 链路心跳：用来区分"固件卡死"和"传感器不回话"
static void report_link_stats(uint32_t now_ms, uint32_t last_err_cnt)
{
    uint8_t err_buf[MODBUS_RX_BUF_SIZE];

    tx_reset();
    tx_addf("[STAT] %4lu s | frame=%lu timeout=%lu badframe=%lu uart_err=%lu maxgap=%lums | state=%u\r\n",
            (unsigned long)(now_ms / 1000),
            (unsigned long)modbus_get_frame_count(),
            (unsigned long)modbus_get_timeout_count(),
            (unsigned long)modbus_get_error_count(),
            (unsigned long)modbus_get_uart_error_count(),
            (unsigned long)modbus_get_max_frame_gap(),
            (unsigned)g_bus_state);

    // 坏帧计数有变化，就把现场打出来看看传感器到底回了什么
    //（0x83 = 非法数据地址，0x02 = 非法寄存器数量）
    if (modbus_get_error_count() != last_err_cnt) {
        uint16_t n = modbus_get_last_error(err_buf, sizeof(err_buf));
        tx_addf("[BAD ]");
        tx_add_hex(err_buf, n);
        tx_addf("  n=%u maxgap=%ums\r\n", (unsigned)n, (unsigned)modbus_get_last_error_maxgap());
    }
    tx_flush();
}

//=========================== main ==========================================

int mote_main(void)
{
    board_init();   // 内部已 uart_init()：调试口 = DK 板载 VCOM (115200 8N1)

    // ⚠ 必须在任何 uart_send_string 之前注册回调，否则发送等待会死锁
    uart_setCallbacks(cb_uartTxDone, cb_uartRxCb);
    uart_enableInterrupts();

    tx_reset();
    tx_addf("\r\n[BOOT] Renke RS-CFSFX-N01-3D / Modbus-RTU 4800 8N1 addr=0x01\r\n");
    tx_addf("[BOOT] sensor: UARTE1 P0.27(TX)/P0.26(RX) | debug: UART0 P0.06/P0.08 = J-Link VCOM\r\n");
    tx_flush();

    modbus_init();

    // 上电先读一次版本寄存器，确认 485 链路通不通
    trigger_read_sensor_version();

    uint32_t t_req     = modbus_millis();
    uint32_t t_report  = t_req;
    uint32_t t_stats   = t_req;
    uint32_t last_errc = 0;

    while (1) {
        // 核心动力源：必须高频轮询，整流状态机全靠它推进
        modbus_poll();

        // ---- 1. 传感器版本（上电只读一次）----
        if (sensor_version.is_ready) {
            sensor_version.is_ready = false;
            tx_reset();
            tx_addf("\r\n[VER ] software=%u hardware=%u\r\n",
                    (unsigned)sensor_version.software_version,
                    (unsigned)sensor_version.hardware_version);
            tx_flush();
        }

        // ---- 2. 风场数据：按 REPORT_PERIOD_MS 打印一次 ----
        if (wind_field_data.is_ready) {
            wind_field_data.is_ready = false;
            if ((uint32_t)(modbus_millis() - t_report) >= REPORT_PERIOD_MS) {
                t_report = modbus_millis();
                report_wind_data();
            }
        }

        // ---- 3. 链路心跳 ----
        if ((uint32_t)(modbus_millis() - t_stats) >= STATS_PERIOD_MS) {
            uint32_t now = modbus_millis();
            report_link_stats(now, last_errc);
            last_errc = modbus_get_error_count();
            t_stats   = now;
        }

        // ---- 4. 发起下一次请求：总线空闲且距上次请求已够久 ----
        if (g_bus_state == MODBUS_STATE_IDLE &&
            (uint32_t)(modbus_millis() - t_req) >= MODBUS_POLL_PERIOD_MS) {
            t_req = modbus_millis();
            trigger_read_wind_field_data();
        }
    }
    return 0;
}
