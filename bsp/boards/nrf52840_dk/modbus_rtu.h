#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#include "board_info.h"
#include "board.h"
#include "nrf52840_bitfields.h"
#include "nrf52840.h"
#include <stdbool.h>

#define MODBUS_RX_BUF_SIZE    64

// 兜底静默断帧阈值：正常帧靠应答自报的"有效字节数"判整帧收齐（见 modbus_poll），
//   实测该传感器应答会在帧中间停顿 ~175-210ms（内部测量周期打断 UART 发送），
#define MODBUS_FRAME_GAP_MS   400
// 发出请求后等不到任何一个字节的超时
#define MODBUS_RX_TIMEOUT_MS  200
// 发送兜底超时：一次请求 8 字节 @4800bps 8N1 = 80bit ≈ 16.7ms。
//   超过 60ms 还没拿到 ENDTX 就判定发送事件丢了，状态机必须复位——
//   否则一次毛刺就让总线永久卡在 TX，数据永久停、而且没有任何计数可查。
#define MODBUS_TX_TIMEOUT_MS  60
// 失败重发 / 连续两帧之间要求的总线静默：Modbus 规定 3.5T，
//   4800bps 下 ≈ 8ms，这里取 12ms 留余量。
#define MODBUS_INTER_FRAME_GUARD_MS  12
// 一次事务内允许的立即重发次数（0 = 只能靠上层周期轮询重试）。
//   传感器自身约 3~5% 的应答会中途夭折（实测），不重发就得白等一个轮询周期。
#define MODBUS_MAX_RETRY      1
// RX 缓冲能容纳的最大寄存器数：(64 - 5) / 2 = 29（Modbus 协议上限是 125）
#define MODBUS_MAX_REG_CNT    ((MODBUS_RX_BUF_SIZE - 5) / 2)

// Modbus 异步总线状态机定义
typedef enum {
    MODBUS_STATE_IDLE = 0,      // 总线空闲，准备好发送新指令
    MODBUS_STATE_TX,            // 串口正在发送数据（EasyDMA在后台工作）
    MODBUS_STATE_RX_WAIT,       // 发送完毕，正在异步等待接收字节
    MODBUS_STATE_RETRY_WAIT,    // 本次事务失败，等满 3.5T 静默后立即重发同一请求
    MODBUS_STATE_PARSE          // 收齐或断帧，进入解析状态
} modbus_bus_state_t;

// 全局状态机指针：应用层做节流判断和打印用，统一在头文件声明，避免各处 extern 重复
extern modbus_bus_state_t g_bus_state;


/**
 * @brief 万能业务解析函数指针
 * @param buf 接收到的完整 Modbus 回应帧起始指针（包含地址码、功能码、数据区及CRC）
 * @param len 回应帧的有效总字节数
 */
typedef void (*modbus_parser_t)(const uint8_t *buf, uint16_t len);

/* ======================================================================== */
/* 🚀 核心 API                                */
/* ======================================================================== */

/**
 * @brief 初始化 UARTE1 外设寄存器及 NVIC 中断
 * @note 绑定 4800 波特率, 8N1 格式，启动单字节 EasyDMA 循环监听
 */
void modbus_init(void);

/**
 * @brief 状态机主轮询引擎
 * @note 必须挂在主循环(背景任务)中频繁调用，内部无任何阻塞，耗时低于 1微秒
 */
void modbus_poll(void);

/**
 * @brief 通用非阻塞读取保持寄存器接口 (功能码 0x03)
 * @param dev_addr       目标从机设备地址
 * @param start_reg      起始寄存器十六进制地址
 * @param reg_cnt        准备读取的寄存器连续个数（1 ~ MODBUS_MAX_REG_CNT）
 * @param parse_callback 针对本次读取量身定制的业务解析函数指针（回调）
 * @return true 发送成功，EasyDMA 已接管总线；false 总线正忙或参数非法，拒绝本次请求
 * @note  返回值必须被检查：漏检 = 请求被静默丢弃，且不留任何痕迹
 */
bool modbus_read_registers(uint8_t dev_addr, uint16_t start_reg, uint16_t reg_cnt, modbus_parser_t parse_callback);

/* ======================================================================== */
/*  诊断 / 节流接口（应用层用）                      */
/* ======================================================================== */

/** @brief 系统毫秒时间戳，供应用层做请求节流（32 位回绕，求差即可） */
uint32_t modbus_millis(void);

/** @brief 累计解析成功的合法帧数 */
uint32_t modbus_get_frame_count(void);

/** @brief 累计超时次数（发出去后一个字节都没收到） */
uint32_t modbus_get_timeout_count(void);

/** @brief 累计坏帧次数（长度/CRC/功能码异常，含 0x83 异常应答） */
uint32_t modbus_get_error_count(void);

/** @brief 累计 UARTE 硬件错误次数（帧错误/溢出） */
uint32_t modbus_get_uart_error_count(void);

/** @brief 累计发送超时次数（ENDTX 丢失，状态机被强制复位） */
uint32_t modbus_get_tx_timeout_count(void);

/** @brief 累计立即重发次数（坏帧/超时后自动补发） */
uint32_t modbus_get_retry_count(void);

/** @brief 累计 RX 缓冲溢出字节数（收到的字节超出 MODBUS_RX_BUF_SIZE 被丢弃） */
uint32_t modbus_get_overflow_count(void);

/** @brief 累计被拒绝的非法请求次数（reg_cnt 越界 / 回调为空） */
uint32_t modbus_get_reject_count(void);

/** @brief 最后一个 Modbus 异常应答的异常码（0 = 没有异常应答） */
uint8_t modbus_get_last_exception(void);

/**
 * @brief 取出最后一条坏帧的现场字节（例如传感器回 0x83 异常码时很有用）
 * @return 实际拷贝的字节数
 */
uint16_t modbus_get_last_error(uint8_t *dst, uint16_t dst_size);

/** @brief 开机以来观测到的最大帧内字节间隔(ms) */
uint16_t modbus_get_max_frame_gap(void);

/** @brief 最后一帧坏帧内的最大字节间隔(ms) */
uint16_t modbus_get_last_error_maxgap(void);

#endif
