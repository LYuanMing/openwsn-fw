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

// Modbus 异步总线状态机定义
typedef enum {
    MODBUS_STATE_IDLE = 0,      // 总线空闲，准备好发送新指令
    MODBUS_STATE_TX,            // 串口正在发送数据（EasyDMA在后台工作）
    MODBUS_STATE_RX_WAIT,       // 发送完毕，正在异步等待接收字节
    MODBUS_STATE_PARSE          // 10ms总线无数据（满足3.5T断帧），进入解析状态
} modbus_bus_state_t;


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
 * @param reg_cnt        准备读取的寄存器连续个数
 * @param parse_callback 针对本次读取量身定制的业务解析函数指针（回调）
 * @return true 发送成功，EasyDMA 已接管总线；false 总线正忙，拒绝本次叠加发送
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

/** @brief 累计坏帧次数（长度/CRC/功能码异常） */
uint32_t modbus_get_error_count(void);

/** @brief 累计 UARTE 硬件错误次数（帧错误/溢出） */
uint32_t modbus_get_uart_error_count(void);

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
