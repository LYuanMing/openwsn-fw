#ifndef  __RENKE_WIND_SENSOR_H__
#define  __RENKE_WIND_SENSOR_H__

#include "modbus_rtu.h"

#define WIND_FIELD_MODBUS_ADDR              0x01
#define WIND_FIELD_DATA_ADDR                0x0000
#define WIND_FIELD_DATA_RANGE               0x10
#define CALIBRATION_PARAMS_ADDR             0x0050
#define CALIBRATION_PARAMS_RANGE            0x05
#define COMMUNICATION_PARAMS_ADDR           0x07D0
#define COMMUNICATION_PARAMS_RANGE          0x02
#define SENSOR_VERSION_ADDR                 0x07D5
#define SENSOR_VERSION_RANGE                0x02

typedef struct {
    uint16_t software_version;
    uint16_t hardware_version;
    volatile bool is_ready;
} dev_version_t;


typedef struct {
    uint16_t filter_length;     // 0x0050: 滤波长度
    float    coef_a;            // 0x0051-0x0052: 风速系数 A
    float    coef_b;            // 0x0053-0x0054: 风速系数 B
    volatile bool is_ready;     // 数据就绪标志位
} dev_calibration_param_t;

/**
 * @brief 2. 设备通信参数结构体 (对应 0x07D0 - 0x07D1)
 */
typedef struct {
    uint8_t  modbus_addr;       // 0x07D0: Modbus 设备地址
    uint32_t baud_rate;         // 0x07D1: 实际转换后的波特率数值 (如 9600)
    uint8_t  baud_rate_idx;     // 0x07D1: 原始波特率索引 (0-6)
    volatile bool is_ready;     // 数据就绪标志位
} dev_communication_param_t;

typedef struct {
    // --- 矢量平均指标 ---
    float vector_avg_wind_speed;      // 0x0000: 矢量平均风速 (m/s)
    float vector_avg_hor_direction;   // 0x0001: 矢量平均水平风向 (°)
    float vector_avg_ver_direction;   // 0x0002: 矢量平均垂直风向 (°)

    // --- 标量平均指标 ---
    float scalar_avg_wind_speed;      // 0x0003: 标量平均风速 (m/s)
    float scalar_avg_hor_direction;   // 0x0004: 标量平均水平风向 (°)
    float scalar_avg_ver_direction;   // 0x0005: 标量平均垂直风向 (°)

    // --- 基础环境指标 ---
    uint16_t wind_level;              // 0x0006: 风级
    float acoustic_temperature;       // 0x0007: 声温 (℃)
    float acoustic_velocity;          // 0x0008: 声速 (m/s)

    // --- 实时测值指标 ---
    float realtime_wind_speed;        // 0x000A: 实时风速 (m/s)
    float realtime_hor_direction;     // 0x000B: 实时水平风向 (°)
    float realtime_ver_direction;     // 0x000C: 实时垂直风向 (°)

    // --- 三维正交轴向风速 ---
    float x_axis_wind_speed;          // 0x000D: X轴风速 (m/s)
    float y_axis_wind_speed;          // 0x000E: Y轴风速 (m/s)
    float z_axis_wind_speed;          // 0x000F: Z轴风速 (m/s)

    volatile bool is_ready;           // 核心就绪标志位，供上层 OpenWSN 消费
} dev_wind_field_data_t;


extern dev_calibration_param_t calibration_params;
extern dev_communication_param_t communication_params;
extern dev_wind_field_data_t wind_field_data;
extern dev_version_t sensor_version;

void trigger_read_calibration_params(void);
void trigger_read_communication_params(void);
void trigger_read_wind_field_data(void);
void trigger_read_sensor_version(void);

#endif