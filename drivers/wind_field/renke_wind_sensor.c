#include "renke_wind_sensor.h"

dev_calibration_param_t calibration_params = {0};
dev_communication_param_t communication_params = {0};
dev_wind_field_data_t wind_field_data = {0};
dev_sensor_version_t sensor_version = {0};

void parse_calibration_param_callback(const uint8_t* buf, uint16_t len);
void parse_communication_param_callback(const uint8_t* buf, uint16_t len);
void parse_wind_field_data_callback(const uint8_t* buf, uint16_t len);
void parse_sensor_version_callback(const uint8_t* buf, uint16_t len);


bool trigger_read_calibration_params(void)
{
    return modbus_read_registers(WIND_FIELD_MODBUS_ADDR, CALIBRATION_PARAMS_ADDR, CALIBRATION_PARAMS_RANGE, parse_calibration_param_callback);
}

bool trigger_read_communication_params(void)
{
    return modbus_read_registers(WIND_FIELD_MODBUS_ADDR, COMMUNICATION_PARAMS_ADDR, COMMUNICATION_PARAMS_RANGE, parse_communication_param_callback);
}

bool trigger_read_wind_field_data(void)
{
    return modbus_read_registers(WIND_FIELD_MODBUS_ADDR, WIND_FIELD_DATA_ADDR, WIND_FIELD_DATA_RANGE, parse_wind_field_data_callback);
}

bool trigger_read_sensor_version(void)
{
    return modbus_read_registers(WIND_FIELD_MODBUS_ADDR, SENSOR_VERSION_ADDR, SENSOR_VERSION_RANGE, parse_sensor_version_callback);
}

static float uint8_to_float_abcd(const uint8_t *p) {
    uint32_t temp = ((uint32_t)p[0] << 24) |
                    ((uint32_t)p[1] << 16) |
                    ((uint32_t)p[2] << 8)  |
                    (uint32_t)p[3];
    float f;
    memcpy(&f, &temp, 4);
    return f;
}


void parse_calibration_param_callback(const uint8_t* buf, uint16_t len)
{
    uint8_t byte_cnt = buf[2]; // Modbus 帧第3字节为数据长度
    // 长度先行保护：byte_cnt 是帧里的声明值，不保证真的收到了那么多字节
    if (byte_cnt == 10 && len >= (uint16_t)(3 + byte_cnt + 2)) {
        // 1. 提取 0x0050 滤波长度 (INT16U)
        calibration_params.filter_length = (buf[3] << 8) | buf[4];
        
        // 2. 提取 0x0051-0x0052 风速系数 A (Float) -> 传入 buf[5] 起始的4个字节
        // 优先选用 uint8_to_float_abcd，如果出来的值是天文数字，直接改换 cdab 函数
        calibration_params.coef_a = uint8_to_float_abcd(&buf[5]);
        
        // 3. 提取 0x0053-0x0054 风速系数 B (Float) -> 传入 buf[9] 起始的4个字节
        calibration_params.coef_b = uint8_to_float_abcd(&buf[9]);
        
        // 挂起就绪旗帜
        calibration_params.is_ready = true;
    }
}

void parse_communication_param_callback(const uint8_t* buf, uint16_t len)
{
    uint8_t byte_cnt = buf[2];
    if (byte_cnt == 4 && len >= (uint16_t)(3 + byte_cnt + 2)) {
        // 1. 提取 0x07D0 Modbus 地址 (INT8U 存储在 16 位寄存器的低字节)
        communication_params.modbus_addr = buf[4]; 
        
        // 2. 提取 0x07D1 波特率索引
        communication_params.baud_rate_idx = buf[6];
        
        // 3. 将索引值动态翻译成可供驱动直接使用的实际通信速率
        switch (communication_params.baud_rate_idx) {
            case 0: communication_params.baud_rate = 2400;   break;
            case 1: communication_params.baud_rate = 4800;   break;
            case 2: communication_params.baud_rate = 9600;   break;
            case 3: communication_params.baud_rate = 19200;  break;
            case 4: communication_params.baud_rate = 38400;  break;
            case 5: communication_params.baud_rate = 57600;  break;
            case 6: communication_params.baud_rate = 115200; break;
            default: communication_params.baud_rate = 4800;  break; // 异常安全缺省
        }
        
        communication_params.is_ready = true;
    }
}

void parse_wind_field_data_callback(const uint8_t* buf, uint16_t len)
{
    uint8_t byte_cnt = buf[2]; // 获取回传的数据字节数
    // 长度先行保护：下面最多读到 buf[34]，必须先确认帧真的有这么长
    
    if (byte_cnt == 32 && len >= (uint16_t)(3 + byte_cnt + 2)) { // 严格核验是否收齐 32 字节数据

        // 0. 留一份原始字节，方便上板核对标度/字节序
        memcpy(wind_field_data.raw, &buf[3], WIND_FIELD_RAW_LEN);
        
        // 1. 矢量平均指标 (0x0000 - 0x0002)
        wind_field_data.vector_avg_wind_speed    = (float)((buf[3]  << 8) | buf[4])  / 100.0f;
        wind_field_data.vector_avg_hor_direction = (float)((buf[5]  << 8) | buf[6])  / 10.0f;
        wind_field_data.vector_avg_ver_direction = (float)((buf[7]  << 8) | buf[8])  / 10.0f;

        // 2. 标量平均指标 (0x0003 - 0x0005)
        wind_field_data.scalar_avg_wind_speed    = (float)((buf[9]  << 8) | buf[10]) / 100.0f;
        wind_field_data.scalar_avg_hor_direction = (float)((buf[11] << 8) | buf[12]) / 10.0f;
        wind_field_data.scalar_avg_ver_direction = (float)((buf[13] << 8) | buf[14]) / 10.0f;

        // 3. 风级 (0x0006)
        wind_field_data.wind_level = (buf[15] << 8) | buf[16];

        // 4. 声温 (0x0007) ⚡ 必须强转 int16_t 处理符号位
        int16_t raw_temp = (int16_t)((buf[17] << 8) | buf[18]);
        wind_field_data.acoustic_temperature = (float)raw_temp / 10.0f;

        // 5. 声速 (0x0008)
        wind_field_data.acoustic_velocity = (float)((buf[19] << 8) | buf[20]) / 100.0f;

        // 💡 寄存器 0x0009 属于空洞占位，数据处于 buf[21] 和 buf[22]，直接跳过不予理踩

        // 6. 实时测值指标 (0x000A - 0x000C)
        wind_field_data.realtime_wind_speed    = (float)((buf[23] << 8) | buf[24]) / 100.0f;
        wind_field_data.realtime_hor_direction = (float)((buf[25] << 8) | buf[26]) / 10.0f;
        wind_field_data.realtime_ver_direction = (float)((buf[27] << 8) | buf[28]) / 10.0f;

        // 7. 三维轴向风速 (0x000D - 0x000F) ⚡ 必须强转 int16_t 处理正反向风速
        int16_t raw_x_speed = (int16_t)((buf[29] << 8) | buf[30]);
        int16_t raw_y_speed = (int16_t)((buf[31] << 8) | buf[32]);
        int16_t raw_z_speed = (int16_t)((buf[33] << 8) | buf[34]);

        wind_field_data.x_axis_wind_speed = (float)raw_x_speed / 100.0f;
        wind_field_data.y_axis_wind_speed = (float)raw_y_speed / 100.0f;
        wind_field_data.z_axis_wind_speed = (float)raw_z_speed / 100.0f;

        // 打上帧身份：序号 + 本地时间戳。上层靠它们才能区分"刚到的数"和"卡住的旧数"
        wind_field_data.t_ms = modbus_millis();
        wind_field_data.seq++;

        // 释放接力棒，通知网络层或主循环消费数据
        wind_field_data.is_ready = true;
    }
}


void parse_sensor_version_callback(const uint8_t* buf, uint16_t len)
{
    uint8_t byte_cnt = buf[2]; // 获取回传的数据字节数
    if (byte_cnt == 4 && len >= (uint16_t)(3 + byte_cnt + 2)) { 
        // 补全逻辑：高位左移 8 位拼接低位
        sensor_version.software_version = (buf[3] << 8) | buf[4];
        sensor_version.hardware_version = (buf[5] << 8) | buf[6];
        
        // 拉高就绪旗帜
        sensor_version.is_ready = true;
    }
}


uint32_t wind_field_data_age_ms(void) {
    if (wind_field_data.seq == 0) {
        return 0xFFFFFFFFu;      // never received a frame: treat as infinitely old
    }
    return (uint32_t)(modbus_millis() - wind_field_data.t_ms);
}

bool wind_field_data_is_fresh(uint32_t max_age_ms) {
    return (wind_field_data.seq != 0) && (wind_field_data_age_ms() <= max_age_ms);
}
