/**
    \brief Definition of the nrf52480 ADC driver.
    \author Frank Senf <frank.senf@imms.de>, July 2018.
    \author Tengfei Chang <tengfeichang@hkust-gz.edu.cn>, April, 2023
*/


#include "adc.h"


//=========================== defines =========================================

//=========================== typedef =========================================

//=========================== variables =======================================

volatile int16_t adc_buffer;
//=========================== prototype =======================================

//=========================== public ==========================================
void adc_init(void) {
    // 1. 配置输入引脚 (P0.02 = AIN0)
    NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput0; 
    
    // 2. 配置通道 0 的电气特性
    NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos)  |
                              (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos)|
                              (SAADC_CH_CONFIG_TACQ_10us       << SAADC_CH_CONFIG_TACQ_Pos)  |
                              (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos)  |
                              (SAADC_CH_CONFIG_BURST_Disabled  << SAADC_CH_CONFIG_BURST_Pos);

    // 3. 设置全局分辨率 (10位)
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_10bit;

    // 4. 设置采样率模式为 Task 触发
    NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos);

    // 5. 使能 SAADC
    NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

    // 6. 校准偏移（保持同步死等，因为只在开机初始化时跑一次）
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
    while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0);
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;

    // =========================================================================
    // 新增：配置 SAADC 中断
    // =========================================================================
    NRF_SAADC->INTENSET = SAADC_INTENSET_STARTED_Msk | SAADC_INTENSET_END_Msk; // 开启 END 事件中断（DMA完成时触发）
    NVIC_SetPriority(SAADC_IRQn, 3);              // 设置一个合理的低优先级，不打扰无线射频
    NVIC_EnableIRQ(SAADC_IRQn);                   // 使能内核的 SAADC 中断
}

void adc_start_sampling(void) {
    NRF_SAADC->RESULT.PTR = (uint32_t)&adc_buffer;
    NRF_SAADC->RESULT.MAXCNT = 1; 

    NRF_SAADC->TASKS_START = 1;
}

//=========================== private =========================================