/**
    \brief Definition of the nrf52480 ADC driver.
    \author Frank Senf <frank.senf@imms.de>, July 2018.
    \author Tengfei Chang <tengfeichang@hkust-gz.edu.cn>, April, 2023
*/


#include "adc.h"

static int16_t adc_buffer;
//=========================== defines =========================================

//=========================== typedef =========================================

//=========================== variables =======================================

//=========================== prototype =======================================

//=========================== public ==========================================

void adc_init(void) {
    // 1. Configure input pin for channel 0 (P0.02 = AIN0)
    NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput0; 
    
    // 2. Configure electrical characteristics for channel 0
    // - GAIN: 1/6 (Input range up to 3.6V with 0.6V reference)
    // - REFSEL: Internal 0.6V reference
    // - TACQ: 10us acquisition time (Suitable for most source impedances)
    // - MODE: Single-ended (SE)
    // - BURST: Disabled (Single sample per trigger)
    NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_GAIN_Gain1_6    << SAADC_CH_CONFIG_GAIN_Pos)  |
                              (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos)|
                              (SAADC_CH_CONFIG_TACQ_10us       << SAADC_CH_CONFIG_TACQ_Pos)  |
                              (SAADC_CH_CONFIG_MODE_SE         << SAADC_CH_CONFIG_MODE_Pos)  |
                              (SAADC_CH_CONFIG_BURST_Disabled  << SAADC_CH_CONFIG_BURST_Pos);

    // 3. Set global resolution (8, 10, or 12 bit)
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_10bit;

    // 4. Set sample rate mode
    // MODE_Task: Conversion is triggered manually by TASKS_SAMPLE
    NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos);

    // 5. Enable the SAADC peripheral
    NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);

    // 6. Calibrate Offset
    // Recommended after initialization to minimize measurement error
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
    while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0); // Wait for calibration to complete
    NRF_SAADC->EVENTS_CALIBRATEDONE = 0;          // Clear the event flag
}

int16_t adc_read(void) {
    // 1. Configure the EasyDMA target buffer
    // PTR must point to a RAM address
    NRF_SAADC->RESULT.PTR = (uint32_t)&adc_buffer;
    NRF_SAADC->RESULT.MAXCNT = 1; // Number of samples to write to RAM

    // 2. Start the SAADC and wait for the STARTED event
    // This powers up the ADC and prepares the DMA
    NRF_SAADC->TASKS_START = 1;
    while (NRF_SAADC->EVENTS_STARTED == 0);
    NRF_SAADC->EVENTS_STARTED = 0;

    // 3. Trigger the actual sampling
    // This starts the conversion of the configured channel(s)
    NRF_SAADC->TASKS_SAMPLE = 1;
    while (NRF_SAADC->EVENTS_END == 0);
    NRF_SAADC->EVENTS_END = 0;

    // 4. Stop the SAADC to save power
    // The ADC remains enabled but the DMA and internal LDOs are paused
    NRF_SAADC->TASKS_STOP = 1;
    while (NRF_SAADC->EVENTS_STOPPED == 0);
    NRF_SAADC->EVENTS_STOPPED = 0;

    return adc_buffer;
}

//=========================== private =========================================