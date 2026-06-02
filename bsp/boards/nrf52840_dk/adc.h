/**
    \brief Declaration of the nrf52480 ADC driver.
    \author Frank Senf <frank.senf@imms.de>, July 2018.
*/

#ifndef __ADC_H__
#define __ADC_H__

#include "board_info.h"

//=========================== define ==========================================
#define VOLTAGE_REF 600
#define VOLTAGE_GAIN 6
#define ADC_RAW_TO_MV(raw)  (((uint32_t)(raw) * (VOLTAGE_REF * VOLTAGE_GAIN)) >> 10)
//=========================== typedef =========================================

//=========================== module variables ================================
extern volatile int16_t adc_buffer;
//=========================== prototypes ======================================

void adc_init(void);
void adc_start_sampling(void);

#endif // __ADC_SENSOR_H__
