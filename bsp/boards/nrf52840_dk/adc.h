/**
    \brief Declaration of the nrf52480 ADC driver.
    \author Frank Senf <frank.senf@imms.de>, July 2018.
*/

#ifndef __ADC_H__
#define __ADC_H__

#include "board_info.h"
#include "board.h"
#include "nrf52840_bitfields.h"
#include "nrf52840.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== module variables ================================

//=========================== prototypes ======================================

void adc_init(void);
int16_t adc_read(void);

#endif // __ADC_SENSOR_H__
