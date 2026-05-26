#ifndef _ENERGY_AWARE_H_
#define _ENERGY_AWARE_H_

#include "stdint.h"

#define CAPACITANCE 10     // unit: mF

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    uint16_t e_surplus;
    uint8_t throttle_factor;
}energy_vars_t;

extern energy_vars_t energy_vars;

void task_energyMeasurement(void);
void energyMeasurementInit(void);


#ifdef __cplusplus
}
#endif

#endif