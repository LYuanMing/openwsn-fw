#ifndef _ENERGY_AWARE_H_
#define _ENERGY_AWARE_H_

#include "stdint.h"
#include "IEEE802154E.h"
#include "idmanager.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAPACITANCE_MF             10     // unit: mF
#define CALCULATE_ENERGY_UJ(volt_mv) \
    ((uint32_t)(((uint32_t)(CAPACITANCE_MF) * (uint32_t)(volt_mv) * (uint32_t)(volt_mv)) / 2000))


#define MAX_VOLTAGE_MV             4200   // uint: mV
#define MIN_VOLTAGE_MV             1800   // uint: mV
#define E_MAX                   CALCULATE_ENERGY_UJ(MAX_VOLTAGE_MV) // uint: uJ
#define E_MIN                   CALCULATE_ENERGY_UJ(MIN_VOLTAGE_MV) // uint: uJ

#define W_PUNISHMENT            1
#define LAMBDA_SENSITIVITY      2




extern const uint16_t F_ENERGY_LUT[];
extern const uint16_t F_ENERGY_LUT_SIZE;

typedef struct{
    uint32_t e_surplus_uJ; // unit: uJ
    uint32_t voltage_mV;
    uint8_t throttle_factor;
}energy_vars_t;

extern energy_vars_t energy_vars;

void task_energyMeasurement(void);
void energyMeasurementInit(void);
uint16_t rpl_energy_penalty(void);
void update_throttle_factor(void);

#ifdef __cplusplus
}
#endif

#endif