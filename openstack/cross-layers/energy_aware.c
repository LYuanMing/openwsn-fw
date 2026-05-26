#include "energy_aware.h"
#include "openserial.h"
#include "IEEE802154E.h"
#include "idmanager.h"

energy_vars_t energy_vars;
extern ieee154e_vars_t ieee154e_vars;

static uint8_t index;

void energyMeasurementInit(void)
{
    index = 0;
    energy_vars.e_surplus = 0;
    energy_vars.throttle_factor = 0;
}

void task_energyMeasurement(void)
{
    if (idmanager_getIsDAGroot() == TRUE) {
        energy_vars.throttle_factor = 0;
    } else {
        openserial_printf("asn: %d,energy_measurement\r\n", ieee154e_vars.slotOffset);
        index += 1;
        if(index % 8 == 0) {
            energy_vars.throttle_factor =3;
            openserial_printf("energy throttle factor: %d\r\n", energy_vars.throttle_factor);
        } 
    }
}

