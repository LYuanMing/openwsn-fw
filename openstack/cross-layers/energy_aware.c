#include "energy_aware.h"
#include "openserial.h"
#include "adc.h"
#include "debugpins.h"
#include "opentimers.h"

energy_vars_t energy_vars;
extern ieee154e_vars_t ieee154e_vars;

// renamed from `index` for toolchain compatibility: newlib declares the legacy
// BSD function index() in <strings.h> (pulled in via board_info.h -> <string.h>),
// which collides with a file-scope variable of that name. SEGGER's libc does
// not declare index(), so the original name compiled under Embedded Studio.
static uint8_t ea_index;
static uint8_t direction = 0;
static uint16_t current_voltage = 4200;
// set when the working point is changed at run time over the serial port
// ('V' command); it suppresses the scheduled two-phase join switch, which
// would otherwise overwrite the operator's value at EA_JOIN_SLOTS
static uint8_t voltage_pinned = 0;
#define FIXED_ENERGY 0
#define ENERGY_CYCLE 0

// ---- two-phase join (user idea, 2026-09-26) -------------------------------
// "join at full duty, then switch to the working point after a given ASN"
//
// Why: at 2200 mV update_throttle_factor() yields throttle_factor=3, i.e. the
// node uses dedicated cells in only 1/8 of the slotframes (IEEE802154E.c:1017;
// the shared/minimal cell is exempt), so the join is contended.  The per-leaf
// isolation test (tools/run_leaf_isolation.py, 2026-09-26, SF53 V2200) showed
// every leaf CAN join alone (+6.5..+12.1 uA) but the N=5 production runs sit
// 7.3 uA below the sum of the five isolated deltas - i.e. the 5th child is not
// *unreachable*, it just cannot establish itself while all five contend.
//
// Phase 1: current_voltage = EA_JOIN_MV (>=3500 -> throttle_factor 0, full
//          duty) so the whole topology forms and every child holds a cell.
// Phase 2: once the 32-bit ASN counter reaches EA_JOIN_SLOTS, current_voltage
//          becomes EA_TARGET_MV (= the working point apply_config sets) and the
//          throttle re-applies.  The report window [3600, 5400] s is well after
//          the switch, so the metric still describes the throttled state - the
//          question it answers becomes "is the 5-child state SUSTAINABLE at
//          this working point", which is the quantity the matrix is about.
//
// Compiled out unless apply_config.py --join-seconds is given (EA_JOIN_MV 0).
// 1 slot = 10 ms (PORT_TsSlotDuration 328 ticks @ 32768 Hz) -> 100 slots/s.
#ifndef EA_JOIN_MV
#define EA_JOIN_MV 0
#endif
#ifndef EA_TARGET_MV
#define EA_TARGET_MV 4200
#endif
#ifndef EA_JOIN_SLOTS
#define EA_JOIN_SLOTS 4294967295u
#endif

const uint16_t F_ENERGY_LUT[] = {
    // the energy consumed from 0 (full) to 1 (empty)
    1024, 1090, 1161, 1236, 1316, 1401, 1491, 1588, // 0.0000 to 0.21875
    1689, 1799, 1915, 2038, 2170, 2311, 2460, 2620, // 0.2500 to 0.46875
    2783, 2963, 3154, 3358, 3575, 3806, 4052, 4314, // 0.5000 to 0.71875
    4593, 4890, 5206, 5543, 5901, 6283, 6689, 7122, // 0.7500 to 0.96875
    7566  // Index 32: Enorm = 1.0 (1.8V), f = e^2.0 * 1024 = 7566
};
const uint16_t F_ENERGY_LUT_SIZE =
    sizeof(F_ENERGY_LUT)/sizeof(F_ENERGY_LUT[0]);

void energyMeasurementInit(void)
{
    ea_index = 0;
    energy_vars.e_surplus_uJ = 0;
    energy_vars.throttle_factor = 0;
    // The DAG root never enters the sampling branch of task_energyMeasurement(),
    // so seed the reported working point here; the host reads it back with a
    // bare 'V' command and shows it next to the root in the topology view.
    energy_vars.voltage_mV = current_voltage;
    adc_init();
}

// Runtime working-point override, called from the serial command handler
// (openserial.c, SERFRAME_PC2MOTE_SETVOLTAGE).  Clamps to the same range the
// compile-time configuration uses, then makes the value effective immediately:
// waiting for the next ea_index%4==0 tick would leave the throttle factor stale.
void energy_aware_setVoltage(uint16_t mv)
{
    if (mv < MIN_VOLTAGE_MV) {
        mv = MIN_VOLTAGE_MV;
    }
    if (mv > MAX_VOLTAGE_MV) {
        mv = MAX_VOLTAGE_MV;
    }
    voltage_pinned = 1;
    current_voltage = mv;
    energy_vars.voltage_mV = mv;
    energy_vars.e_surplus_uJ = CALCULATE_ENERGY_UJ(mv);
    update_throttle_factor();
}

void task_energyMeasurement(void)
{
    if (idmanager_getIsDAGroot() == TRUE) {
        energy_vars.throttle_factor = 0;
        energy_vars.voltage_mV = current_voltage;
        energy_vars.e_surplus_uJ = E_MAX;
    } else {
        ea_index += 1;
        if(ea_index % 4 == 0) {
#if EA_JOIN_MV > 0
            // two-phase join: switch from the join working point to the target
            // one after EA_JOIN_SLOTS slots (see the EA_JOIN_* block above)
            if ((voltage_pinned == 0) && (current_voltage == EA_JOIN_MV)) {
                uint32_t asn_low;
                asn_low = (uint32_t)ieee154e_vars.asn.bytes0and1 |
                          ((uint32_t)ieee154e_vars.asn.bytes2and3 << 16);
                if (asn_low >= (uint32_t)EA_JOIN_SLOTS) {
                    current_voltage = EA_TARGET_MV;
                    openserial_printf("EA two-phase: join phase over, "
                                      "switching to target voltage\r\n");
                }
            }
#endif
#if FIXED_ENERGY 
            energy_vars.voltage_mV = current_voltage;
            energy_vars.e_surplus_uJ = CALCULATE_ENERGY_UJ(energy_vars.voltage_mV);
            update_throttle_factor();
#elif ENERGY_CYCLE
            if (direction == 0) {
                // ---------- 放电阶段 ----------
                if (current_voltage >= (MIN_VOLTAGE_MV + 100)) {
                    current_voltage -= 100;
                } else {
                    current_voltage = MIN_VOLTAGE_MV;
                    direction = 1;
                }
            } else {
                  // ---------- 充电阶段 ----------
                  if (current_voltage <= (MAX_VOLTAGE_MV - 100)) {
                      current_voltage += 100;
                  } else {
                      current_voltage = MAX_VOLTAGE_MV;
                      direction = 0;
                  }
            }
            energy_vars.voltage_mV = current_voltage;
            energy_vars.e_surplus_uJ = CALCULATE_ENERGY_UJ(current_voltage);
            update_throttle_factor();
#else
        adc_start_sampling();
#endif
            
            //openserial_printf("current energy: %d\r\n", energy_vars.e_surplus_uJ);
        } 
    }

}

// Implemented by openstack/03b-IPv6/icmpv6rpl.c; weak so a build without RPL
// still links.  Guarded before every call.
__attribute__ ((weak)) void icmpv6rpl_triggerDIO(uint8_t burst);

// ---- announcing a working-point change (decided 2026-11-11) ---------------
//
// Announcing at once is a *debug* aid.  The experiment measures energy and
// throughput with a voltage that moves continuously, and an immediate DIO per
// change would (a) turn the control traffic into a function of the voltage
// dynamics instead of the protocol, and (b) crowd the shared cell - the same
// failure mode that EB_PORTION=1 produced earlier.
//
// So: mode 0 (default) does nothing at all, i.e. the behaviour of the reference
// implementation; the rank still changes, it is simply announced on the normal
// DIO schedule.  Mode 1 sends DIO_BURST copies at once (measure switch latency),
// mode 2 sends one.  Both are floored by eaDioTminS seconds, because a floor of
// a few seconds is what keeps a continuously moving voltage from turning into a
// DIO per slotframe: even with the floor the worst-case extra load is one
// announcement per tmin, and only while the operator asked for it.
static uint8_t  eaDioMode = OPENWSN_EA_DIO_ON_VOLTAGE;
static uint8_t  eaDioTminS = 15;
static uint32_t eaLastForcedMs = 0;

// The floor is measured with the free-running counter rather than the ASN: a
// node that has not synchronized yet still counts slots, and when it locks onto
// the network the ASN can jump, which would defeat the floor exactly during a
// join.  opentimers_getValue() wraps sctimer_readCounter(), i.e. RTC0 with
// PRESCALER=0: 32768 ticks per second, not milliseconds (v26 got this wrong and
// the floor never fired: 11.8 s looked like 386k "ms").
#define EA_TICKS_PER_S 32768u

void energy_aware_setDioMode(uint8_t mode, uint8_t tminS)
{
    eaDioMode = mode;
    eaDioTminS = tminS;
}

uint8_t energy_aware_getDioMode(void)
{
    return eaDioMode;
}

uint8_t energy_aware_getDioTmin(void)
{
    return eaDioTminS;
}

void update_throttle_factor(void)
{
    uint8_t tf;

    if (energy_vars.voltage_mV >= 3500) {
        tf = 0;
    } else if (energy_vars.voltage_mV >= 3000) {
        tf = 1;
    } else if (energy_vars.voltage_mV >= 2500) {
        tf = 2;
    } else {
        tf = 3;
    }

    if (tf != energy_vars.throttle_factor) {
        energy_vars.throttle_factor = tf;
        if (eaDioMode != 0 && icmpv6rpl_triggerDIO != NULL) {
            uint32_t now = opentimers_getValue();
            if (eaDioTminS != 0 &&
                    (uint32_t)(now - eaLastForcedMs) <
                    (uint32_t)eaDioTminS * EA_TICKS_PER_S) {
                return;     // announced too recently: leave it to the schedule
            }
            eaLastForcedMs = now;
            icmpv6rpl_triggerDIO(eaDioMode == 1 ? 5 : 1);
        }
    }
}

void SAADC_IRQHandler(void) {
    if (NRF_SAADC->EVENTS_STARTED != 0) {
        NRF_SAADC->EVENTS_STARTED = 0;
        NRF_SAADC->TASKS_SAMPLE = 1;
    }

    if (NRF_SAADC->EVENTS_END != 0) {
        NRF_SAADC->EVENTS_END = 0;
        NRF_SAADC->TASKS_STOP = 1; 
    }


    if (NRF_SAADC->EVENTS_STOPPED != 0) {
        NRF_SAADC->EVENTS_STOPPED = 0;
        // now the sampling is finished, then we update the voltage
        if (adc_buffer < 0) {
            energy_vars.voltage_mV = 0;
        } else {
            energy_vars.voltage_mV = (ADC_RAW_TO_MV(adc_buffer)) * 3;
        }
        update_throttle_factor();
        adc_buffer = 0;
        energy_vars.e_surplus_uJ = CALCULATE_ENERGY_UJ(energy_vars.voltage_mV);
    }
}

uint16_t rpl_energy_penalty(void) {

    // energy-related result
    uint32_t e_consumed;
    uint32_t e_norm;
    uint16_t f_penalty = F_ENERGY_LUT[0];
    uint32_t  lut_index;
    uint32_t remainder;

    if (energy_vars.e_surplus_uJ >= E_MAX) {
        f_penalty = F_ENERGY_LUT[0];
    } else if (energy_vars.e_surplus_uJ <= E_MIN) {
        f_penalty = F_ENERGY_LUT[F_ENERGY_LUT_SIZE - 1];
    } else {
        e_consumed = E_MAX - energy_vars.e_surplus_uJ;

        // original equation: e_norm = (e_consumed * 1024) / 72000
        // `59652 / 4194304` approximates `1024 / 72000` and `/ 4194304` equals to `>> 22`
        e_norm = (e_consumed * 59652) >> 22;

        lut_index = e_norm >> 5;    // `e_norm / 32` equals to `e_norm >> 5`
        remainder = e_norm & 0x1F;  // `e_norm % 32` equals to `e_norm & 0x1F`

        if (lut_index >= F_ENERGY_LUT_SIZE - 1) {
            lut_index = F_ENERGY_LUT_SIZE - 2;
            remainder = 31;
        }

        // linear interpolation 
        f_penalty = (uint16_t) (F_ENERGY_LUT[lut_index] + (((F_ENERGY_LUT[lut_index + 1] - F_ENERGY_LUT[lut_index]) * remainder) >> 5));
    }
    return f_penalty;
}