#ifndef __UINJECT_H
#define __UINJECT_H

/**
\addtogroup AppUdp
\{
\addtogroup uinject
\{
*/

#include "config.h"
#include "opentimers.h"
#include "udp.h"

//=========================== define ==========================================

// One report per node every 10 s (2026-11-10: was 2000 ms; the host-side
// topology viewer does not need a 2 s refresh, and a slower rate leaves airtime
// to the throttled nodes and shrinks the serial log).
#define UINJECT_PERIOD_MS 10000

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    opentimers_id_t timerId;   ///< periodic timer which triggers transmission
    uint16_t counter;  ///< incrementing counter which is written into the packet
    uint16_t period;  ///< uinject packet sending period>
    bool busySendingUinject;  ///< TRUE when busy sending an uinject
} uinject_vars_t;

//=========================== prototypes ======================================

void uinject_init(void);
void uinject_requestDump(void);
void uinject_requestSetVoltage(uint8_t eui7, uint16_t mv);
void uinject_onParentChanged(void);
void uinject_dumpTopology(void);

/**
\}
\}
*/

#endif

