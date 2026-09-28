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

// Packet period. Overridable at build time for the Run-L lambda ladder:
//   scons ... apps=uinject uinject_period_ms=500
#ifndef UINJECT_PERIOD_MS
#define UINJECT_PERIOD_MS 2000
#endif

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

/**
\}
\}
*/

#endif

