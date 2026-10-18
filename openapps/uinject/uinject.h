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

// Period of the perf3 status line (serial only, never goes over the air).
#ifndef PERF3_STATUS_PERIOD_MS
#define PERF3_STATUS_PERIOD_MS 2000
#endif

// Host-driven energy state (EMULATION ONLY -- leave 0 for real-light runs).
//
// When 1, the node accepts a 5-byte datagram 'EMV' + uint16 LE millivolts on
// WKP_UDP_INJECT and uses it as its supply voltage instead of the ADC.  The
// host then runs energy_emulator.py, which integrates the capacitor model.
// Requires ea_set_voltage_mv() in energy_aware.c -- see EMUL_NOTES.md.
#ifndef PERF3_HOST_VOLTAGE
#define PERF3_HOST_VOLTAGE 0
#endif

//=========================== typedef =========================================

//=========================== variables =======================================

typedef struct {
    opentimers_id_t timerId;   ///< periodic timer which triggers transmission
    opentimers_id_t statusTimerId; ///< periodic timer which emits the P3 status line
    uint16_t counter;  ///< incrementing counter which is written into the packet
    uint16_t period;  ///< uinject packet sending period>
    bool busySendingUinject;  ///< TRUE when busy sending an uinject

    // ---- perf3 instrumentation -------------------------------------------
    // Every timer tick that passes the traffic-rate gate is ONE OFFER, i.e. a
    // sample the application asked the stack to deliver.  PDR must be built on
    // this denominator: using "packets handed to the socket" as the
    // denominator hides every packet the node never even tried to send.
    //
    // Invariant, checked by parse.py for every run:
    //     n_gen == n_tx + d_busy + d_nocell + d_nosync + d_noparent + d_fail
    uint32_t n_gen;      ///< packets offered by the application
    uint32_t n_tx;       ///< offers accepted by sock_udp_send
    uint32_t d_busy;     ///< dropped: previous packet still in flight
    uint32_t d_nocell;   ///< dropped: no negotiated TX cell to the parent
    uint32_t d_nosync;   ///< dropped: not synchronised
    uint32_t d_noparent; ///< dropped: no preferred parent
    uint32_t d_fail;     ///< dropped: sock_udp_send returned <= 0
    uint32_t n_sent_done;///< SOCK_ASYNC_MSG_SENT: MAC confirmed the send
    uint32_t n_rx;       ///< SOCK_ASYNC_MSG_RECV: a uinject packet arrived here
#if PERF3_HOST_VOLTAGE
    uint32_t n_emv;      ///< emulation voltage commands accepted from the host
#endif
} uinject_vars_t;

//=========================== prototypes ======================================

void uinject_init(void);

/**
\}
\}
*/

#endif
