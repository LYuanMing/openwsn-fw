#include "config.h"
#include "board.h"
#if OPENWSN_UINJECT_C

#include "opendefs.h"
#include "uinject.h"
#include "sock.h"
#include "async.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "scheduler.h"
#include "IEEE802154E.h"
#include "schedule.h"
#include "icmpv6rpl.h"
#include "idmanager.h"
#include "openrandom.h"
#include "msf.h"
#include "energy_aware.h"      // perf3: energy_vars (voltage / throttle factor)
#include "neighbors.h"         // perf3: neighbors_getNumNeighbors()

//=========================== defines =========================================

// perf3 2026-10-04 READ-ONLY: in-flight timing of a uinject packet.
// FREEZE_20261003.md appendix E: this send path has NO timeout, so the stall can
// only be observed by measuring it.  Declared near the top because the
// SOCK_ASYNC_MSG_SENT callback that reads them comes before the sender.
// perf3 2026-10-04: monotonic slot counter, the only clock available in this
// tree that still advances while the owning TSCH slot is never activated.
static uint32_t perf3_nowSlots(void) {
    uint32_t on, total;
    ieee154e_getTicsInfo(&on, &total);
    return total;
}
static uint32_t perf3_txStartTick = 0;
volatile uint32_t perf3_txLastSlots = 0;   // in-flight duration, in SLOTS
volatile uint32_t perf3_txLastMs = 0;
volatile uint32_t perf3_txMaxSlots = 0;
volatile uint32_t perf3_txMaxMs = 0;
volatile uint32_t perf3_txDone = 0;

// perf3 2026-10-05 READ-ONLY: where the preferred parent's TX cell lives.
// Appendix L: hasNegotiatedCellToNeighbor() returns FALSE ~50% of the time
// while getNumberOfNegotiatedCells() returns 1 at P3-sampling instants -- same
// predicate both times, so the cell really comes and goes.  These five say
// whether it is MISSING, or present at another slotOffset / channelOffset, or
// flipped to shared / autonomous -- which decides whether a fix belongs in 6P
// renegotiation or in MSF's shared-cell backoff.
static uint8_t perf3_cellTotal = 0;
static uint8_t perf3_cellDedicated = 0;
static uint8_t perf3_cellShared = 0;
static uint8_t perf3_cellAuto = 0;
static uint16_t perf3_cellFirstDedSlot = 0;
static uint8_t perf3_allTxTotal = 0;
static uint8_t perf3_allTxDedicated = 0;
static uint8_t perf3_allTxShared = 0;
static uint8_t perf3_rxCells = 0;
static uint8_t perf3_txrxCells = 0;
static uint8_t perf3_minimalCells = 0;
static uint16_t perf3_cellDump[18];
static int perf3_cellDumpFilled = 0;
// perf3 2026-10-05: read-only, defined in IEEE802154E.c.  How many slots were
// actually activated, and how many of those carried a TX/TXRX cell.
extern volatile uint32_t perf3_activatedSlots;
extern volatile uint32_t perf3_activatedTxSlots;
extern volatile uint32_t perf3_syncAckCount;
// perf3 2026-10-05: send-done notifications leaving the MAC.  Compare with
// uinject_vars.n_sent_done (the other end of the same edge) to localise a break.
extern volatile uint32_t perf3_notifSendDoneCalls;
// perf3 2026-10-05: the sixtop task's own view of the chain (defined in sixtop.c).
extern uint32_t perf3_getSixtopTaskCounters(uint32_t *runs, uint32_t *got, uint32_t *null);
static uint32_t perf3_stRuns = 0, perf3_stGot = 0, perf3_stNull = 0;
// perf3 2026-10-05: per-creator split inside task_sixtopNotifSendDone (sixtop.c:405).
extern void perf3_getSixtopCreatorSplit(uint32_t *sixTop, uint32_t *res, uint32_t *app,
                                     uint32_t *other, uint8_t *appL4);
static uint32_t perf3_stSixTop = 0, perf3_stRes = 0, perf3_stApp = 0, perf3_stOther = 0;
static uint8_t perf3_stAppL4 = 0;
// perf3 2026-10-05: the three exits of forwarding_sendDone() (forwarding.c:229).
extern volatile uint32_t perf3_fwdSendDone;
extern volatile uint32_t perf3_fwdRelayedFreed;
extern volatile uint32_t perf3_fwdOwnPacket;
extern volatile uint32_t perf3_fwdUdpDone;
extern volatile uint32_t perf3_fwdWrongProto;
extern volatile uint8_t perf3_fwdOwnCreator;
extern volatile uint8_t perf3_fwdOwnL4Proto;
extern volatile uint32_t perf3_fwdUdpCreatorSeen;
extern volatile uint32_t perf3_fwdOtherCreatorSeen;
extern volatile uint8_t perf3_fwdUdpCreatorL4;
// perf3 2026-10-05: the send path's exits (forwarding.c / udp.c).
extern volatile uint32_t perf3_fwdSendCalls;
extern volatile uint32_t perf3_fwdIphcFail;
extern volatile uint32_t perf3_fwdIphcOk;
extern volatile uint32_t perf3_fwdRouteFail;
extern volatile uint32_t perf3_fwdRouteOk;
extern volatile uint32_t perf3_udpTransmitCalls;
extern volatile uint32_t perf3_udpFreedNoNotify;
extern volatile uint32_t perf3_udpAccepted;
extern volatile uint32_t perf3_txDoneOk;
extern volatile uint32_t perf3_txDoneFail;
extern void ieee154e_getHopInfo(uint8_t *o_asn, uint8_t *o_single, uint8_t *o_freq, uint16_t *o_sum);
static uint8_t perf3_asnOffset = 0, perf3_singleCh = 0, perf3_freq = 0;
static uint16_t perf3_tmplSum = 0;
static uint32_t perf3_uinjectCallbackEntries = 0;
static uint8_t perf3_numDeSync = 0;
static uint32_t perf3_stackActiveSlots = 0;
static uint32_t perf3_syncAckPrev = 0;
static uint32_t perf3_syncAckPerSf = 0;
static uint32_t perf3_offeredTotal = 0;
static uint32_t perf3_fetchedTotal = 0;
static uint32_t perf3_okTrueTotal = 0;
static uint32_t perf3_okBlockedTotal = 0;
static uint8_t perf3_havePacket = 0;
static uint8_t perf3_numTxAttempts = 0;
static uint8_t perf3_retriesLeft = 0;
static uint32_t perf3_actSlotsPrev = 0;
static uint32_t perf3_actTxPrev = 0;
static uint32_t perf3_actSlotsPerSf = 0;
static uint32_t perf3_actTxPerSf = 0;

// perf3 2026-10-05 READ-ONLY: what the cell table holds AT THE MOMENT a packet
// is offered.  perf3_probeDedicatedNow = total TX cells (dedicated+shared),
// perf3_dedicatedNow = the dedicated ones.  Compared against the P3-time
// snapshot (fields 28..32) this shows whether the dedicated cell flickers.
static uint8_t perf3_probeDedicatedNow = 0;
static uint8_t perf3_dedicatedNow = 0;

#define UINJECT_TRAFFIC_RATE 1 ///> the value X indicates 1 packet/X minutes

// perf3 payload tail appended after the original 30-byte layout.  The first
// 30 bytes stay byte-identical so existing decoders keep working.
#define PERF3_TAIL_LEN 11      ///< v_mv(2) + f(1) + n_gen(4) + n_tx(4)
#define UINJECT_MAX_LEN 50     ///< must stay <= payload[] below

//=========================== variables =======================================

static sock_udp_t _sock;
static uinject_vars_t uinject_vars;

static const uint8_t uinject_payload[] = "uinject";
static const uint8_t uinject_dst_addr[] = {
        0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};

//=========================== prototypes ======================================

void uinject_sock_handler(sock_udp_t *sock, sock_async_flags_t type, void *arg);

void _uinject_timer_cb(opentimers_id_t id);

void _uinject_status_cb(opentimers_id_t id);

void _uinject_task_cb(void);

//=========================== public ==========================================

void uinject_init(void) {

    // clear local variables
    memset(&_sock, 0, sizeof(sock_udp_t));
    memset(&uinject_vars, 0, sizeof(uinject_vars_t));


    sock_udp_ep_t local;
    local.family = AF_INET6;
    local.port = WKP_UDP_INJECT;

    if (sock_udp_create(&_sock, &local, NULL, 0) < 0) {
        openserial_printf("Could not create socket\n");
        return;
    }

    openserial_printf("Created a UDP socket\n");

    sock_udp_set_cb(&_sock, uinject_sock_handler, NULL);

    // start periodic timer
    uinject_vars.period = UINJECT_PERIOD_MS;
    uinject_vars.timerId = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_UDP);
    opentimers_scheduleIn(
            uinject_vars.timerId,
            UINJECT_PERIOD_MS,
            TIME_MS,
            TIMER_PERIODIC,
            _uinject_timer_cb
    );

    // perf3: independent status timer, so the reporting cadence does not move
    // with the (much slower) packet period.
    uinject_vars.statusTimerId = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_UDP);
    opentimers_scheduleIn(
            uinject_vars.statusTimerId,
            PERF3_STATUS_PERIOD_MS,
            TIME_MS,
            TIMER_PERIODIC,
            _uinject_status_cb
    );
}

//=========================== private =========================================

void get_temperature(int32_t* result)
{
  NRF_TEMP->EVENTS_DATARDY = 0;
  NRF_TEMP->TASKS_START = 1;
  while (NRF_TEMP->EVENTS_DATARDY == 0) {}
  *result = NRF_TEMP->TEMP;
  return;
}

void uinject_sock_handler(sock_udp_t *sock, sock_async_flags_t type, void *arg) {
    (void) arg;

    char buf[50];

    if (type & SOCK_ASYNC_MSG_RECV) {
        // Link-level truth: of n_tx packets handed to the stack, n_sent_done
        // were confirmed sent by the MAC, and n_rx were received here (only
        // nonzero if this node is the destination of the uinject traffic).
        uinject_vars.n_rx++;
        sock_udp_ep_t remote;
        int16_t res;

        if ((res = sock_udp_recv(sock, buf, sizeof(buf), 0, &remote)) >= 0) {
#if PERF3_HOST_VOLTAGE
            // Emulation only: 'EMV' + uint16 LE millivolts. See EMUL_NOTES.md
            if (res >= 5 && buf[0] == 'E' && buf[1] == 'M' && buf[2] == 'V') {
                uint16_t mv = (uint8_t)buf[3] | ((uint16_t)(uint8_t)buf[4] << 8);
                ea_set_voltage_mv(mv);
                uinject_vars.n_emv++;
                return;
            }
#endif
            openserial_printf("Received %d bytes from remote endpoint:\n", res);
            openserial_printf(" - port: %d", remote.port);
            openserial_printf(" - addr: ", remote.port);
            for(int i=0; i < 16; i ++)
                openserial_printf("%x ", remote.addr.ipv6[i]);

            openserial_printf("\n\n");
            openserial_printf("Msg received: %s\n\n", buf);
        }
    }

    if (type & SOCK_ASYNC_MSG_SENT) {
        // perf3 2026-10-05 READ-ONLY: the receiving end of the send-done edge.
        // Compared with perf3_notifSendDoneCalls (MAC side) it says whether the
        // break is inside the callback chain or upstream of it.
        perf3_uinjectCallbackEntries++;
        owerror_t error = *(uint8_t*)arg;
        if (error == E_FAIL) {
            LOG_ERROR(COMPONENT_UINJECT, ERR_MAXRETRIES_REACHED,
                    (errorparameter_t) uinject_vars.counter,
                    (errorparameter_t) 0);
        }
        // allow send next uinject packet
        // perf3 2026-10-04 READ-ONLY: how long the packet stayed in flight.
        // FREEZE_20261003.md appendix E: this send path has NO timeout, so the
        // stall can only be seen by timing it.  Unsigned wrap-around is fine.
        {
            // opentimers has no "read the clock" call in this tree, so use
            // ieee154e_stats.numTicsTotal -- a monotonic slot counter that keeps
            // counting even while the owning slot is never activated.
            uint32_t dummyOn, nowSlots;
            ieee154e_getTicsInfo(&dummyOn, &nowSlots);
            if (perf3_txStartTick != 0) {
                uint32_t d = nowSlots - perf3_txStartTick;   // wrap-around is fine
                perf3_txLastSlots = d;
                perf3_txLastMs = d * SLOTDURATION;            // ms, nominal
                if (d > perf3_txMaxSlots) {
                    perf3_txMaxSlots = d;
                }
                perf3_txDone++;
            }
        }
        uinject_vars.busySendingUinject = FALSE;
        uinject_vars.n_sent_done++;
    }
}


void _uinject_timer_cb(opentimers_id_t id) {
    // calling the task directly as the timer_cb function is executed in
    // task mode by opentimer already
    // perf3: RATE==1 means strictly periodic -- the old random gate used
    // `openrandom_get16b() < 0xffff`, which skips a tick every ~65536 and
    // would punch holes into the offer sequence.
    if (UINJECT_TRAFFIC_RATE <= 1 ||
        openrandom_get16b() < (0xffff / UINJECT_TRAFFIC_RATE)) {
        _uinject_task_cb();
    }
}

// perf3: one ASCII line per PERF3_STATUS_PERIOD_MS, serial only.
// Everything here is either a counter maintained above or a getter that
// already exists in the tree -- no other firmware file is touched.
// perf3: real number of TX cells currently negotiated with the preferred
// parent.  msf_getPreviousNumCellsUsed() reports how many cells were USED in
// the last MSF window (0 whenever MSF never fired), which cannot distinguish
// "one uplink slot" from "no cell".  Under perf3_max_tx_cells=1 this counter is
// the direct evidence of the one-slot state.
static uint8_t perf3_negotiatedTxCells(void) {
    open_addr_t par;
    if (icmpv6rpl_getPreferredParentEui64(&par) == FALSE) {
        return 0;
    }
    return (uint8_t)schedule_getNumberOfNegotiatedCells(&par, CELLTYPE_TX);
}

void _uinject_status_cb(opentimers_id_t id) {
    // EUI64[7] of the preferred parent: tells apart "the packet reached the
    // DAG root" from "it died one hop earlier".
    extern uint32_t perf3_iphc_rx;
extern uint32_t perf3_iphc_fwd;
extern uint32_t perf3_iphc_bridge;
    int parent7 = -1;
    open_addr_t _par;
    if (icmpv6rpl_getPreferredParentEui64(&_par)) {
        parent7 = (int)_par.addr_64b[7];
    }
    uint8_t asnArray[5];
    uint32_t asn32;
    uint32_t ticksOn;
    uint32_t ticksTotal;

    // NOTE: the root is deliberately NOT muted here.  It has no throttle of
    // its own (throttle_factor is hard-coded 0), but its neighbours_getNumNeighbors()
    // is the only way to tell "the packet never arrived" apart from
    // "it arrived but was not bridged" when debugging the D-frame path.

    ieee154e_getAsn(asnArray);
    asn32 = ((uint32_t)asnArray[1] << 24) | ((uint32_t)asnArray[2] << 16) |
            ((uint32_t)asnArray[3] << 8)  | ((uint32_t)asnArray[4]);
    ieee154e_getTicsInfo(&ticksOn, &ticksTotal);

    // perf3 2026-10-05 READ-ONLY: sample the cell table at the SAME instant the
    // P3 line prints, so it can be compared against the per-packet probe.
    {
        slotOffset_t so;
        perf3_cellTotal = schedule_debugCountTxCells(&_par,
                                &perf3_cellDedicated,
                                &perf3_cellShared,
                                &perf3_cellAuto,
                                &so);
        perf3_cellFirstDedSlot = so;
        // Also count EVERY TX cell: a DAG root has no preferred parent,
        // so the neighbour-scoped count above is 0 by construction there.
        perf3_allTxTotal = schedule_debugCountAllTxCells(&perf3_allTxDedicated,
                                                             &perf3_allTxShared,
                                                             &perf3_cellAuto);
        // Count RX-side cells too: msf_rx_cells_used counts cells USED, not
        // PRESENT, so it cannot answer whether a minimal cell exists.
        schedule_debugCountRxCells(&perf3_rxCells, &perf3_txrxCells, &perf3_minimalCells);
        // Dump the actual (slotOffset, channelOffset) of every active cell so that
        // both ends of a link can be compared.  Refresh on every print: filling it
        // once captured the table BEFORE 6P finished negotiating (the child showed
        // only its RX minimal cell while allTxTotal already said 2 TX cells),
        // which is exactly the kind of stale snapshot that invents a conclusion.
        schedule_debugDumpCells(perf3_cellDump);
        // Hopping identity: calculateFrequency() = 11 + chTemplate[(asnOffset +
        // channelOffset) % NUM_CHANNELS], so the two ends of a link must share
        // asnOffset and the template, not merely channelOffset.
        ieee154e_getHopInfo(&perf3_asnOffset, &perf3_singleCh, &perf3_freq, &perf3_tmplSum);
        // Deltas over the last slotframe: how many slots were activated and how
        // many carried a TX cell.  A dedicated cell that exists but is never
        // activated shows up here as allTX>0 with actTx==0.
        {
            uint32_t a = perf3_activatedSlots - perf3_actSlotsPrev;
            uint32_t t = perf3_activatedTxSlots - perf3_actTxPrev;
            perf3_actSlotsPrev = perf3_activatedSlots;
            perf3_actTxPrev = perf3_activatedTxSlots;
            perf3_actSlotsPerSf = a;
            perf3_actTxPerSf = t;
            perf3_syncAckPerSf = perf3_syncAckCount - perf3_syncAckPrev;
            perf3_syncAckPrev = perf3_syncAckCount;
            ieee154e_getSyncStats(&perf3_numDeSync, &perf3_stackActiveSlots);
            perf3_getFetchCounters(&perf3_offeredTotal, &perf3_fetchedTotal,
                                   &perf3_okTrueTotal, &perf3_okBlockedTotal);
            perf3_getPendingPacketState(&perf3_havePacket, &perf3_numTxAttempts,
                                           &perf3_retriesLeft);
            perf3_getSixtopTaskCounters(&perf3_stRuns, &perf3_stGot, &perf3_stNull);
            perf3_getSixtopCreatorSplit(&perf3_stSixTop, &perf3_stRes, &perf3_stApp,
                                        &perf3_stOther, &perf3_stAppL4);
        }
    }

    openserial_printf(
        "P3,%x,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%x,%x,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        (unsigned int)asn32,
        (int)energy_vars.throttle_factor,
        (int)energy_vars.voltage_mV,
        (int)neighbors_getNumNeighbors(),
        (int)uinject_vars.n_gen,
        (int)uinject_vars.n_tx,
        (int)uinject_vars.d_busy,
        (int)uinject_vars.d_nocell,
        (int)uinject_vars.d_nosync,
        (int)uinject_vars.d_noparent,
        (int)uinject_vars.d_fail,
        (int)perf3_negotiatedTxCells(),
        (int)msf_getPreviousNumCellsUsed(CELLTYPE_RX),
        (unsigned int)ticksOn,
        (unsigned int)ticksTotal,
        (int)uinject_vars.n_sent_done,
        (int)uinject_vars.n_rx,
        (int)parent7,
        (int)perf3_iphc_rx,
        (int)perf3_iphc_fwd,
        (int)perf3_iphc_bridge,
        // perf3: the period this firmware was ACTUALLY compiled with.  The
        // load axis of fig (a) is only meaningful if the offered rate is known,
        // and the command-line uinject_period_ms=... is silently ignored unless
        // openapps/SConscript actually appends the define.  Measured 2026-10-02:
        // a run recorded notes=period_ms=15000 while the board was built at
        // 1000, so the node offered 60 pkt/min instead of 4, the whole lambda
        // ladder was 15x too fast, and every power point saturated at C_max.
        // Report the real value so parse.py can gate on it rather than trust
        // the CSV.
        (int)UINJECT_PERIOD_MS,
        // perf3 2026-10-04 READ-ONLY: in-flight duration of one uinject packet,
        // i.e. from sock_udp_send() to the SOCK_ASYNC_MSG_SENT callback.
        // FREEZE_20261003.md appendix E says this path has NO timeout, so this is
        // the only place the stall can be measured.  Fields 1..23 are unchanged.
        //  24) txLastMs 25) txMaxMs 26) txDone 27) txStartTick(0 = none pending)
        (int)perf3_txLastMs,
        (int)perf3_txMaxMs,
        (int)perf3_txDone,
        (int)perf3_txStartTick,
        // perf3 2026-10-05 READ-ONLY: WHERE the TX cell of the preferred parent
        // lives in scheduleBuf.  Appendix L narrowed the freeze to "the cell is
        // absent about half the time"; these say whether it is missing or just
        // moved / turned shared / turned autonomous.
        //  28) cellTotal 29) cellDedicated 30) cellSharedCells
        //  31) cellAuto 32) firstDedicatedSlotOffset
        (int)perf3_cellTotal,
        (int)perf3_cellDedicated,
        (int)perf3_cellShared,
        (int)perf3_cellAuto,
        (int)perf3_cellFirstDedSlot,
        //  33) probeTotalNow 34) probeDedicatedNow (per-packet snapshot)
        (int)perf3_probeDedicatedNow,
        (int)perf3_dedicatedNow,
        //  35) allTxTotal 36) allTxDedicated 37) allTxShared
        // Whole-table counts: a DAG root has no preferred parent, so
        // the neighbour-scoped counts above are 0 by construction there.
        (int)perf3_allTxTotal,
        (int)perf3_allTxDedicated,
        (int)perf3_allTxShared,
        //  38) actSlotsPerSf 39) actTxPerSf 40) actSlotsTotal
        // appendix D's missing observation: how many slots were ACTUALLY
        // activated in the last slotframe, and how many carried a TX cell.
        // A dedicated cell that exists but is never activated shows up as
        // allTxTotal>0 together with actTxPerSf==0.
        (int)perf3_actSlotsPerSf,
        (int)perf3_actTxPerSf,
        (int)perf3_activatedSlots,
        //  41) syncAckPerSf 42) numDeSync 43) stackActiveSlots
        // Appendix T's decisive pair: syncAckPerSf is the rate of EB
        // receptions (the only thing that refills deSyncTimeout).  If the stall
        // is EB starvation this drops to 0 in the same window in which
        // act_slots_per_sf drops to 0.
        (int)perf3_syncAckPerSf,
        (int)perf3_numDeSync,
        (int)perf3_stackActiveSlots,
        //  44) offeredTotal 45) fetchedTotal 46) okTrueTotal 47) okBlockedTotal
        // The decisive pair for appendix U: on a unicast cell the OkToSend gate
        // opened (offered) -- did a packet actually come out (fetched)?
        // fetched<<offered means the queued packet's next hop never matches the
        // activated cell's neighbour, i.e. it is never offered to a cell at all.
        (int)perf3_offeredTotal,
        (int)perf3_fetchedTotal,
        (int)perf3_okTrueTotal,
        (int)perf3_okBlockedTotal,
        //  48) havePacket 49) numTxAttempts 50) retriesLeft
        // State of the packet the MAC is holding.  activity_tie5() runs in
        // state S_RXACKLISTEN -- i.e. AFTER the packet went out and the node is
        // waiting for the link-layer ACK -- and it is the ONLY place that
        // decrements retriesLeft and eventually calls notif_sendDone(E_FAIL).
        //   attempts>0 + retriesLeft unchanged  => the FSM is parked in
        //                                        S_RXACKLISTEN waiting for an ACK
        //                                        that never comes (no timeout)
        //   attempts==0                        => fetched but never even tried
        (int)perf3_havePacket,
        (int)perf3_numTxAttempts,
        (int)perf3_retriesLeft,
        //  51) notifSendDoneCalls 52) uinjectCallbackEntries
        // The two ends of the send-done edge: the MAC's notif_sendDone() and
        // uinject's own callback.  notif >> callback => the break is in between.
        (int)perf3_notifSendDoneCalls,
        (int)perf3_uinjectCallbackEntries,
        //  53) stTaskRuns 54) stTaskGotPacket 55) stTaskNullPacket
        // The sixtop task's own counters.  notif > runs  => the scheduler dropped
        // the task;  runs > got  => the task ran but the queue no longer held
        // the packet;  got > 0 while uinjectCallbackEntries == 0 => the break is
        // further downstream (RES/socket layer).
        (int)perf3_stRuns,
        (int)perf3_stGot,
        (int)perf3_stNull,
        //  56) fwdSendDone 57) fwdRelayedFreed 58) fwdOwnPacket
        //  59) fwdUdpDone 60) fwdWrongProto 61) fwdOwnCreator 62) fwdOwnL4Proto
        // forwarding_sendDone() has three exits; only the UDP one reaches
        // uinject.  ownCreator/ownL4Proto record WHO the packet claims to
        // belong to and WHICH protocol it carries.
        (int)perf3_fwdSendDone,
        (int)perf3_fwdRelayedFreed,
        (int)perf3_fwdOwnPacket,
        (int)perf3_fwdUdpDone,
        (int)perf3_fwdWrongProto,
        (int)perf3_fwdOwnCreator,
        (int)perf3_fwdOwnL4Proto,
        //  63) fwdUdpCreatorSeen 64) fwdOtherCreatorSeen 65) fwdUdpL4
        // Per-creator split: only fwdUdpCreatorSeen counts the
        // application's own UDP packets (creator=COMPONENT_SOCK_TO_UDP).
        (int)perf3_fwdUdpCreatorSeen,
        (int)perf3_fwdOtherCreatorSeen,
        (int)perf3_fwdUdpCreatorL4,
        //  66) stSixTop 67) stRes 68) stApp 69) stOther 70) stAppL4
        // Which branch of task_sixtopNotifSendDone's switch each
        // send-done took.  stApp==0 with stTaskGot>0 means the
        // application's own packet never arrives at this task.
        (int)perf3_stSixTop,
        (int)perf3_stRes,
        (int)perf3_stApp,
        (int)perf3_stOther,
        (int)perf3_stAppL4,
        //  71) fwdSendCalls 72) fwdIphcOk 73) fwdIphcFail
        //  74) fwdRouteOk 75) fwdRouteFail 76) udpTransmitCalls
        //  77) udpFreedNoNotify 78) udpAccepted
        // udpFreedNoNotify>0 is the smoking gun: the packet was freed
        // without any send-done, so busySendingUinject can never clear.
        (int)perf3_fwdSendCalls,
        (int)perf3_fwdIphcOk,
        (int)perf3_fwdIphcFail,
        (int)perf3_fwdRouteOk,
        (int)perf3_fwdRouteFail,
        (int)perf3_udpTransmitCalls,
        (int)perf3_udpFreedNoNotify,
        (int)perf3_udpAccepted,
        //  79) rxCells 80) txrxCells 81) minimalCells
        //  rxCells==0 with minimalCells==0 means the 6TiSCH minimal cell
        //  created in msf_init() is genuinely absent from scheduleBuf.
        (int)perf3_rxCells,
        (int)perf3_txrxCells,
        (int)perf3_minimalCells,
        //  82..99) cellDump[18] = 6 cells x (slotOffset, channelOffset, packed)
        //  packed = type*100 + shared*10 + auto;  slotOffset 0xFFFF = unused.
        //  Both ends of a link MUST agree on (slotOffset, channelOffset).
        (int)perf3_cellDump[0],
        (int)perf3_cellDump[1],
        (int)perf3_cellDump[2],
        (int)perf3_cellDump[3],
        (int)perf3_cellDump[4],
        (int)perf3_cellDump[5],
        (int)perf3_cellDump[6],
        (int)perf3_cellDump[7],
        (int)perf3_cellDump[8],
        (int)perf3_cellDump[9],
        (int)perf3_cellDump[10],
        (int)perf3_cellDump[11],
        (int)perf3_cellDump[12],
        (int)perf3_cellDump[13],
        (int)perf3_cellDump[14],
        (int)perf3_cellDump[15],
        (int)perf3_cellDump[16],
        (int)perf3_cellDump[17],
        //  100) txDoneOk 101) txDoneFail
        //  MAC-level outcome split.  txDoneOk large while the peer
        //  receives nothing => frames leave the node but are not demodulated.
        (int)perf3_txDoneOk,
        (int)perf3_txDoneFail,
        //  102) asnOffset 103) singleChannel 104) freq 105) tmplSum
        //  asnOffset/105 differ between the two ends => they hop to
        //  different frequencies and the link is dead while both sides
        //  still believe it is up.
        (int)perf3_asnOffset,
        (int)perf3_singleCh,
        (int)perf3_freq,
        (int)perf3_tmplSum
    );
}

void _uinject_task_cb(void) {
    uint8_t asnArray[5];
    open_addr_t parentNeighbor;
    bool foundNeighbor;

    // perf3: ONE OFFER is counted here, before any early return, because the
    // application took a sample regardless of whether the stack can carry it.
    uinject_vars.n_gen++;
    uint32_t offerId = uinject_vars.n_gen;

    // don't run if not synch
    if (ieee154e_isSynch() == FALSE) {
        uinject_vars.d_nosync++;
        return;
    }

    // don't OFFER on dagroot, but keep the status timer alive: the root's
    // neighbours_getNumNeighbors() is needed to debug the D-frame path.
    if (idmanager_getIsDAGroot()) {
        opentimers_destroy(uinject_vars.timerId);
        return;
    }

    foundNeighbor = icmpv6rpl_getPreferredParentEui64(&parentNeighbor);
    if (foundNeighbor == FALSE) {
        uinject_vars.d_noparent++;
        return;
    }

    // perf3 2026-10-05 READ-ONLY.  P3 samples the cell table every ~5 s, but
    // this runs per packet.  Appendix J saw "dedicated cell present" and
    // "d_nocell rising" at the same time, which is only possible if the cell
    // comes and goes.  Count what the SAME predicate sees here, and how many
    // dedicated cells the table holds right now, so the two can be compared
    // directly instead of inferred.
    {
        uint8_t dd = 0, ss = 0, aa = 0;
        slotOffset_t so;
        perf3_probeDedicatedNow = schedule_debugCountTxCells(&parentNeighbor,
                                                     &dd, &ss, &aa, &so);
        perf3_dedicatedNow = dd;
    }
    if (schedule_hasNegotiatedCellToNeighbor(&parentNeighbor, CELLTYPE_TX) == FALSE) {
        uinject_vars.d_nocell++;
        return;
    }

    if (uinject_vars.busySendingUinject == TRUE) {
        // don't continue if I'm still sending a previous uinject packet
        uinject_vars.d_busy++;
        return;
    }

    // if you get here, send a packet
    sock_udp_ep_t remote;
    remote.port = WKP_UDP_INJECT;
    remote.family = AF_INET6;
    memcpy(remote.addr.ipv6, uinject_dst_addr, sizeof(uinject_dst_addr));

    uint8_t payload[UINJECT_MAX_LEN];
    uint8_t len = 0;
    // add 'uinject' string
    memcpy(&payload[len], uinject_payload, sizeof(uinject_payload) - 1);
    len += sizeof(uinject_payload) - 1;
    // add counter
    payload[len++] = (uint8_t)(uinject_vars.counter & 0x00ff);
    payload[len++] = (uint8_t)((uinject_vars.counter & 0xff00) >> 8);
    uinject_vars.counter++;
    // add asn
    ieee154e_getAsn(asnArray);
    memcpy(&payload[len], asnArray, sizeof(asnArray));
    len += sizeof(asnArray);
    // add tx cells used
    payload[len++] = perf3_negotiatedTxCells();
    // add rx cells used
    payload[len++] = msf_getPreviousNumCellsUsed(CELLTYPE_RX);
    // add 16b addr
    payload[len++] = (uint8_t)(idmanager_getMyID(ADDR_16B)->addr_16b[1]);
    payload[len++] = (uint8_t)(idmanager_getMyID(ADDR_16B)->addr_16b[0]);
    int32_t temp;
    get_temperature(&temp);
    payload[len++] = (uint8_t)temp & 0xff;
    payload[len++] = (uint8_t)((temp & 0xff00) >> 8);
    payload[len++] = (uint8_t)((temp & 0xff0000) >> 16);
    payload[len++] = (uint8_t)((temp & 0xff000000) >> 24);
    // add ticks info
    uint32_t ticksOn;
    uint32_t ticksInTotal;
    ieee154e_getTicsInfo(&ticksOn, &ticksInTotal);
    memcpy(&payload[len],  &ticksOn, sizeof(ticksOn));
    len += sizeof(ticksOn);
    memcpy(&payload[len],  &ticksInTotal, sizeof(ticksInTotal));
    len += sizeof(ticksInTotal);

    // ---- perf3 tail (11 B, appended so the first 30 B stay unchanged) ----
    uint16_t v_mv = (uint16_t)energy_vars.voltage_mV;
    payload[len++] = (uint8_t)(v_mv & 0x00ff);
    payload[len++] = (uint8_t)((v_mv & 0xff00) >> 8);
    payload[len++] = energy_vars.throttle_factor;
    memcpy(&payload[len], &uinject_vars.n_gen, sizeof(uint32_t));
    len += sizeof(uint32_t);
    memcpy(&payload[len], &uinject_vars.n_tx, sizeof(uint32_t));
    len += sizeof(uint32_t);
    (void)offerId;

    if (sock_udp_send(&_sock, payload, len, &remote) > 0) {
        // set busySending to TRUE
        uinject_vars.busySendingUinject = TRUE;
        // perf3 2026-10-04 READ-ONLY: stamp the send so the SOCK_ASYNC_MSG_SENT
        // callback can measure how long it stayed in flight.  opentimers' tick
        // counter keeps running even while the owning TSCH slot is never
        // activated, which is exactly the case we need to see.
        perf3_txStartTick = perf3_nowSlots();
        // perf3: only a send the stack accepted counts towards n_tx
        uinject_vars.n_tx++;
    } else {
        uinject_vars.d_fail++;
    }
}

#endif /* OPENWSN_UINJECT_C */
