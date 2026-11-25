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
#include "energy_aware.h"

//=========================== defines =========================================

#define UINJECT_TRAFFIC_RATE 1 ///> the value X indicates 1 packet/X minutes

// Payload layout (2026-11-10): the trailing 5 bytes carry the energy-aware
// working point and the topology identity, so a DAG root can print one line
// per received packet for the host-side visualiser.
//   [30..31] current_voltage (mV, little endian)
//   [32]     throttle_factor
//   [33]     own EUI64[7]
//   [34]     preferred parent EUI64[7] (0 when there is no parent yet)
#define UINJECT_OFF_MV       30
#define UINJECT_OFF_TF       32
#define UINJECT_OFF_NODE     33
#define UINJECT_OFF_PARENT   34
#define UINJECT_PAYLOAD_LEN  35

// A send that never gets its SOCK_ASYNC_MSG_SENT callback (the packet was
// dropped in the queue, which happens once this node also forwards its
// children's traffic) would keep busySendingUinject TRUE forever and silence
// the mote until the next reset. Observed 2026-11-10 on the A/B/C
// force-topology run: the parent of C stopped reporting while it was still
// forwarding C's packets. After this many timer ticks the flag is cleared and
// a new packet is sent (2 ticks = 20 s at the 10 s UINJECT_PERIOD_MS; the value
// used to be 5 ticks of 2 s = 10 s).
#define UINJECT_BUSY_TIMEOUT_TICKS 2

// Root-side snapshot of the last report of every child.  A host that connects
// later asks for the whole tree with 'T' instead of waiting one uinject period
// per node (a forwarding node waits out the busy timeout, and a node that just
// lost its parent can stay quiet for tens of seconds).
#define UINJECT_CACHE_N          8
#define UINJECT_DUMP_PERIOD_MS   500
#define UINJECT_DUMP_TIMER_ENABLE 1

// Downlink control (2026-11-11): a host that is wired to the DAG root only must
// still be able to retune every node.  'W' <eui7> <mV hi> <mV lo> arrives on the
// root's serial port, the root broadcasts "SETVOLT"+eui7+mV over the shared cell
// (the tree is uplink-only, there is no TX cell from a parent down to a child),
// and the leaf whose EUI64[7] matches applies it and reports back at once.
// Layout: "SETVOLT"(7) eui7 mV(lo,hi) seq hops
// The sequence byte lets a node drop a copy that was queued before a newer
// command: the shared cell is slow (one packet per slotframe, 1/4 chance with
// the MAC's ALOHA gate), so three copies of an old command can still be in the
// mesh when the next one arrives, and applying them out of order made a node
// flip 2200 -> 4200 -> 2200.
#define UINJECT_CMD_LEN      12
// A broadcast has no ACK, and the root can only reach the mesh through the
// shared cell, so queue the command a few times; the MAC sends one copy per
// shared cell (~2 s apart), which also rides out a lost frame.
#define UINJECT_CMD_REPEAT   3
// A node two hops away from the root usually cannot hear the root's shared-cell
// broadcast at all (measured 2026-11-11: 0x61 received under 3% of the root's
// EBs, and none of 3 SETVOLT copies).  Every node that hears a command meant for
// somebody else repeats it once with one hop less, so the command walks down the
// tree; the TTL keeps it from circulating for ever.
#define UINJECT_CMD_TTL      3
// Ignore the copies of a command this node just relayed (the root sends three,
// and a rebroadcast may come back through another neighbour).
#define UINJECT_RELAY_TICKS  8   // 8 x 500 ms = 4 s
#define UINJECT_QUICK_TICKS  40   // 40 x 500 ms = 20 s of report retries after an event
#define UINJECT_SNAP_TICKS   60   // root: re-dump the cached tree every 30 s

typedef struct {
    uint8_t  valid;
    uint8_t  eui;
    uint8_t  parent;
    uint8_t  tf;
    uint16_t mv;
} uinject_cacheEntry_t;


//=========================== variables =======================================

static sock_udp_t _sock;
static uinject_vars_t uinject_vars;
static uint8_t busyTicks;   ///< timer ticks spent waiting for MSG_SENT
static uinject_cacheEntry_t uinject_cache[UINJECT_CACHE_N];
static volatile uint8_t uinject_dumpRequested;
static opentimers_id_t uinject_dumpTimerId;
static volatile uint8_t uinject_setVoltPending;
static volatile uint8_t uinject_setVoltEui;
static volatile uint16_t uinject_setVoltMv;
static volatile uint8_t uinject_setVoltSeq;    // root: bumped for every 'W'
// Downlink commands carry an 8-bit sequence number.  Two things have to hold:
//   1. a trailing copy of an *older* command must not undo a newer one (the
//      shared cell delivers copies seconds apart, and out of order);
//   2. after the root restarts its counter starts again at 0, so a plain
//      "newer must be greater" test would make every node refuse every command
//      until the counter climbed past the value it last saw -- unacceptable
//      when the leaves sit somewhere else and cannot be reset.
// The rule below keeps the monotonic test and adds an escape hatch: a value
// that keeps coming back (UINJECT_SEQ_FORCE identical receptions) is treated as
// a restarted root rather than a stray copy.  One click is three copies plus
// the GUI's retries, so the escape hatch opens after about two clicks.
#define UINJECT_SEQ_FORCE 6
static uint8_t uinject_lastSeq;      // last sequence number applied
static uint8_t uinject_candSeq;      // a sequence number currently being refused
static uint8_t uinject_candCount;    // how many times it was seen
static volatile uint8_t uinject_quickTicks;
static uint8_t uinject_snapTicks;
static volatile uint8_t uinject_relayEui;
static volatile uint16_t uinject_relayMv;
static volatile uint8_t uinject_relayHops;
static volatile uint8_t uinject_relayTicks;

static const uint8_t uinject_setvolt_cmd[] = "SETVOLT";
// ff02::1a (all RPL nodes), NOT ff02::1: iphc_sendFromForwarding() refuses to
// let a DAG root originate anything but ff02::1a
// (openstack/03a-IPHC/iphc.c:76-80), so ff02::1 made the root log
// ERR_BRIDGE_MISMATCH and lock up.  DIO uses this same address.
static const uint8_t uinject_bcast_addr[] = {
        0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a
};

static const uint8_t uinject_payload[] = "uinject";
static const uint8_t uinject_dst_addr[] = {
        0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};

//=========================== prototypes ======================================

void uinject_sock_handler(sock_udp_t *sock, sock_async_flags_t type, void *arg);

void _uinject_timer_cb(opentimers_id_t id);

void _uinject_task_cb(void);

static void _uinject_dumpTimer_cb(opentimers_id_t id);

static void _uinject_sendSetVoltage(uint8_t eui, uint16_t mv, uint8_t seq, uint8_t hops);

static void uinject_cacheStore(uint8_t eui, uint8_t parent, uint16_t mv, uint8_t tf);

void uinject_cacheFromPacket(uint8_t *buffer, uint8_t len);

//=========================== public ==========================================

void uinject_init(void) {

    // clear local variables
    memset(&_sock, 0, sizeof(sock_udp_t));
    memset(&uinject_vars, 0, sizeof(uinject_vars_t));
    busyTicks = 0;

    // snapshot of the children's last reports, dumped on demand ('T')
    memset(uinject_cache, 0, sizeof(uinject_cache));
    uinject_dumpRequested = FALSE;
    uinject_setVoltPending = FALSE;
    uinject_quickTicks = 0;
    uinject_snapTicks = 0;
    uinject_relayTicks = 0;
    uinject_relayHops = 0;
    uinject_candCount = 0;
#if UINJECT_DUMP_TIMER_ENABLE
    uinject_dumpTimerId = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_UDP);
    opentimers_scheduleIn(
            uinject_dumpTimerId,
            UINJECT_DUMP_PERIOD_MS,
            TIME_MS,
            TIMER_PERIODIC,
            _uinject_dumpTimer_cb
    );
#endif

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

//=========================== cache ===========================================

static void uinject_cacheStore(uint8_t eui, uint8_t parent, uint16_t mv, uint8_t tf) {
    uint8_t i;

    for (i = 0; i < UINJECT_CACHE_N; i++) {
        if (uinject_cache[i].valid && uinject_cache[i].eui == eui) {
            break;
        }
    }
    if (i == UINJECT_CACHE_N) {
        for (i = 0; i < UINJECT_CACHE_N; i++) {
            if (uinject_cache[i].valid == FALSE) {
                break;
            }
        }
    }
    if (i == UINJECT_CACHE_N) {
        i = 0;  // table full: recycle the first slot
    }
    uinject_cache[i].valid  = TRUE;
    uinject_cache[i].eui    = eui;
    uinject_cache[i].parent = parent;
    uinject_cache[i].mv     = mv;
    uinject_cache[i].tf     = tf;
}

// Same line format as the live print in uinject_sock_handler(), so the host
// parses a snapshot and a live report with the same regexp.
void uinject_dumpTopology(void) {
    uint8_t i;

    for (i = 0; i < UINJECT_CACHE_N; i++) {
        if (uinject_cache[i].valid == FALSE) {
            continue;
        }
        openserial_printf("TOPO root=%x node=%x parent=%x mv=%d tf=%d snap=1\r\n",
                (int)idmanager_getMyID(ADDR_64B)->addr_64b[7],
                (int)uinject_cache[i].eui,
                (int)uinject_cache[i].parent,
                (int)uinject_cache[i].mv,
                (int)uinject_cache[i].tf);
    }
}

// Called by the DAG root's openbridge path (openstack/03a-IPHC/openbridge.c).
// On a root, a unicast IPv6 packet never reaches the local UDP stack: iphc_receive()
// hands it to openbridge_receive() and it leaves over the serial bridge as a 'D'
// frame (see openstack/03a-IPHC/iphc.c:281-309).  The snapshot cache therefore
// has to be fed from that path, not from uinject_sock_handler().
void uinject_cacheFromPacket(uint8_t *buffer, uint8_t len) {
    uint8_t *p;
    uint16_t mv;

    if (len < UINJECT_PAYLOAD_LEN) {
        return;
    }
    p = &buffer[len - UINJECT_PAYLOAD_LEN];
    if (memcmp(p, uinject_payload, 7) != 0) {
        return;
    }
    mv = (uint16_t)p[UINJECT_OFF_MV] |
         ((uint16_t)p[UINJECT_OFF_MV + 1] << 8);
    uinject_cacheStore(p[UINJECT_OFF_NODE], p[UINJECT_OFF_PARENT], mv,
                       p[UINJECT_OFF_TF]);
    // Emit the live line from here (task context, same format as the snapshot),
    // because on a root the socket handler below never runs.
    openserial_printf("TOPO root=%x node=%x parent=%x mv=%d tf=%d\r\n",
            (int)idmanager_getMyID(ADDR_64B)->addr_64b[7],
            (int)p[UINJECT_OFF_NODE],
            (int)p[UINJECT_OFF_PARENT],
            (int)mv,
            (int)p[UINJECT_OFF_TF]);
}

// Called from the serial ISR: only raise a flag, the printing happens in the
// timer task below (printing from the ISR overflows the ISR stack).
void uinject_requestDump(void) {
    uinject_dumpRequested = TRUE;
}

// Called from the serial ISR ('W' frame) and from the uinject timer task.
void uinject_requestSetVoltage(uint8_t eui7, uint16_t mv) {
    uinject_setVoltSeq++;      // newer command: older copies become stale
    uinject_setVoltEui = eui7;
    uinject_setVoltMv = mv;
    uinject_setVoltPending = UINJECT_CMD_REPEAT;
}

// Called by RPL right after the preferred parent changed.
void uinject_onParentChanged(void) {
    uinject_quickTicks = UINJECT_QUICK_TICKS;
}

static void _uinject_dumpTimer_cb(opentimers_id_t id) {
    (void)id;
    if (uinject_dumpRequested) {
        uinject_dumpRequested = FALSE;
        uinject_dumpTopology();
    }
    if (idmanager_getIsDAGroot()) {
        // Keep the host's tree fresh without a 'T': a node that went silent
        // still shows its last known working point.
        if (++uinject_snapTicks >= UINJECT_SNAP_TICKS) {
            uinject_snapTicks = 0;
            uinject_dumpTopology();
        }
        if (uinject_setVoltPending) {
            uinject_setVoltPending--;
            _uinject_sendSetVoltage(uinject_setVoltEui, uinject_setVoltMv,
                                    uinject_setVoltSeq, UINJECT_CMD_TTL);
        }
    }
    if (uinject_relayTicks) {
        if (--uinject_relayTicks == 0) {
            uinject_relayHops = 0;      // window closed: accept a new command
        }
    }
    if (!idmanager_getIsDAGroot() && uinject_quickTicks) {
        // Report immediately after a parent change or a voltage command, and
        // keep retrying until a packet actually goes out (right after a parent
        // switch there is no TX cell to the new parent yet).
        _uinject_task_cb();
        if (uinject_vars.busySendingUinject) {
            uinject_quickTicks = 0;
        } else {
            uinject_quickTicks--;
        }
    }
}

static void _uinject_sendSetVoltage(uint8_t eui, uint16_t mv, uint8_t seq,
                                     uint8_t hops) {
    uint8_t payload[UINJECT_CMD_LEN];
    sock_udp_ep_t remote;

    memcpy(payload, uinject_setvolt_cmd, 7);
    payload[7] = eui;
    payload[8] = (uint8_t)(mv & 0xff);
    payload[9] = (uint8_t)((mv >> 8) & 0xff);
    payload[10] = seq;
    payload[11] = hops;

    memset(&remote, 0, sizeof(remote));
    remote.port = WKP_UDP_INJECT;
    remote.family = AF_INET6;
    memcpy(remote.addr.ipv6, uinject_bcast_addr, sizeof(uinject_bcast_addr));

    openserial_printf("SETVOLT tx eui=%x mv=%d seq=%d hops=%d\r\n",
                      (int)eui, (int)mv, (int)seq, (int)hops);
    sock_udp_send(&_sock, payload, sizeof(payload), &remote);
}

void uinject_sock_handler(sock_udp_t *sock, sock_async_flags_t type, void *arg) {
    (void) arg;

    char buf[50];

    if (type & SOCK_ASYNC_MSG_RECV) {
        sock_udp_ep_t remote;
        int16_t res;

        res = sock_udp_recv(sock, buf, sizeof(buf), 0, &remote);
        if (res >= 0) {
            if (res >= UINJECT_CMD_LEN &&
                    memcmp((uint8_t*)buf, uinject_setvolt_cmd, 7) == 0) {
                uint16_t mv = (uint16_t)(uint8_t)buf[8] |
                              ((uint16_t)(uint8_t)buf[9] << 8);
                uint8_t seq = (uint8_t)buf[10];
                uint8_t hops = (uint8_t)buf[11];
                // downlink command broadcast by the root: apply if it is for me
                if ((uint8_t)buf[7] == idmanager_getMyID(ADDR_64B)->addr_64b[7]) {
                    if (seq == uinject_lastSeq) {
                        // a copy of the command already in force
                        if (++uinject_candCount < UINJECT_SEQ_FORCE) {
                            openserial_printf("SETVOLT stale seq=%d\r\n", (int)seq);
                            return;
                        }
                    } else if ((int8_t)(seq - uinject_lastSeq) < 0) {
                        // older than what is in force: a stray copy, unless the
                        // root restarted and keeps sending this same value
                        if (seq == uinject_candSeq) {
                            uinject_candCount++;
                        } else {
                            uinject_candSeq = seq;
                            uinject_candCount = 1;
                        }
                        if (uinject_candCount < UINJECT_SEQ_FORCE) {
                            openserial_printf("SETVOLT stale seq=%d\r\n", (int)seq);
                            return;
                        }
                        openserial_printf("SETVOLT force seq=%d\r\n", (int)seq);
                    }
                    uinject_lastSeq = seq;
                    uinject_candSeq = seq;
                    uinject_candCount = 0;
                    energy_aware_setVoltage(mv);
                    uinject_quickTicks = UINJECT_QUICK_TICKS;
                    openserial_printf("SETVOLT rx mv=%d seq=%d\r\n", (int)mv, (int)seq);
                } else if (hops > 0 && uinject_relayHops == 0 &&
                           idmanager_getIsDAGroot() == FALSE) {
                    // (the root originated the command, it must not relay its
                    // own copy back into the mesh)
                    // Not mine, but a node further down the tree may not hear the
                    // root: repeat it once with one hop less.  uinject_relayHops
                    // doubles as the "window open" flag, so the three copies the
                    // root sends produce a single relay.
                    uinject_relayEui = (uint8_t)buf[7];
                    uinject_relayMv = mv;
                    uinject_relayHops = hops;      // marks the window open
                    uinject_relayTicks = UINJECT_RELAY_TICKS;
                    openserial_printf("SETVOLT relay eui=%x mv=%d\r\n",
                                      (int)uinject_relayEui, (int)uinject_relayMv);
                    _uinject_sendSetVoltage(uinject_relayEui, uinject_relayMv,
                                            seq, (uint8_t)(hops - 1));
                }
                return;
            }
            if (idmanager_getIsDAGroot() && res >= UINJECT_PAYLOAD_LEN) {
                // A child reported in.  Emit one machine-readable line so the
                // host-side topology viewer can rebuild the tree and show each
                // node's energy-aware working point:
                //   TOPO root=<eui7> node=<eui7> parent=<eui7> mv=<mV> tf=<n>
                uint8_t *p = (uint8_t*)buf;
                uint16_t mv;
                mv = (uint16_t)p[UINJECT_OFF_MV] |
                     ((uint16_t)p[UINJECT_OFF_MV + 1] << 8);
                uinject_cacheStore(p[UINJECT_OFF_NODE], p[UINJECT_OFF_PARENT], mv,
                                   p[UINJECT_OFF_TF]);
                openserial_printf("TOPO root=%x node=%x parent=%x mv=%d tf=%d\r\n",
                        (int)idmanager_getMyID(ADDR_64B)->addr_64b[7],
                        (int)p[UINJECT_OFF_NODE],
                        (int)p[UINJECT_OFF_PARENT],
                        (int)mv,
                        (int)p[UINJECT_OFF_TF]);
            } else {
                openserial_printf("Received %d bytes from remote endpoint:\n", res);
                openserial_printf(" - port: %d", remote.port);
                openserial_printf(" - addr: ", remote.port);
                for(int i=0; i < 16; i ++)
                    openserial_printf("%x ", remote.addr.ipv6[i]);

                openserial_printf("\n\n");
                openserial_printf("Msg received: %s\n\n", buf);
            }
        }
    }

    if (type & SOCK_ASYNC_MSG_SENT) {
        owerror_t error = *(uint8_t*)arg;
        if (error == E_FAIL) {
            LOG_ERROR(COMPONENT_UINJECT, ERR_MAXRETRIES_REACHED,
                    (errorparameter_t) uinject_vars.counter,
                    (errorparameter_t) 0);
        }
        // allow send next uinject packet
        uinject_vars.busySendingUinject = FALSE;
        busyTicks = 0;
    }
}


void _uinject_timer_cb(opentimers_id_t id) {
    // calling the task directly as the timer_cb function is executed in
    // task mode by opentimer already
    if (openrandom_get16b() < (0xffff / UINJECT_TRAFFIC_RATE)) {
        _uinject_task_cb();
    }
}

void _uinject_task_cb(void) {
    uint8_t asnArray[5];
    open_addr_t parentNeighbor;
    bool foundNeighbor;
    // don't run if not synch
    if (ieee154e_isSynch() == FALSE) {
        return;
    }

    // don't run on dagroot
    if (idmanager_getIsDAGroot()) {
        opentimers_destroy(uinject_vars.timerId);
        return;
    }

    foundNeighbor = icmpv6rpl_getPreferredParentEui64(&parentNeighbor);
    if (foundNeighbor == FALSE) {
        return;
    }

    if (schedule_hasNegotiatedCellToNeighbor(&parentNeighbor, CELLTYPE_TX) == FALSE) {
        return;
    }

    if (uinject_vars.busySendingUinject == TRUE) {
        // Don't continue if I'm still sending a previous uinject packet, but do
        // not wait forever: a node that also forwards its children's traffic can
        // lose the SOCK_ASYNC_MSG_SENT callback, and without this timeout it would
        // stay silent until the next reset.
        if (busyTicks < UINJECT_BUSY_TIMEOUT_TICKS) {
            busyTicks++;
            return;
        }
        busyTicks = 0;
        uinject_vars.busySendingUinject = FALSE;
    }
    busyTicks = 0;

    // if you get here, send a packet
    sock_udp_ep_t remote;
    remote.port = WKP_UDP_INJECT;
    remote.family = AF_INET6;
    memcpy(remote.addr.ipv6, uinject_dst_addr, sizeof(uinject_dst_addr));

    uint8_t payload[50];
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
    payload[len++] = msf_getPreviousNumCellsUsed(CELLTYPE_TX);
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
    // add energy-aware working point + topology identity (see the UINJECT_OFF_*
    // block at the top of this file)
    payload[len++] = (uint8_t)(energy_vars.voltage_mV & 0xff);
    payload[len++] = (uint8_t)((energy_vars.voltage_mV >> 8) & 0xff);
    payload[len++] = energy_vars.throttle_factor;
    payload[len++] = idmanager_getMyID(ADDR_64B)->addr_64b[7];
    payload[len++] = parentNeighbor.addr_64b[7];

    if (sock_udp_send(&_sock, payload, len, &remote) > 0) {
        // set busySending to TRUE
        uinject_vars.busySendingUinject = TRUE;
    }
}

#endif /* OPENWSN_UINJECT_C */
