#include "config.h"

#if OPENWSN_UDP_C

#include "sock_internal.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "scheduler.h"
#include "udp.h"
#include "openqueue.h"
#include "forwarding.h"
// perf3 2026-10-05 READ-ONLY: udp_transmit exit counters
volatile uint32_t perf3_udpTransmitCalls = 0;
volatile uint32_t perf3_udpFreedNoNotify = 0;
volatile uint32_t perf3_udpAccepted = 0;

//=========================== variables =======================================

//=========================== prototypes ======================================

//=========================== public ==========================================

void udp_sendDone(OpenQueueEntry_t *msg, owerror_t error) {
    msg->owner = COMPONENT_UDP;
    sock_senddone_internal(msg, error);
    openqueue_freePacketBuffer(msg);
}

void udp_receive(OpenQueueEntry_t *msg) {

    msg->owner = COMPONENT_UDP_TO_SOCK;
    msg->l4_sourcePortORicmpv6Type = packetfunctions_ntohs((uint8_t * ) & (((udp_ht *) (msg->payload))->source_port));
    msg->l4_destination_port = packetfunctions_ntohs((uint8_t * ) & (((udp_ht *) (msg->payload))->dest_port));

    // verify checksum

    scheduler_push_task(sock_receive_internal, TASKPRIO_UDP);

    packetfunctions_tossHeader(&msg, sizeof(udp_ht));
    msg->l4_length = msg->length;
    msg->l4_payload = msg->payload;
}

void udp_transmit(OpenQueueEntry_t *msg) {
    msg->l4_protocol_compressed = FALSE;
    msg->l4_protocol = IANA_UDP;

    packetfunctions_reserveHeader(&msg, sizeof(udp_ht));
    packetfunctions_htons(msg->l4_sourcePortORicmpv6Type, &(msg->payload[0]));
    packetfunctions_htons(msg->l4_destination_port, &(msg->payload[2]));
    packetfunctions_htons(msg->length, &(msg->payload[4]));
    packetfunctions_calculateChecksum(msg, (uint8_t * ) & (((udp_ht *) msg->payload)->checksum));

    // perf3 2026-10-05 -- BEHAVIOUR FIX (not merely an observation).
    //
    // forwarding_send() returns E_FAIL when the next hop cannot be resolved.
    // Measured cause (FREEZE_20261003.md appendix AA): a 6P reply of
    // IANA_6TOP_RC_BUSY makes msf.c:220 call neighbors_setNeighborNoResource(),
    // which sets f6PNORES on that neighbour for good.  From then on
    // icmpv6rpl_getPreferredParentEui64() (icmpv6rpl.c:307) returns FALSE
    // because of that flag, forwarding_getNextHop() discards the FALSE and
    // leaves l2_nextORpreviousHop == ADDR_NONE, and
    // forwarding_send_internal_RoutingTable() (forwarding.c:519) returns E_FAIL.
    //
    // The old code freed the packet here WITHOUT telling the socket layer.  The
    // application then waited forever for a send-done that could not arrive:
    // uinject clears busySendingUinject only in its SOCK_ASYNC_MSG_SENT callback
    // (uinject.c:148), so it stayed TRUE and every later packet was shed by
    // d_busy -- the "transmit freeze".
    //
    // Silently discarding a packet the caller is waiting on violates the socket
    // contract.  udp_sendDone() is the notification path the success case
    // already uses (forwarding.c:292); it also frees the buffer, so the explicit
    // free must NOT happen here any more.
    perf3_udpTransmitCalls++;
    if (forwarding_send(msg) == E_FAIL) {
        perf3_udpFreedNoNotify++;
        udp_sendDone(msg, E_FAIL);
    } else {
        perf3_udpAccepted++;
    }
}

#endif /* OPENWSN_UDP_C */
