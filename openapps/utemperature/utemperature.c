#include "utemperature.h"
#include "sock.h"
#include "openserial.h"
#include "async.h"
#include "opentimers.h"
#include "scheduler.h"
#include "IEEE802154E.h"
#include "idmanager.h"
#include "icmpv6rpl.h"
#ifdef NRF52840_DK
#include "nrf52840.h"
#endif

static sock_udp_t _sock;
void utemperature_sock_handler(sock_udp_t *sock, sock_async_flags_t type, void *arg);
void utemperature_get_temperature(int32_t* result);
void utemperature_timer_cb(opentimers_id_t id);

opentimers_id_t timerID = 0;
bool busySendingUinject = FALSE;
static const uint8_t prefix_payload[] = "temperature: ";
static const uint8_t dst_addr[] = {
        0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};
static uint8_t payload[50];
static uint8_t len = 0;
static int32_t temperature;


void utemperature_init(void)
{
    memset(&_sock, 0, sizeof(sock_udp_t));
    sock_udp_ep_t local;
    local.family = AF_INET6;
    local.port = WKP_UDP_INJECT;

    if (sock_udp_create(&_sock, &local, NULL, 0) < 0) {
        openserial_printf("Could not create socket\n");
        return;
    }
    openserial_printf("Created a UDP socket\n");

    sock_udp_set_cb(&_sock, utemperature_sock_handler, NULL);

    timerID = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_UDP);
    opentimers_scheduleIn(
            timerID,
            UINJECT_PERIOD_MS,
            TIME_MS,
            TIMER_PERIODIC,
            utemperature_timer_cb
    );
}


void utemperature_sock_handler(sock_udp_t *sock, sock_async_flags_t type, void *arg)
{
    (void) arg;

    char buf[50];

    if (type & SOCK_ASYNC_MSG_RECV) {
        sock_udp_ep_t remote;
        int16_t res;

        if ((res = sock_udp_recv(sock, buf, sizeof(buf), 0, &remote)) >= 0) {
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
        owerror_t error = *(uint8_t*)arg;
        if (error == E_FAIL) {            
            openserial_printf("Fail to receive packet\r\n");

        }
        // allow send next uinject packet
        busySendingUinject = FALSE;
    }
}

void utemperature_get_temperature(int32_t* result)
{
  *result = -1;
  NRF_TEMP->EVENTS_DATARDY = 0;
  NRF_TEMP->TASKS_START = 1;
  while (NRF_TEMP->EVENTS_DATARDY == 0) {}
  *result = NRF_TEMP->TEMP;
  return;
}

void utemperature_timer_cb(opentimers_id_t id)
{
    bool foundNeighbor;
    open_addr_t parentNeighbor;


    // don't run if not synch
    if (ieee154e_isSynch() == FALSE) {
        return;
    }

    // don't run on dagroot
    if (idmanager_getIsDAGroot()) {
        opentimers_destroy(id);
        return;
    }

    foundNeighbor = icmpv6rpl_getPreferredParentEui64(&parentNeighbor);
    if (foundNeighbor == FALSE) {
        return;
    }

    if (schedule_hasNegotiatedCellToNeighbor(&parentNeighbor, CELLTYPE_TX) == FALSE) {
        return;
    }

    if (busySendingUinject == TRUE) {
        // don't continue if I'm still sending a previous uinject packet
        return;
    }

    // if you get here, send a packet
    sock_udp_ep_t remote;
    remote.port = WKP_UDP_INJECT;
    remote.family = AF_INET6;
    memcpy(remote.addr.ipv6, dst_addr, sizeof(dst_addr));

    len = 0;
    memcpy(&payload[len], prefix_payload, sizeof(prefix_payload) - 1);
    len += sizeof(prefix_payload) - 1;
    
    utemperature_get_temperature(&temperature);
    if(temperature != -1 && temperature <= 60) {
        payload[len++] = '0' + ((temperature / 10) % 10);
        payload[len++] = '0' + ((temperature) % 10);
        
        if (sock_udp_send(&_sock, payload, len, &remote) > 0) {
            busySendingUinject = TRUE;
        }
    }
}
