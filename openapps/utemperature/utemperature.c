#include "utemperature.h"
#include "sock.h"
#include "openserial.h"
#include "async.h"
#include "opentimers.h"
#include "scheduler.h"
#include "IEEE802154E.h"
#include "idmanager.h"
#include "icmpv6rpl.h"
#include "openrandom.h"
#include "msf.h"

#if ENERGY_THROTTLE
#include "energy_aware.h"
#endif

#ifdef NRF52840_DK
#include "nrf52840.h"
#endif

//=========================== defines =========================================

#define utemperature_TRAFFIC_RATE 1 ///> the value X indicates 1 packet per 2X seconds


//=========================== variables =======================================
typedef struct {
    opentimers_id_t timerId;   ///< periodic timer which triggers transmission
    uint16_t counter;  ///< incrementing counter which is written into the packet
    uint16_t period;  ///< utemperature packet sending period>
    bool busySendingutemperature;  ///< TRUE when busy sending an utemperature
} utemperature_vars_t;

static sock_udp_t _sock;
static utemperature_vars_t utemperature_vars;

static const uint8_t utemperature_payload[] = "utemperature";
static const uint8_t utemperature_dst_addr[] = {
        0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};

//=========================== prototypes ======================================

void utemperature_sock_handler(sock_udp_t *sock, sock_async_flags_t type, void *arg);

void _utemperature_timer_cb(opentimers_id_t id);

void _utemperature_task_cb(void);

//=========================== public ==========================================

void utemperature_init(void) {

    // clear local variables
    memset(&_sock, 0, sizeof(sock_udp_t));
    memset(&utemperature_vars, 0, sizeof(utemperature_vars_t));

    sock_udp_ep_t local;
    local.family = AF_INET6;
    local.port = WKP_UDP_INJECT;

    if (sock_udp_create(&_sock, &local, NULL, 0) < 0) {
        openserial_printf("Could not create socket\n");
        return;
    }

    openserial_printf("Created a UDP socket\n");

    sock_udp_set_cb(&_sock, utemperature_sock_handler, NULL);

    // start periodic timer
    utemperature_vars.period = UINJECT_PERIOD_MS;
    utemperature_vars.timerId = opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_UDP);
    opentimers_scheduleIn(
            utemperature_vars.timerId,
            UINJECT_PERIOD_MS,
            TIME_MS,
            TIMER_PERIODIC,
            _utemperature_timer_cb
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

void utemperature_sock_handler(sock_udp_t *sock, sock_async_flags_t type, void *arg) {
    (void) arg;

    char buf[50];

    if (type & SOCK_ASYNC_MSG_RECV) {
        sock_udp_ep_t remote;
        int16_t res;

        if ((res = sock_udp_recv(sock, buf, sizeof(buf), 0, &remote)) >= 0) {
            //openserial_printf("Received %d bytes from remote endpoint:\n", res);
            //openserial_printf(" - port: %d", remote.port);
            //openserial_printf(" - addr: ", remote.port);
            openserial_printf("msg received!\r\n");
        }
    }

    if (type & SOCK_ASYNC_MSG_SENT) {
        owerror_t error = *(uint8_t*)arg;
        if (error == E_FAIL) {
            LOG_ERROR(COMPONENT_UINJECT, ERR_MAXRETRIES_REACHED,
                    (errorparameter_t) utemperature_vars.counter,
                    (errorparameter_t) 0);
        }
        // allow send next utemperature packet
        utemperature_vars.busySendingutemperature = FALSE;
    }
}


void _utemperature_timer_cb(opentimers_id_t id) {
    // calling the task directly as the timer_cb function is executed in
    // task mode by opentimer already
    if (openrandom_get16b() < (0xffff / utemperature_TRAFFIC_RATE)) {
        _utemperature_task_cb();
    }
}

void _utemperature_task_cb(void) {
    uint8_t asnArray[5];
    open_addr_t parentNeighbor;
    bool foundNeighbor;
    // don't run if not synch
    if (ieee154e_isSynch() == FALSE) {
        return;
    }

    // don't run on dagroot
    if (idmanager_getIsDAGroot()) {
        opentimers_destroy(utemperature_vars.timerId);
        return;
    }

    foundNeighbor = icmpv6rpl_getPreferredParentEui64(&parentNeighbor);
    if (foundNeighbor == FALSE) {
        return;
    }

    if (schedule_hasNegotiatedCellToNeighbor(&parentNeighbor, CELLTYPE_TX) == FALSE) {
        return;
    }

    if (utemperature_vars.busySendingutemperature == TRUE) {
        // don't continue if I'm still sending a previous utemperature packet
        return;
    }

    // if you get here, send a packet
    sock_udp_ep_t remote;
    remote.port = WKP_UDP_INJECT;
    remote.family = AF_INET6;
    memcpy(remote.addr.ipv6, utemperature_dst_addr, sizeof(utemperature_dst_addr));

    uint8_t payload[50];
    uint8_t len = 0;
    // add 'utemperature' string
    memcpy(&payload[len], utemperature_payload, sizeof(utemperature_payload) - 1);
    len += sizeof(utemperature_payload) - 1;

    ieee154e_getAsn(asnArray);
    msf_getPreviousNumCellsUsed(CELLTYPE_TX);
    msf_getPreviousNumCellsUsed(CELLTYPE_RX);

    // add my address
    open_addr_t* myAddress;
    myAddress = idmanager_getMyID(ADDR_64B); 
    
    memcpy(&payload[len], myAddress->addr_64b, 8);
    len += 8;

    //int32_t temp;
    //get_temperature(&temp);
    payload[len++] = (uint8_t)energy_vars.voltage_mV & 0xff;
    payload[len++] = (uint8_t)((energy_vars.voltage_mV & 0xff00) >> 8);
    payload[len++] = (uint8_t)((energy_vars.voltage_mV & 0xff0000) >> 16);
    payload[len++] = (uint8_t)((energy_vars.voltage_mV & 0xff000000) >> 24);
    
    if (sock_udp_send(&_sock, payload, len, &remote) > 0) {
        // set busySending to TRUE
        openserial_printf("send a packet\r\n");
        utemperature_vars.busySendingutemperature = TRUE;
    }
}