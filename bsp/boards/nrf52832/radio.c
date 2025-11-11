/**
\brief nRF52832 definition of the "radio" bsp module.

\author: Tengfei Chang <tengfei.chang@inria.fr> August 2020
*/

#include "nrf52.h"
#include "nrf52_bitfields.h"
#include "board.h"
#include "radio.h"
#include "debugpins.h"
#include "leds.h"

//=========================== defines =========================================

#define RADIO_POWER_POWER_POS       0

#define STATE_DISABLED              0
#define STATE_RXRU                  1
#define STATE_RXIDLE                2
#define STATE_RX                    3
#define STATE_RXDISABLE             4
#define STATE_TXTU                  9
#define STATE_TXIDLE                10
#define STATE_TX                    11
#define STATE_TXDIABLE              12

#define MAX_PACKET_SIZE           (255)       ///< maximal size of radio packet (one more byte at the beginning needed to store the length)
#define CRC_POLYNOMIAL            (0x11021)   ///< polynomial used for CRC calculation in 802.15.4 frames (x^16 + x^12 + x^5 + 1)

#define RADIO_CRCINIT_24BIT       0x555555
#define RADIO_CRCPOLY_24BIT       0x0000065B  /// ref: https://devzone.nordicsemi.com/f/nordic-q-a/44111/crc-register-values-for-a-24-bit-crc

#define INTERFRAM_SPACING         (150)       // in us

#define BLE_ACCESS_ADDR           0x8E89BED6  // the actual address is 0xD6, 0xBE, 0x89, 0x8E

#define RADIO_TXPOWER             0x00 // in 2-compilant format  0xec == -20db

//=========================== variables =======================================

typedef struct {
    radio_capture_cbt         startFrame_cb;
    radio_capture_cbt         endFrame_cb;
    radio_state_t             state;
    uint8_t                   payload[MAX_PACKET_SIZE] __attribute__ ((aligned));
} radio_vars_t;

radio_vars_t radio_vars;

//=========================== private =========================================

uint32_t ble_channel_to_frequency(uint8_t channel);

static void hfclock_start(void);
static void hfclock_stop(void);

//=========================== public ==========================================

//===== admin

void radio_init(void) {

    uint8_t i;

    // clear variables
    memset(&radio_vars,0,sizeof(radio_vars_t));

    // set radio configuration parameters
    NRF_RADIO->TXPOWER     = (uint32_t)RADIO_TXPOWER;

    // configure packet
    NRF_RADIO->PCNF0       = 
        (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) | 
        (((0UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) |
        (((8UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk);

    NRF_RADIO->PCNF1       = 
        (((RADIO_PCNF1_ENDIAN_Little)    << RADIO_PCNF1_ENDIAN_Pos)  & RADIO_PCNF1_ENDIAN_Msk)  |
        (((3UL)                          << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)   |
        (((0UL)                          << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |
        ((((uint32_t)MAX_PACKET_SIZE)    << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)  |
        ((RADIO_PCNF1_WHITEEN_Enabled    << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk);

        
    NRF_RADIO->CRCPOLY     = RADIO_CRCPOLY_24BIT;
    NRF_RADIO->CRCCNF      = 
        (((RADIO_CRCCNF_SKIPADDR_Skip) << RADIO_CRCCNF_SKIPADDR_Pos) & RADIO_CRCCNF_SKIPADDR_Msk) |
        (((RADIO_CRCCNF_LEN_Three)     << RADIO_CRCCNF_LEN_Pos)      & RADIO_CRCCNF_LEN_Msk);

    NRF_RADIO->CRCINIT     = RADIO_CRCINIT_24BIT;

    NRF_RADIO->TXADDRESS   = 0;
    NRF_RADIO->RXADDRESSES = 1;

    NRF_RADIO->MODE        = ((RADIO_MODE_MODE_Ble_1Mbit) << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk;
    NRF_RADIO->TIFS        = INTERFRAM_SPACING;
    NRF_RADIO->PREFIX0     = ((BLE_ACCESS_ADDR & 0xff000000) >> 24);
    NRF_RADIO->BASE0       = ((BLE_ACCESS_ADDR & 0x00ffffff) << 8 );

    NRF_RADIO->PACKETPTR   = (uint32_t)(radio_vars.payload);

    // set priority and disable interrupt in NVIC
    NVIC->IP[((uint32_t)RADIO_IRQn)]     = 
        (uint8_t)(
            (RADIO_PRIORITY << (8 - __NVIC_PRIO_BITS)) & (uint32_t)0xff
        );
    NVIC->ICER[((uint32_t)RADIO_IRQn)>>5] = 
       ((uint32_t)1) << ( ((uint32_t)RADIO_IRQn) & 0x1f);

    // enable address and payload interrupts 
    NRF_RADIO->INTENSET   = 
        RADIO_INTENSET_ADDRESS_Set    << RADIO_INTENSET_ADDRESS_Pos |
        RADIO_INTENSET_END_Set        << RADIO_INTENSET_END_Pos;
    
    NVIC->ICPR[((uint32_t)RADIO_IRQn)>>5] = 
       ((uint32_t)1) << ( ((uint32_t)RADIO_IRQn) & 0x1f);
    NVIC->ISER[((uint32_t)RADIO_IRQn)>>5] = 
       ((uint32_t)1) << ( ((uint32_t)RADIO_IRQn) & 0x1f);

    radio_vars.state        = RADIOSTATE_STOPPED;
}

void radio_setStartFrameCb(radio_capture_cbt cb) {
    
    radio_vars.startFrame_cb  = cb;
}

void radio_setEndFrameCb(radio_capture_cbt cb) {

    radio_vars.endFrame_cb    = cb;
}

//===== reset

void radio_reset(void) {

    // reset is implemented by power off and power radio
    NRF_RADIO->POWER = ((uint32_t)(0)) << RADIO_POWER_POWER_POS;
    NRF_RADIO->POWER = ((uint32_t)(1)) << RADIO_POWER_POWER_POS;

    radio_vars.state    = RADIOSTATE_STOPPED;
}

//===== RF admin

void radio_setFrequency(uint8_t channel, radio_freq_t tx_or_rx) {

    NRF_RADIO->FREQUENCY     = ble_channel_to_frequency(channel);
    NRF_RADIO->DATAWHITEIV   = channel; 

    radio_vars.state            = RADIOSTATE_FREQUENCY_SET;
}


uint32_t radio_get_frequency(void) {

    // return value, in MHz
    return (2400 + (NRF_RADIO->FREQUENCY & 0x0000007F));
}

void radio_rfOn(void) {
    
#ifdef  SUPER_LOW_POWER
    hfclock_start();
#endif
    // power on radio
    NRF_RADIO->POWER = ((uint32_t)(1)) << RADIO_POWER_POWER_POS;

    radio_vars.state    = RADIOSTATE_STOPPED;
}

void radio_rfOff(void) {

    radio_vars.state  = RADIOSTATE_TURNING_OFF;

    NRF_RADIO->EVENTS_DISABLED = 0;

    // stop radio
    NRF_RADIO->TASKS_DISABLE = (uint32_t)(1);

    while(NRF_RADIO->EVENTS_DISABLED==0);

    // wiggle debug pin
    debugpins_radio_clr();
    leds_radio_off();
    radio_vars.state  = RADIOSTATE_RFOFF;

#ifdef  SUPER_LOW_POWER
    hfclock_stop();
#endif

}

int8_t radio_getFrequencyOffset(void){

    // not supported
    return 0;
}

//===== TX

void radio_loadPacket(uint8_t* packet, uint16_t len) {

    radio_vars.state  = RADIOSTATE_LOADING_PACKET;

    ///< note: 1st byte should be the payload size (for Nordic), and
    ///   the two last bytes are used by the MAC layer for CRC
    if ((len > 0) && (len <= MAX_PACKET_SIZE)) {
        radio_vars.payload[0]= len;
        memcpy(&radio_vars.payload[1], packet, len);
    }

    // (re)set payload pointer
    NRF_RADIO->PACKETPTR = (uint32_t)(radio_vars.payload);

    radio_vars.state  = RADIOSTATE_PACKET_LOADED;
}

void radio_txEnable(void) {

    radio_vars.state  = RADIOSTATE_ENABLING_TX;

    NRF_RADIO->EVENTS_READY = (uint32_t)0;

    NRF_RADIO->TASKS_TXEN = (uint32_t)1;
    while(NRF_RADIO->EVENTS_READY==0);

    // wiggle debug pin
    debugpins_radio_set();
    leds_radio_on();

    radio_vars.state  = RADIOSTATE_TX_ENABLED;
}

void radio_txNow(void) {

    NRF_RADIO->TASKS_START = (uint32_t)1;

    radio_vars.state  = RADIOSTATE_TRANSMITTING;
}

//===== RX

void radio_rxEnable(void) {

    radio_vars.state  = RADIOSTATE_ENABLING_RX;

    if (NRF_RADIO->STATE != STATE_RX){

       // turn off radio first
       radio_rfOff();
        
        NRF_RADIO->EVENTS_READY = (uint32_t)0;

        NRF_RADIO->TASKS_RXEN  = (uint32_t)1;

        while(NRF_RADIO->EVENTS_READY==0);
    }

    // wiggle debug pin
    debugpins_radio_set();
    leds_radio_on();
}

void radio_rxNow(void) {

    NRF_RADIO->TASKS_START = (uint32_t)1;

    radio_vars.state  = RADIOSTATE_LISTENING;
}

void radio_getReceivedFrame(
      uint8_t* bufRead,
      uint8_t* lenRead,
      uint8_t  maxBufLen,
      int8_t*  rssi,
      uint8_t* lqi,
      bool*    crc
   ) {

    // check for length parameter; if too long, payload won't fit into memory
    uint8_t len;

    len = radio_vars.payload[0];

    if (len == 0) {
        return; 
    }

    if (len > MAX_PACKET_SIZE) { 
        len = MAX_PACKET_SIZE; 
    }

    if (len > maxBufLen) { 
        len = maxBufLen; 
    }

    // copy payload
    memcpy(bufRead, &radio_vars.payload[1], len);

    // store other parameters
    *lenRead = len;

    *rssi = (int8_t)(0-NRF_RADIO->RSSISAMPLE);

    *crc = (NRF_RADIO->CRCSTATUS == 1U);
}

void radio_get_crc(uint8_t* crc24){

    uint32_t crc;
    crc = NRF_RADIO->RXCRC;

    crc24[0] = (uint8_t)((crc & 0x00ff0000) >> 16);
    crc24[1] = (uint8_t)((crc & 0x0000ff00) >> 8);
    crc24[2] = (uint8_t)((crc & 0x000000ff) >> 0);
}

//=========================== private =========================================

uint32_t ble_channel_to_frequency(uint8_t channel) {

    uint32_t frequency;
    
    if (channel<=10) {

        frequency = 4+2*channel;
    } else {
        if (channel >=11 && channel <=36) {
            
            frequency = 28+2*(channel-11);
        } else {
            switch(channel) {
                case 37:
                    frequency = 2;
                break;
                case 38:
                    frequency = 26;
                break;
                case 39:
                    frequency = 80;
                break;
                default:
                    // something goes wrong
                    frequency = 2;

            }
        }
    }

    return frequency;
}

static void hfclock_start(void) {
    
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

static void hfclock_stop(void) {
    
    // check clock source
    if((NRF_CLOCK->HFCLKSTAT & 0x00000001) != 0) {
        
        // clock running?
        if((NRF_CLOCK->HFCLKSTAT & 0x00010000) != 0) {
            
            NRF_CLOCK->TASKS_HFCLKSTOP = 1;
            while((NRF_CLOCK->HFCLKSTAT & 0x00000001) != 0);
        }
    }
}

//=========================== callbacks =======================================

kick_scheduler_t    radio_isr(void){

    uint32_t time_stampe;

    time_stampe = NRF_RTC0->COUNTER;

    // start of frame (payload)
    if (NRF_RADIO->EVENTS_ADDRESS){

        // start sampling rssi
        NRF_RADIO->TASKS_RSSISTART = (uint32_t)1;

        if (radio_vars.startFrame_cb!=NULL){
            radio_vars.startFrame_cb(time_stampe);
        }
        
        NRF_RADIO->EVENTS_ADDRESS = (uint32_t)0;
        return KICK_SCHEDULER;
    }

    // END 
    if (NRF_RADIO->EVENTS_END) {
        if (radio_vars.endFrame_cb!=NULL){
            radio_vars.endFrame_cb(time_stampe);
        }

        NRF_RADIO->EVENTS_END = (uint32_t)0;
        return KICK_SCHEDULER;
    }

    return DO_NOT_KICK_SCHEDULER;
}

//=========================== interrupt handlers ==============================

void RADIO_IRQHandler(void) {

    debugpins_isr_set();

    radio_isr();

    debugpins_isr_clr();
}
