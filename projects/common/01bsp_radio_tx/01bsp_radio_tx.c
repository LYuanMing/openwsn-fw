/**
\brief This program shows the use of the "radio" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

The board running this program will send a packet on channel CHANNEL every
TIMER_PERIOD ticks. The packet contains LENGTH_PACKET bytes. The first byte
is the packet number, which increments for each transmitted packet. The
remainder of the packet contains an incrementing bytes.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2014.
*/

#include "stdint.h"
#include "string.h"
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"

//=========================== defines =========================================
#define LEN_WITHOUT_CRC 20
#define LENGTH_PACKET   LEN_WITHOUT_CRC+LENGTH_CRC // maximum length is 127 bytes
#define CHANNEL         11            // 24ghz: 11 = 2.405GHz, subghz: 11 = 865.325 in  FSK operating mode #1
#define TIMER_PERIOD    490    // (32768>>1) = 500ms @ 32kHz

//=========================== variables =======================================

typedef struct {
    uint8_t              num_scTimerCompare;
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
    uint8_t              txpk_txNow;
    uint8_t              txpk_buf[LENGTH_PACKET];
    uint8_t              txpk_len;
    uint8_t              txpk_num;
    

    uint8_t uart_lastTxByteIndex;
    volatile   uint8_t uartDone;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void cb_scTimerCompare(void);
void cb_startFrame(PORT_TIMER_WIDTH timestamp);
void cb_endFrame(PORT_TIMER_WIDTH timestamp);


void send_string(const char* str);
void cb_uartTxDone(void);
uint8_t cb_uartRxCb(void);

static inline uint8_t check_sum(const uint8_t* buffer, int size);

uint8_t stringToSend[256] = {0};
uint16_t length = 0;
//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint8_t  i;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();

    // add radio callback functions
    uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    uart_enableInterrupts();
    length = strlen("uart is ok!\r\n");
    send_string("uart is ok!\r\n");


    sctimer_set_callback(cb_scTimerCompare);
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_TX);
    radio_rfOff();

    length = 7;
    send_string("start\r\n");
    // start periodic overflow
    sctimer_setCompare(sctimer_readCounter()+ TIMER_PERIOD);
    sctimer_enable();
    while(1) {
        // wait for timer to elapse
        app_vars.txpk_txNow = 0;
        while (app_vars.txpk_txNow==0) {
            board_sleep();
        }
        // freq type only effects on scum port
        radio_setFrequency(CHANNEL, FREQ_TX);
        // led
        leds_error_toggle();

        // prepare packet
        app_vars.txpk_num++;
        app_vars.txpk_len           = LENGTH_PACKET;
        app_vars.txpk_buf[0] = 'e';
        app_vars.txpk_buf[1] = 'e';
        app_vars.txpk_buf[2] = 'e';
        app_vars.txpk_buf[3] = 'e';
        app_vars.txpk_buf[4] = 'e';
        app_vars.txpk_buf[5] = 'e';
        app_vars.txpk_buf[6] = 'e';
        app_vars.txpk_buf[7] = 'e';
        app_vars.txpk_buf[8] = 'e';
        app_vars.txpk_buf[9] = 'e';
        app_vars.txpk_buf[10] = 'e';
        app_vars.txpk_buf[11] = 'e';
        app_vars.txpk_buf[12] = 'e';
        app_vars.txpk_buf[13] = 'e';
        app_vars.txpk_buf[14] = 'e';
        app_vars.txpk_buf[15] = 'e';
        app_vars.txpk_buf[16] = 'e';
        app_vars.txpk_buf[17] = 'e';
        app_vars.txpk_buf[18] = 'e';
        app_vars.txpk_buf[LEN_WITHOUT_CRC - 1] = 0;
        uint8_t sum = check_sum(&app_vars.txpk_buf, LEN_WITHOUT_CRC);
        app_vars.txpk_buf[LEN_WITHOUT_CRC - 1] = sum;
        //for (i=0;i<app_vars.txpk_len;i++) {
        //    app_vars.txpk_buf[i] = 'a' + i;
        //}

        // send packet
        radio_loadPacket(app_vars.txpk_buf,app_vars.txpk_len);
        radio_rfOff(); // important!!! radio_rfOff must be executed before transimitting
        radio_txEnable();
        radio_txNow();
    }
}

//=========================== callbacks =======================================

void cb_scTimerCompare(void) {

    // update debug vals
    app_dbg.num_scTimerCompare++;
    // ready to send next packet
    app_vars.txpk_txNow = 1;
    // schedule again
    sctimer_setCompare(sctimer_readCounter()+ TIMER_PERIOD);
}

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {

   // update debug vals
   app_dbg.num_startFrame++;

   // led
   leds_sync_on();

}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
   // update debug vals
   app_dbg.num_endFrame++;

   // led
   leds_sync_off();
}


void cb_uartTxDone(void) {
   app_vars.uart_lastTxByteIndex++;
   if (app_vars.uart_lastTxByteIndex<length) {
      uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
   } else {
      app_vars.uartDone = 1;
   }
}

uint8_t cb_uartRxCb(void) {
   uint8_t byte;
   
   // toggle LED
   leds_error_toggle();
   
   // read received byte
   byte = uart_readByte();
   
   // echo that byte over serial
   uart_writeByte(byte);
   
   return 0;
}

static inline uint8_t check_sum(const uint8_t* buffer, int size)
{
    uint8_t sum = 0;
    while(size > 1) {
        sum += *buffer++;
        size -= sizeof(uint8_t);
    }

    return (~sum) + 1;
}

void send_string(const char* str)
{
    strcpy(stringToSend, str);
    for (uint16_t i = length; i < sizeof(stringToSend); i++)
    stringToSend[i] = 0;
    app_vars.uartDone = 0;
    app_vars.uart_lastTxByteIndex = 0;
    uart_writeByte(stringToSend[app_vars.uart_lastTxByteIndex]);
    while(app_vars.uartDone==0);
}

