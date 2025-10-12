/**
\brief This program shows the use of the "sctimer" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

Load this program onto your board, and start running. It will enable the sctimer. 
The sctimer is periodic, of period SCTIMER_PERIOD ticks. Each time it elapses:
    - the frame debugpin toggles
    - the error LED toggles

\author Tengfei Chang <tengfei.chang@eecs.berkeley.edu>, April 2017.
*/

#include "stdint.h"
#include "string.h"
#include "board.h"
#include "debugpins.h"
#include "leds.h"
#include "sctimer.h"

//=========================== defines =========================================

//#define SCTIMER_PERIOD     32768 // @32kHz = 1s
#define SCTIMER_PERIOD     16384    // @32kHz = 500ms

//=========================== variables =======================================

typedef struct {
   uint16_t num_compare;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void cb_compare(void);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {  
   
   // initialize board. 
   board_init();
   leds_init();
   sctimer_set_callback(cb_compare);
   sctimer_setCompare(sctimer_readCounter()+SCTIMER_PERIOD);
   
   while (1) {
      board_sleep();
      if(app_vars.num_compare % 2 == 0) {
        leds_sync_toggle();
      }  
      if(app_vars.num_compare % 3 == 0) {
        leds_radio_toggle();
      } 
      if(app_vars.num_compare % 4 == 0) {
        leds_debug_toggle();
      }
      leds_error_toggle();
   }
}

//=========================== callbacks =======================================

void cb_compare(void) {
   
   // toggle pin
   debugpins_frame_toggle();
   
   // increment counter
   app_vars.num_compare++;
   
   // schedule again
   sctimer_setCompare(sctimer_readCounter()+SCTIMER_PERIOD);
}
