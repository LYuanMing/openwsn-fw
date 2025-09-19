/**
\brief This is a program which shows how to use the bsp modules for the board
       and leds.

\note: Since the bsp modules for different platforms have the same declaration,
       you can use this project with any platform.

Load this program on your boards. The LEDs should start blinking furiously.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2014.
*/

#include "stdint.h"
#include "stdio.h"
// bsp modules required
#include "board.h"
#include "leds.h"
#include "sctimer.h"

#define SECOND     32768 // @32kHz = 1s
#define MILLISECOND           SECOND / 1000

#define TASK1_1

uint8_t timer_flag = 0;
void some_delay(void);

void timer_callback(void);
/**
\brief The program starts executing here.
*/

int mote_main(void) {uint8_t i;
   
  board_init();
  #ifdef TASK1_1
  #else
  sctimer_set_callback(timer_callback);
  #endif
  while (TRUE) {
    
   // error LED functions
   leds_error_on();          some_delay();
   leds_error_off();         some_delay();

   // sync LED functions
   leds_sync_on();           some_delay();
   leds_sync_off();          some_delay();
   
   // debug LED functions
   leds_debug_on();          some_delay();
   leds_debug_off();         some_delay();
   
   // radio LED functions
   leds_radio_on();          some_delay();
   leds_radio_off();         some_delay();
   

  }

   board_reset();
   
   return 0;
}
#ifdef TASK1_1
void some_delay(void)
{
    for (uint32_t i = 0; i <= 0xfffff; i++);
}
#else
void some_delay(void)
{

    timer_flag = 0;
    sctimer_setCompare(sctimer_readCounter() + 50 * MILLISECOND);
    while(timer_flag == 0) {}
}

void timer_callback(void)
{
    timer_flag = 1;
}
#endif