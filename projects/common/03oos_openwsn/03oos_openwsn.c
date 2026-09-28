/**
\brief This project runs the full OpenWSN stack.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2010
*/

#include "config.h"
#include "board.h"
#include "scheduler.h"
#include "openstack.h"
#include "opendefs.h"
#include "energy_aware.h"
#include "debugpins.h"
#include "uart.h"

int mote_main(void) {
   
   // initialize
   board_init();
   uart_bootProbe("P|");
   scheduler_init();
   uart_bootProbe("S|");
   openstack_init();
   uart_bootProbe("B|");
   uart_bootProbe(uart_hasCallbacks() ? "T|" : "t|");
   energyMeasurementInit();
   uart_bootProbe("C|");
   // start
   scheduler_start();
   uart_bootProbe("!!scheduler_start returned");

   return 0; // this line should never be reached
}

