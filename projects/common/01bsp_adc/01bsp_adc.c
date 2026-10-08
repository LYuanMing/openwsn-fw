#include "board.h"
#include "adc.h"

int mote_main(void)
{
  board_init();
  adc_init();


  int16_t result = 0;
  double voltage = 0;
  while(1){
    result = 0;
    result = adc_read();
    voltage = (result * 3.6f) / 1024.0f;
  }

  return 0;
}