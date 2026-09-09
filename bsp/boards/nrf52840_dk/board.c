/**
 * Author: Tamas Harczos (tamas.harczos@imms.de)
 * Date:   Apr 2018
 * Description: nRF52840-specific definition of the "board" bsp module.
 */

#include "nrf52840.h"
#include "board.h"
#include "leds.h"
#include "sctimer.h"
#include "debugpins.h"
#include "uart.h"
#include "radio.h"
#include "spi.h"
#include "radio.h"
#include "sensors.h"
#include "i2c.h"


//=========================== variables =======================================

//=========================== prototypes ======================================

void enable_dcdc(void);
//=========================== main ============================================

extern int mote_main(void);

int main(void) {
    return mote_main();
}

//=========================== public ==========================================

void board_init(void) {


    leds_init();
    
    //debugpins_init();
#if SUPER_LOW_POWER
#else
    uart_init();
#endif
    sctimer_init();
    radio_init();

#if SUPER_LOW_POWER
#else
    i2c_init();
#endif
    // configure dcdc
    enable_dcdc();
}

/**
 * Puts the board to sleep
 */
void board_sleep(void) {
    // 1. 核心修复：如果使用了 FPU，必须在休眠前强行清除 FPU 中断挂起状态
    #if defined(__FPU_USED) && (__FPU_USED == 1)
    __set_FPSCR(__get_FPSCR() & ~(0x0000009FUL)); // 清除所有 FPU 异常标志
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);              // 清除 FPU 挂起的中断
    #endif

    // 2. 标准的 Nordic 官方无 OS 休眠序列
    __WFE();
    __SEV();
    __WFE(); // 这三步确保事件寄存器彻底被清干净
}

/**
 * Resets the board
 */
void board_reset(void) {

    NVIC_SystemReset();
}

//=========================== private =========================================

void enable_dcdc(void) {
    NRF_POWER->DCDCEN = 1;

    if (NRF_POWER->MAINREGSTATUS & POWER_MAINREGSTATUS_MAINREGSTATUS_Msk) {
        NRF_POWER->DCDCEN0 = 1;
    }
}
//=========================== interrupt handlers ==============================
