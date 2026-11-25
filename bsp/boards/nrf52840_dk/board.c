/**
 * Author: Tamas Harczos (tamas.harczos@imms.de)
 * Date:   Apr 2018
 * Description: nRF52840-specific definition of the "board" bsp module.
 */

#include "nrf52840.h"
#include "config.h"
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

// --- DIAGNOSTIC (temporary): RAM vector table + RAM-resident RTC0 trampoline --
// Question: does the CPU ever *enter* the wake-up interrupt?  With the vector
// table in RAM we already know the exception entry succeeds; now the first
// instructions of the RTC0 handler also live in RAM, so they execute even if
// flash is not readable at that instant.  The trampoline records the clock state
// and then jumps to the real (flash) handler.  If the counter moves, the wake
// interrupt was delivered; if it stops at some value, it was not.
#define DIAG_VTOR_IN_RAM 0
#if DIAG_VTOR_IN_RAM
#define DIAG_MAGIC 0xD1A6F00D
// [0] enters, [1] HFCLKSTAT at entry, [2] RTC0 COUNTER at entry,
// [3] real RTC0 handler (filled at boot), [4] EVENTS_HFCLKSTARTED at entry,
// [5] magic
static volatile uint32_t board_diag[8] __attribute__((aligned(4)));
static uint32_t board_ramVectors[64] __attribute__((aligned(256)));
static uint32_t board_ramStub[32] __attribute__((aligned(4)));

__attribute__((naked, noinline)) static void board_rtcStub(void) {
    __asm volatile(
        "movw r0, #:lower16:board_diag\n\t"
        "movt r0, #:upper16:board_diag\n\t"
        "ldr  r1, [r0]\n\t"
        "adds r1, #1\n\t"
        "str  r1, [r0]\n\t"
        "movw r2, #0x0410\n\t"   // HFCLKSTAT
        "movt r2, #0x4000\n\t"
        "ldr  r3, [r2]\n\t"
        "str  r3, [r0, #4]\n\t"
        "movw r4, #0x0000\n\t"   // 0x00010000 = HFCLKSTAT.STATE (running)
        "movt r4, #0x0001\n\t"
        "tst  r3, r4\n\t"
        "bne  1f\n\t"
        "ldr  r5, [r0, #24]\n\t" // entries seen with HFCLK NOT running
        "adds r5, #1\n\t"
        "str  r5, [r0, #24]\n\t"
        "str  r1, [r0, #28]\n\t" // entry counter of the last such entry
        "1:\n\t"
        "movw r2, #0xB504\n\t"   // RTC0 COUNTER
        "movt r2, #0x4000\n\t"
        "ldr  r3, [r2]\n\t"
        "str  r3, [r0, #8]\n\t"
        "movw r2, #0x0500\n\t"   // EVENTS_HFCLKSTARTED
        "movt r2, #0x4000\n\t"
        "ldr  r3, [r2]\n\t"
        "str  r3, [r0, #16]\n\t"
        "movw r2, #0x5000\n\t"   // a word inside .text: proves flash is readable
        "movt r2, #0x0000\n\t"   // right now, in RAM code, before jumping to flash
        "ldr  r3, [r2]\n\t"
        "str  r3, [r0, #32]\n\t" // board_diag[8]
        "ldr  r3, [r0, #12]\n\t" // real handler, patched at boot
        "bx   r3\n\t"
    );
}

static void board_relocateVectors(void) {
    const uint32_t* flashVectors = (const uint32_t*)0x00000000;
    uint32_t* stub = board_ramStub;
    uint32_t i;
    for (i = 0; i < 64; i++) {
        board_ramVectors[i] = flashVectors[i];
    }
    // copy the trampoline (pure movw/movt code, no literals, no relocations)
    // NOTE: a function designator carries the Thumb bit in its low address bit,
    // so the source pointer must be masked -- otherwise the copy is shifted by
    // one byte and the RAM copy executes as garbage (observed: CFSR=UNALIGNED).
    const uint32_t* src = (const uint32_t*)(((uint32_t)board_rtcStub) & ~1u);
    for (i = 0; i < sizeof(board_ramStub)/sizeof(board_ramStub[0]); i++) {
        stub[i] = src[i];
    }
    board_diag[3] = flashVectors[27];              // RTC0_IRQn = 11 -> vector 27
    board_diag[5] = DIAG_MAGIC;
    board_ramVectors[27] = ((uint32_t)board_ramStub) | 1u;
    SCB->VTOR = (uint32_t)board_ramVectors;
}
#endif

//=========================== prototypes ======================================

void enable_dcdc(void);
//=========================== main ============================================

extern int mote_main(void);

int main(void) {
    return mote_main();
}

//=========================== public ==========================================

void board_init(void) {

#if DIAG_VTOR_IN_RAM
    board_relocateVectors();
#endif
    debugpins_init();
    sctimer_init();
    radio_init();

#if SUPER_LOW_POWER
#else
    leds_init();
    uart_init();
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
