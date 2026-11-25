/**
 * \brief Minimal SEGGER-RTT up-channel writer (nRF52840 board).
 *
 * The structure below is the one the J-Link firmware expects to find in RAM
 * (SEGGER_RTT_CB): a 16-byte ID string, the number of up/down buffers, and the
 * buffer descriptors.  We implement only what a uni-directional log needs:
 * one up-channel named "Terminal", no locking (single producer: the console
 * mirror in openserial.c, always called with interrupts disabled).
 *
 * The arrays are sized like SEGGER's defaults (3 + 3 descriptors) even though
 * only entry 0 is used: a host that walks the array by index stays inside our
 * memory instead of reading whatever follows.
 */

#include "rtt_console.h"

//=========================== defines =========================================

#define RTT_MAX_UP_BUFFERS    3
#define RTT_MAX_DOWN_BUFFERS  3
#define RTT_UP_BUFFER_SIZE    1024
#define RTT_DOWN_BUFFER_SIZE  16

//=========================== typedefs =======================================

typedef struct {
    const char *sName;
    char       *pBuffer;
    uint32_t    SizeOfBuffer;
    uint32_t    WrOff;
    uint32_t    RdOff;
    uint32_t    Flags;      ///< 0 = no blocking: a full buffer drops data
} rtt_buffer_t;

typedef struct {
    char         acID[16];          ///< "SEGGER RTT"
    int32_t      MaxNumUpBuffers;
    int32_t      MaxNumDownBuffers;
    rtt_buffer_t aUp[RTT_MAX_UP_BUFFERS];
    rtt_buffer_t aDown[RTT_MAX_DOWN_BUFFERS];
} rtt_cb_t;

//=========================== variables =======================================

static char rtt_upBuffer[RTT_UP_BUFFER_SIZE];
static char rtt_downBuffer[RTT_DOWN_BUFFER_SIZE];
static char rtt_channelName[] = "Terminal";

// 16-byte aligned: the probe searches RAM for the ID string on word boundaries
static rtt_cb_t rtt_cb __attribute__((aligned(16))) = {
    .acID              = "SEGGER RTT",
    .MaxNumUpBuffers   = 1,
    .MaxNumDownBuffers = 1,
    .aUp = {
        {
            .sName        = rtt_channelName,
            .pBuffer      = rtt_upBuffer,
            .SizeOfBuffer = RTT_UP_BUFFER_SIZE,
            .WrOff        = 0,
            .RdOff        = 0,
            .Flags        = 0,
        },
    },
    .aDown = {
        {
            .sName        = rtt_channelName,
            .pBuffer      = rtt_downBuffer,
            .SizeOfBuffer = RTT_DOWN_BUFFER_SIZE,
            .WrOff        = 0,
            .RdOff        = 0,
            .Flags        = 0,
        },
    },
};

//=========================== public ==========================================

void rtt_console_writeByte(uint8_t byte) {
    rtt_buffer_t *b = &rtt_cb.aUp[0];
    uint32_t      wr = b->WrOff;
    uint32_t      next = wr + 1;

    if (next == b->SizeOfBuffer) {
        next = 0;
    }
    if (next == b->RdOff) {
        // ring full and the host is not reading: drop the byte
        return;
    }
    b->pBuffer[wr] = (char)byte;
    b->WrOff = next;
}

void rtt_console_write(const char *data, unsigned len) {
    while (len--) {
        rtt_console_writeByte((uint8_t)*data++);
    }
}
