/**
 * \brief SEGGER-RTT up-channel that carries this board's serial output.
 *
 * RTT needs no UART and no host-side library: the J-Link reads a control block
 * that lives in the target's RAM over SWD, while the CPU keeps running.  The
 * board without a wired UART (the battery-free mote) therefore still delivers
 * the same byte stream its UART would carry - HDLC frames including the 'D'
 * frames that hold every child's uinject report.
 *
 * The control block is static and pre-initialised, so there is nothing to call
 * at startup; the first write makes it visible to the probe.
 */

#ifndef __RTT_CONSOLE_H
#define __RTT_CONSOLE_H

#include "opendefs.h"

/**
\brief Append one byte to the RTT up-buffer (drops it when the host is not
       reading and the ring is full, which is SEGGER's Flags=0 behaviour).
*/
void rtt_console_writeByte(uint8_t byte);

/**
\brief Append a string.
*/
void rtt_console_write(const char *data, unsigned len);

#endif /* __RTT_CONSOLE_H */
