r"""OpenWSN mote<->PC HDLC framing (Python 3).

Wire format (see code/openwsn-fw/drivers/common/openserial.c, inputHdlcOpen /
inputHdlcWrite / inputHdlcClose):

    0x7e  <escaped(payload + CRC16)>  0x7e

    escape:  0x7d 0x5d  -> 0x7d
             0x7d 0x5e  -> 0x7e
    CRC:     CRC-16/X-25 (init 0xffff, reflected poly 0x8408), appended little
             endian, so that CRC over payload+CRC equals 0xf0b8.

The table and the framing rules are the ones used by
E:\Project\6TiSCH\OpenWSN_serial_parser\serial_parser.py (class OpenHdlc),
rewritten for Python 3 bytes objects (the original mixes str and chr()).
"""

HDLC_FLAG = 0x7E
HDLC_ESCAPE = 0x7D
HDLC_FLAG_ESCAPED = 0x5E
HDLC_ESCAPE_ESCAPED = 0x5D
HDLC_CRCINIT = 0xFFFF
HDLC_CRCGOOD = 0xF0B8

# Software flow control (XON/XOFF) escaping, applied by the mote's uart_writeByte
# *before* the bytes reach the wire (bsp/boards/uart.h):
#     0x11 (XON)  -> [0x12, 0x01]
#     0x13 (XOFF) -> [0x12, 0x03]
#     0x12 (ESC)  -> [0x12, 0x02]
# Raw 0x11/0x13 are flow control and must be dropped. This layer sits *below*
# HDLC, so it has to be undone before the 0x7e framing is parsed; without it any
# frame containing 0x11/0x13/0x12 fails its CRC (long frames almost always do).
XON = 0x11
XOFF = 0x13
XONXOFF_ESCAPE = 0x12
XONXOFF_MASK = 0x10

FCS16TAB = (
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
)


class HdlcError(Exception):
    """Raised when a frame cannot be decoded (bad flags or bad CRC)."""


def crc16(data, crc=HDLC_CRCINIT):
    for b in data:
        crc = (crc >> 8) ^ FCS16TAB[(crc ^ b) & 0xFF]
    return crc


def encode(payload):
    """payload (bytes) -> HDLC frame including both 0x7e flags."""
    crc = 0xFFFF - crc16(payload)
    body = bytes(payload) + bytes((crc & 0xFF, (crc >> 8) & 0xFF))
    body = body.replace(b"\x7d", b"\x7d\x5d")
    body = body.replace(b"\x7e", b"\x7d\x5e")
    return b"\x7e" + body + b"\x7e"


def decode(frame):
    """HDLC frame (with flags) -> payload bytes."""
    if len(frame) < 4:
        raise HdlcError("frame too short")
    if frame[0] != HDLC_FLAG or frame[-1] != HDLC_FLAG:
        raise HdlcError("missing flag")
    body = frame[1:-1]
    body = body.replace(b"\x7d\x5e", b"\x7e")
    body = body.replace(b"\x7d\x5d", b"\x7d")
    if len(body) < 3:
        raise HdlcError("payload too short")
    if crc16(body) != HDLC_CRCGOOD:
        raise HdlcError("wrong CRC")
    return body[:-2]


class XonXoffUnescaper(object):
    """Undo the mote's XON/XOFF escaping (bsp/boards/uart.h).

    Mirrors serial_parser.py:311-319: 0x12 introduces the next byte, which has to
    be XORed with XONXOFF_MASK; unescaped 0x11/0x13 are flow control and dropped.
    """

    def __init__(self):
        self._escaping = False

    def feed(self, data):
        out = bytearray()
        for b in data:
            if b == XONXOFF_ESCAPE:
                self._escaping = True
            elif self._escaping:
                out.append(b ^ XONXOFF_MASK)
                self._escaping = False
            elif b != XON and b != XOFF:
                out.append(b)
        return bytes(out)


class Framer(object):
    """Byte-stream to HDLC-frame splitter (mirror of moteProbe.run()).

    xonxoff=True undoes the mote's software-flow-control escaping, which is what
    a UART delivers.  Same bytes, but taken before that escaping (the RTT mirror
    in openserial.c, see host/rtt_link.py), need xonxoff=False: there 0x11/0x13/
    0x12 are ordinary payload bytes.
    """

    def __init__(self, xonxoff=True):
        self._buf = bytearray()
        self._busy = False
        self.frames = 0
        self.errors = 0
        self._unescape = XonXoffUnescaper() if xonxoff else None

    def feed(self, data):
        out = []
        if self._unescape is None:
            stream = data
        else:
            stream = self._unescape.feed(data)
        for b in stream:
            if b == HDLC_FLAG:
                if self._busy and self._buf:
                    out.append(b"\x7e" + bytes(self._buf) + b"\x7e")
                    self.frames += 1
                self._buf = bytearray()
                self._busy = True
            elif self._busy:
                self._buf.append(b)
        return out
