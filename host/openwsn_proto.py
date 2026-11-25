"""OpenWSN serial protocol: frame types, TOPO line parsing, 'V' command.

Frame layout mote -> PC (code/openwsn-fw/drivers/common/openserial.h):

    'F' <addr16 lo> <addr16 hi> <asn[5]> <text>      printf output
    'D' <addr16 lo> <addr16 hi> <asn[5]> <myEUI64[8]> <prevhopEUI64[8]> <packet>
                                                    data forwarded by openbridge
    'S' <asn[5]> <isSync> <...>                      periodic status

PC -> mote:

    'V' <mV hi> <mV lo>   set the energy-aware working point (clamped by the
                          firmware to [1800, 4200]); a bare 'V' only reports it
    'S' <bytes...>        echo back as a 'D' frame
    'R'                   become DAG root
    'Q'                   reset the board

The DAG root prints one line per uinject packet it receives from a child
(openapps/uinject/uinject.c):

    TOPO root=<eui7> node=<eui7> parent=<eui7> mv=<mV> tf=<throttle>

and answers a voltage command with

    EA voltage=<mV> mV tf=<throttle> eui=<eui7>
"""

import re
import struct

FRAME_MOTE2PC_DATA = ord("D")
FRAME_MOTE2PC_STATUS = ord("S")
FRAME_MOTE2PC_VERBOSE = ord("V")
FRAME_MOTE2PC_INFO = ord("I")
FRAME_MOTE2PC_WARNING = ord("W")
FRAME_MOTE2PC_SUCCESS = ord("U")
FRAME_MOTE2PC_ERROR = ord("E")
FRAME_MOTE2PC_CRITICAL = ord("C")
FRAME_MOTE2PC_SNIFFED_PACKET = ord("P")
FRAME_MOTE2PC_PRINTF = ord("F")

FRAME_PC2MOTE_SETROOT = ord("R")
FRAME_PC2MOTE_RESET = ord("Q")
FRAME_PC2MOTE_DATA = ord("D")
FRAME_PC2MOTE_TRIGGERSERIALECHO = ord("S")
FRAME_PC2MOTE_SETVOLTAGE = ord("V")

TOPO_RE = re.compile(
    r"TOPO\s+root=(?P<root>[0-9a-fA-F]+)\s+node=(?P<node>[0-9a-fA-F]+)"
    r"\s+parent=(?P<parent>[0-9a-fA-F]+)\s+mv=(?P<mv>\d+)\s+tf=(?P<tf>\d+)"
    r"(?P<snap>\s+snap=1)?"     # root-side cache replay (uinject_dumpTopology)
)
EA_VOLTAGE_RE = re.compile(
    r"EA\s+voltage=(?P<mv>\d+)\s+mV\s+tf=(?P<tf>\d+)(?:\s+eui=(?P<eui>[0-9a-fA-F]+))?"
)


# uinject trailer (openapps/uinject/uinject.c): every packet the app injects
# ends with
#     "uinject" <23 bytes of state> <mV lo> <mV hi> <throttle> <eui7> <parent7>
UINJECT_MAGIC = b"uinject"
UINJECT_PAYLOAD_LEN = 35
UINJECT_OFF_COUNTER = 7        # uint16 LE, +1 per built packet (uinject.c:542-544)
UINJECT_OFF_MV = 30
UINJECT_OFF_TF = 32
UINJECT_OFF_NODE = 33
UINJECT_OFF_PARENT = 34
D_FRAME_HEADER = 24            # 'D' + addr16(2) + asn(5) + myEUI(8) + prevhop(8)


def parse_uinject(payload):
    """Return the uinject trailer of a 'D' payload, or None if absent.

    payload is the whole 'D' payload, header included, so the 35-byte trailer
    is simply the tail of it.
    """
    if len(payload) < D_FRAME_HEADER + UINJECT_PAYLOAD_LEN:
        return None
    tail = payload[-UINJECT_PAYLOAD_LEN:]
    if tail[: len(UINJECT_MAGIC)] != UINJECT_MAGIC:
        return None
    return {
        "counter": tail[UINJECT_OFF_COUNTER] | (tail[UINJECT_OFF_COUNTER + 1] << 8),
        "mv": tail[UINJECT_OFF_MV] | (tail[UINJECT_OFF_MV + 1] << 8),
        "tf": tail[UINJECT_OFF_TF],
        "eui": tail[UINJECT_OFF_NODE],
        "parent": tail[UINJECT_OFF_PARENT],
        "prevhop": payload[23],
    }


def parse_frame(payload):
    """payload (bytes, CRC already removed) -> dict."""
    if not payload:
        return {"type": "empty", "raw": b""}
    kind = payload[0]
    if kind == FRAME_MOTE2PC_PRINTF:
        text = payload[8:].decode("utf-8", "replace") if len(payload) >= 8 else ""
        return {
            "type": "printf",
            "addr16": payload[1:3],
            "asn": payload[3:8],
            "text": text,
            "raw": payload,
        }
    if kind == FRAME_MOTE2PC_DATA:
        frame = {
            "type": "data",
            "addr16": payload[1:3],
            "payload": payload[3:],
            "raw": payload,
        }
        # openserial_printData() (drivers/common/openserial.c) writes
        # 'D' + addr16(2) + asn(5) + myEUI64(8) + prevhopEUI64(8) + packet
        if len(payload) >= D_FRAME_HEADER:
            frame["asn"] = payload[3:8]
            frame["my_eui64"] = payload[8:16]
            frame["prevhop_eui64"] = payload[16:24]
            frame["packet"] = payload[24:]
            uinject = parse_uinject(payload)
            if uinject is not None:
                frame["uinject"] = uinject
        return frame
    if kind == FRAME_MOTE2PC_STATUS:
        return {"type": "status", "asn": payload[1:6], "raw": payload}
    return {"type": "other", "kind": chr(kind) if 32 <= kind < 127 else "0x%02x" % kind, "raw": payload}


def parse_text(text):
    """Look for the machine-readable lines the firmware prints.

    Returns a list of dicts: {'kind': 'topo', ...} or {'kind': 'voltage', ...}
    """
    events = []
    for m in TOPO_RE.finditer(text):
        events.append(
            {
                "kind": "topo",
                "root": int(m.group("root"), 16),
                "node": int(m.group("node"), 16),
                "parent": int(m.group("parent"), 16),
                "mv": int(m.group("mv")),
                "tf": int(m.group("tf")),
                # 'snap=1' marks a root-side snapshot ('T' or the 30 s re-dump):
                # it repeats the last known state, so it must not refresh the
                # node's "seconds ago" age.
                "snap": m.group("snap") is not None,
            }
        )
    for m in EA_VOLTAGE_RE.finditer(text):
        events.append(
            {
                "kind": "voltage",
                "mv": int(m.group("mv")),
                "tf": int(m.group("tf")),
                "eui": int(m.group("eui"), 16) if m.group("eui") else None,
            }
        )
    return events


def set_voltage_payload(mv):
    """Build the PC->mote payload for 'V' (3 bytes, big-endian mV)."""
    mv = max(0, min(0xFFFF, int(mv)))
    return bytes((FRAME_PC2MOTE_SETVOLTAGE,)) + struct.pack(">H", mv)


def dump_payload():
    """Ask a DAG root for a snapshot of every child it has heard from ('T')."""
    return bytes([0x54])


# 'W' <eui7> <mV hi> <mV lo>: the DAG root broadcasts "SETVOLT"+eui7+mV over the
# mesh, and the node whose EUI64[7] matches applies it (openapps/uinject).  This
# is the only way to retune a node that has no serial port of its own.
FRAME_PC2MOTE_SETVOLTAGE_REMOTE = ord("W")


def remote_voltage_payload(eui7, mv):
    """Build the PC->root payload for 'W' (4 bytes: eui7 + big-endian mV)."""
    mv = max(0, min(0xFFFF, int(mv)))
    return bytes((FRAME_PC2MOTE_SETVOLTAGE_REMOTE, eui7 & 0xFF)) + struct.pack(">H", mv)


def read_voltage_payload():
    """Bare 'V': the mote answers with its current value."""
    return bytes((FRAME_PC2MOTE_SETVOLTAGE,))


# 'M' <mode> <tmin_s>: how a throttle-factor change is announced.  Mode 0 (the
# default the experiment images boot with) says nothing extra - the rank still
# changes, it just waits for the normal DIO schedule; 1 = immediate burst,
# 2 = one immediate DIO.  tmin_s floors how often a forced announcement may
# happen (0 = no floor).  Volatile: a reset returns to the compiled default.
FRAME_PC2MOTE_EADIOMODE = ord("M")


def dio_mode_payload(mode, tmin_s):
    """Build the PC->mote payload for 'M' (3 bytes)."""
    return bytes((FRAME_PC2MOTE_EADIOMODE, int(mode) & 0xFF, max(0, min(255, int(tmin_s)))))


def read_dio_mode_payload():
    """Bare 'M': the mote answers with its current mode and floor."""
    return bytes((FRAME_PC2MOTE_EADIOMODE,))


def echo_payload(data=b"ping"):
    return bytes((FRAME_PC2MOTE_TRIGGERSERIALECHO,)) + bytes(data)


def fmt_eui(eui7):
    return "%02X" % (eui7 & 0xFF)
