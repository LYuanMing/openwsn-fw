"""Read/set the voltage of one mote, directly or through the DAG root.

  python set_voltage.py COM13 2200            # 'V' on that node's own UART
  python set_voltage.py COM13                 # read it back
  python set_voltage.py COM10 --eui D9 3000   # 'W' to the root, relayed over the mesh

The --eui form is the deployment case: only the root is plugged in, and it
broadcasts "SETVOLT"+eui7+mV to ff02::1; the node whose EUI64[7] matches
applies it (openapps/uinject/uinject.c) and reports the new value in its next
uinject, which the root prints as a TOPO line.
"""
import sys
import time

import link
import openwsn_proto

port = "COM10"
eui = None
mvs = []
args = list(sys.argv[1:])
for i, a in enumerate(args):
    if a == "--eui" and i + 1 < len(args):
        eui = int(args[i + 1], 16)
        args[i + 1] = None
    elif a == "--eui":
        pass
args = [a for a in args if a is not None and a != "--eui"]
for a in args:
    if mvs == [] and not a.isdigit():
        port = a
    else:
        mvs.append(int(a))
mvs = mvs or [None]

replies = []


def emit(kind, payload):
    if kind == "frame" and payload.get("type") == "printf":
        text = payload.get("text", "").strip()
        if "EA voltage" in text or "SETVOLT" in text or "TOPO" in text:
            replies.append(text)
            print("  <-", text)
    elif kind == "frame" and payload.get("type") == "data" and "uinject" in payload:
        u = payload["uinject"]
        print("  <- uinject node=%02X parent=%02X mv=%d tf=%d"
              % (u["eui"], u["parent"], u["mv"], u["tf"]))


lk = link.SerialLink(port, emit=emit)
lk.start()
time.sleep(1.0)
for mv in mvs:
    if eui is not None:
        if mv is None:
            print("%s: ask root for a snapshot ('T')" % port)
            lk.send(openwsn_proto.dump_payload())
        else:
            print("%s: W eui=%02X %d mV (relayed by the root)" % (port, eui, mv))
            lk.send(openwsn_proto.remote_voltage_payload(eui, mv))
    elif mv is None:
        print("%s: read" % port)
        lk.send(openwsn_proto.read_voltage_payload())
    else:
        print("%s: set %d mV" % (port, mv))
        lk.send(openwsn_proto.set_voltage_payload(mv))
    time.sleep(1.5)
# leave the link open a little longer so the relayed reply can come back
time.sleep(6.0 if eui is not None else 0.5)
lk.stop()
print("replies=%d" % len(replies))
