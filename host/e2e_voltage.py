"""End-to-end: set a leaf's voltage over its own serial port, watch the root's D frames."""
import sys
import time

import link
import model
import openwsn_proto

leaf_port = sys.argv[1] if len(sys.argv) > 1 else "COM11"
root_port = sys.argv[2] if len(sys.argv) > 2 else "COM10"
mv = int(sys.argv[3]) if len(sys.argv) > 3 else 3000

seen = {}


def root_emit(kind, payload):
    if kind == "frame" and payload.get("type") == "data":
        u = payload.get("uinject")
        if u:
            seen.setdefault(u["eui"], []).append((time.time(), u["mv"], u["tf"]))
    elif kind == "frame" and payload.get("type") == "printf":
        text = payload.get("text", "")
        if "EA voltage" in text:
            print("  [leaf serial] %s" % text.strip())


def leaf_emit(kind, payload):
    if kind == "frame" and payload.get("type") == "printf":
        text = payload.get("text", "")
        if "EA voltage" in text:
            print("  [leaf serial] %s" % text.strip())


root = link.SerialLink(root_port, emit=root_emit)
leaf = link.SerialLink(leaf_port, emit=leaf_emit)
root.start()
leaf.start()
time.sleep(1.0)

print("before: set %s -> %d mV" % (leaf_port, mv))
print("  write ok:", leaf.send(openwsn_proto.set_voltage_payload(mv)))
time.sleep(2.0)
print("  now back to 4200")
print("  write ok:", leaf.send(openwsn_proto.set_voltage_payload(4200)))
time.sleep(6.0)
leaf.stop()
root.stop()

for eui, samples in sorted(seen.items()):
    mvs = [s[1] for s in samples]
    print("node %02X: %d reports, mv=%s tf=%s" % (eui, len(samples), sorted(set(mvs)), sorted(set(s[2] for s in samples))))
