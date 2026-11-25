"""Live check: feed one mote's serial port into TopologyModel for N seconds."""
import sys
import time

import link
import model

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

m = model.TopologyModel()
counters = {"frame": 0, "badframe": 0, "data": 0}


def emit(kind, payload):
    if kind == "frame":
        counters["frame"] += 1
        if payload.get("type") == "data":
            counters["data"] += 1
        for line in m.handle_frame(port, payload):
            if "TOPO" in line:
                print(line)
    elif kind == "badframe":
        counters["badframe"] += 1
    elif kind == "error":
        print("ERR", payload)


lk = link.SerialLink(port, emit=emit)
lk.start()
t0 = time.time()
while time.time() - t0 < secs:
    time.sleep(0.5)
lk.stop()

print("frames=%d data=%d bad=%d" % (counters["frame"], counters["data"], counters["badframe"]))
print("root=%s" % ("0x%02X" % m.root if m.root is not None else None))
depth = m.depth_map()
for n in sorted(m.nodes.values(), key=lambda x: (depth.get(x.eui7, 9), x.eui7)):
    print("  eui=%s depth=%d parent=%s mv=%s tf=%s pkts=%d port=%s" % (
        n.label, depth.get(n.eui7, -1),
        ("0x%02X" % n.parent) if n.parent is not None else "-",
        n.mv, n.tf, n.packets, n.port))
print("ports=%s" % {k: "0x%02X" % v for k, v in m.port_eui.items()})
