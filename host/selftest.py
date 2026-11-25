r"""Headless self-test for the HDLC / protocol / model layers.

    venv\Scripts\python.exe selftest.py

No serial port is needed: the test only checks framing, parsing and the model.
"""

import struct

import hdlc
import model
import openwsn_proto


def check(name, cond, detail=""):
    print("%-38s %s %s" % (name, "OK" if cond else "FAIL", detail))
    if not cond:
        raise SystemExit("self-test failed: " + name)


def main():
    # -- framing ---------------------------------------------------------
    for payload in (b"S", b"V", b"V\x0b\xb8", bytes(range(0, 40)), b"\x7e\x7d\x7e"):
        frame = hdlc.encode(payload)
        check("roundtrip %r" % payload[:6], hdlc.decode(frame) == payload, frame.hex())

    check("frame starts/ends with 0x7e", hdlc.encode(b"S")[:1] == b"\x7e" and hdlc.encode(b"S")[-1:] == b"\x7e")
    check("echo frame == 7e5366907e", hdlc.encode(b"S").hex() == "7e5366907e", hdlc.encode(b"S").hex())
    check("bare V frame == 7e56cbc77e", hdlc.encode(b"V").hex() == "7e56cbc77e", hdlc.encode(b"V").hex())
    vpay = openwsn_proto.set_voltage_payload(3000)
    check("set-voltage payload is V + big-endian mV", vpay == bytes((0x56, 0x0B, 0xB8)), vpay.hex())
    check("set-voltage frame roundtrips", hdlc.decode(hdlc.encode(vpay)) == vpay,
          hdlc.encode(vpay).hex())

    bad = bytearray(hdlc.encode(b"S"))
    bad[2] ^= 0xFF
    try:
        hdlc.decode(bytes(bad))
        check("bad CRC rejected", False)
    except hdlc.HdlcError as exc:
        check("bad CRC rejected", "CRC" in str(exc), str(exc))

    # -- stream framer ---------------------------------------------------
    framer = hdlc.Framer()
    stream = b"\x00\x11" + hdlc.encode(b"F\x00\x00\x00\x00\x00\x00\x00hi") + hdlc.encode(b"S")
    frames = framer.feed(stream)
    check("framer splits 2 frames", len(frames) == 2, str(len(frames)))
    check("framer payload 1", hdlc.decode(frames[0]) == b"F\x00\x00\x00\x00\x00\x00\x00hi")

    # -- frame parsing ---------------------------------------------------
    payload = b"F\xe4\x00" + bytes(5) + b"TOPO root=d9 node=e4 parent=d9 mv=3000 tf=2\r\n"
    parsed = openwsn_proto.parse_frame(payload)
    check("printf frame type", parsed["type"] == "printf")
    check("printf text", parsed["text"].startswith("TOPO"), parsed["text"].strip())

    events = openwsn_proto.parse_text(parsed["text"])
    check("one topo event", len(events) == 1 and events[0]["kind"] == "topo")
    ev = events[0]
    check("topo fields", (ev["root"], ev["node"], ev["parent"], ev["mv"], ev["tf"]) == (0xD9, 0xE4, 0xD9, 3000, 2))

    check("live topo is not a snapshot", ev["snap"] is False)

    # the root replays its cache with a trailing snap=1 (uinject_dumpTopology)
    snap = openwsn_proto.parse_text(
        "TOPO root=d9 node=cd parent=d9 mv=2200 tf=3 snap=1\r\n")[0]
    check("snapshot flagged", snap["snap"] is True)
    check("snapshot fields", (snap["node"], snap["parent"], snap["mv"]) == (0xCD, 0xD9, 2200))

    ev2 = openwsn_proto.parse_text("EA voltage=2200 mV tf=3 eui=cd")[0]
    check("voltage event", (ev2["mv"], ev2["tf"], ev2["eui"]) == (2200, 3, 0xCD))


    # -- model -----------------------------------------------------------
    m = model.TopologyModel()
    # A live TOPO printf is the text twin of a 'D' frame; the model takes its state
    # from the D frame (that copy carries the uinject counter), so the printf is
    # neither logged nor applied.  A snap=1 replay is printf-only and still used.
    live = m.handle_frame("COM10", parsed)
    check("live TOPO printf ignored", live == [] and 0xE4 not in m.nodes)
    m.apply_event({"kind": "topo", "root": 0xD9, "node": 0xE4, "parent": 0xD9, "mv": 3000, "tf": 2}, "COM10")
    m.apply_event({"kind": "topo", "root": 0xD9, "node": 0xCD, "parent": 0xD9, "mv": 2200, "tf": 3}, "COM10")
    m.apply_event({"kind": "topo", "root": 0xD9, "node": 0x18, "parent": 0xE4, "mv": 4200, "tf": 0}, "COM10")
    check("root recorded", m.root == 0xD9)

    # a snap=1 replay must not look like a fresh report: values update, age does not
    m2 = model.TopologyModel()
    m2.apply_event({"kind": "topo", "root": 0xD9, "node": 0x99, "parent": 0xD9,
                    "mv": 3600, "tf": 1}, "COM10")
    snap_node = m2.nodes[0x99]
    snap_node.last_seen -= 100.0          # pretend the last real report was 100 s ago
    m2.apply_event({"kind": "topo", "root": 0xD9, "node": 0x99, "parent": 0xD9,
                    "mv": 2400, "tf": 2, "snap": True}, "COM10")
    check("snapshot updates mv", (snap_node.mv, snap_node.tf) == (2400, 2))
    check("snapshot keeps age", snap_node.age() > 90.0 and snap_node.packets == 1,
          "age=%.0f packets=%d" % (snap_node.age(), snap_node.packets))
    check("3 nodes + root", len(m.nodes) == 4, str(sorted(hex(k) for k in m.nodes)))
    depth = m.depth_map()
    check("depth root=0", depth[0xD9] == 0)
    check("depth children=1", depth[0xE4] == 1 and depth[0xCD] == 1)
    check("depth grandchild=2", depth[0x18] == 2)
    check("packets counted", m.packets == 3)

    m.apply_event({"kind": "voltage", "mv": 3500, "tf": 1, "eui": 0xE4}, "COM13")
    check("voltage applied", m.nodes[0xE4].mv == 3500)
    check("port binding learned", m.port_eui.get("COM13") == 0xE4)
    check("bound_port lookup", m.bound_port(0xE4) == "COM13")

    # a bare V reply without eui uses the port binding
    m.apply_event({"kind": "voltage", "mv": 1900, "tf": 5, "eui": None}, "COM13")
    check("binding used when eui absent", m.nodes[0xE4].mv == 1900)

    # The RTT link gets the serial byte stream *before* the UART's XON/XOFF
    # escaping (openserial.c outputHdlcByte -> rtt_console.c), so its framer must
    # not undo that layer: 0x11/0x13/0x12 are ordinary payload bytes there.
    raw = b"\x7e\x56\x11\x13\x12\xaa\xbb\x7e"
    check("framer keeps 0x11/0x13/0x12 when xonxoff=False",
          hdlc.Framer(xonxoff=False).feed(raw) == [raw], raw.hex())
    check("framer drops them when xonxoff=True (serial)",
          hdlc.Framer().feed(raw) != [raw])

    # -- 'D' frame carrying the uinject trailer --------------------------
    def d_frame(mv, tf, eui, parent, my=0xE4, seq=1):
        tail = (
            openwsn_proto.UINJECT_MAGIC
            + struct.pack("<H", seq)      # uinject counter, offset 7 (uinject.c:542)
            + bytes(21)
            + struct.pack("<H", mv)
            + bytes((tf, eui, parent))
        )
        assert len(tail) == openwsn_proto.UINJECT_PAYLOAD_LEN
        packet = bytes(range(31)) + tail
        my_eui = bytes((0xFB, 0x72, 0x2F, 0xD5, 0x4C, 0xFA, 0x68, my))
        prev = bytes((0xD5, 0x76, 0xCC, 0x6C, 0xB7, 0xC2, 0x06, eui))
        return b"D" + b"\x68\xe4" + bytes(5) + my_eui + prev + packet

    parsed_d = openwsn_proto.parse_frame(d_frame(3300, 2, 0xD9, 0xE4, seq=10))
    check("data frame type", parsed_d["type"] == "data")
    check("data frame carries uinject", "uinject" in parsed_d)
    u = parsed_d["uinject"]
    check("uinject fields", (u["mv"], u["tf"], u["eui"], u["parent"]) == (3300, 2, 0xD9, 0xE4), str(u))
    check("uinject counter parsed", u["counter"] == 10, str(u["counter"]))
    check("data frame my eui", parsed_d["my_eui64"][7] == 0xE4)
    check("data frame prevhop", parsed_d["prevhop_eui64"][7] == 0xD9)

    m2 = model.TopologyModel()
    m2.handle_frame("COM10", parsed_d)
    check("root from D frame", m2.root == 0xE4)
    check("node from D frame", 0xD9 in m2.nodes and m2.nodes[0xD9].mv == 3300)
    check("node parent from D frame", m2.nodes[0xD9].parent == 0xE4)
    check("depth of D-frame child", m2.depth_map()[0xD9] == 1)
    m2.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(3000, 3, 0xD9, 0xE4, seq=11)))
    check("re-report updates mv", m2.nodes[0xD9].mv == 3000 and m2.nodes[0xD9].packets == 2)
    check("root port bound", m2.port_eui.get("COM10") == 0xE4)
    m2.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(4100, 0, 0xCD, 0xD9, seq=5)))
    check("grandchild depth", m2.depth_map()[0xCD] == 2, str(m2.depth_map()))

    # a snap=1 replay still travels through handle_frame (printf only, no counter)
    m2.apply_event({"kind": "topo", "root": 0xE4, "node": 0xD9, "parent": 0xE4,
                    "mv": 3000, "tf": 3}, "COM10")
    snap_payload = b"F\xe4\x00" + bytes(5) + b"TOPO root=e4 node=d9 parent=e4 mv=2500 tf=1 snap=1\r\n"
    out = m2.handle_frame("COM10", openwsn_proto.parse_frame(snap_payload))
    check("snapshot printf still applied",
          len(out) == 1 and m2.nodes[0xD9].mv == 2500 and m2.nodes[0xD9].tf == 1, str(out))

    # -- late uinject packets ------------------------------------------
    # The counter in the payload is the only ordering information the host has:
    # a packet built before the parent switch can arrive after a newer one and
    # would drag the topology back to the old parent for a moment.
    m3 = model.TopologyModel()
    m3.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(3000, 2, 0xD9, 0xE4, seq=100)))
    check("first report accepted", m3.nodes[0xD9].parent == 0xE4 and m3.nodes[0xD9].seq == 100)
    lines = m3.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(3100, 3, 0xD9, 0xCD, seq=99)))
    check("older packet dropped", m3.nodes[0xD9].parent == 0xE4 and m3.nodes[0xD9].mv == 3000,
          "parent=%s mv=%s" % (m3.nodes[0xD9].parent, m3.nodes[0xD9].mv))
    check("older packet logged", len(lines) == 1 and "late packet" in lines[0], str(lines))
    check("stale counter counted", m3.stale_reports == 1)
    m3.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(3200, 0, 0xD9, 0xCD, seq=100)))
    check("duplicate counter dropped",
          (m3.nodes[0xD9].parent, m3.nodes[0xD9].mv, m3.nodes[0xD9].packets) == (0xE4, 3000, 1),
          "parent=%s mv=%s packets=%d" % (m3.nodes[0xD9].parent, m3.nodes[0xD9].mv,
                                          m3.nodes[0xD9].packets))
    m3.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(3300, 0, 0xD9, 0xCD, seq=101)))
    check("newer packet applied", m3.nodes[0xD9].parent == 0xCD and m3.nodes[0xD9].seq == 101)

    # counter wrap: 0xFFFF -> 0 is +1, not a reboot
    m4 = model.TopologyModel()
    m4.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(3000, 0, 0xD9, 0xE4, seq=0xFFFF)))
    m4.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(3100, 1, 0xD9, 0xCD, seq=0)))
    check("counter wrap accepted", m4.nodes[0xD9].seq == 0 and m4.nodes[0xD9].parent == 0xCD)

    # a rebooted node starts over at a low counter: accept after a quiet spell
    m5 = model.TopologyModel()
    m5.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(3000, 0, 0xD9, 0xE4, seq=600)))
    m5.nodes[0xD9].last_seen -= model.SEQ_QUIET_S + 5
    m5.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(2800, 2, 0xD9, 0xCD, seq=5)))
    check("restart after quiet accepted",
          m5.nodes[0xD9].seq == 5 and m5.nodes[0xD9].parent == 0xCD)

    # a big backwards jump is a restart even inside the quiet window
    m6 = model.TopologyModel()
    m6.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(3000, 0, 0xD9, 0xE4, seq=600)))
    m6.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(2800, 2, 0xD9, 0xCD, seq=1)))
    check("big backwards jump accepted as restart",
          m6.nodes[0xD9].seq == 1 and m6.nodes[0xD9].parent == 0xCD)

    # but a small backwards step inside the quiet window is still a late packet
    m7 = model.TopologyModel()
    m7.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(3000, 0, 0xD9, 0xE4, seq=600)))
    m7.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(2800, 2, 0xD9, 0xCD, seq=595)))
    check("small backwards step dropped", m7.nodes[0xD9].parent == 0xE4 and m7.nodes[0xD9].seq == 600)

    # -- offline eviction ------------------------------------------------
    # A node the root stopped hearing leaves the tree; the root itself does not
    # (it is the board on the serial port and only relays the others' reports).
    m8 = model.TopologyModel()
    m8.apply_event({"kind": "topo", "root": 0xE4, "node": 0xD9, "parent": 0xE4,
                    "mv": 3000, "tf": 0}, "COM10")
    m8.apply_event({"kind": "topo", "root": 0xE4, "node": 0x18, "parent": 0xD9,
                    "mv": 3000, "tf": 0}, "COM10")
    m8.nodes[0xD9].last_seen -= m8.offline_after + 5
    gone = m8.prune_offline()
    check("silent node evicted", [e for e, _ in gone] == [0xD9] and 0xD9 not in m8.nodes, str(gone))
    check("root kept", 0xE4 in m8.nodes and m8.nodes[0xE4].is_root)
    check("orphan detached", m8.nodes[0x18].parent is None)
    m8.apply_event({"kind": "topo", "root": 0xE4, "node": 0xD9, "parent": 0xE4,
                    "mv": 3000, "tf": 0, "snap": True}, "COM10")
    check("snapshot does not resurrect", 0xD9 not in m8.nodes)
    m8.handle_frame("COM10", openwsn_proto.parse_frame(d_frame(2900, 1, 0xD9, 0xE4, seq=7)))
    check("live packet brings it back", 0xD9 in m8.nodes and m8.nodes[0xD9].mv == 2900)
    check("comeback reported", m8.returned == [0xD9], str(m8.returned))

    print("\nall checks passed")
    print("known frames: echo=%s  V=%s  V+3000=%s" % (
        hdlc.encode(b"S").hex(), hdlc.encode(b"V").hex(),
        hdlc.encode(openwsn_proto.set_voltage_payload(3000)).hex()))


if __name__ == "__main__":
    main()
