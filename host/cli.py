r"""Command-line version of the monitor: useful to check wiring without the GUI.

    python cli.py --port COM10 --read
    python cli.py --port COM10 --port COM11 --set 3000
    python cli.py --port COM10 --set 3000 --remote D9   (root relays it, no cable)
    python cli.py --port COM10 --listen 30

Every received 'F' (printf) frame is printed as  HH:MM:SS PORT | text, and
every 'D' frame that carries a uinject trailer as
HH:MM:SS PORT | uinject node=.. parent=.. mv=.. tf=..
TOPO / EA voltage lines are additionally summarised at the end.
"""

import argparse
import queue
import sys
import time

import link
import openwsn_proto


def main(argv=None):
    ap = argparse.ArgumentParser(description="OpenWSN serial monitor / voltage setter")
    ap.add_argument("--port", action="append", required=True, help="serial port, repeatable")
    ap.add_argument("--set", dest="mv", type=int, help="send 'V' <mv> to every port")
    ap.add_argument("--remote", metavar="EUI7",
                    help="send 'W' <eui7> <mv> to the root, which relays it over the mesh "
                         "(use with --set; needs no cable to the target node)")
    ap.add_argument("--read", action="store_true", help="send a bare 'V' (report current value)")
    ap.add_argument("--listen", type=float, default=3.0, help="seconds to listen (default 3)")
    ap.add_argument("--quiet", action="store_true", help="only print TOPO / EA lines")
    args = ap.parse_args(argv)

    q = queue.Queue()
    links = {}
    for port in args.port:
        lk = link.SerialLink(port, 115200, emit=lambda k, p, pt=port: q.put((pt, k, p)))
        links[port] = lk
        lk.start()

    time.sleep(0.6)  # let the ports open
    if args.remote is not None:
        eui = int(args.remote, 16)
        if args.mv is None:
            print("--remote needs --set <mV>")
            return 2
        payload = openwsn_proto.remote_voltage_payload(eui, args.mv)
        for port, lk in links.items():
            ok = lk.send(payload)
            print("%s  TX  W eui=%02X %d mV  %s" % (port, eui, args.mv, "ok" if ok else "FAILED"))
    elif args.mv is not None:
        payload = openwsn_proto.set_voltage_payload(args.mv)
        for port, lk in links.items():
            print("%s  TX  V %d mV  %s" % (port, args.mv, "ok" if lk.send(payload) else "FAILED"))
    if args.read or (args.mv is None and args.remote is None):
        for port, lk in links.items():
            lk.send(openwsn_proto.read_voltage_payload())

    deadline = time.time() + args.listen
    seen = []
    try:
        while time.time() < deadline:
            try:
                port, kind, payload = q.get(timeout=0.2)
            except queue.Empty:
                continue
            if kind == "frame":
                if payload.get("type") == "printf":
                    text = payload.get("text", "").rstrip()
                    events = openwsn_proto.parse_text(text)
                    if events or not args.quiet:
                        stamp = time.strftime("%H:%M:%S")
                        print("%s %-6s | %s" % (stamp, port, text))
                    seen.extend(events)
                elif payload.get("type") == "data":
                    u = payload.get("uinject")
                    stamp = time.strftime("%H:%M:%S")
                    if u:
                        # the root forwards every uinject packet a child sent
                        print("%s %-6s | uinject node=%02X parent=%02X mv=%d tf=%d"
                              % (stamp, port, u["eui"], u["parent"], u["mv"], u["tf"]))
                    elif not args.quiet:
                        print("%s %-6s | data %s"
                              % (stamp, port, payload.get("payload", b"")[:24].hex()))
            elif kind in ("error", "badframe"):
                if not args.quiet:
                    print("        %-6s | %s: %s" % (port, kind, payload))
    except KeyboardInterrupt:
        pass
    finally:
        for lk in links.values():
            lk.stop()

    if seen:
        print("\nsummary:")
        for ev in seen:
            if ev["kind"] == "topo":
                print("  TOPO  root=%02X node=%02X parent=%02X mv=%d tf=%d" % (
                    ev["root"], ev["node"], ev["parent"], ev["mv"], ev["tf"]))
            else:
                print("  EA    mv=%d tf=%d eui=%s" % (
                    ev["mv"], ev["tf"], "-" if ev["eui"] is None else "%02X" % ev["eui"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
