"""Probe a serial port: read raw bytes, then send 'V' and 'S', report what comes back."""
import sys
import time

import serial

import hdlc
import openwsn_proto

for port in sys.argv[1:]:
    print("=== %s" % port)
    try:
        s = serial.Serial(port, 115200, timeout=0.2)
    except Exception as exc:
        print("  open failed: %s" % exc)
        continue
    try:
        time.sleep(0.3)
        s.reset_input_buffer()
        quiet = s.read(2000)
        print("  idle bytes: %d %s" % (len(quiet), quiet[:24].hex()))
        for name, payload in (("V", openwsn_proto.read_voltage_payload()),
                              ("S", openwsn_proto.echo_payload(b"probe"))):
            s.write(hdlc.encode(payload))
            s.flush()
            time.sleep(1.2)
            data = s.read(4000)
            print("  after %s: %d bytes %s" % (name, len(data), data[:32].hex()))
            if b"EA voltage" in data:
                i = data.find(b"EA voltage")
                print("    -> %r" % data[i:i + 40])
    finally:
        s.close()
