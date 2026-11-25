import sys, time, serial
sys.path.insert(0, r"E:\Project\6TiSCH\battery-free\host")
import hdlc
port = sys.argv[1]; n = int(sys.argv[2])
s = serial.Serial(port, 115200, timeout=0.2)
s.dtr = True; s.rts = True
time.sleep(0.3)
s.reset_input_buffer()
payload = bytes([0x53]) + bytes((i*3+1) & 0xff for i in range(n))
s.write(hdlc.encode(payload)); s.flush()
fr = hdlc.Framer()
end = time.time() + 4
got = 0; ok = 0; bad = 0
while time.time() < end:
    b = s.read(4096)
    if not b: continue
    for frame in fr.feed(b):
        if len(frame) > 2 and frame[1] == 0x44:
            got += 1
            try:
                p = hdlc.decode(frame)
                ok += 1
                echo = p[1+7:]
                print("n=%d OK framelen=%d echolen=%d match=%s" % (n, len(frame), len(echo), echo == payload[1:]))
            except hdlc.HdlcError:
                bad += 1
                print("n=%d BAD framelen=%d" % (n, len(frame)))
print("n=%d sent=%d frames=%d ok=%d bad=%d" % (n, len(payload), got, ok, bad))
