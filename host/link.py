"""One serial link to a mote, running HDLC framing on a background thread."""

import re
import threading
import time

import serial
from serial.tools import list_ports

import hdlc
import openwsn_proto


def list_serial_ports():
    """[(device, description, hwid), ...] sorted by COM number."""
    ports = []
    for p in list_ports.comports():
        ports.append((p.device, p.description or "", p.hwid or ""))
    ports.sort(key=lambda t: int("".join(c for c in t[0] if c.isdigit()) or 0))
    return ports


def probe_serial(port):
    """J-Link serial number behind a CDC port, or None.

    SEGGER's CDC ports carry the probe's serial in the USB descriptor, e.g.
    'USB VID:PID=1366:1061 SER=001050208732' -> '1050208732'.  That is what lets
    the GUI recognise a COM port and its probe's RTT ring as one board.
    """
    for device, _desc, hwid in list_serial_ports():
        if device != port:
            continue
        hwid = hwid or ""
        if "VID:PID=1366:" not in hwid:
            return None
        m = re.search(r"SER=0*(\d+)", hwid)
        return m.group(1) if m else None
    return None


class SerialLink(object):
    """Reads a mote's serial port, emits decoded frames through emit(kind, payload).

    Kinds:
        open / close      port opened or lost
        error             open/read error (the link keeps retrying)
        frame             decoded frame dict (openwsn_proto.parse_frame)
        badframe          HDLC/CRC error
        stats             (bytes, frames) counters
    """

    def __init__(self, port, baudrate=115200, emit=None):
        self.port = port
        self.baudrate = baudrate
        self._emit = emit or (lambda kind, payload: None)
        self._thread = None
        self._stop = threading.Event()
        self._ser = None
        self._write_lock = threading.Lock()
        self.is_open = False
        self.rx_bytes = 0
        self.rx_frames = 0
        self.tx_frames = 0
        self.last_error = ""

    # -- lifecycle ---------------------------------------------------------
    def start(self):
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="link-" + self.port, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout=2.0)
        self._thread = None
        self._close_serial()

    # -- io ----------------------------------------------------------------
    def send(self, payload):
        """HDLC-wrap payload and write it; returns True on success."""
        ser = self._ser
        if ser is None:
            self.last_error = "port not open"
            return False
        frame = hdlc.encode(payload)
        try:
            with self._write_lock:
                ser.write(frame)
                ser.flush()
            self.tx_frames += 1
            return True
        except Exception as exc:  # noqa: BLE001 - surfaced to the UI
            self.last_error = str(exc)
            self._emit("error", str(exc))
            return False

    def _close_serial(self):
        ser = self._ser
        self._ser = None
        self.is_open = False
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    def _run(self):
        framer = hdlc.Framer()
        while not self._stop.is_set():
            try:
                ser = serial.Serial(self.port, self.baudrate, timeout=0.2)
            except Exception as exc:  # noqa: BLE001
                self.last_error = str(exc)
                self._emit("error", str(exc))
                self._stop.wait(1.0)
                continue
            self._ser = ser
            self.is_open = True
            self.last_error = ""
            self._emit("open", self.port)
            while not self._stop.is_set():
                try:
                    data = ser.read(512)
                except Exception as exc:  # noqa: BLE001
                    self.last_error = str(exc)
                    self._emit("error", str(exc))
                    break
                if not data:
                    continue
                self.rx_bytes += len(data)
                for frame in framer.feed(data):
                    try:
                        payload = hdlc.decode(frame)
                    except hdlc.HdlcError as exc:
                        self._emit("badframe", str(exc))
                        continue
                    self.rx_frames += 1
                    try:
                        parsed = openwsn_proto.parse_frame(payload)
                    except Exception as exc:  # noqa: BLE001
                        self._emit("badframe", "parse: %s" % exc)
                        continue
                    self._emit("frame", parsed)
                self._emit("stats", (self.rx_bytes, self.rx_frames))
            self._close_serial()
            self._emit("close", self.port)
