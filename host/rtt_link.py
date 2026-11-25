"""RTT link: a mote's serial byte stream, read through its J-Link probe.

The battery-free board has no wired UART, but its firmware mirrors every byte the
serial port would carry into a SEGGER-RTT ring in RAM
(bsp/boards/nrf52840_dk/rtt_console.c, called from openserial.c).  The J-Link
reads that ring over SWD while the CPU keeps running, so a host attached to the
board's probe sees exactly what it would see on a COM port: the same HDLC frames,
including the 'D' frames that hold every child's uinject report and the 'F'
frames that carry the TOPO lines.  Nothing above this layer can tell the
difference from a serial link.

This module runs SEGGER's JLinkRTTLogger for one probe serial number and tails the
file it writes.  Two details matter:
  * '-USB <SN>': the logger is told which probe to use, so it never shows the
    J-Link selection dialog;
  * the mirrored stream is taken *before* the UART's XON/XOFF escaping, so the
    framer must not undo that layer (hdlc.Framer(xonxoff=False)).

The link emits the same ('frame', parsed) and ('badframe', err) events a
SerialLink emits, so model.handle_frame() and the topology view need no
special case.
"""

import atexit
import os
import re
import subprocess
import tempfile
import threading
import time

import hdlc
import openwsn_proto

JLINK_DIR = os.environ.get("JLINK_DIR", r"D:\SEGGER\JLink_V810")
JLINK_EXE = os.path.join(JLINK_DIR, "JLink.exe")
RTT_LOGGER = os.path.join(JLINK_DIR, "JLinkRTTLogger.exe")

DEVICE = "nRF52840_xxAA"
SPEED_KHZ = 4000
# nRF52840 has 256 KB of RAM at 0x20000000; searching it explicitly beats
# relying on whatever the probe's device database thinks the target is.
RTT_SEARCH = "0x20000000 0x00040000"
CHANNEL = "0"

POLL_S = 0.2
RETRY_S = 5.0
CREATE_NO_WINDOW = 0x08000000

EMU_RE = re.compile(
    r"J-Link\[\d+\]:\s*Connection:\s*USB,\s*Serial number:\s*(?P<sn>\d+)"
    r",\s*ProductName:\s*(?P<product>[^,\r\n]+)"
)

_probe_cache = (0.0, [])

# JLinkRTTLogger holds the probe exclusively, so a logger that outlives this
# process makes every later scan/flash fail with "emulator in use".  Keep track
# of the ones we spawned and take them down when the host exits (a thread stuck
# in Popen() at exit would otherwise leave its child behind).
_live_procs = set()
_live_lock = threading.Lock()


def _kill_live_procs():
    with _live_lock:
        procs = list(_live_procs)
        _live_procs.clear()
    for proc in procs:
        try:
            if proc.poll() is None:
                proc.kill()
        except Exception:  # noqa: BLE001
            pass


atexit.register(_kill_live_procs)


def list_rtt_probes(max_age=10.0):
    """[(serial, product), ...] for every J-Link plugged into this PC.

    'ShowEmuList' from a command file enumerates the probes without ever
    showing the selection dialog the DLL API pops up when several are attached.
    """
    global _probe_cache
    stamp, cached = _probe_cache
    if time.time() - stamp < max_age:
        return list(cached)
    found = []
    cmd_file = None
    try:
        fd, cmd_file = tempfile.mkstemp(prefix="wsn_emu_", suffix=".txt")
        with os.fdopen(fd, "w", encoding="ascii") as fh:
            fh.write("ShowEmuList\nqc\n")
        out = subprocess.run(
            [JLINK_EXE, "-NoGui", "1", "-CommandFile", cmd_file],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=30, creationflags=CREATE_NO_WINDOW if os.name == "nt" else 0,
        ).stdout.decode("utf-8", "replace")
        for m in EMU_RE.finditer(out):
            found.append((m.group("sn"), m.group("product").strip()))
    except Exception:  # noqa: BLE001 - a probe scan must never break the UI
        found = list(cached)
    finally:
        if cmd_file:
            try:
                os.unlink(cmd_file)
            except OSError:
                pass
    _probe_cache = (time.time(), found)
    return list(found)


def explain(message):
    """Turn the logger's one-line error into something actionable."""
    low = message.lower()
    if "could not connect to target" in low:
        return "连不上目标板（没上电 / SWD 没接好 / 探针选错板子）"
    if "could not find rtt control block" in low:
        return "目标在跑，但 RAM 里没有 RTT 控制块（固件没带 rtt_console.c？）"
    if "in use" in low or "could not be opened" in low:
        return "探针被占用（先关掉烧录脚本或另一个 RTT/GDB 会话）"
    return message


class RttLink(object):
    """One probe's RTT up-channel, decoded into the same events as a serial port.

    Mirrors the SerialLink interface the GUI uses (is_open, rx_bytes, rx_frames,
    last_error, start/stop/send) plus 'is_rtt', which the GUI uses to skip the
    commands that only exist on a serial port.
    """

    is_rtt = True

    def __init__(self, serial_no, emit=None, device=DEVICE, speed=SPEED_KHZ,
                 search_ranges=RTT_SEARCH):
        self.serial = str(serial_no)
        self.port = "RTT:" + self.serial
        self.device = device
        self.speed = speed
        self.search_ranges = search_ranges
        self._emit = emit or (lambda kind, payload: None)
        self._thread = None
        self._proc = None
        self._stop = threading.Event()
        self._log_path = None
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
        self._thread = threading.Thread(target=self._run, name="rtt-" + self.serial,
                                        daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        proc = self._proc
        if proc is not None and proc.poll() is None:
            try:
                proc.kill()
            except Exception:  # noqa: BLE001
                pass
        t = self._thread
        if t is not None:
            t.join(timeout=3.0)
        self._thread = None
        self.is_open = False

    # -- io ----------------------------------------------------------------
    def send(self, payload):
        """Uplink only: the firmware does not read RTT down-channel 0."""
        self.last_error = "RTT 链路只读（板上行日志），命令请走 root 的 'W'"
        return False

    # -- internals ---------------------------------------------------------
    def _run(self):
        """Keep one logger per probe alive: a flash, a cable wiggle or a probe
        that was busy restarts the link by itself, like the serial link does."""
        while not self._stop.is_set():
            self._attempt()
            if self._stop.is_set():
                return
            self.is_open = False
            self._emit("close", self.port)
            self._stop.wait(RETRY_S)

    def _attempt(self):
        fd, path = tempfile.mkstemp(prefix="wsn_rtt_", suffix=".log")
        os.close(fd)
        args = [
            RTT_LOGGER,
            "-Device", self.device,
            "-If", "SWD",
            "-Speed", str(self.speed),
            "-USB", self.serial,
            "-RTTSearchRanges", self.search_ranges,
            "-RTTChannel", CHANNEL,
            path,
        ]
        try:
            self._proc = subprocess.Popen(
                args,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                creationflags=CREATE_NO_WINDOW if os.name == "nt" else 0,
            )
        except Exception as exc:  # noqa: BLE001
            self.last_error = "启动 JLinkRTTLogger 失败: %s" % exc
            self._emit("error", self.last_error)
            _unlink(path)
            return
        with _live_lock:
            _live_procs.add(self._proc)
        threading.Thread(target=self._read_status, daemon=True).start()
        try:
            self._tail(path)
        finally:
            self.is_open = False
            proc, self._proc = self._proc, None
            with _live_lock:
                _live_procs.discard(proc)
            if proc is not None and proc.poll() is None:
                try:
                    proc.kill()
                except Exception:  # noqa: BLE001
                    pass
            _unlink(path)

    def _read_status(self):
        """Follow the logger's stdout: it says whether the control block was found."""
        proc = self._proc
        if proc is None or proc.stdout is None:
            return
        notified = False
        last_line = ""
        err_line = ""
        for raw in iter(proc.stdout.readline, b""):
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue
            last_line = line
            if not err_line and ("ERROR" in line or "Could not" in line or "Failed" in line):
                err_line = line
            if "Control Block...OK" in line:
                self.is_open = True
                self.last_error = ""
                if not notified:
                    notified = True
                    self._emit("open", self.port)
            elif "Could not find" in line:
                self.last_error = "RTT 控制块没找到（固件没编 rtt_console.c？板子没在跑？）"
                self._emit("error", self.last_error)
            elif line.startswith("S/N:"):
                continue
            if self._stop.is_set():
                return
        if not notified and not self._stop.is_set():
            self.last_error = "JLinkRTTLogger 退出: %s" % explain(err_line or last_line or "没有输出")
            self._emit("error", self.last_error)

    def _tail(self, path):
        framer = hdlc.Framer(xonxoff=False)
        offset = 0
        while not self._stop.is_set():
            chunk = b""
            try:
                with open(path, "rb") as fh:
                    fh.seek(offset)
                    chunk = fh.read()
                    offset = fh.tell()
            except FileNotFoundError:
                pass
            except OSError as exc:
                self.last_error = str(exc)
                self._emit("error", str(exc))
            if chunk:
                self.rx_bytes += len(chunk)
                for frame in framer.feed(chunk):
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
            proc = self._proc
            if proc is not None and proc.poll() is not None:
                # _run() reports the close and restarts the logger
                self.last_error = self.last_error or "JLinkRTTLogger 已退出"
                return
            self._stop.wait(POLL_S)


def _unlink(path):
    try:
        os.unlink(path)
    except OSError:
        pass
