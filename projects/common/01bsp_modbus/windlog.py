#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
windlog.py - continuous wind-sensor logger for the 01bsp_modbus firmware.

The nRF52840-DK prints one text block per wind frame on its VCOM port:

    ===== RS-CFSFX-N01-3D wind @12345 ms | seq=61 age=0ms  =====
    vec  : 0.45 m/s  123.4 deg  2.5 deg
    sca  : 0.51 m/s  120.0 deg  2.1 deg
    env  : level 1 | 26.20 C | 347.92 m/s
    rt   : 0.44 m/s  121.1 deg  2.4 deg
    xyz  : X 0.10  Y -0.36  Z -0.22 m/s
    raw32: 00 2D 04 D2 ... (32 bytes)
    [STAT]  62 s | frame=309 timeout=0 badframe=2 retry=1 ...

This script reads that stream and appends one CSV row per wind frame, flushing as
it goes, so an interrupted or multi-day run still leaves a complete file. It also

  * re-decodes every field from raw32 and compares it with the printed value
    (a scaling / byte-order / offset bug shows up as a mismatch count),
  * watches the link counters and dumps every bad frame,
  * reopens the port by itself if the board is unplugged and replugged.

Usage
    python windlog.py                              COM34, run until Ctrl-C
    python windlog.py COM34 --out wind_0915.csv    own file (starts a new one)
    python windlog.py COM34 --duration 300         stop after 300s
    python windlog.py --replay cap.txt             parse an existing capture
    python windlog.py --list                       enumerate serial ports

Files next to --out:  <out>             the CSV (appended if it already exists)
                      <out>.raw.txt     every line as it arrived
                      <out>.summary.txt end-of-run summary

The CSV header is Chinese by default (--header zh) and such a file is written as
UTF-8-with-BOM, because Excel on a zh-CN system otherwise reads a CSV as GBK and
shows mojibake. --header en writes the ASCII labels and no BOM. Either label set
is accepted when appending, and an existing file keeps the labels it already has.

Console output is ASCII-only on purpose: this machine's console codepage is GBK.
"""

import argparse
import csv
import os
import re
import sys
import time

DEFAULT_PORT = "COM34"
DEFAULT_BAUD = 115200
RECONNECT_DELAY_S = 2.0
MAX_TAIL = 2000            # records kept in RAM for the detailed summary
HEARTBEAT_S = 60.0

# --------------------------------------------------------------------------
# register map: raw32[i] is register (i/2), high byte first. Offsets and
# signedness follow renke_wind_sensor.c and the datasheet type column:
#   0x0000-0x0006 U16, 0x0007 S16, 0x0008 U16, 0x000A-0x000C U16,
#   0x000D-0x000F S16, 0x0009 = datasheet hole (not decoded).
#   (offset, column, scale, signed)
# --------------------------------------------------------------------------
LAYOUT = [
    (0,  "vec_speed", 100.0, False),   # 0x0000 vector avg wind speed
    (2,  "vec_hor",    10.0, False),   # 0x0001 vector avg horizontal dir
    (4,  "vec_ver",    10.0, False),   # 0x0002 vector avg vertical dir
    (6,  "sca_speed", 100.0, False),   # 0x0003 scalar avg wind speed
    (8,  "sca_hor",    10.0, False),   # 0x0004 scalar avg horizontal dir
    (10, "sca_ver",    10.0, False),   # 0x0005 scalar avg vertical dir
    (12, "level",       1.0, False),   # 0x0006 wind level
    (14, "temp_c",     10.0, True),    # 0x0007 acoustic temperature
    (16, "sound_ms",  100.0, False),   # 0x0008 sound speed
    (20, "rt_speed",  100.0, False),   # 0x000A realtime wind speed
    (22, "rt_hor",     10.0, False),   # 0x000B realtime horizontal dir
    (24, "rt_ver",     10.0, False),   # 0x000C realtime vertical dir
    (26, "x",         100.0, True),    # 0x000D X axis
    (28, "y",         100.0, True),    # 0x000E Y axis
    (30, "z",         100.0, True),    # 0x000F Z axis
]
VALUE_COLS = [c for _, c, _, _ in LAYOUT]
COLUMNS = (["host_time", "host_epoch", "boot_ms", "seq", "age_ms", "stale"]
           + VALUE_COLS + ["raw32"])

# Chinese column labels, used by default (--header zh). Unit suffixes live in the
# label; no commas inside, so csv does not need to quote anything. A new Chinese
# CSV is written as UTF-8-with-BOM so Excel on a zh-CN system does not read it as
# GBK mojibake; --header en keeps the plain ASCII labels and no BOM.
COLUMNS_ZH = ["主机时间", "主机时间戳", "板端毫秒", "帧序号", "数据龄期ms", "是否陈旧",
              "矢量平均风速(m/s)", "矢量平均水平风向(deg)", "矢量平均垂直风向(deg)",
              "标量平均风速(m/s)", "标量平均水平风向(deg)", "标量平均垂直风向(deg)",
              "风级", "声温(C)", "声速(m/s)",
              "实时风速(m/s)", "实时水平风向(deg)", "实时垂直风向(deg)",
              "X轴风速(m/s)", "Y轴风速(m/s)", "Z轴风速(m/s)", "原始32字节"]
HEADER_SETS = {"zh": COLUMNS_ZH, "en": COLUMNS}
assert len(COLUMNS_ZH) == len(COLUMNS) == len(set(COLUMNS_ZH)) == len(set(COLUMNS))

# --------------------------------------------------------------------------
RE_HEAD = re.compile(r"wind @(\d+) ms(?:\s*\|\s*seq=(\d+) age=(\d+)ms(\s+<<STALE>>)?)?")
RE_VEC = re.compile(r"^vec\s+:\s+(-?[\d.]+) m/s\s+(-?[\d.]+) deg\s+(-?[\d.]+) deg")
RE_SCA = re.compile(r"^sca\s+:\s+(-?[\d.]+) m/s\s+(-?[\d.]+) deg\s+(-?[\d.]+) deg")
RE_ENV = re.compile(r"^env\s+:\s+level (\d+) \| (-?[\d.]+) C \| (-?[\d.]+) m/s")
RE_RT = re.compile(r"^rt\s+:\s+(-?[\d.]+) m/s\s+(-?[\d.]+) deg\s+(-?[\d.]+) deg")
RE_XYZ = re.compile(r"^xyz\s+:\s+X (-?[\d.]+)\s+Y (-?[\d.]+)\s+Z (-?[\d.]+) m/s")
RE_RAW = re.compile(r"^raw32:(.*)$")
# [STAT] is parsed as generic key=value pairs so captures from older firmware
# revisions (no retry/txtmo/ovf/rej/drop) still parse.
RE_STAT = re.compile(r"\[STAT\]\s*\d+ s \|(.*)$")
RE_KV = re.compile(r"([A-Za-z_]+)=(\d+)")
RE_BAD = re.compile(r"\[BAD \](.*?)n=(\d+) maxgap=(\d+)ms(?:\s+exc=0x([0-9A-Fa-f]+))?")
RE_TS_PREFIX = re.compile(r"^\[\s*[\d.]+\]\s")      # windlog's own raw-log prefix

STAT_NAMES = ["frame", "timeout", "badframe", "retry", "txtmo", "ovf", "rej",
              "drop", "uart_err", "maxgap", "state"]


def decode_raw(raw):
    """Independently re-decode the 32 raw payload bytes -> {column: value}."""
    out = {}
    for off, name, scale, signed in LAYOUT:
        v = (raw[off] << 8) | raw[off + 1]
        if signed and v >= 0x8000:
            v -= 0x10000
        out[name] = v / scale
    return out


def check_raw(rec):
    """Compare the printed values with a fresh decode of raw32."""
    if not rec.get("raw32"):
        return []
    derived = decode_raw(rec["raw32"])
    bad = []
    for off, name, scale, signed in LAYOUT:
        if name not in rec:
            continue
        tol = 0.5 if scale == 1.0 else (0.006 * scale + 1e-6)
        if abs(derived[name] - rec[name]) > tol:
            bad.append((name, rec[name], derived[name]))
    return bad


class Stats(object):
    """Running aggregates, so the summary never needs every record in RAM."""

    def __init__(self):
        self.n = 0
        self.first_boot_ms = None
        self.last_boot_ms = None
        self.first_seq = None
        self.last_seq = None
        self.restarts = 0
        self.prev_boot_ms = None
        self.agg = {c: [None, None, 0.0] for c in VALUE_COLS}   # min, max, sum
        self.age_min = None
        self.age_max = None
        self.age_sum = 0.0
        self.age_n = 0
        self.stale = 0
        self.checked = 0
        self.mismatch = 0
        self.mismatch_detail = []
        self.no_raw = 0
        self.tail = []

    def add(self, rec):
        self.n += 1
        b = rec.get("boot_ms")
        if b is not None:
            if self.prev_boot_ms is not None and b < self.prev_boot_ms:
                self.restarts += 1
            self.prev_boot_ms = b
            if self.first_boot_ms is None:
                self.first_boot_ms = b
            self.last_boot_ms = b
        if rec.get("seq") is not None:
            if self.first_seq is None:
                self.first_seq = rec["seq"]
            self.last_seq = rec["seq"]
        if rec.get("age_ms") is not None:
            a = rec["age_ms"]
            self.age_min = a if self.age_min is None else min(self.age_min, a)
            self.age_max = a if self.age_max is None else max(self.age_max, a)
            self.age_sum += a
            self.age_n += 1
        if rec.get("stale"):
            self.stale += 1
        for c in VALUE_COLS:
            if c in rec:
                v = rec[c]
                slot = self.agg[c]
                slot[0] = v if slot[0] is None else min(slot[0], v)
                slot[1] = v if slot[1] is None else max(slot[1], v)
                slot[2] += v
        if rec.get("raw32"):
            self.checked += 1
            bad = check_raw(rec)
            if bad:
                self.mismatch += 1
                if len(self.mismatch_detail) < 5:
                    self.mismatch_detail.append((rec.get("seq"), bad))
        else:
            self.no_raw += 1
        self.tail.append(rec)
        if len(self.tail) > MAX_TAIL:
            del self.tail[0]


class Parser(object):
    """Feed text lines in; every completed wind frame calls on_record(rec)."""

    def __init__(self, on_record=None, on_event=None):
        self.cur = None
        self.on_record = on_record
        self.on_event = on_event          # (kind, payload) for stat/bad/boot
        self.stats = []
        self.bad_frames = []
        self.boot = []
        self.dropped = 0
        self.unknown = 0

    def feed(self, line, host_epoch=None):
        line = line.rstrip("\r\n")
        if not line.strip():
            return
        line = RE_TS_PREFIX.sub("", line)
        if line.startswith("#"):          # fixture files may carry comments
            return

        m = RE_HEAD.search(line)
        if m:
            if self.cur is not None:
                self.dropped += 1
            self.cur = {
                "boot_ms": int(m.group(1)),
                "seq": int(m.group(2)) if m.group(2) is not None else None,
                "age_ms": int(m.group(3)) if m.group(3) is not None else None,
                "stale": bool(m.group(4)),
                "host_epoch": round(host_epoch, 3) if host_epoch else "",
                "host_time": (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(host_epoch))
                              + ".%03d" % int((host_epoch % 1) * 1000)) if host_epoch else "",
            }
            return

        if line.startswith(("[BOOT]", "[WARN]", "[VER ]")):
            self.boot.append(line)
            if self.on_event:
                self.on_event("boot", line)
            return

        m = RE_STAT.search(line)
        if m:
            vals = {k: int(v) for k, v in RE_KV.findall(m.group(1))}
            self.stats.append(vals)
            if self.on_event:
                self.on_event("stat", vals)
            return

        m = RE_BAD.search(line)
        if m:
            hexpart = m.group(1).strip()
            try:
                data = [int(t, 16) for t in hexpart.split()] if hexpart else []
            except ValueError:
                data = []
            item = (data, int(m.group(2)), int(m.group(3)),
                    int(m.group(4), 16) if m.group(4) else 0)
            self.bad_frames.append(item)
            if self.on_event:
                self.on_event("bad", item)
            return

        if self.cur is None:
            if not line.startswith(("vec", "sca", "env", "rt", "xyz", "raw32")):
                self.unknown += 1
            return

        m = RE_VEC.match(line)
        if m:
            self.cur.update(vec_speed=float(m.group(1)), vec_hor=float(m.group(2)),
                            vec_ver=float(m.group(3)))
            return
        m = RE_SCA.match(line)
        if m:
            self.cur.update(sca_speed=float(m.group(1)), sca_hor=float(m.group(2)),
                            sca_ver=float(m.group(3)))
            return
        m = RE_ENV.match(line)
        if m:
            self.cur.update(level=float(m.group(1)), temp_c=float(m.group(2)),
                            sound_ms=float(m.group(3)))
            return
        m = RE_RT.match(line)
        if m:
            self.cur.update(rt_speed=float(m.group(1)), rt_hor=float(m.group(2)),
                            rt_ver=float(m.group(3)))
            return
        m = RE_XYZ.match(line)
        if m:
            self.cur.update(x=float(m.group(1)), y=float(m.group(2)), z=float(m.group(3)))
            return
        m = RE_RAW.match(line)
        if m:
            toks = m.group(1).split()
            raw = None
            if len(toks) == 32:
                try:
                    raw = [int(t, 16) for t in toks]
                except ValueError:
                    raw = None
            self.cur["raw32"] = raw
            rec = self.cur
            self.cur = None
            if self.on_record:
                self.on_record(rec)
            return
        self.unknown += 1


# --------------------------------------------------------------------------
# CSV handling
# --------------------------------------------------------------------------
def fmt_row(rec):
    row = []
    for c in COLUMNS:
        v = rec.get(c, "")
        if c == "raw32":
            v = " ".join("%02X" % b for b in v) if v else ""
        elif c == "stale":
            v = "1" if v else "0"
        elif c == "level":
            v = "%d" % int(v) if v != "" else ""
        elif v != "" and isinstance(v, float):
            v = "%.2f" % v
        row.append(v)
    return row


class CsvSink(object):
    """Streaming, append-safe CSV writer with a header compatibility check."""

    def __init__(self, path, header, truncate=False):
        self.path = path
        self.header = header
        self.rows = 0
        known = [",".join(v) for v in HEADER_SETS.values()]
        new = truncate or not os.path.exists(path) or os.path.getsize(path) == 0
        if not new:
            # utf-8-sig strips a BOM if the existing file has one
            with open(path, "r", encoding="utf-8-sig", errors="replace") as fh:
                head = fh.readline().rstrip("\r\n")
            if head not in known:
                raise SystemExit(
                    "ERROR: %s exists with an unknown header:\n  %s\n"
                    "       pick a new --out name, or pass --truncate to overwrite."
                    % (path, head))
            # an existing file keeps its own header/labels; rows are positional
        bom = header == COLUMNS_ZH
        # append mode must never write a second BOM in the middle of the file
        self.fh = open(path, "w" if new else "a", newline="",
                       encoding=("utf-8-sig" if bom else "utf-8") if new else "utf-8")
        self.w = csv.writer(self.fh)
        if new:
            self.w.writerow(header)
            self.fh.flush()

    def write(self, rec):
        self.w.writerow(fmt_row(rec))
        self.rows += 1
        self.fh.flush()
        return self.rows

    def sync(self):
        try:
            self.fh.flush()
            os.fsync(self.fh.fileno())
        except (OSError, ValueError):
            pass

    def close(self):
        try:
            self.fh.flush()
            self.fh.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# input sources
# --------------------------------------------------------------------------
def iter_replay(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        raw = fh.read()
    if "---- text ----" in raw:
        body = raw.split("---- text ----", 1)[1].split("---- hex ----", 1)[0]
    else:
        body = raw
    for line in body.split("\n"):
        yield line, None


def iter_live(port, baud, duration, log=None):
    """Yield (line, host_epoch). Reopens the port by itself after a dropout."""
    import serial
    t0 = time.time()
    ser = None
    buf = b""
    try:
        while True:
            if duration > 0 and time.time() - t0 >= duration:
                return
            if ser is None:
                try:
                    ser = serial.Serial(port, baud, bytesize=8, parity="N",
                                        stopbits=1, timeout=0.2)
                    buf = b""
                    if log:
                        log("opened %s" % port, False)
                except Exception as exc:
                    if log:
                        log("open failed (%s: %s) - retry in %.0fs"
                            % (type(exc).__name__, exc, RECONNECT_DELAY_S), True)
                    time.sleep(RECONNECT_DELAY_S)
                    continue
            try:
                chunk = ser.read(512)
            except Exception as exc:
                if log:
                    log("read failed (%s: %s) - reopening" % (type(exc).__name__, exc), True)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                continue
            if not chunk:
                yield "", None
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                yield line.decode("ascii", "replace"), time.time()
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


def list_ports():
    try:
        import serial.tools.list_ports as lp
    except ImportError:
        print("pyserial not installed: python -m pip install pyserial")
        return
    ports = list(lp.comports())
    if not ports:
        print("no serial port found (is the DK plugged in?)")
        return
    for p in ports:
        print("%-8s %s" % (p.device, p.description))


# --------------------------------------------------------------------------
def summarize(stats, parser, wall, replay, csv_path, raw_path, reconnects):
    out = []
    add = out.append
    add("")
    add("=" * 68)
    add("windlog summary")
    add("=" * 68)
    if parser.boot:
        add("boot/warn lines:")
        for b in parser.boot:
            add("  " + b.strip())
    if replay:
        add("run               : replay")
    else:
        add("run               : %.1f s wall%s" % (wall,
                                                   "  (port reconnects: %d)" % reconnects
                                                   if reconnects else ""))
    add("wind frames logged: %d   (incomplete blocks: %d, unparsed lines: %d)"
        % (stats.n, parser.dropped, parser.unknown))
    if csv_path:
        add("csv               : %s   (+%d rows this run)" % (csv_path, stats.n))
    if raw_path:
        add("raw log           : %s" % raw_path)

    span_s = None
    if stats.first_boot_ms is not None and stats.last_boot_ms is not None:
        span_s = (stats.last_boot_ms - stats.first_boot_ms) / 1000.0
        if stats.restarts:
            add("board time span   : %d runs concatenated (board restarted %d time(s))"
                % (stats.restarts + 1, stats.restarts))
        else:
            add("board time span   : %.1f s  (%d .. %d ms)"
                % (span_s, stats.first_boot_ms, stats.last_boot_ms))
    if stats.first_seq is not None and stats.last_seq is not None and not stats.restarts:
        add("seq span          : %d .. %d  (= %d frames)"
            % (stats.first_seq, stats.last_seq, stats.last_seq - stats.first_seq))
        if span_s and span_s > 0:
            add("frame rate        : %.2f frames/s  (200ms poll -> ceiling 5.00)"
                % ((stats.last_seq - stats.first_seq) / span_s))
    elif stats.first_seq is None:
        add("seq / age columns : not present in this capture (older firmware)")
    if stats.age_n:
        add("age_ms            : min %d  max %d  mean %.1f   (stale marked: %d)"
            % (stats.age_min, stats.age_max, stats.age_sum / stats.age_n, stats.stale))

    if stats.n:
        add("")
        add("values                       min          mean           max")
        for c in VALUE_COLS:
            lo, hi, tot = stats.agg[c]
            if lo is None:
                continue
            add("  %-14s %12.2f %13.2f %13.2f" % (c, lo, tot / stats.n, hi))

    add("")
    add("raw32 cross-check (independent re-decode):")
    add("  frames with raw32 : %d%s" % (stats.checked,
                                       "" if not stats.no_raw else "  (%d without)" % stats.no_raw))
    add("  mismatching frames: %d" % stats.mismatch)
    for seq, bad in stats.mismatch_detail:
        add("    seq=%s: %s" % (seq, bad))

    devs = []
    for r in stats.tail:
        if "sound_ms" in r and "temp_c" in r:
            devs.append(abs(r["sound_ms"] - (331.3 + 0.606 * r["temp_c"])))
    if devs:
        add("  |sound - (331.3+0.606*T)| : mean %.2f m/s, max %.2f m/s (last %d frames)"
            % (sum(devs) / len(devs), max(devs), len(devs)))

    if parser.stats:
        add("")
        add("link counters (last - first):")
        a, b = parser.stats[0], parser.stats[-1]
        for n in STAT_NAMES:
            if n == "state" or n not in a or n not in b:
                continue
            add("  %-9s %+d   (last %d)" % (n, b[n] - a[n], b[n]))
        if "state" in b:
            add("  state           : %d" % b["state"])
        if "frame" in a and "frame" in b and span_s and span_s > 0 and not stats.restarts:
            add("  good frames/s from counters : %.2f" % ((b["frame"] - a["frame"]) / span_s))
        missing = [n for n in STAT_NAMES if n not in b]
        if missing:
            add("  (not reported by this firmware revision: %s)" % ", ".join(missing))

    add("")
    add("bad frames shown  : %d" % len(parser.bad_frames))
    for data, n, mg, exc in parser.bad_frames[:10]:
        add("  %s  n=%d maxgap=%dms exc=0x%02X"
            % (" ".join("%02X" % b for b in data), n, mg, exc))
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description="continuous wind-sensor logger (01bsp_modbus)")
    ap.add_argument("port", nargs="?", default=DEFAULT_PORT,
                    help="serial port (default %s)" % DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to log, 0 = until Ctrl-C (default 0)")
    ap.add_argument("--out", default="windlog.csv", help="CSV file (appended if present)")
    ap.add_argument("--header", choices=["zh", "en"], default="zh",
                    help="CSV column labels: zh = Chinese + UTF-8 BOM (default), "
                         "en = ASCII without BOM")
    ap.add_argument("--truncate", action="store_true", help="overwrite an existing CSV")
    ap.add_argument("--no-csv", action="store_true", help="do not write the CSV")
    ap.add_argument("--raw", default=None,
                    help="raw line log (default <out>.raw.txt, '' to skip)")
    ap.add_argument("--replay", metavar="FILE", default=None,
                    help="parse a saved capture instead of a live port")
    ap.add_argument("--quiet", action="store_true", help="no live status line")
    ap.add_argument("--fsync", type=float, default=5.0,
                    help="fsync the CSV at most every N seconds (default 5)")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = ap.parse_args(argv)

    if args.list:
        list_ports()
        return 0

    live = args.replay is None
    raw_path = args.raw
    if raw_path is None:
        raw_path = os.path.splitext(args.out)[0] + ".raw.txt" if args.out else ""
    if not live:
        raw_path = ""            # replaying needs no raw copy

    t_start = time.time()
    state = {"reconnects": 0, "last_sync": time.time(), "last_beat": time.time(),
             "last_stat": None, "last_bad": None}
    stats = Stats()

    def log_line(msg, is_error):
        if is_error:
            state["reconnects"] += 1
        print("  [%7.1fs] %s" % (time.time() - t_start, msg), flush=True)

    def on_record(rec):
        if csv_sink:
            csv_sink.write(rec)
            now = time.time()
            if args.fsync > 0 and now - state["last_sync"] >= args.fsync:
                csv_sink.sync()
                state["last_sync"] = now
        stats.add(rec)
        if live and not args.quiet:
            print("\r  %7.1fs  rows=%-7d seq=%-6s age=%-4s vec=%-6s rt=%-6s T=%-6s v=%-8s %s"
                  % (time.time() - t_start, stats.n, rec.get("seq"), rec.get("age_ms"),
                     rec.get("vec_speed"), rec.get("rt_speed"), rec.get("temp_c"),
                     rec.get("sound_ms"), "STALE" if rec.get("stale") else ""),
                  end="", flush=True)

    def on_event(kind, payload):
        if kind == "bad":
            state["last_bad"] = payload
            if not args.quiet:
                data, n, mg, exc = payload
                print("\r  [%7.1fs] BAD frame: %s n=%d maxgap=%dms exc=0x%02X"
                      % (time.time() - t_start,
                         " ".join("%02X" % b for b in data), n, mg, exc), flush=True)
        elif kind == "stat":
            state["last_stat"] = payload
        elif kind == "boot":
            if not args.quiet:
                print("\r  [%7.1fs] %s" % (time.time() - t_start, payload.strip()), flush=True)

    csv_sink = None
    if not args.no_csv:
        try:
            csv_sink = CsvSink(args.out, HEADER_SETS[args.header], truncate=args.truncate)
        except SystemExit as exc:
            print(exc)
            return 2
        except OSError as exc:
            print("ERROR: cannot open %s: %s" % (args.out, exc))
            return 2

    raw_fh = open(raw_path, "w", encoding="utf-8") if raw_path else None
    parser = Parser(on_record=on_record, on_event=on_event)

    print("windlog: %s -> %s" % ("replay " + args.replay if not live else args.port,
                                 args.out if not args.no_csv else "(no csv)"))
    if live:
        print("logging until Ctrl-C (Ctrl-C stops and writes the summary)")
    try:
        source = (iter_replay(args.replay) if not live
                  else iter_live(args.port, args.baud, args.duration, log=log_line))
        for line, epoch in source:
            if not line:
                now = time.time()          # idle tick: service fsync + heartbeat
                if csv_sink and args.fsync > 0 and now - state["last_sync"] >= args.fsync:
                    csv_sink.sync()
                    state["last_sync"] = now
                if raw_fh:
                    raw_fh.flush()
                if live and not args.quiet and now - state["last_beat"] >= HEARTBEAT_S:
                    state["last_beat"] = now
                    cnt = ",".join("%s=%s" % kv for kv in sorted((state["last_stat"] or {}).items()))
                    print("\r  [%7.1fs] alive: %d rows logged | %s"
                          % (now - t_start, stats.n, cnt or "no [STAT] yet"), flush=True)
                continue
            if raw_fh:
                raw_fh.write("[%8.3f] %s\n" % (time.time() - t_start, line))
            parser.feed(line, epoch)
    except KeyboardInterrupt:
        print("\n  stopped by user")
    except Exception as exc:
        print("\n  ERROR %s: %s" % (type(exc).__name__, exc))
        if live:
            print("  (port busy / wrong name? try --list)")
    finally:
        if raw_fh:
            raw_fh.close()
        if csv_sink:
            csv_sink.sync()
            csv_sink.close()
        if live and not args.quiet:
            print("")

    wall = time.time() - t_start
    text = summarize(stats, parser, wall, replay=not live,
                     csv_path=args.out if not args.no_csv else None,
                     raw_path=raw_path, reconnects=state["reconnects"])
    print(text)
    if args.out and not args.no_csv:
        try:
            with open(os.path.splitext(args.out)[0] + ".summary.txt", "w",
                      encoding="utf-8") as fh:
                fh.write(text + "\n")
        except OSError:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
