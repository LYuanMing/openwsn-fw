r"""OpenWSN topology + per-node voltage monitor (Tkinter).

Data path:
    leaf --uinject--> root --serial--> HDLC 'D' frames --> uinject trailer
    (mv, tf, eui, parent) --> this GUI draws the tree and each node's working
    point.  A 'D' frame is  'D' + addr16(2) + asn(5) + myEUI64(8) +
    prevhopEUI64(8) + packet, and the packet ends with the uinject trailer
    "...uinject" + 23 bytes of state + mV(LE2) + tf + eui7 + parent7.

The battery-free board has no usable UART, and it is meant to be the DAG root
with every other node deployed out of reach.  So the firmware mirrors the whole
serial byte stream into a SEGGER-RTT ring (drivers/common/openserial.c feeds
outputHdlcByte() into bsp/boards/nrf52840_dk/rtt_console.c), and the host reads it
back over SWD:
    root --RTT up-channel 0 "Terminal"--> JLinkRTTLogger --file--> host/rtt_link.py
        --hdlc.Framer(xonxoff=False)--> the same ('frame', {...}) events --> model
    Identical bytes, identical framing, so nothing here knows the port is not a
    COM port: the 'D' frames carry every child's report and the root's own
    'F'/TOPO lines come along too.  The mirror taps the stream *before* the
    UART's XON/XOFF escaping, hence xonxoff=False.

Control path (two ways, the first one needs no cable to the node):
    this GUI --HDLC 'W' <eui7> <mv hi> <mv lo>--> root's UART --> root sends
    "SETVOLT"+eui7+mV as UDP to ff02::1 --> the matching node applies it via
    energy_aware_setVoltage() and reports back in its next uinject.
    this GUI --HDLC 'V' <mv hi> <mv lo>--> a node's own UART --> same setter.
    An RTT link is uplink only: a root reached that way takes no 'W'/'V'/'M'/'T'
    (the model learns the tree from the root's own snapshots instead).

Run:  venv\Scripts\python.exe wsn_gui.py            (auto-find the root)
      venv\Scripts\python.exe wsn_gui.py COM10 RTT:602719539
"""

import json
import queue
import sys
import time
import traceback
import tkinter as tk
from tkinter import ttk
from pathlib import Path

import link
import model
import openwsn_proto
import rtt_link

BAUD = 115200
QUICK_MV = (1800, 2200, 2600, 3000, 3500, 4200)
NO_PORT = "（未选择）"
# A relayed command walks down through the shared cell (one broadcast per
# slotframe, and the MAC sends app broadcasts at the first opportunity), and a
# throttled node reports back only on its next uplink slot: measured 2026-11-11
# the round trip is 2-6 s, but a node at tf=3 that just changed parent has taken
# 30 s+.  The window has to outlast that, otherwise the GUI says "not confirmed"
# for a command that did land.
REMOTE_RETRY_S = 20.0     # wait this long for a relayed node to report back
REMOTE_RETRIES = 3        # extra 'W' sends before giving up
LAST_PORTS = Path(__file__).resolve().with_name("last_ports.json")
ERROR_LOG = Path(__file__).resolve().with_name("gui_error.log")


def log_exception(where):
    """Append a traceback to gui_error.log instead of letting it kill a timer."""
    try:
        with open(ERROR_LOG, "a", encoding="utf-8") as fh:
            fh.write("%s [%s]\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), where))
            traceback.print_exc(file=fh)
    except Exception:  # noqa: BLE001 - logging must never break the UI
        pass


def mv_color(mv):
    if mv is None:
        return "#d9d9d9"
    if mv >= 3600:
        return "#7ed37e"
    if mv >= 3000:
        return "#c9e07a"
    if mv >= 2400:
        return "#f5c26b"
    return "#ef8a7a"


FULL_MV = 4200


def node_fill(node):
    """Box colour.  The DAG root never sends its own uinject row - it has no
    parent to report through - so its mV is always unknown.  Treat it as a
    fully charged battery instead of the grey "unknown" colour."""
    if node.is_root:
        return mv_color(FULL_MV)
    return mv_color(node.mv)


def node_mv_text(node):
    """Voltage caption under a node box.  The root shows none (its voltage is
    not measured), so the caller collapses that line."""
    if node.is_root:
        return ""
    return "?" if node.mv is None else "%d mV" % node.mv


class App(object):
    def __init__(self, master):
        self.master = master
        master.title("OpenWSN 拓扑 / 电压监控")
        master.geometry("1220x780")
        self.model = model.TopologyModel()
        self.links = {}
        self.q = queue.Queue()
        self.last_version = -1
        self.port_rows = {}
        self.node_rows = {}
        self.node_order = []
        # ("W" retry state) broadcasts are lossy: the root can only send them in
        # the shared cell and a node may be transmitting in that slot.  Resend
        # until the node reports the new value, then stop.
        self.pending_remote = None      # (eui, mv, next_try, tries_left)
        self._build_ui()
        self.refresh_ports()
        self._probes = {}
        # No port on the command line: reconnect the ports used last time, or
        # (first run) probe the J-Link CDC ports and keep the ones that talk.
        if len(sys.argv) > 1:
            master.after(300, lambda: [self.connect_port(p) for p in sys.argv[1:]])
        else:
            master.after(300, self.autostart)
        master.after(100, self._drain)
        master.after(400, self._tick)
        master.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        self.master.rowconfigure(1, weight=3)
        self.master.rowconfigure(3, weight=1)
        self.master.columnconfigure(0, weight=3)
        self.master.columnconfigure(1, weight=2)

        # toolbar
        bar = ttk.Frame(self.master, padding=(6, 6))
        bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Label(bar, text="串口:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=34, state="readonly")
        self.port_box.pack(side="left", padx=4)
        ttk.Button(bar, text="刷新", command=self.refresh_ports).pack(side="left")
        ttk.Button(bar, text="自动查找 root", command=self.autodetect).pack(side="left", padx=4)
        ttk.Button(bar, text="连接", command=self.connect_selected).pack(side="left", padx=4)
        ttk.Button(bar, text="全部断开", command=self.disconnect_all).pack(side="left")
        ttk.Button(bar, text="演示数据", command=self.demo).pack(side="left", padx=8)
        ttk.Button(bar, text="清空视图", command=self.clear_view).pack(side="left")
        ttk.Label(bar, text="    离线:").pack(side="left")
        self.offline_var = tk.IntVar(value=int(model.OFFLINE_AFTER_S))
        self.offline_box = ttk.Spinbox(bar, from_=30, to=3600, increment=30, width=5,
                                       textvariable=self.offline_var,
                                       command=self._apply_offline)
        self.offline_box.pack(side="left", padx=(2, 0))
        self.offline_box.bind("<Return>", lambda e: self._apply_offline())
        ttk.Label(bar, text="s 无包即移除").pack(side="left")
        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(bar, textvariable=self.status_var, foreground="#555").pack(side="right")

        # canvas
        left = ttk.Frame(self.master, padding=(6, 0))
        left.grid(row=1, column=0, sticky="nsew")
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)
        ttk.Label(left, text="拓扑（root 居中，子节点按父节点分层；颜色=电压）").grid(row=0, column=0, sticky="w")
        self.canvas = tk.Canvas(left, background="white", highlightthickness=1,
                                highlightbackground="#bbb")
        self.canvas.grid(row=1, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda e: self.redraw())

        # right side: ports + nodes
        right = ttk.Frame(self.master, padding=(6, 0))
        right.grid(row=1, column=1, sticky="nsew")
        right.rowconfigure(1, weight=1)
        right.rowconfigure(3, weight=2)
        right.columnconfigure(0, weight=1)
        ttk.Label(right, text="串口（选中一行作为电压目标）").grid(row=0, column=0, sticky="w")
        self.port_tree = ttk.Treeview(right, columns=("port", "state", "node", "rx", "err"),
                                      show="headings", height=5)
        for col, text, width in (("port", "端口", 70), ("state", "状态", 70),
                                 ("node", "节点", 60), ("rx", "帧", 50), ("err", "错误", 120)):
            self.port_tree.heading(col, text=text)
            self.port_tree.column(col, width=width, anchor="w")
        self.port_tree.grid(row=1, column=0, sticky="nsew")
        self.port_tree.bind("<<TreeviewSelect>>", lambda e: self._sync_target())

        ttk.Label(right, text="节点（来自 root 收到的 uinject）").grid(row=2, column=0, sticky="w", pady=(6, 0))
        self.node_tree = ttk.Treeview(right, columns=("eui", "parent", "mv", "tf", "pkt", "age", "port"),
                                      show="headings", height=10)
        for col, text, width in (("eui", "EUI7", 55), ("parent", "父", 45), ("mv", "mV", 60),
                                 ("tf", "tf", 40), ("pkt", "包", 50), ("age", "秒前", 55),
                                 ("port", "端口", 65)):
            self.node_tree.heading(col, text=text)
            self.node_tree.column(col, width=width, anchor="w")
        self.node_tree.grid(row=3, column=0, sticky="nsew")
        self.node_tree.bind("<<TreeviewSelect>>", self._node_selected)

        # voltage panel
        panel = ttk.LabelFrame(self.master, text="电压控制（'V' 直连 / 'W' 经 root 下发）", padding=(6, 4))
        panel.grid(row=2, column=0, columnspan=2, sticky="ew", padx=6, pady=4)
        ttk.Label(panel, text="目标:").pack(side="left")
        self.target_var = tk.StringVar()
        self.target_box = ttk.Combobox(panel, textvariable=self.target_var, width=20, state="readonly")
        self.target_box.pack(side="left", padx=4)
        self._target_values = []
        ttk.Label(panel, text="mV:").pack(side="left", padx=(8, 2))
        self.mv_var = tk.StringVar(value="3000")
        ttk.Entry(panel, textvariable=self.mv_var, width=7).pack(side="left")
        ttk.Button(panel, text="设置", command=self.send_voltage).pack(side="left", padx=4)
        ttk.Button(panel, text="读取", command=self.read_voltage).pack(side="left")
        for mv in QUICK_MV:
            ttk.Button(panel, text=str(mv), width=5,
                       command=lambda v=mv: self.set_mv_and_send(v)).pack(side="left", padx=1)
        self.all_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(panel, text="广播到全部端口", variable=self.all_var).pack(side="left", padx=8)
        # Even a locally attached node can be driven through the root, so the
        # remote path stays testable without unplugging anything.
        self.remote_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(panel, text="经 root 下发", variable=self.remote_var).pack(side="left", padx=4)

        # Announce a working-point change at once ('M').  Off by default: the
        # experiment runs with a voltage that moves all the time, and an
        # announcement per change would make the control traffic a function of
        # the voltage instead of the protocol.  This switch is volatile on the
        # mote too - a reset returns to the compiled default (off).
        diof = ttk.Frame(panel)
        diof.pack(side="left", padx=(12, 0))
        self.dio_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(diof, text="电压变化立即发 DIO（调试）",
                        variable=self.dio_var).pack(side="left")
        ttk.Label(diof, text=" 最小间隔 s:").pack(side="left")
        self.dio_tmin_var = tk.StringVar(value="15")
        ttk.Entry(diof, textvariable=self.dio_tmin_var, width=4).pack(side="left")
        ttk.Button(diof, text="下发", width=5,
                   command=self.send_dio_mode).pack(side="left", padx=4)

        # log
        logf = ttk.Frame(self.master, padding=(6, 0, 6, 6))
        logf.grid(row=3, column=0, columnspan=2, sticky="nsew")
        logf.rowconfigure(1, weight=1)
        logf.columnconfigure(0, weight=1)
        ttk.Label(logf, text="日志").grid(row=0, column=0, sticky="w")
        self.log = tk.Text(logf, height=8, wrap="none", font=("Consolas", 9))
        self.log.grid(row=1, column=0, sticky="nsew")
        sb = ttk.Scrollbar(logf, orient="vertical", command=self.log.yview)
        sb.grid(row=1, column=1, sticky="ns")
        self.log.configure(yscrollcommand=sb.set, state="disabled")

    # --------------------------------------------------------------- ports
    def refresh_ports(self):
        ports = link.list_serial_ports()
        # Never preselect a port: the first entries are usually J-Link/CH340
        # adapters that have nothing to do with the motes (COM3..COM6 here).
        values = [NO_PORT] + ["%s  %s" % (p[0], p[1]) for p in ports]
        # A battery-free node has no usable UART, but its J-Link can read the
        # RTT ring the firmware publishes (host/rtt_link.py): one entry per
        # probe, e.g. "RTT:602719539  J-Link PLUS".
        values += ["RTT:%s  %s" % (sn, name) for sn, name in rtt_link.list_rtt_probes()]
        self.port_box["values"] = values
        if self.port_var.get() not in values:
            self.port_var.set(NO_PORT)

    def _selected_port(self):
        text = self.port_var.get()
        if not text or text == NO_PORT:
            return None
        return text.split()[0]

    def connect_selected(self):
        port = self._selected_port()
        if port:
            self.connect_port(port)
        else:
            self.status_var.set("先选中要连的串口（本次测试是 COM10 / COM11 / COM13）")

    def connect_port(self, port):
        if port in self.links:
            self.status_var.set("%s 已连接" % port)
            return
        if port.startswith("RTT:"):
            # J-Link RTT: the node's log over SWD, no UART involved.  Uplink
            # only, so 'V'/'W' still go through the root.
            lk = rtt_link.RttLink(port.split(":", 1)[1],
                                  emit=lambda kind, payload, p=port: self.q.put((p, kind, payload)))
            self.links[port] = lk
            lk.start()
            self.log_line("连接 %s（RTT 上行日志）" % port)
            self._save_last_ports()
            return
        lk = link.SerialLink(port, BAUD, emit=lambda kind, payload, p=port: self.q.put((p, kind, payload)))
        self.links[port] = lk
        lk.start()
        self.log_line("连接 %s @ %d" % (port, BAUD))
        self._save_last_ports()

    def disconnect_all(self):
        for port, lk in list(self.links.items()):
            lk.stop()
            self.log_line("断开 %s" % port)
        self.links.clear()
        self._save_last_ports()
        self._refresh_port_tree()

    def _refresh_port_tree(self):
        seen = set()
        for port, lk in self.links.items():
            seen.add(port)
            values = (port, "已连接" if lk.is_open else "打开中",
                      openwsn_proto.fmt_eui(self.model.port_eui[port]) if port in self.model.port_eui else "-",
                      lk.rx_frames, lk.last_error[:40])
            if port in self.port_rows:
                self.port_tree.item(self.port_rows[port], values=values)
            else:
                self.port_rows[port] = self.port_tree.insert("", "end", values=values)
        for port in list(self.port_rows):
            if port not in seen:
                self.port_tree.delete(self.port_rows.pop(port))
        self.status_var.set("已连接: %s" % (", ".join(sorted(self.links)) or "无"))

    def _sync_target(self):
        """A row picked in the 串口 table selects the node behind that port."""
        sel = self.port_tree.selection()
        if not sel:
            return
        eui = self.model.port_eui.get(self.port_tree.item(sel[0], "values")[0])
        if eui is not None:
            self._select_target(eui)

    # ------------------------------------------------------------- autostart
    def _save_last_ports(self):
        try:
            LAST_PORTS.write_text(json.dumps(sorted(self.links)), encoding="utf-8")
        except Exception as exc:  # noqa: BLE001 - never break the UI over this
            self.log_line("保存端口列表失败: %s" % exc)

    def _load_last_ports(self):
        try:
            ports = json.loads(LAST_PORTS.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001 - missing/corrupt file = first run
            return []
        available = {p[0] for p in link.list_serial_ports()}
        available |= {"RTT:" + sn for sn, _ in rtt_link.list_rtt_probes()}
        return [p for p in ports if p in available]

    def autostart(self):
        """Called at startup when no port was given on the command line."""
        if self.links:
            return
        saved = self._load_last_ports()
        if saved:
            for port in saved:
                self.connect_port(port)
            self.log_line("自动连接上次的端口: %s" % ", ".join(saved))
            return
        self.autodetect()

    def autodetect(self):
        """Open every candidate link for a few seconds and keep the ones that talk.

        Two kinds of candidates are probed together:

        * the J-Link CDC ports -- on a stock nRF52840-DK the mote's UART comes
          out there.  The DAG root prints status/D frames continuously; a leaf
          is silent until it is asked, so this normally finds exactly the root.
        * every J-Link probe's RTT ring (host/rtt_link.py) -- the battery-free
          board has no wired UART at all, its firmware mirrors the serial byte
          stream into RAM and the probe reads it over SWD.

        Whatever stays silent is closed again, so the scan is read-only and
        leaves nothing behind on the boards."""
        cands = [p[0] for p in link.list_serial_ports() if "JLink" in (p[1] or "")]
        cands = [p for p in cands if p not in self.links and p not in self._probes]
        rtt = ["RTT:%s" % sn for sn, _ in rtt_link.list_rtt_probes()]
        rtt = [p for p in rtt if p not in self.links and p not in self._probes]
        if not cands and not rtt:
            self.log_line("没有可扫描的 J-Link 串口或 RTT 探针（先确认板子上电）")
            return
        self.log_line("扫描 %d 个 J-Link 串口 + %d 个 RTT 探针找 root ..."
                      % (len(cands), len(rtt)))
        for port in cands:
            lk = link.SerialLink(port, BAUD,
                                 emit=lambda kind, payload, p=port: self.q.put((p, kind, payload)))
            self._probes[port] = lk
            lk.start()
        for port in rtt:
            lk = rtt_link.RttLink(port.split(":", 1)[1],
                                  emit=lambda kind, payload, p=port: self.q.put((p, kind, payload)))
            self._probes[port] = lk
            lk.start()
        # A probe scan is slower than a COM scan: RttLink has to spawn
        # JLinkRTTLogger and that process has to find the RTT control block in
        # RAM before the first frame can arrive (usually < 2 s, retried every 5 s
        # after a failure), so leave the RTT candidates a little more time.
        self.master.after(4500 if rtt else 3000, self._finish_autodetect)

    def _finish_autodetect(self):
        probes = self._probes
        self._probes = {}
        live = []
        for port, lk in probes.items():
            if lk.rx_frames > 0:
                live.append((port, lk))
            else:
                lk.stop()
        if not live:
            self.log_line("没扫到会说话的串口或 RTT 探针：确认 root 已上电并在发 D 帧")
            self._refresh_port_tree()
            return
        # A stock DK answers on two links at once: its J-Link CDC port and its
        # probe's RTT ring (the firmware mirrors the serial stream into RAM, see
        # bsp/boards/nrf52840_dk/rtt_console.c).  The probe serial is in the CDC
        # port's USB descriptor, so the pairing is exact; keep the serial link
        # and leave the probe free for flashing.
        serial_sns = set()
        for port, _lk in live:
            if not port.startswith("RTT:"):
                sn = link.probe_serial(port)
                if sn:
                    serial_sns.add(sn)
        chosen = {}
        opened = []
        for port, lk in sorted(live, key=lambda item: item[0].startswith("RTT:")):
            if port.startswith("RTT:") and port.split(":", 1)[1] in serial_sns:
                self.log_line("%s 与同一探针的串口重复，只用串口" % port)
                lk.stop()
                continue
            eui = self.model.port_eui.get(port)
            if eui is not None and eui in chosen:
                self.log_line("%s 与 %s 是同一个节点 0x%02x，关闭 %s"
                              % (chosen[eui], port, eui, port))
                lk.stop()
                continue
            if eui is not None:
                chosen[eui] = port
            self.links[port] = lk
            opened.append(port)
        self.log_line("会说话的链路: %s" % ", ".join(sorted(opened)))
        self._save_last_ports()
        for port in opened:
            self.log_line("%s 已打开" % port)
            if port.startswith("RTT:"):
                # RTT only carries the mote's output: 'V'/'W'/'T' have to travel
                # over the root's serial link (see send_voltage).
                continue
            self.links[port].send(openwsn_proto.read_voltage_payload())
            self.links[port].send(openwsn_proto.read_dio_mode_payload())
            self.links[port].send(openwsn_proto.dump_payload())
        self._refresh_port_tree()

    # ------------------------------------------------------------- voltage
    @staticmethod
    def _eui_of(text):
        tok = text.split()[0] if text and text.strip() else None
        if not tok:
            return None
        try:
            return int(tok, 16)
        except ValueError:
            return None

    def _target_label(self, node):
        label = openwsn_proto.fmt_eui(node.eui7)
        if node.is_root:
            label += " root"
        port = self.model.bound_port(node.eui7)
        label += (" @" + port) if port else " 经root"
        return label

    def _refresh_targets(self):
        """Rebuild the 目标 dropdown from the nodes the root has reported.

        Every node is listed, whether or not it is plugged into this PC: a
        node without a serial port of its own gets its command relayed by the
        root ('W'), which is the deployment case (only the root is attached).
        """
        values = [self._target_label(n) for n in self._sorted_nodes()]
        if values == self._target_values:
            return
        old = self._eui_of(self.target_var.get())
        self._target_values = values
        self.target_box["values"] = values
        if old is not None and any(self._eui_of(v) == old for v in values):
            self._select_target(old)           # keep the user's choice, new label
        elif self.model.root is not None and any(self._eui_of(v) == self.model.root for v in values):
            self._select_target(self.model.root)
        else:
            self.target_var.set(values[0] if values else "")

    def _select_target(self, eui):
        for value in self._target_values:
            if self._eui_of(value) == eui:
                self.target_var.set(value)
                return

    def _target_node(self):
        eui = self._eui_of(self.target_var.get())
        if eui is None:
            return None
        return self.model.nodes.get(eui)

    def _root_port(self):
        """The serial port of the DAG root (the one that relays 'W')."""
        if self.model.root is not None:
            port = self.model.bound_port(self.model.root)
            if port in self.links:
                return port
        for port in sorted(self.links):
            if self.model.port_eui.get(port) == self.model.root:
                return port
        return None

    def _node_selected(self, _event=None):
        """Pick the node's own port (if it is connected) as the 'V' target."""
        sel = self.node_tree.selection()
        if not sel:
            return
        values = self.node_tree.item(sel[0], "values")
        eui = None
        for node in self.model.nodes.values():
            if node.label == values[0]:
                eui = node.eui7
                break
        if eui is None:
            return
        self._select_target(eui)
        node = self.model.nodes.get(eui)
        if node is not None and node.mv is not None:
            self.mv_var.set(str(node.mv))

    def set_mv_and_send(self, mv):
        self.mv_var.set(str(mv))
        self.send_voltage()

    def _no_target(self):
        msg = "没选中目标节点：先连上 root，等它报出节点后再在“目标”里选"
        self.log_line(msg)
        self.status_var.set(msg)
        try:
            self.master.bell()
        except tk.TclError:
            pass

    def send_voltage(self):
        if self.all_var.get():
            try:
                mv = int(self.mv_var.get())
            except ValueError:
                self.log_line("mV 必须是整数")
                return
            payload = openwsn_proto.set_voltage_payload(mv)
            for port in sorted(self.links):
                ok = self.links[port].send(payload)
                self.log_line("TX %s  V %d mV  %s"
                              % (port, mv, "ok" if ok else "失败: " + self.links[port].last_error))
            return
        node = self._target_node()
        if node is None:
            self._no_target()
            return
        try:
            mv = int(self.mv_var.get())
        except ValueError:
            self.log_line("mV 必须是整数")
            return
        own = self.model.bound_port(node.eui7)
        if own is not None and getattr(self.links.get(own), "is_rtt", False):
            own = None      # RTT carries the log only: 'W' through the root
        if own is not None and not self.remote_var.get():
            # The node is plugged in here: talk to it directly, one hop, no
            # dependency on the mesh at all.
            payload = openwsn_proto.set_voltage_payload(mv)
            ok = self.links[own].send(payload)
            self.log_line("TX %s  V %d mV -> 节点 %s  %s"
                          % (own, mv, node.label, "ok" if ok else "失败: " + self.links[own].last_error))
            return
        root_port = self._root_port()
        if root_port is None:
            msg = "没连上 root：'W' 命令必须由 root 转发到网状网"
            self.log_line(msg)
            self.status_var.set(msg)
            return
        payload = openwsn_proto.remote_voltage_payload(node.eui7, mv)
        ok = self.links[root_port].send(payload)
        self.log_line("TX %s  W eui=%s %d mV（经 root 下发）  %s"
                      % (root_port, node.label, mv,
                         "ok" if ok else "失败: " + self.links[root_port].last_error))
        # A broadcast has no ACK: wait for the node's own report, resend if it
        # does not show up (openapps/uinject queues 3 copies per command).
        if ok:
            self.log_line("已下发：等待节点 %s 上报新的工作点（最多 %d s，超时会自动重发）"
                          % (node.label, int(REMOTE_RETRY_S * (REMOTE_RETRIES + 1))))
        self.pending_remote = (node.eui7, mv, time.time() + REMOTE_RETRY_S, REMOTE_RETRIES)

    def send_dio_mode(self):
        """Send 'M' <mode> <tmin_s> to every connected mote."""
        if not self.links:
            msg = "先连上串口：'M' 是发给节点自身的设置"
            self.log_line(msg)
            self.status_var.set(msg)
            return
        try:
            tmin = int(self.dio_tmin_var.get())
        except ValueError:
            self.log_line("最小间隔必须是整数秒（0 = 不限制）")
            return
        mode = 1 if self.dio_var.get() else 0
        payload = openwsn_proto.dio_mode_payload(mode, tmin)
        for port in sorted(self.links):
            ok = self.links[port].send(payload)
            self.log_line("TX %s  M mode=%d tmin=%ds  %s"
                          % (port, mode, tmin,
                             "ok" if ok else "失败: " + self.links[port].last_error))

    def read_voltage(self):
        node = self._target_node()
        if node is None:
            self._no_target()
            return
        own = self.model.bound_port(node.eui7)
        if own is not None and getattr(self.links.get(own), "is_rtt", False):
            own = None      # nothing to ask over RTT: fall back to the root
        if own is not None:
            self.links[own].send(openwsn_proto.read_voltage_payload())
            self.log_line("TX %s  读取节点 %s" % (own, node.label))
            return
        root_port = self._root_port()
        if root_port is None:
            self.log_line("没连上 root：远端节点只能靠 root 的快照（'T'）")
            return
        self.links[root_port].send(openwsn_proto.dump_payload())
        self.log_line("TX %s  'T' 让 root 重放节点 %s 的缓存" % (root_port, node.label))

    # ---------------------------------------------------------------- loop
    def _drain(self):
        # A single bad frame must not kill this timer: the Tk callbacks are a
        # chain, so an exception here would freeze the UI for good while the
        # model keeps growing in the background.
        try:
            while True:
                port, kind, payload = self.q.get_nowait()
                try:
                    self._handle(port, kind, payload)
                except Exception:  # noqa: BLE001
                    log_exception("handle %s %s" % (port, kind))
        except queue.Empty:
            pass
        self.master.after(100, self._drain)

    def _handle(self, port, kind, payload):
        if kind == "frame":
            for line in self.model.handle_frame(port, payload):
                self._append(line)
            self._sync_target()
        elif kind == "open":
            # learn the board's node id and working point right away; the root
            # answers 'T' with a snapshot of the children it already heard from,
            # so the canvas fills in one go instead of node by node
            if port in self.links:
                self.log_line("%s 已打开" % port)
                if not getattr(self.links[port], "is_rtt", False):
                    self.links[port].send(openwsn_proto.read_voltage_payload())
                    self.links[port].send(openwsn_proto.read_dio_mode_payload())
                    self.links[port].send(openwsn_proto.dump_payload())
        elif kind == "close":
            self.log_line("%s 已关闭" % port)
        elif kind == "error":
            if port in self.links or port not in self._probes:
                self.log_line("%s 错误: %s" % (port, payload))
        elif kind == "badframe":
            self.log_line("%s 坏帧: %s" % (port, payload))

    def _check_remote(self):
        """Resend a relayed 'W' until the target reports the commanded value."""
        if self.pending_remote is None:
            return
        eui, mv, due, tries = self.pending_remote
        node = self.model.nodes.get(eui)
        if node is not None and node.mv == mv and node.last_seen >= due - REMOTE_RETRY_S:
            self.log_line("节点 %s 已确认 %d mV" % (node.label, mv))
            self.pending_remote = None
            return
        if time.time() < due:
            return
        if tries <= 0:
            self.log_line("节点 %s 没有确认 %d mV（%d s 内没收到它的上报）：可再点一次设置"
                          % (openwsn_proto.fmt_eui(eui), mv,
                             int(REMOTE_RETRY_S * (REMOTE_RETRIES + 1))))
            self.pending_remote = None
            return
        root_port = self._root_port()
        if root_port is None:
            self.pending_remote = None
            return
        self.links[root_port].send(openwsn_proto.remote_voltage_payload(eui, mv))
        self.log_line("重发 W eui=%s %d mV（第 %d 次）"
                      % (openwsn_proto.fmt_eui(eui), mv, REMOTE_RETRIES - tries + 2))
        self.pending_remote = (eui, mv, time.time() + REMOTE_RETRY_S, tries - 1)

    def _tick(self):
        try:
            self._check_remote()
            self._apply_offline()
            self._prune_offline()
            self._refresh_port_tree()
            self._refresh_targets()
            if self.model.version != self.last_version:
                self.last_version = self.model.version
                self.redraw()
            self._refresh_node_tree()
        except Exception:  # noqa: BLE001 - keep ticking whatever happens
            log_exception("tick")
        self.master.after(400, self._tick)

    def _apply_offline(self):
        """Push the spinbox value into the model (30 s .. 1 h)."""
        try:
            value = float(self.offline_var.get())
        except Exception:  # noqa: BLE001 - half-typed text is not an error
            return
        self.model.offline_after = max(30.0, min(value, 3600.0))

    def _prune_offline(self):
        """Drop nodes the root stopped hearing, and log both directions."""
        for eui, age in self.model.prune_offline():
            self.log_line("节点 %s 离线：%.0f s 没收到包，从拓扑移除"
                          % (openwsn_proto.fmt_eui(eui), age))
        while self.model.returned:
            eui = self.model.returned.pop(0)
            self.log_line("节点 %s 重新上线：又收到包，加回拓扑" % openwsn_proto.fmt_eui(eui))

    def _sorted_nodes(self):
        return sorted(self.model.nodes.values(), key=lambda n: (0 if n.is_root else 1, n.eui7))

    def _refresh_node_tree(self):
        nodes = self._sorted_nodes()
        order = [n.eui7 for n in nodes]
        if order != self.node_order:
            # rows are inserted in discovery order, so rebuild when the sort changes
            for item in self.node_tree.get_children():
                self.node_tree.delete(item)
            self.node_rows.clear()
            self.node_order = order
        seen = set()
        for n in nodes:
            seen.add(n.eui7)
            values = (n.label, "-" if n.parent is None else openwsn_proto.fmt_eui(n.parent),
                      "-" if n.mv is None else n.mv, "-" if n.tf is None else n.tf,
                      n.packets, "%.0f" % n.age(), n.port or "-")
            if n.eui7 in self.node_rows:
                self.node_tree.item(self.node_rows[n.eui7], values=values)
            else:
                self.node_rows[n.eui7] = self.node_tree.insert("", "end", values=values)
        for eui in list(self.node_rows):
            if eui not in seen:
                self.node_tree.delete(self.node_rows.pop(eui))

    # -------------------------------------------------------------- canvas
    def redraw(self):
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(), 400)
        h = max(c.winfo_height(), 300)
        if not self.model.nodes:
            c.create_text(w / 2, h / 2, text="等待 root 转发 uinject 包 ...\n(先连接 root 的串口)",
                          fill="#888", justify="center", font=("Segoe UI", 11))
            return
        depth = self.model.depth_map()
        levels = {}
        for eui, d in depth.items():
            levels.setdefault(d, []).append(eui)
        for d in levels:
            levels[d].sort()
        max_depth = max(levels) if levels else 0
        box_w, box_h = 96, 58
        margin_y = 60
        pos = {}
        for d, euis in sorted(levels.items()):
            n = len(euis)
            span = min(w - 40, n * (box_w + 18))
            step = span / max(n, 1)
            x0 = (w - span) / 2 + step / 2
            y = margin_y + d * (box_h + 55)
            for i, eui in enumerate(euis):
                pos[eui] = (x0 + i * step, y)
        # edges
        for node in self.model.nodes.values():
            if node.parent is not None and node.parent in pos and node.eui7 in pos:
                x1, y1 = pos[node.parent]
                x2, y2 = pos[node.eui7]
                c.create_line(x1, y1 + box_h / 2, x2, y2 - box_h / 2, fill="#8a8a8a", width=2,
                              arrow="last", arrowshape=(10, 12, 4))
        # nodes
        for eui, (x, y) in pos.items():
            node = self.model.nodes[eui]
            fill = node_fill(node)
            outline = "#1f4e1f" if node.is_root else "#666"
            c.create_rectangle(x - box_w / 2, y - box_h / 2, x + box_w / 2, y + box_h / 2,
                               fill=fill, outline=outline, width=2 if node.is_root else 1)
            title = ("ROOT " if node.is_root else "") + openwsn_proto.fmt_eui(eui)
            c.create_text(x, y - 14, text=title, font=("Segoe UI", 9, "bold"))
            mv_text = node_mv_text(node)
            if mv_text:
                c.create_text(x, y + 4, text=mv_text, font=("Segoe UI", 9))
            tf_text = "tf -" if node.tf is None else "tf %d" % node.tf
            # no voltage caption on the root: let the tf line take that row
            c.create_text(x, y + (4 if node.is_root else 19), text=tf_text,
                          font=("Segoe UI", 8), fill="#444")
        # legend
        lx = 12
        ly = h - 22
        c.create_text(lx, ly - 14, text="电压:", anchor="w", fill="#666", font=("Segoe UI", 8))
        for i, (lo, hi, label) in enumerate(((3600, 4200, ">=3.6V"), (3000, 3600, "3.0-3.6"),
                                             (2400, 3000, "2.4-3.0"), (1800, 2400, "<2.4"))):
            c.create_rectangle(lx + i * 90, ly - 6, lx + i * 90 + 14, ly + 6,
                               fill=mv_color((lo + hi) // 2), outline="#999")
            c.create_text(lx + i * 90 + 20, ly, text=label, anchor="w", font=("Segoe UI", 8), fill="#555")
        c.create_text(w - 12, 14, anchor="e", fill="#666", font=("Segoe UI", 9),
                      text="节点 %d  uinject 包 %d  已离线 %d"
                      % (len(self.model.nodes), self.model.packets, len(self.model.dropped)))

    # ---------------------------------------------------------------- misc
    def log_line(self, text):
        self._append("%s %s" % (time.strftime("%H:%M:%S"), text))

    def _append(self, line):
        if not line:
            return
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > 3000:
            self.log.delete("1.0", "500.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def demo(self):
        for i, (node, parent, mv) in enumerate(((0xE4, 0xD9, 3000), (0xCD, 0xD9, 2200),
                                               (0x18, 0xE4, 4200), (0xBA, 0xE4, 1800))):
            self.model.apply_event({"kind": "topo", "root": 0xD9, "node": node,
                                    "parent": parent, "mv": mv, "tf": i % 4}, "DEMO")
        self.redraw()
        self._refresh_node_tree()
        self.log_line("已注入演示拓扑")

    def clear_view(self):
        self.model.reset()
        self.node_rows.clear()
        self.node_order = []
        for item in self.node_tree.get_children():
            self.node_tree.delete(item)
        self.last_version = -1
        self.redraw()
        self.log_line("视图已清空")

    def on_close(self):
        for lk in self.links.values():
            lk.stop()
        self.master.destroy()


def main():
    master = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except tk.TclError:
        pass
    App(master)
    master.mainloop()


if __name__ == "__main__":
    main()
