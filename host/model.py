"""Topology + voltage model, fed by the frames the root prints."""

import time

import openwsn_proto

# With throttle_factor > 0 the firmware prints one line per skipped slotframe
# (openstack/02a-MAClow/IEEE802154E.c:1025,1055) - ~87 lines/s at tf=3.  They
# are pure noise for the viewer, so they are counted instead of logged.
# A node whose *parent* is throttled prints the second wording once per skipped
# slotframe of its own schedule (measured on 0x61 under 0xCD at 2000 mV: one
# line every ~2 s), so both spellings are filtered.
SKIP_SLOT = "skip slot"

# A uinject packet carries a 16-bit counter that the reporting node increments on
# every packet it builds (openwsn-fw/openapps/uinject/uinject.c:542-544).  Packets
# from different paths are not delivered in build order: one built before a parent
# switch can reach the root seconds after a newer one, and it still carries the old
# parent.  Anything older than what a node already reported is therefore dropped.
# A node that restarts begins again at 0, so a backwards jump is accepted when the
# node has been quiet for a while or when the counter fell by a lot.
SEQ_REBOOT_BACK = 100
SEQ_QUIET_S = 30.0

# A node the root stops hearing is gone as far as the viewer is concerned: the
# radio link is lost, and keeping its last value would draw a dead board as if it
# were still there.  Five minutes with no packet at all is far longer than the
# slowest live node (throttle factor 3 still reports every few seconds), so a
# node ever dropped for this reason really was silent, not just unlucky.
OFFLINE_AFTER_S = 300.0


class Node(object):
    __slots__ = ("eui7", "parent", "mv", "tf", "last_seen", "packets", "port",
                 "is_root", "seq")

    def __init__(self, eui7, parent=None, mv=None, tf=None, port=None):
        self.eui7 = eui7
        self.parent = parent
        self.mv = mv
        self.tf = tf
        self.last_seen = time.time()
        self.packets = 0
        self.port = port
        self.is_root = False
        self.seq = None             # last uinject counter accepted for this node

    @property
    def label(self):
        return openwsn_proto.fmt_eui(self.eui7)

    def age(self):
        return time.time() - self.last_seen


class TopologyModel(object):
    """State the GUI draws: root, nodes, edges, per-port bindings, log."""

    def __init__(self):
        self.root = None
        self.nodes = {}
        self.port_eui = {}          # serial port -> eui7 (learned from replies)
        self.log = []
        self.version = 0
        self.packets = 0
        self.last_topo = None
        self.skip_slots = 0         # filtered "SF n I skip slot" lines
        self.stale_reports = 0      # late uinject packets dropped by the seq check
        self.last_stale = None      # (eui7, seq, kept seq) of the last dropped one
        self.offline_after = OFFLINE_AFTER_S   # seconds of silence -> evicted
        self.dropped = {}           # eui7 -> when it was evicted
        self.returned = []          # eui7s that came back, for the GUI to log

    # -- ingestion ---------------------------------------------------------
    def note(self, text, port=None):
        """Append a log line; returns it."""
        line = text.rstrip()
        if SKIP_SLOT in line:
            self.skip_slots += 1
            return ""
        stamp = time.strftime("%H:%M:%S")
        entry = "%s %s" % (stamp, line)
        self.log.append(entry)
        if len(self.log) > 4000:
            del self.log[:1000]
        return entry

    def handle_frame(self, port, frame):
        """Feed one decoded frame; returns a list of log lines to show."""
        lines = []
        if frame.get("type") == "printf":
            text = frame.get("text", "")
            events = openwsn_proto.parse_text(text)
            live_topo = any(
                ev["kind"] == "topo" and not ev.get("snap") for ev in events)
            if text.strip() and not live_topo:
                lines.append(self.note(text, port))
            for ev in events:
                if ev["kind"] == "topo" and not ev.get("snap"):
                    # The same packet also leaves the root as a 'D' frame
                    # (openbridge), and only that copy carries the uinject counter
                    # needed to drop late packets.  Let apply_uinject() own the
                    # live topology state, otherwise a late packet would still
                    # move the node back to its old parent for a moment.
                    continue
                self.apply_event(ev, port)
        elif frame.get("type") == "data":
            if "uinject" in frame:
                node = self.apply_uinject(frame, port)
                if node is not None:
                    lines.append(
                        self.note(
                            "TOPO root=%s node=%s parent=%s mv=%d tf=%d seq=%d"
                            % (
                                openwsn_proto.fmt_eui(self.root),
                                node.label,
                                openwsn_proto.fmt_eui(node.parent),
                                node.mv,
                                node.tf,
                                node.seq if node.seq is not None else -1,
                            ),
                            port,
                        )
                    )
                elif self.last_stale is not None:
                    eui7, seq, kept = self.last_stale
                    self.last_stale = None
                    kept_node = self.nodes.get(eui7)
                    lines.append(
                        self.note(
                            "TOPO late packet dropped node=%s seq=%d < %d (keep parent=%s)"
                            % (
                                openwsn_proto.fmt_eui(eui7),
                                seq,
                                kept,
                                openwsn_proto.fmt_eui(kept_node.parent)
                                if kept_node is not None and kept_node.parent is not None
                                else "?",
                            ),
                            port,
                        )
                    )
            else:
                payload = frame.get("payload", b"")
                lines.append(self.note("DATA %s" % payload[:32].hex(), port))
        return lines

    def apply_event(self, ev, port=None):
        if ev["kind"] == "topo":
            self._apply_topo(ev, port)
        elif ev["kind"] == "voltage":
            self._apply_voltage(ev, port)
        self.version += 1

    def _apply_topo(self, ev, port):
        root = ev["root"]
        self.root = root
        root_node = self.nodes.get(root)
        if root_node is None:
            root_node = Node(root, port=port)
            self.nodes[root] = root_node
        root_node.is_root = True
        if not ev.get("snap"):
            root_node.last_seen = time.time()

        node = self.nodes.get(ev["node"])
        if node is None:
            if ev.get("snap") and ev["node"] in self.dropped:
                # The root replays its cache ('T' and every UINJECT_SNAP_TICKS):
                # that is history, so a node evicted for being silent must not
                # come back from it.  Only a live packet brings a node back.
                return
            # No port: a TOPO line arrives on the root's serial link, so it says
            # nothing about where the reporting node is plugged in.  Only the
            # root is really attached to the port the line came from.
            node = self._get_or_create(ev["node"])
        node.parent = ev["parent"]
        node.mv = ev["mv"]
        node.tf = ev["tf"]
        if ev.get("snap"):
            # The root replays its cache on 'T' and every UINJECT_SNAP_TICKS, so a
            # snap=1 line is history: it must not look like the node just spoke.
            # mv/parent/tf above are still refreshed (that is the point of the
            # re-dump), only the age keeps running, so a node that went away
            # fades to red instead of looking alive.
            pass
        else:
            node.last_seen = time.time()
            node.packets += 1
        # A TOPO line always arrives on the root's port, so it says nothing about
        # where the reporting node is plugged in.  Only the root's own port is
        # learned here; a leaf's port comes from the 'V' reply on its own link
        # (_apply_voltage), otherwise the table shows '-'.
        if port and ev["node"] == root and node.port is None:
            node.port = port
        self.packets += 1
        self.last_topo = time.time()

    def _apply_voltage(self, ev, port):
        eui = ev.get("eui")
        if eui is None and port is not None:
            eui = self.port_eui.get(port)
        if eui is None:
            return
        if port:
            self.port_eui[port] = eui
        # a 'V' reply is a live frame too, so it also brings an evicted node back
        node = self._get_or_create(eui)
        if node.port is None:
            node.port = port
        node.mv = ev["mv"]
        node.tf = ev["tf"]
        node.last_seen = time.time()
        if port:
            node.port = port
        if node.eui7 == self.root:
            node.is_root = True

    def apply_uinject(self, frame, port=None):
        """Fold one 'D' frame carrying a uinject trailer into the model.

        The root is taken from the frame's own EUI64 (openserial_printData
        stamps the sender), the reporting node from the uinject trailer, and
        the parent from the trailer's last byte.
        """
        u = frame["uinject"]
        my = frame.get("my_eui64") or b""
        prev = frame.get("prevhop_eui64") or b""
        root = my[7] if len(my) == 8 else self.root
        eui = u.get("eui")
        if eui is None and len(prev) == 8:
            eui = prev[7]
        if root is None or eui is None:
            return None
        now = time.time()
        self.root = root
        root_node = self.nodes.get(root)
        if root_node is None:
            root_node = Node(root)
            self.nodes[root] = root_node
        root_node.is_root = True
        root_node.last_seen = now
        if port:
            root_node.port = port
            self.port_eui[port] = root
        node = self._get_or_create(eui)
        seq = u.get("counter")
        if seq is not None and node.seq is not None:
            delta = ((seq - node.seq + 0x8000) & 0xFFFF) - 0x8000
            if delta == 0:
                self.stale_reports += 1
                self.last_stale = (eui, seq, node.seq)
                return None
            if delta < 0:
                rebooted = (-delta > SEQ_REBOOT_BACK or
                            (now - node.last_seen) > SEQ_QUIET_S)
                if not rebooted:
                    self.stale_reports += 1
                    self.last_stale = (eui, seq, node.seq)
                    return None
        if seq is not None:
            node.seq = seq
        node.parent = u.get("parent")
        node.mv = u.get("mv")
        node.tf = u.get("tf")
        node.last_seen = now
        node.packets += 1
        self.packets += 1
        self.last_topo = now
        self.version += 1
        return node

    def _get_or_create(self, eui7):
        """Return the node, creating it (and noting a comeback) if needed."""
        node = self.nodes.get(eui7)
        if node is not None:
            return node
        node = Node(eui7)
        self.nodes[eui7] = node
        if eui7 in self.dropped:
            # it was evicted for being silent and is reporting again
            del self.dropped[eui7]
            self.returned.append(eui7)
        return node

    # -- derived -----------------------------------------------------------
    def children(self, eui7):
        return [n for n in self.nodes.values() if n.parent == eui7 and not n.is_root]

    def depth_map(self):
        """BFS depth from the root; unlinked nodes get depth 1."""
        depth = {}
        if self.root is None:
            for n in self.nodes.values():
                depth[n.eui7] = 0 if n.is_root else 1
            return depth
        depth[self.root] = 0
        frontier = [self.root]
        while frontier:
            nxt = []
            for e in frontier:
                for child in self.children(e):
                    if child.eui7 not in depth:
                        depth[child.eui7] = depth[e] + 1
                        nxt.append(child.eui7)
            frontier = nxt
        for n in self.nodes.values():
            depth.setdefault(n.eui7, 1)
        return depth

    def prune_offline(self, now=None):
        """Evict nodes silent for longer than offline_after.

        Returns [(eui7, age)] for the ones removed.  The root is never evicted:
        it is the board on the serial port and only relays the other nodes'
        reports, so a mesh where everyone went quiet must not erase the root.
        A child of an evicted node loses its parent link, so it is drawn off the
        tree instead of hanging under a node that is no longer there; the next
        report gives it the real parent again.
        """
        if not self.offline_after or self.offline_after <= 0:
            return []
        now = time.time() if now is None else now
        gone = []
        for eui in list(self.nodes):
            node = self.nodes[eui]
            if node.is_root or eui == self.root:
                continue
            age = now - node.last_seen
            if age <= self.offline_after:
                continue
            gone.append((eui, age))
            del self.nodes[eui]
            self.dropped[eui] = now
            for other in self.nodes.values():
                if other.parent == eui:
                    other.parent = None
        if gone:
            self.version += 1
        return gone

    def bound_port(self, eui7):
        for port, e in self.port_eui.items():
            if e == eui7:
                return port
        return None

    def reset(self):
        self.root = None
        self.nodes.clear()
        self.port_eui.clear()
        self.dropped.clear()
        del self.returned[:]
        self.packets = 0
        self.skip_slots = 0
        self.version += 1
