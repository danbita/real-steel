"""
console_link.py - the wire between mimic_receiver_v11.py and operator_console.py.

    receiver --telemetry (UDP 5006, ~30 Hz)--> console
    receiver <--commands (UDP 5007)----------- console

Stdlib only. One JSON object per datagram. Nothing here knows which pose
sender is running: the console only ever talks to the receiver.

WINDOWS NOTE: sending UDP to a local port nobody has bound makes Windows queue
an ICMP "port unreachable", which surfaces as ConnectionResetError on that
SAME socket's next call. So every class here keeps send and receive on
separate sockets and swallows OSError on send - a console that isn't running
must never disturb the receiver (whose landmark loop treats OSError as "no
more packets").
"""
import json
import socket
import time

HOST = "127.0.0.1"
TELEM_PORT = 5006       # receiver -> console
CMD_PORT = 5007         # console -> receiver
MAX_DATAGRAM = 8192     # every message we send must fit in one of these

MAX_RATE_MIN, MAX_RATE_MAX = 0.1, 3.0   # rad/s; 3.0 = STS3215 safe limit
CONNECTED_S = 1.0       # telemetry older than this = receiver not connected


def encode(msg):
    return json.dumps(msg, separators=(",", ":")).encode()


def decode(blob):
    try:
        msg = json.loads(blob.decode())
    except (ValueError, UnicodeDecodeError):
        return None
    return msg if isinstance(msg, dict) else None


class _Sender:
    """Fire-and-forget UDP to one address; never raises."""

    def __init__(self, port):
        self.addr = (HOST, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, msg):
        try:
            self.sock.sendto(encode(msg), self.addr)
        except OSError:
            pass                    # nobody listening: that's fine


class _Listener:
    """Non-blocking UDP receiver. If the port is taken, it disables itself
    with a warning instead of crashing the program that owns it."""

    def __init__(self, port, what):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((HOST, port))
            self.sock.setblocking(False)
        except OSError as e:
            print(f"console_link: cannot listen for {what} on {HOST}:{port} "
                  f"({e}); continuing without it.")
            self.sock.close()
            self.sock = None

    def poll(self):
        """All messages waiting right now, oldest first."""
        out = []
        while self.sock is not None:
            try:
                blob, _ = self.sock.recvfrom(65535)
            except OSError:         # BlockingIOError = empty; others: give up
                break
            msg = decode(blob)
            if msg is not None:
                out.append(msg)
        return out


# ---------------------------------------------------------------------------
# receiver side
# ---------------------------------------------------------------------------
class TelemetryPublisher(_Sender):
    def __init__(self):
        super().__init__(TELEM_PORT)


class CommandListener(_Listener):
    def __init__(self):
        super().__init__(CMD_PORT, "console commands")


class ControlState:
    """What the console can change on the receiver, and how each command
    changes it. Kept free of MuJoCo so it can be tested on its own.

    apply(cmd) returns a list of events the receiver must act on:
        ("stopped", bool)       freeze / unfreeze
        ("max_rate", float)     rad/s, already clamped
        ("gate", bool)          create / drop the StepGate
        ("record_start", path)
        ("record_stop", path)
    """

    def __init__(self, max_rate, gate_on, recording=None):
        self.stopped = False
        self.max_rate = float(max_rate)
        self.gate_on = bool(gate_on)
        self.recording = recording      # path being recorded to, or None

    def apply(self, cmd):
        c = cmd.get("cmd")
        if c == "stop" and not self.stopped:
            self.stopped = True
            return [("stopped", True)]
        if c == "resume" and self.stopped:
            self.stopped = False
            return [("stopped", False)]
        if c == "set":
            ev = []
            if "max_rate" in cmd:
                try:
                    v = float(cmd["max_rate"])
                except (TypeError, ValueError):
                    v = float("nan")
                if v == v:                              # not NaN
                    v = min(MAX_RATE_MAX, max(MAX_RATE_MIN, v))
                    if v != self.max_rate:
                        self.max_rate = v
                        ev.append(("max_rate", v))
            if "gate" in cmd and bool(cmd["gate"]) != self.gate_on:
                self.gate_on = bool(cmd["gate"])
                ev.append(("gate", self.gate_on))
            return ev
        if c == "record_start" and self.recording is None and cmd.get("path"):
            self.recording = str(cmd["path"])
            return [("record_start", self.recording)]
        if c == "record_stop" and self.recording is not None:
            path, self.recording = self.recording, None
            return [("record_stop", path)]
        return []


# ---------------------------------------------------------------------------
# console side
# ---------------------------------------------------------------------------
class ReceiverLink:
    """The console's view of the receiver: latest telemetry + send commands."""

    def __init__(self):
        self._rx = _Listener(TELEM_PORT, "receiver telemetry")
        self._tx = _Sender(CMD_PORT)
        self.latest = None
        self.stamp = 0.0            # time.time() the latest telemetry arrived

    def poll(self):
        """Read everything waiting. Returns the new telemetry messages."""
        got = [m for m in self._rx.poll() if m.get("type") == "telemetry"]
        if got:
            self.latest = got[-1]
            self.stamp = time.time()
        return got

    @property
    def connected(self):
        return self.latest is not None and time.time() - self.stamp < CONNECTED_S

    def send(self, cmd, **kw):
        self._tx.send({"cmd": cmd, **kw})
