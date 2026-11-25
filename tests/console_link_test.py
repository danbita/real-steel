"""
console_link_test.py - checks for the operator console wire (console_link.py).

    python tests/console_link_test.py

Stdlib only (no MuJoCo, no Dear PyGui). Checks the command -> receiver-state
rules in ControlState, that a full-size telemetry message fits one datagram,
and a real UDP round trip on the console ports (skipped if the ports are in
use, e.g. a receiver or console is running).
"""
import importlib.util as il
import os
import socket
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
_spec = il.spec_from_file_location(
    "console_link", os.path.join(ROOT, "live mimic system", "console_link.py"))
cl = il.module_from_spec(_spec); _spec.loader.exec_module(cl)

FAILS = []


def check(name, cond):
    print(f"{'PASS' if cond else 'FAIL'}  {name}")
    if not cond:
        FAILS.append(name)


def test_stop_resume():
    s = cl.ControlState(3.0, False)
    check("stop -> stopped event", s.apply({"cmd": "stop"}) == [("stopped", True)])
    check("stop again -> no event", s.apply({"cmd": "stop"}) == [])
    check("resume -> event", s.apply({"cmd": "resume"}) == [("stopped", False)])
    check("resume again -> no event", s.apply({"cmd": "resume"}) == [])


def test_max_rate():
    s = cl.ControlState(3.0, False)
    check("max_rate set", s.apply({"cmd": "set", "max_rate": 1.5}) == [("max_rate", 1.5)])
    check("max_rate same -> no event", s.apply({"cmd": "set", "max_rate": 1.5}) == [])
    check("max_rate clamped high",
          s.apply({"cmd": "set", "max_rate": 99}) == [("max_rate", cl.MAX_RATE_MAX)])
    check("max_rate clamped low",
          s.apply({"cmd": "set", "max_rate": -1}) == [("max_rate", cl.MAX_RATE_MIN)])
    check("max_rate garbage ignored",
          s.apply({"cmd": "set", "max_rate": "fast"}) == [])
    check("max_rate NaN ignored",
          s.apply({"cmd": "set", "max_rate": float("nan")}) == []
          and s.max_rate == cl.MAX_RATE_MIN)


def test_gate():
    s = cl.ControlState(3.0, False)
    check("gate on", s.apply({"cmd": "set", "gate": True}) == [("gate", True)])
    check("gate on again -> no event", s.apply({"cmd": "set", "gate": True}) == [])
    check("gate off", s.apply({"cmd": "set", "gate": False}) == [("gate", False)])
    check("rate + gate in one command",
          s.apply({"cmd": "set", "max_rate": 2.0, "gate": True})
          == [("max_rate", 2.0), ("gate", True)])


def test_record():
    s = cl.ControlState(3.0, False)
    check("record_start needs a path", s.apply({"cmd": "record_start"}) == [])
    check("record_start", s.apply({"cmd": "record_start", "path": "a.npz"})
          == [("record_start", "a.npz")])
    check("second record_start ignored",
          s.apply({"cmd": "record_start", "path": "b.npz"}) == [])
    check("record_stop returns the path",
          s.apply({"cmd": "record_stop"}) == [("record_stop", "a.npz")])
    check("record_stop when idle -> no event", s.apply({"cmd": "record_stop"}) == [])
    d = cl.ControlState(3.0, False, recording="dump.npz")
    check("--dump counts as recording",
          d.apply({"cmd": "record_start", "path": "x.npz"}) == [])


def test_unknown():
    s = cl.ControlState(3.0, False)
    check("unknown command ignored", s.apply({"cmd": "explode"}) == [])
    check("no cmd key ignored", s.apply({}) == [])
    check("decode rejects junk", cl.decode(b"\xff\xfe") is None
          and cl.decode(b"[1,2]") is None and cl.decode(b"{") is None)


def test_size():
    # worst case: every joint in the v11 scene, long names, all flags on
    joints = [f"{s}_{j}" for s in "LR" for j in
              ("shoulder_pitch", "shoulder_roll", "shoulder_twist", "elbow",
               "hip_flex", "hip_abd", "knee")]
    msg = {"type": "telemetry", "t": 12345.678, "joints": joints,
           "limits": [[-3.1416, 3.1416]] * len(joints),
           "target": [-1.2345] * len(joints), "qpos": [-1.2345] * len(joints),
           "rate_limited": joints, "gated": joints, "npkt": 10 ** 9,
           "pkt_age": 123.456, "pkt_kind": "landmarks", "stopped": True,
           "max_rate": 3.0, "gate_on": True,
           "recording": "C:/" + "x" * 200 + "/runs/20261120-120000.npz"}
    n = len(cl.encode(msg))
    check(f"worst-case telemetry fits one datagram ({n} B < {cl.MAX_DATAGRAM})",
          n < cl.MAX_DATAGRAM)
    check("telemetry round-trips", cl.decode(cl.encode(msg)) == msg)


def test_udp_round_trip():
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    busy = False
    for port in (cl.TELEM_PORT, cl.CMD_PORT):
        try:
            probe.bind((cl.HOST, port))
        except OSError:
            busy = True
        probe.close()
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.close()
    if busy:
        print("SKIP  UDP round trip (ports 5006/5007 in use - receiver or console running?)")
        return
    # a command sent before anyone listens must not raise (Windows ICMP case)
    link = cl.ReceiverLink()
    link.send("stop")
    link.send("stop")
    check("send with no receiver does not raise", True)

    listener, pub = cl.CommandListener(), cl.TelemetryPublisher()
    link.send("set", max_rate=2.0)
    pub.send({"type": "telemetry", "t": 1.0, "stopped": False})
    pub.send({"type": "other"})
    time.sleep(0.05)
    got = listener.poll()
    check("command arrives at receiver", {"cmd": "set", "max_rate": 2.0} in got)
    link.poll()
    check("telemetry arrives at console",
          link.latest is not None and link.latest.get("t") == 1.0)
    check("console reports connected", link.connected)
    check("non-telemetry ignored", link.latest.get("type") == "telemetry")


def main():
    for t in (test_stop_resume, test_max_rate, test_gate, test_record,
              test_unknown, test_size, test_udp_round_trip):
        t()
    print(f"\n{'all ok' if not FAILS else f'{len(FAILS)} FAILED'}")
    raise SystemExit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
