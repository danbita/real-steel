"""
servo_test.py - move a physical servo from the computer, via the Uno running
hardware/uno_servo/uno_servo.ino.

    pip install pyserial
    python hardware/servo_test.py              # interactive
    python hardware/servo_test.py --sweep      # gentle back-and-forth on ch 0
    python hardware/servo_test.py --list       # show serial ports

Interactive commands:
    90          move ch 0 to 90 deg
    1 120       move ch 1 to 120 deg
    sweep [ch]  60 -> 120 -> 90
    off         servos go limp
    q           quit (also goes limp)

Angles are clamped to SAFE_LO..SAFE_HI (30..150) so a mounted servo can't be
driven into its end stops on the first try. --wide allows 0..180.
"""
import argparse
import sys
import time

import serial
import serial.tools.list_ports

BAUD = 115200
SAFE_LO, SAFE_HI = 30.0, 150.0
HINTS = ("usbmodem", "usbserial", "wchusbserial", "ch340", "arduino", "ttyacm", "ttyusb")


def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        text = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
        if any(h in text for h in HINTS):
            return p.device
    return None


class Board:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.5)
        time.sleep(2.0)                     # opening the port resets the Uno
        self.ser.reset_input_buffer()
        reply = self.cmd("?")
        if not reply.startswith("ATOM-SERVO"):
            raise RuntimeError(f"no answer from the firmware (got {reply!r}) - "
                               "is uno_servo.ino uploaded and Serial Monitor closed?")
        print("board:", reply)

    def cmd(self, line):
        self.ser.write((line + "\n").encode())
        return self.ser.readline().decode(errors="replace").strip()

    def move(self, ch, deg):
        r = self.cmd(f"M {ch} {deg:.1f}")
        print(" ", r)
        return r.startswith("OK")

    def off(self):
        print(" ", self.cmd("OFF"))

    def close(self):
        try:
            self.off()
        finally:
            self.ser.close()


def sweep(b, ch, clamp):
    for deg in (60, 120, 60, 120, 90):
        b.move(ch, clamp(deg))
        time.sleep(1.2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    ap.add_argument("--sweep", action="store_true", help="run one sweep on --ch and exit")
    ap.add_argument("--ch", type=int, default=0)
    ap.add_argument("--wide", action="store_true", help="allow 0..180 instead of 30..150")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    a = ap.parse_args()

    if a.list:
        for p in serial.tools.list_ports.comports():
            print(f"{p.device:35s} {p.description}")
        return

    port = a.port or find_port()
    if not port:
        sys.exit("No Uno found. Plug it in, then try --list. (Elegoo boards use a CH340 "
                 "chip; it shows up as /dev/cu.usbserial-* or /dev/cu.wchusbserial*.)")
    print("port:", port)

    lo, hi = (0.0, 180.0) if a.wide else (SAFE_LO, SAFE_HI)

    def clamp(d):
        c = max(lo, min(hi, d))
        if c != d:
            print(f"  (clamped {d} -> {c}; use --wide for full range)")
        return c

    b = Board(port)
    try:
        if a.sweep:
            sweep(b, a.ch, clamp)
            return
        print("type an angle (e.g. 90), '<ch> <deg>', 'sweep', 'off', or 'q'")
        while True:
            try:
                line = input("> ").strip().lower()
            except EOFError:
                break
            if not line:
                continue
            if line in ("q", "quit", "exit"):
                break
            parts = line.split()
            try:
                if parts[0] == "off":
                    b.off()
                elif parts[0] == "sweep":
                    sweep(b, int(parts[1]) if len(parts) > 1 else a.ch, clamp)
                elif len(parts) == 1:
                    b.move(a.ch, clamp(float(parts[0])))
                else:
                    b.move(int(parts[0]), clamp(float(parts[1])))
            except ValueError:
                print("  ? try: 90   |   1 120   |   sweep   |   off   |   q")
    except KeyboardInterrupt:
        pass
    finally:
        b.close()
        print("servos off, port closed")


if __name__ == "__main__":
    main()
