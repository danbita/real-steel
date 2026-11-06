"""
servo_spec.py - THE one place the sim learns what servos Atom is built from.

Every simulated number that comes from a servo datasheet - torque ceiling,
speed, mass, travel - is read from here by:

    sim36/prep_v11.py                 servo body mass in the link inertias
    sim36/gen_humanoid_v11.py         URDF <limit effort/velocity>, header
    sim36/build_humanoid_v11_scene.py actuator forcerange
    live mimic system/mimic_receiver_v11.py, sim36/mimic_from_video_v11.py,
    dual camera setup/arm_receiver.py, dual camera setup/mimic_receiver_v4.py
                                      slew-limit sanity check

Change servo class HERE, re-run prep -> gen -> build, and the whole chain
follows. Do not re-type these figures anywhere else.

CART A (docs/atom_shopping.html): 3x HTS-35H on the shoulder, 1x HTS-20H on
the elbow, one 3S / 11.1 V rail. The CAD (cad/shoulder_v6) is already cut for
the HTS-35H; this file brings the simulation in line with it.

Datasheet figures (Hiwonder product pages, 11.1 V):
    HTS-35H  35 kg*cm, 0.18 s/60 deg, 64 g, 0-240 deg, 40 x 20 x 40.5 mm body
    HTS-20H  20 kg*cm, 0.18 s/60 deg, 58 g, 0-240 deg, same body family
Hiwonder labels the torque a REFERENCE value, i.e. stall-class, not a rating
for continuous load. As before, the sim uses it as the actuator's hard
ceiling; plan sustained loads at ~1/3 of it.
"""
import math

KGCM = 9.80665 / 100.0          # 1 kg*cm in N*m


class Servo:
    def __init__(self, name, kgcm, s_per_60, mass_kg, travel_deg):
        self.name = name
        self.kgcm = kgcm
        self.stall_nm = kgcm * KGCM                      # actuator ceiling
        self.speed = math.radians(60.0) / s_per_60       # rad/s, no load
        self.mass = mass_kg
        self.travel = math.radians(travel_deg)           # total usable travel

    def __repr__(self):
        return (f"{self.name}: {self.stall_nm:.2f} N*m, {self.speed:.2f} rad/s, "
                f"{self.mass * 1000:.0f} g, {math.degrees(self.travel):.0f} deg")


HTS_35H = Servo("Hiwonder HTS-35H", 35.0, 0.18, 0.064, 240.0)
HTS_20H = Servo("Hiwonder HTS-20H", 20.0, 0.18, 0.058, 240.0)

SHOULDER = HTS_35H              # pitch, roll, twist
ELBOW = HTS_20H

# Live/offline command slew limit. Was justified against the STS3215 (4.8
# rad/s) and the XH540 (3.14 rad/s) in turn; against the HTS-35H's 5.82 rad/s
# no-load it is 52% of no-load speed, which leaves the margin a loaded servo
# needs. Kept at 3.0 so tracking behaviour is unchanged.
MAX_RATE = 3.0

assert MAX_RATE < min(SHOULDER.speed, ELBOW.speed), "slew limit beyond servo speed"


if __name__ == "__main__":
    for s in (SHOULDER, ELBOW):
        print(s)
