"""
operator_console.py - small debugging window for the live mimic system.

    python "live mimic system/operator_console.py"

Talks only to mimic_receiver_v11.py (telemetry in on UDP 5006, commands out on
5007, see console_link.py), so it works the same with any pose sender: 1, 2 or
3 cameras. Start order doesn't matter; closing it never affects the robot.
How to use it: operator_console_GUIDE.txt in the repo root.

Needs:  pip install dearpygui
"""
import collections
import math
import os
import time

import dearpygui.dearpygui as dpg

import importlib.util as _il
HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
_spec = _il.spec_from_file_location(
    "console_link", os.path.join(HERE, "console_link.py"))
cl = _il.module_from_spec(_spec); _spec.loader.exec_module(cl)

ARM = [f"{s}_{j}" for s in "LR"
       for j in ("shoulder_pitch", "shoulder_roll", "shoulder_twist", "elbow")]
HIST_S = 10.0           # seconds of history in the plot
NO_DATA_S = 1.0         # sender packet older than this = "no data"

LABEL_W, BAR_W, ROW_H = 270, 300, 28
RED = (230, 70, 60)
GREEN = (70, 200, 90)
AMBER = (240, 170, 40)
GREY = (150, 150, 150)
WHITE = (235, 235, 235)


class Console:
    def __init__(self):
        self.link = cl.ReceiverLink()
        self.hist = {j: collections.deque() for j in ARM}   # (t, target, qpos)
        self.pkts = collections.deque()                     # (wall t, npkt)
        self.was_connected = False

    # -- commands ------------------------------------------------------------
    def toggle_stop(self, *_):
        tm = self.link.latest
        if not self.link.connected:
            return
        # STOP is idempotent, so a double press can only ever stop
        self.link.send("resume" if tm.get("stopped") else "stop")

    def send_rate(self, *_):
        self.link.send("set", max_rate=float(dpg.get_value("rate")))

    def send_gate(self, *_):
        self.link.send("set", gate=bool(dpg.get_value("gate")))

    def toggle_record(self, *_):
        tm = self.link.latest
        if not self.link.connected:
            return
        if tm.get("recording"):
            self.link.send("record_stop")
        else:
            name = time.strftime("%Y%m%d-%H%M%S") + ".npz"
            self.link.send("record_start",
                           path=os.path.join(REPO_ROOT, "runs", name))

    # -- per frame -----------------------------------------------------------
    def update(self):
        now = time.time()
        for m in self.link.poll():
            self.pkts.append((now, m.get("npkt", 0)))
            self._add_history(m)
        while self.pkts and now - self.pkts[0][0] > 1.0:
            self.pkts.popleft()

        connected = self.link.connected
        tm = self.link.latest if connected else None
        if connected and not self.was_connected:
            # (re)connected: show the receiver's real settings
            dpg.set_value("rate", float(tm.get("max_rate", 3.0)))
            dpg.set_value("gate", bool(tm.get("gate_on", False)))
        self.was_connected = connected

        self._draw_stop(tm)
        self._draw_link(tm)
        self._draw_bars(tm)
        self._draw_plot(tm)
        self._draw_record(tm)

    def _add_history(self, m):
        t = float(m.get("t", 0.0))
        h0 = self.hist[ARM[0]]
        if h0 and t < h0[-1][0]:              # receiver restarted
            for h in self.hist.values():
                h.clear()
        idx = {n: i for i, n in enumerate(m.get("joints", []))}
        for j in ARM:
            if j in idx:
                i = idx[j]
                self.hist[j].append((t, m["target"][i], m["qpos"][i]))
                while self.hist[j][0][0] < t - HIST_S:
                    self.hist[j].popleft()

    def _draw_stop(self, tm):
        if tm is None:
            dpg.set_item_label("stop", "STOP   (receiver not connected)")
            dpg.bind_item_theme("stop", "th_grey")
        elif tm.get("stopped"):
            dpg.set_item_label("stop", "RESUME   (Space)  -  robot is stopped")
            dpg.bind_item_theme("stop", "th_green")
        else:
            dpg.set_item_label("stop", "STOP   (Space)")
            dpg.bind_item_theme("stop", "th_red")

    def _draw_link(self, tm):
        if tm is None:
            dpg.set_value("link_sender", "sender: -")
            dpg.configure_item("link_sender", color=GREY)
            dpg.set_value("link_receiver", "receiver: not connected")
            dpg.configure_item("link_receiver", color=RED)
            return
        dpg.set_value("link_receiver", "receiver: OK")
        dpg.configure_item("link_receiver", color=GREEN)
        age = tm.get("pkt_age")
        if age is None or age > NO_DATA_S:
            dpg.set_value("link_sender", "sender: no data")
            dpg.configure_item("link_sender", color=RED)
            return
        rate = 0.0
        if len(self.pkts) >= 2:
            (t0, n0), (t1, n1) = self.pkts[0], self.pkts[-1]
            if t1 > t0:
                rate = (n1 - n0) / (t1 - t0)
        dpg.set_value("link_sender",
                      f"sender: {tm.get('pkt_kind')}  |  {rate:.0f} pkt/s  |  "
                      f"last {age * 1000:.0f} ms ago")
        dpg.configure_item("link_sender", color=GREEN)

    def _draw_bars(self, tm):
        dpg.delete_item("bars", children_only=True)
        if tm is None:
            dpg.draw_text((0, 4), "waiting for receiver...", color=GREY,
                          size=16, parent="bars")
            return
        idx = {n: i for i, n in enumerate(tm["joints"])}
        limited = set(tm.get("rate_limited", []))
        gated = set(tm.get("gated", []))
        rows = [j for j in ARM if j in idx]
        for r, j in enumerate(rows):
            i = idx[j]
            lo, hi = tm["limits"][i]
            tgt, act = tm["target"][i], tm["qpos"][i]
            y = r * ROW_H + 4
            yc = y + ROW_H / 2 - 2

            def x_of(v):
                f = (v - lo) / (hi - lo) if hi > lo else 0.5
                return LABEL_W + min(1.0, max(0.0, f)) * BAR_W

            if j in gated:
                band = RED
            elif j in limited:
                band = AMBER
            else:
                band = None
            if band:
                dpg.draw_rectangle((0, y), (LABEL_W + BAR_W + 12, y + ROW_H - 4),
                                   color=band, fill=(*band, 60), parent="bars")
            dpg.draw_text((6, y + 4),
                          f"{j.replace('_', ' '):17s}{math.degrees(tgt):+5.0f}"
                          f"{math.degrees(act):+6.0f}",
                          color=WHITE, size=15, parent="bars")
            dpg.draw_rectangle((LABEL_W, yc - 4), (LABEL_W + BAR_W, yc + 4),
                               color=GREY, fill=(70, 70, 70), parent="bars")
            if lo < 0 < hi:                               # zero tick
                xz = x_of(0.0)
                dpg.draw_line((xz, yc - 8), (xz, yc + 8), color=GREY,
                              parent="bars")
            dpg.draw_circle((x_of(tgt), yc), 7, color=WHITE, thickness=2,
                            parent="bars")
            dpg.draw_circle((x_of(act), yc), 4, color=(90, 170, 255),
                            fill=(90, 170, 255), parent="bars")

    def _draw_plot(self, tm):
        j = dpg.get_value("plot_joint")
        h = self.hist.get(j)
        if not h:
            dpg.set_value("s_target", [[], []])
            dpg.set_value("s_actual", [[], []])
            return
        tnow = h[-1][0]
        xs = [t - tnow for t, _, _ in h]
        dpg.set_value("s_target", [xs, [math.degrees(v) for _, v, _ in h]])
        dpg.set_value("s_actual", [xs, [math.degrees(v) for _, _, v in h]])
        if tm is not None and j in tm["joints"]:
            lo, hi = tm["limits"][tm["joints"].index(j)]
            pad = 0.05 * (hi - lo)
            dpg.set_axis_limits("y_axis", math.degrees(lo - pad),
                                math.degrees(hi + pad))

    def _draw_record(self, tm):
        rec = tm.get("recording") if tm else None
        if rec:
            dpg.set_item_label("record",
                               f"Stop recording   ({os.path.basename(rec)})")
            dpg.bind_item_theme("record", "th_red")
        else:
            dpg.set_item_label("record", "Record")
            dpg.bind_item_theme("record", 0)


def _button_theme(tag, rgb):
    with dpg.theme(tag=tag):
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (*rgb, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,
                                tuple(min(255, c + 25) for c in rgb) + (255,))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,
                                tuple(max(0, c - 25) for c in rgb) + (255,))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))


def main():
    dpg.create_context()
    con = Console()

    _button_theme("th_red", (190, 40, 35))
    _button_theme("th_green", (40, 150, 60))
    _button_theme("th_grey", (90, 90, 90))

    width = LABEL_W + BAR_W + 40
    with dpg.window(tag="main"):
        dpg.add_button(tag="stop", label="STOP", width=-1, height=64,
                       callback=con.toggle_stop)
        with dpg.group(horizontal=True):
            dpg.add_text("sender: -", tag="link_sender", color=GREY)
            dpg.add_text("   ")
            dpg.add_text("receiver: not connected", tag="link_receiver",
                         color=RED)
        dpg.add_separator()

        dpg.add_text("Joints     white ring = target (what the pose asks for)"
                     "   blue dot = actual", color=GREY)
        dpg.add_text("           amber row = speed-limited   red row = glitch "
                     "filter rejected a jump", color=GREY)
        dpg.add_text("                  deg: target actual", color=GREY)
        dpg.add_drawlist(tag="bars", width=width, height=len(ARM) * ROW_H + 8)
        dpg.add_separator()

        dpg.add_combo(ARM, tag="plot_joint", default_value="L_elbow",
                      label="joint to plot", width=200)
        with dpg.plot(height=220, width=-1):
            dpg.add_plot_legend()
            dpg.add_plot_axis(dpg.mvXAxis, label="seconds ago", tag="x_axis")
            dpg.set_axis_limits("x_axis", -HIST_S, 0.0)
            with dpg.plot_axis(dpg.mvYAxis, label="deg", tag="y_axis"):
                dpg.add_line_series([], [], label="target", tag="s_target")
                dpg.add_line_series([], [], label="actual", tag="s_actual")
        dpg.add_separator()

        dpg.add_slider_float(tag="rate", label="Max speed (rad/s)",
                             default_value=3.0, min_value=cl.MAX_RATE_MIN,
                             max_value=cl.MAX_RATE_MAX, format="%.2f",
                             width=260)
        dpg.add_checkbox(tag="gate", label="Glitch filter",
                         callback=con.send_gate)
        dpg.add_button(tag="record", label="Record", width=260,
                       callback=con.toggle_record)

    # send the speed once, when the slider is released, not on every pixel
    with dpg.item_handler_registry(tag="rate_handlers"):
        dpg.add_item_deactivated_after_edit_handler(callback=con.send_rate)
    dpg.bind_item_handler_registry("rate", "rate_handlers")

    with dpg.handler_registry():
        dpg.add_key_press_handler(dpg.mvKey_Spacebar, callback=con.toggle_stop)

    dpg.create_viewport(title="Atom operator console", width=width + 40,
                        height=780)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("main", True)
    while dpg.is_dearpygui_running():
        con.update()
        dpg.render_dearpygui_frame()
    dpg.destroy_context()


if __name__ == "__main__":
    main()
