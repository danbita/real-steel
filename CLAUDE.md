# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Human motion capture → simulated humanoid → buildable robot ("Atom", ~36 in boxing robot).
MediaPipe PoseLandmarker (Tasks API only — `mp.solutions` was removed in mediapipe ≥ 0.10.31)
extracts landmarks from webcam/video; a retarget/IK step turns them into joint angles that drive a
MuJoCo model. In parallel, the physical shoulder is designed in CadQuery and verified in sim.

There is no build system, test suite, or linter. Everything is standalone Python scripts run
directly. Scripts are run **from the repo root** (paths in docstrings like `python sim/...` are
stale — the folder is `sim36/`).

## Environment
- Python 3.12; libraries: mediapipe, mujoco, opencv, numpy, scipy, matplotlib, pillow. **CadQuery is not installed**, so `cad/*/build_*_cq.py` cannot run without adding it.
- `*.task`, `*.npz`, `*.mp4`, `*.mov` are gitignored (except `media/*.mp4`). The `ref/` and
  `sim/out36/` directories many `sim36/` scripts read/write (e.g. `ref/ref_clip.mp4`,
  `ref/pose_seq.json`) are **not present** in this checkout — those scripts won't run until regenerated.
- On macOS the MuJoCo viewer must run under `mjpython` and cannot share a process with OpenCV
  windows — hence the two-process UDP design. On Windows/Linux plain `python` works.

## Current (v11) pipeline

Folder names contain spaces — quote them.

**Live system** (two processes, UDP 127.0.0.1:5005):
```
python "live mimic system/mimic_receiver_v11.py" [--headless] [--dump out.npz] [--gate] [--ui]
python "live mimic system/pose_sender.py" --landmarks [--source <video|index>] [--log out.npz] [--noshow] [--realtime]
```
- `pose_sender.py` builds a body frame from the person's own hips/shoulders on MediaPipe *world*
  landmarks (camera-angle independent), with One Euro filtering, deadband, visibility hysteresis and
  an L/R swap guard. With `--landmarks` it sends raw world landmarks (what v9+ receivers want);
  without it, it sends 12 joint angles (older receivers).
- `mimic_receiver_v11.py` imports `pose_sender.py` as a module (via `importlib`) to reuse its
  filters/`frame_angles()`, then runs a 4-DOF Gauss-Newton `ArmIK` (pitch, roll, twist, elbow) per
  arm against the real robot kinematics, smooths (`SMOOTH_ALPHA`), slew-limits (`MAX_RATE` = 3.0
  rad/s, matched to the STS3215 servo), optionally applies `StepGate` glitch rejection, and steps
  MuJoCo with position actuators. Many knobs are env vars (`RUN_SECS`, `GATE_STEP`, `GATE_HOLD`, `GATE_MODE`).
- **Known bug:** `mimic_receiver_v11.py` computes `REPO_ROOT = dirname(dirname(HERE))`, which is one
  level above the repo, so `SCENE` points at `../sim36/humanoid_v11_scene.xml`. It should be
  `dirname(HERE)` (as every `sim36/` script does).

**Model generation chain** (CAD → sim). Generated files are outputs — edit the generator, not the output:
1. `cad/shoulder_v6/build_shoulder_v6_cq.py` → per-part STEP/STL + `check_meta.json` (left);
   `cad/shoulder_v6/right/_build_right.py` → mirrored right module.
2. `python sim36/interference_check.py cad/shoulder_v6` — KD-tree sweep of every moving part pair
   through joint travel, kinematics read from `check_meta.json`. This is the shoulder's pass/fail gate.
3. `python sim36/prep_v11.py` → scaled meshes in `live mimic system/meshes36d/` + mass/inertia in
   `sim36/v11_links.json`.
4. `python sim36/gen_humanoid_v11.py` → `live mimic system/humanoid_v11_atom.urdf`.
5. `python sim36/build_humanoid_v11_scene.py` → `sim36/humanoid_v11_scene.xml` (what everything loads).

**Verification / offline tools** in `sim36/` (all reuse the shipping receiver/sender code rather than reimplementing):
- `live_loopback_test.py` — replays recorded landmarks over real UDP into the real receiver; the
  closest thing to an end-to-end test.
- `noise_lib.py` / `noise_experiments.py [--quick]` / `noise_live_check.py` / `noise_summary.py` —
  glitch-injection studies for `MAX_RATE` and `StepGate`.
- `mimic_from_video_v11.py`, `extract_landmarks.py`, `compose_sidebyside_*.py`, `live_diag_render.py` —
  offline video mimic and side-by-side renders. `ATOM_CLIP=<path>` reruns against another clip with
  suffixed outputs.

**Docs renders**: `tools/render/*.py` produce `docs/img_v6/` images/GIFs for the assembly manual
(`docs/assembly/index.html`). `mjanim_v9.py` `chdir`s to the repo root and exec's constants straight
out of the CAD build script, so CAD is the single source of truth.

## Versioning conventions

Files are versioned by suffix and old versions are kept side by side (`mimic_receiver_v4..v11`,
`humanoid_v3..v11_atom.urdf`, `meshes36`/`36b`/`36c`/`36d` = shoulder v3/v4/v5/v6). Work on the
highest version unless told otherwise. Note version numbers of receiver, URDF, generator and shoulder
don't line up (v11 humanoid uses shoulder_v6; some docstrings still say v7/v9). `random files/`,
`offline video pipeline/`, `dual camera setup/` and `armRosFile/` (ROS package) are earlier,
mostly superseded work. The README's repo-layout and usage sections describe that older flat layout.

## Joint conventions

- Sender angles: `*_flex` 0 = hanging, + = forward; `*_abd` 0 = hanging, + = outward; elbow/knee 0 = straight.
- Robot shoulder: `*_shoulder_pitch` (Y, + forward), `*_shoulder_roll` (X), `*_shoulder_twist`, `*_elbow`.
  The CAD module's roll neutral is arm-UP; the module is mounted inverted in the robot so roll 0 is arm hanging.
- Left/right modules are genuinely mirrored and both mount with rpy = 0 — no axis-flip hacks.
- Dumped `qpos`/`cmd` arrays are labelled by joint name; write back through
  `model.jnt_qposadr[mj_name2id(...)]`, never positionally.

## Hardware design rules

`docs/design_principles.md` is binding for any CAD, animation, or sourcing work. The essentials:
only measured, purchasable parts (the servo is the Feetech **STS3215**, 45.4×24.8×36.9 mm, spline
12.5 mm off centre; the README's "HTS-35H" mention is out of date with the CAD); no fictional geometry;
every screw lands in a real bore with exact length and threads in metal (inserts/nuts); assembly
corridors and order are verified geometrically; check full range of motion on both hands; and
visually inspect rendered frames — numeric checks passing is not enough. `cad/shoulder_v6/NOTES.md`
records the shoulder's current design rationale and status.
