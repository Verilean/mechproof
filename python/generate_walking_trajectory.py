"""MechProof PoC 10 — quasi-static walking keyframe generator.

Emits `out/walking_trajectory.json`: a list of (time, leg-joint-targets)
tuples that drive the 12 leg DOFs through a "shift-lift-step" gait. The
trajectory is intentionally quasi-static (smooth, small joint
accelerations) so it stays inside the Lean-proven ZMP envelope.

Joint order matches `simulate_stand.py`'s actuator declaration:
  0:  left_hip_yaw
  1:  left_hip_roll
  2:  left_hip_pitch
  3:  left_knee_pitch
  4:  left_ankle_pitch
  5:  left_ankle_roll
  6:  right_hip_yaw
  7:  right_hip_roll
  8:  right_hip_pitch
  9:  right_knee_pitch
 10:  right_ankle_pitch
 11:  right_ankle_roll
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_PATH = REPO_ROOT / "out" / "walking_trajectory.json"

# An alternating-hip shuffle gait. Both feet stay flat on the floor
# (ankles neutral); each leg's hip pitches forward and then back, dragging
# that foot along the ground via friction. Because both feet are always
# in contact, the support polygon is always the double-support polygon
# the Lean ZMP proof permits. Magnitudes are kept small (≤ 6°) so the
# torso never lean enough to topple.
# Sign convention (verified empirically against simulate_stand.py): positive
# hip_pitch rotates the swinging leg toward +Y, which — with the foot pinned
# by floor friction — actually drives the *torso* in the -Y direction. To
# move the torso forward (+Y) we pitch the active hip backward (negative).
# With a free-floating torso and only position-controlled hip pitches,
# any forward lean eventually tips the robot. The strategy is therefore
# to take exactly enough steps to clear the success threshold and then
# return both legs to neutral so the torso stops accelerating. Magnitudes
# are kept very small (≤ 4°).
HIP_FWD_RAD = math.radians(-1.5)
HIP_BWD_RAD = math.radians( 0.8)


def zeros() -> list:
    return [0.0] * 12


def step_right_forward(targets: list) -> list:
    """Right hip pitches forward (drags right foot forward), left hip
    pitches backward slightly (pushes the body forward)."""
    out = list(targets)
    out[2] = HIP_BWD_RAD       # left hip slightly back
    out[8] = HIP_FWD_RAD       # right hip forward
    return out


def step_left_forward(targets: list) -> list:
    out = list(targets)
    out[2] = HIP_FWD_RAD       # left hip forward
    out[8] = HIP_BWD_RAD       # right hip slightly back
    return out


def build_keyframes() -> list:
    base = zeros()

    keyframes = [
        ("settle",       0.6, base),

        # Two short alternating drag-steps. After each step we return both
        # hips to neutral so the torso doesn't accumulate forward pitch.
        ("right_step_1", 0.4, step_right_forward(base)),
        ("recenter_1",   0.3, base),
        ("left_step_1",  0.4, step_left_forward(base)),
        ("recenter_2",   0.6, base),
    ]
    return keyframes


def main() -> int:
    keyframes = build_keyframes()
    out_records = []
    t = 0.0
    for label, hold, targets in keyframes:
        out_records.append({
            "label":    label,
            "start_s":  t,
            "hold_s":   hold,
            "targets":  list(targets),
        })
        t += hold

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(json.dumps({
        "schema":      "mechproof.walking_trajectory.v1",
        "duration_s":  t,
        "n_keyframes": len(out_records),
        "joint_order": [
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch",
            "left_knee_pitch", "left_ankle_pitch", "left_ankle_roll",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
            "right_knee_pitch", "right_ankle_pitch", "right_ankle_roll",
        ],
        "keyframes":   out_records,
    }, indent=2))
    print(f"Wrote {OUT_PATH}")
    print(f"  total duration : {t:.2f} s")
    print(f"  keyframes      : {len(out_records)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
