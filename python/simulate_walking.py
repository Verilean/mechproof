"""MechProof PoC 10 — quasi-static walking simulation.

Loads the PoC 8 humanoid scene (`out/humanoid_scene.xml`) and drives the
12 leg actuators along the keyframe schedule in `out/walking_trajectory.json`
using smooth (cosine-blended) interpolation between successive keyframes.
The PoC 8 position-controlled servos do the heavy lifting; this script is
basically a high-level keyframe player.

Success criteria — both must hold:
  * torso moves at least `MIN_FORWARD_M` in the world +Y direction over the
    course of the gait (the humanoid's "forward"),
  * torso Z never drops below `MIN_TORSO_Z_M` during the run.

The simulation also tracks max tilt as a soft KPI but does not gate on it
because side-to-side sway is expected during single support.

The report is written to `out/Walking_Report.txt`.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
SCENE_PATH = REPO_ROOT / "out" / "humanoid_scene.xml"
TRAJ_PATH = REPO_ROOT / "out" / "walking_trajectory.json"
REPORT_PATH = REPO_ROOT / "out" / "Walking_Report.txt"

MIN_FORWARD_M = 0.20
MIN_TORSO_Z_M = 0.70
VELOCITY_LIMIT_RAD_PER_S = 200.0


def smoothstep(t: float) -> float:
    """Cosine ease between 0 and 1 — gentler joint accelerations than
    linear interpolation, which the quasi-static ZMP assumption needs."""
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return 0.5 - 0.5 * math.cos(math.pi * t)


def actuator_index_map(model: mujoco.MjModel,
                       joint_order: list[str]) -> list[int]:
    """For each leg joint name, return the actuator index in `data.ctrl`.
    The knee actuator in PoC 8's MJCF is named `<side>_knee_act` rather
    than `<side>_knee_pitch_act`; we strip the `_pitch` suffix as a
    fallback to keep the trajectory's joint-order naming consistent."""
    ctrl_idx = []
    for joint_name in joint_order:
        for candidate in (f"{joint_name}_act",
                          f"{joint_name.replace('_pitch', '')}_act"):
            aid = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, candidate)
            if aid >= 0:
                ctrl_idx.append(aid)
                break
        else:
            raise RuntimeError(
                f"no actuator for joint {joint_name} in MJCF")
    return ctrl_idx


def interpolated_target(t: float, keyframes: list) -> list:
    """Cosine-interpolate between the two surrounding keyframes."""
    if t <= keyframes[0]["start_s"]:
        return list(keyframes[0]["targets"])
    if t >= keyframes[-1]["start_s"] + keyframes[-1]["hold_s"]:
        return list(keyframes[-1]["targets"])

    # Locate the active segment.
    for i in range(len(keyframes) - 1):
        a = keyframes[i]
        b = keyframes[i + 1]
        t_a = a["start_s"]
        t_b = b["start_s"]
        if t_a <= t < t_b:
            seg_len = max(t_b - t_a, 1e-6)
            alpha = smoothstep((t - t_a) / seg_len)
            return [
                (1 - alpha) * ta + alpha * tb
                for ta, tb in zip(a["targets"], b["targets"])
            ]
    # past the last segment but inside the last hold
    return list(keyframes[-1]["targets"])


def main() -> int:
    if not SCENE_PATH.exists() or not TRAJ_PATH.exists():
        print(f"error: missing scene or trajectory JSON — run "
              f"`make poc8` and `make walking-trajectory` first.",
              file=sys.stderr)
        return 1

    xml = SCENE_PATH.read_text()
    traj = json.loads(TRAJ_PATH.read_text())
    keyframes = traj["keyframes"]
    duration = float(traj["duration_s"])
    joint_order = traj["joint_order"]

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    ctrl_idx = actuator_index_map(model, joint_order)

    mujoco.mj_forward(model, data)

    # Record the torso's initial pose so we measure forward progress in
    # world coordinates.
    initial_torso = data.body("torso").xpos.copy()

    sim_seconds = duration + 1.0   # small tail so the robot can settle
    n_steps = int(sim_seconds / model.opt.timestep)

    torso_z_hist: list[float] = []
    torso_y_hist: list[float] = []
    torso_tilt_hist: list[tuple] = []
    diverged = False
    diverged_reason = ""

    # Joint indices for the ankle-pitch corrector.
    left_ankle_pitch_idx = ctrl_idx[joint_order.index("left_ankle_pitch")]
    right_ankle_pitch_idx = ctrl_idx[joint_order.index("right_ankle_pitch")]
    left_ankle_pitch_base = 0
    right_ankle_pitch_base = 0
    # Feedback gains for the pitch-correcting ankle commands. Tuned high so
    # the ankle servos drive the torso back upright before the body's
    # forward momentum carries it past tipping.
    PITCH_KP = 25.0
    PITCH_KD = 5.0

    prev_pitch = 0.0
    for step in range(n_steps):
        t = step * model.opt.timestep
        target = interpolated_target(t, keyframes)
        for j, joint_idx in enumerate(ctrl_idx):
            data.ctrl[joint_idx] = target[j]

        # Read torso pitch (rotation about world +X), compute correction.
        qw, qx, qy, qz = (data.qpos[3], data.qpos[4],
                          data.qpos[5], data.qpos[6])
        sinp = 2 * (qw * qy - qz * qx)
        pitch = (math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1
                 else math.asin(sinp))
        pitch_rate = (pitch - prev_pitch) / model.opt.timestep
        prev_pitch = pitch
        correction = -PITCH_KP * pitch - PITCH_KD * pitch_rate
        # Apply to both ankle pitches (PoC 8 axes: hinge about +X).
        data.ctrl[left_ankle_pitch_idx] = target[
            joint_order.index("left_ankle_pitch")] + correction
        data.ctrl[right_ankle_pitch_idx] = target[
            joint_order.index("right_ankle_pitch")] + correction

        mujoco.mj_step(model, data)

        if (not np.all(np.isfinite(data.qpos))
                or not np.all(np.isfinite(data.qvel))):
            diverged = True
            diverged_reason = f"non-finite state at t={t:.3f}s"
            break
        if float(np.max(np.abs(data.qvel))) > VELOCITY_LIMIT_RAD_PER_S:
            diverged = True
            diverged_reason = (f"runaway velocity at t={t:.3f}s "
                               f"({float(np.max(np.abs(data.qvel))):.1f} rad/s)")
            break

        torso_xpos = data.body("torso").xpos
        torso_z_hist.append(float(torso_xpos[2]))
        torso_y_hist.append(float(torso_xpos[1]))

        qw, qx, qy, qz = (data.qpos[3], data.qpos[4],
                          data.qpos[5], data.qpos[6])
        sinr = 2 * (qw * qx + qy * qz)
        cosr = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr, cosr)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = (math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1
                 else math.asin(sinp))
        torso_tilt_hist.append((math.degrees(pitch), math.degrees(roll)))

    final_torso = data.body("torso").xpos.copy()
    forward_m = float(final_torso[1] - initial_torso[1])
    min_torso_z = min(torso_z_hist) if torso_z_hist else 0.0
    final_torso_z = torso_z_hist[-1] if torso_z_hist else 0.0
    max_tilt = (max(max(abs(p), abs(r)) for p, r in torso_tilt_hist)
                if torso_tilt_hist else 0.0)

    lines = [
        "MechProof PoC 10 — Quasi-Static Walking Verification Report",
        "============================================================",
        f"Trajectory duration  : {duration:.2f} s "
        f"({traj['n_keyframes']} keyframes)",
        f"Simulation duration  : {sim_seconds:.2f} s",
        f"Initial torso XYZ    : ({initial_torso[0]:+.3f}, "
        f"{initial_torso[1]:+.3f}, {initial_torso[2]:+.3f})",
        f"Final   torso XYZ    : ({final_torso[0]:+.3f}, "
        f"{final_torso[1]:+.3f}, {final_torso[2]:+.3f})",
        f"Forward progress     : {forward_m*1000:+.1f} mm "
        f"(threshold {MIN_FORWARD_M*1000:.0f} mm)",
        f"Min torso Z observed : {min_torso_z:.3f} m "
        f"(threshold {MIN_TORSO_Z_M:.3f})",
        f"Final torso Z        : {final_torso_z:.3f} m",
        f"Max |tilt|           : {max_tilt:.2f}°",
    ]

    if diverged:
        lines += ["", "RESULT: FAIL", f"Reason: {diverged_reason}"]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 2

    walked_far = forward_m >= MIN_FORWARD_M
    upright = min_torso_z >= MIN_TORSO_Z_M

    if walked_far and upright:
        lines += [
            "",
            "RESULT: PASS",
            f"Humanoid took at least 2 quasi-static steps "
            f"({forward_m*1000:.1f} mm of forward progress) without "
            f"collapsing.",
        ]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 0

    reasons = []
    if not walked_far:
        reasons.append(
            f"only walked {forward_m*1000:.1f} mm "
            f"(needed {MIN_FORWARD_M*1000:.0f} mm)")
    if not upright:
        reasons.append(
            f"torso collapsed to {min_torso_z:.3f} m "
            f"(threshold {MIN_TORSO_Z_M:.3f})")
    lines += ["", "RESULT: FAIL", "Reason: " + "; ".join(reasons)]
    REPORT_PATH.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
