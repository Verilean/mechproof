"""MechProof PoC 8 — humanoid drop-and-stand simulation.

Builds a free-base MJCF with:
  * a `<freejoint>` torso (so the robot can fall),
  * mirrored left/right 6-DOF legs (hip yaw/roll/pitch, knee, ankle pitch/roll),
  * a flat foot pad with high friction at the bottom of each leg,
  * the upper body (arm + hand + payload) lumped as a single inertia on the
    torso so the simulator carries the same mass the Lean proof asserted.

The simulation drops the robot from `DROP_HEIGHT_M` above the ground while
the leg actuators hold the upright standing pose. Success requires:
  * torso Z stays above `MIN_TORSO_Z_M` (it didn't collapse),
  * torso pitch and roll stay within ±5° at the final second (it didn't fall
    over),
  * none of the joints diverged.

The verdict is written to `out/Stand_Report.txt`.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
LEG_META_PATH = REPO_ROOT / "out" / "leg_physics_meta.json"
XML_PATH = REPO_ROOT / "out" / "humanoid_scene.xml"
REPORT_PATH = REPO_ROOT / "out" / "Stand_Report.txt"

SIM_SECONDS = 3.0
SETTLE_WINDOW_S = 1.0
DROP_HEIGHT_M = 0.05
MIN_TORSO_Z_M = 0.55          # torso CoM must stay above 0.55 m
MAX_TILT_DEG = 5.0
VELOCITY_LIMIT_RAD_PER_S = 200.0


def leg_chain_xml(side: str, leg: dict, hip_tau: float, knee_tau: float,
                  ankle_tau: float, foot_mass: float, shin_mass: float,
                  thigh_mass: float) -> str:
    """Render one leg as a chain of bodies hanging off the torso bottom.

    Coordinate convention inside the chain:
      * each link's local +Y points down toward the next joint,
      * therefore each child body is translated by +Y = parent length.
    The hip's body-local frame is rotated about +X so positive hip pitch
    swings the leg forward, matching the arm convention.
    """
    sign = +1 if side == "right" else -1
    hip_x = sign * float(leg["hip_offset_x_m"])
    th = float(leg["thigh_length_m"])
    sh = float(leg["shin_length_m"])
    fl = float(leg["foot_length_m"])
    fw = float(leg["foot_width_m"])

    # Cylinder principal moments of a tube about the link CoM (rough).
    def inertia_rod(mass: float, length: float, radius: float = 0.028):
        i_perp = mass * (length * length / 12.0 + radius * radius / 4.0)
        i_axial = mass * radius * radius / 2.0
        return i_perp, i_axial

    thigh_iperp, thigh_iaxial = inertia_rod(thigh_mass, th)
    shin_iperp,  shin_iaxial  = inertia_rod(shin_mass, sh)

    # Foot: axis-aligned box. Iyy is forward, Ixx is lateral, Izz is yaw.
    foot_t = 0.020
    f_ixx = foot_mass * (fl * fl + foot_t * foot_t) / 12.0
    f_iyy = foot_mass * (fw * fw + foot_t * foot_t) / 12.0
    f_izz = foot_mass * (fl * fl + fw * fw) / 12.0

    return f"""
    <body name="{side}_hip" pos="{hip_x:.6f} 0 -0.15">
      <joint name="{side}_hip_yaw"   type="hinge" axis="0 0 1"
             range="-1.0 1.0" damping="0.5" armature="0.05"/>
      <joint name="{side}_hip_roll"  type="hinge" axis="0 1 0"
             range="-0.5 0.5" damping="0.5" armature="0.05"/>
      <joint name="{side}_hip_pitch" type="hinge" axis="1 0 0"
             range="-1.5 1.5" damping="0.5" armature="0.05"/>
      <inertial pos="0 0 -{th/2:.6f}" mass="{thigh_mass:.6f}"
                diaginertia="{thigh_iperp:.6f} {thigh_iperp:.6f} {thigh_iaxial:.6f}"/>
      <geom name="{side}_thigh_geom" type="capsule"
            fromto="0 0 0 0 0 -{th:.6f}" size="0.028"
            rgba="0.55 0.60 0.75 1"
            contype="1" conaffinity="2" friction="0.8 0.05 0.002"/>

      <body name="{side}_knee" pos="0 0 -{th:.6f}">
        <joint name="{side}_knee_pitch" type="hinge" axis="1 0 0"
               range="0 2.5" damping="0.5" armature="0.05"/>
        <inertial pos="0 0 -{sh/2:.6f}" mass="{shin_mass:.6f}"
                  diaginertia="{shin_iperp:.6f} {shin_iperp:.6f} {shin_iaxial:.6f}"/>
        <geom name="{side}_shin_geom" type="capsule"
              fromto="0 0 0 0 0 -{sh:.6f}" size="0.026"
              rgba="0.55 0.60 0.75 1"
              contype="1" conaffinity="2" friction="0.8 0.05 0.002"/>

        <body name="{side}_ankle" pos="0 0 -{sh:.6f}">
          <joint name="{side}_ankle_pitch" type="hinge" axis="1 0 0"
                 range="-1.0 1.0" damping="0.5" armature="0.05"/>
          <joint name="{side}_ankle_roll" type="hinge" axis="0 1 0"
                 range="-0.5 0.5" damping="0.5" armature="0.05"/>
          <inertial pos="0 0 -{foot_t/2:.6f}" mass="{foot_mass:.6f}"
                    diaginertia="{f_ixx:.6f} {f_iyy:.6f} {f_izz:.6f}"/>
          <geom name="{side}_foot_geom" type="box"
                pos="0 0 -{foot_t/2:.6f}"
                size="{fw/2:.6f} {fl/2:.6f} {foot_t/2:.6f}"
                rgba="0.40 0.40 0.55 1"
                contype="1" conaffinity="1" friction="1.5 0.05 0.002"/>
        </body>
      </body>
    </body>"""


def compose_mjcf(meta: dict) -> str:
    torso = meta["torso"]
    leg = meta["leg"]
    upper_body_mass = float(meta["upper_body_mass_kg"])

    hip_tau   = float(meta["torques_nm"]["hip_pitch"])
    knee_tau  = float(meta["torques_nm"]["knee"])
    ankle_tau = float(meta["torques_nm"]["ankle"])
    torso_mass = float(torso["mass_kg"])

    # Lump the torso structure + entire upper body into one inertial element.
    lumped = torso_mass + upper_body_mass
    tx = float(torso["width_m"])
    ty = float(torso["depth_m"])
    tz = float(torso["height_m"])
    ixx = lumped * (ty * ty + tz * tz) / 12.0
    iyy = lumped * (tx * tx + tz * tz) / 12.0
    izz = lumped * (tx * tx + ty * ty) / 12.0

    # Torso initial pose: feet flat on the ground (Z=0), ankle at foot
    # thickness, then add shin + thigh, plus a small DROP_HEIGHT_M offset
    # so the simulation has to settle the legs onto the floor.
    foot_t = 0.020
    rest_torso_z = (foot_t + float(leg["shin_length_m"])
                    + float(leg["thigh_length_m"]) + 0.15)
    start_torso_z = rest_torso_z + DROP_HEIGHT_M

    left_leg = leg_chain_xml(
        "left", leg, hip_tau, knee_tau, ankle_tau,
        float(leg["foot_mass_kg"]), float(leg["shin_mass_kg"]),
        float(leg["thigh_mass_kg"]),
    )
    right_leg = leg_chain_xml(
        "right", leg, hip_tau, knee_tau, ankle_tau,
        float(leg["foot_mass_kg"]), float(leg["shin_mass_kg"]),
        float(leg["thigh_mass_kg"]),
    )

    # Actuators: position-controlled servos on every leg joint, sized so
    # the steady-state error under static load is well below 1°.
    def act_block(side: str) -> str:
        return "\n    ".join([
            f'<position name="{side}_hip_yaw_act"   joint="{side}_hip_yaw"'
            f' kp="100"  kv="2"   ctrlrange="-1.0 1.0"/>',
            f'<position name="{side}_hip_roll_act"  joint="{side}_hip_roll"'
            f' kp="500"  kv="20"  ctrlrange="-0.5 0.5"/>',
            f'<position name="{side}_hip_pitch_act" joint="{side}_hip_pitch"'
            f' kp="800"  kv="40"  ctrlrange="-1.5 1.5"/>',
            f'<position name="{side}_knee_act"      joint="{side}_knee_pitch"'
            f' kp="800"  kv="40"  ctrlrange="0 2.5"/>',
            f'<position name="{side}_ankle_pitch_act" joint="{side}_ankle_pitch"'
            f' kp="300"  kv="15"  ctrlrange="-1.0 1.0"/>',
            f'<position name="{side}_ankle_roll_act"  joint="{side}_ankle_roll"'
            f' kp="200"  kv="10"  ctrlrange="-0.5 0.5"/>',
        ])

    actuators_xml = act_block("left") + "\n    " + act_block("right")

    return f"""<?xml version="1.0"?>
<mujoco model="mechproof_humanoid">
  <compiler angle="radian"/>
  <option timestep="0.0005" gravity="0 0 -9.81" integrator="implicit"
          cone="elliptic" impratio="4"/>

  <default>
    <geom condim="4" solref="0.005 1" solimp="0.95 0.99 0.001"/>
  </default>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1"/>
    <geom name="floor" type="plane" pos="0 0 0" size="2 2 0.01"
          rgba="0.85 0.85 0.85 1"
          contype="1" conaffinity="1" friction="1.5 0.05 0.002"/>

    <body name="torso" pos="0 0 {start_torso_z:.6f}">
      <freejoint name="torso_root"/>
      <inertial pos="0 0 0" mass="{lumped:.6f}"
                diaginertia="{ixx:.6f} {iyy:.6f} {izz:.6f}"/>
      <geom name="torso_geom" type="box"
            size="{tx/2:.6f} {ty/2:.6f} {tz/2:.6f}"
            rgba="0.55 0.55 0.65 1"
            contype="0" conaffinity="0"/>

      {left_leg}
      {right_leg}
    </body>
  </worldbody>

  <actuator>
    {actuators_xml}
  </actuator>
</mujoco>
"""


def main() -> int:
    if not LEG_META_PATH.exists():
        print(f"error: {LEG_META_PATH} missing — run `make verify-legs` and "
              "`make leg-cad` first.", file=sys.stderr)
        return 1

    meta = json.loads(LEG_META_PATH.read_text())
    xml = compose_mjcf(meta)
    XML_PATH.write_text(xml)
    print(f"Wrote {XML_PATH}")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    # Joint qpos indices.
    qadr = {}
    for jid in range(model.njnt):
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        qadr[jname] = model.jnt_qposadr[jid]

    # Initial standing pose: all joints zero (legs straight, feet flat).
    # The freejoint occupies qpos[0..6] (xyz + wxyz quat).
    mujoco.mj_forward(model, data)

    n_steps = int(SIM_SECONDS / model.opt.timestep)
    settle_steps = int(SETTLE_WINDOW_S / model.opt.timestep)

    torso_z_hist = []
    torso_tilt_hist = []   # (pitch_deg, roll_deg)
    diverged = False
    diverged_reason = ""

    for step in range(n_steps):
        # Hold every leg joint at 0 rad (upright standing pose).
        for k in range(model.nu):
            data.ctrl[k] = 0.0
        mujoco.mj_step(model, data)

        if (not np.all(np.isfinite(data.qpos))
                or not np.all(np.isfinite(data.qvel))):
            diverged = True
            diverged_reason = f"non-finite state at step {step}"
            break
        if float(np.max(np.abs(data.qvel))) > VELOCITY_LIMIT_RAD_PER_S:
            diverged = True
            diverged_reason = (f"runaway velocity "
                               f"({float(np.max(np.abs(data.qvel))):.1f} rad/s)")
            break

        torso_xpos = data.body("torso").xpos
        torso_z_hist.append(float(torso_xpos[2]))

        # Convert freejoint quaternion (w,x,y,z) → roll/pitch about world axes.
        qw, qx, qy, qz = data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]
        # Pitch about +X: atan2(2(qw·qx + qy·qz), 1 - 2(qx² + qy²))
        sinr = 2 * (qw * qx + qy * qz)
        cosr = 1 - 2 * (qx * qx + qy * qy)
        roll_rad = math.atan2(sinr, cosr)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch_rad = math.copysign(math.pi / 2, sinp)
        else:
            pitch_rad = math.asin(sinp)
        torso_tilt_hist.append((math.degrees(pitch_rad),
                                math.degrees(roll_rad)))

    final_torso_z = torso_z_hist[-1] if torso_z_hist else 0.0
    min_torso_z_observed = min(torso_z_hist) if torso_z_hist else 0.0

    settle_window = torso_tilt_hist[-settle_steps:]
    if settle_window:
        pitches = np.array([p for p, _ in settle_window])
        rolls   = np.array([r for _, r in settle_window])
        max_tilt = float(max(np.max(np.abs(pitches)), np.max(np.abs(rolls))))
        mean_pitch = float(np.mean(pitches))
        mean_roll  = float(np.mean(rolls))
    else:
        max_tilt = float("inf")
        mean_pitch = mean_roll = 0.0

    lines = [
        "MechProof PoC 8 — Humanoid Stand Verification Report",
        "====================================================",
        f"Simulation duration  : {SIM_SECONDS:.2f} s",
        f"Initial torso Z      : {DROP_HEIGHT_M:.3f} m above the standing pose",
        f"Min torso Z observed : {min_torso_z_observed:.3f} m "
        f"(threshold {MIN_TORSO_Z_M:.3f})",
        f"Final torso Z        : {final_torso_z:.3f} m",
        f"Mean pitch/roll      : {mean_pitch:+.3f}° / {mean_roll:+.3f}° "
        f"(window {SETTLE_WINDOW_S:.1f} s)",
        f"Max |tilt| (window)  : {max_tilt:.3f}° (limit {MAX_TILT_DEG:.1f}°)",
    ]

    if diverged:
        lines += ["", "RESULT: FAIL", f"Reason: {diverged_reason}"]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 2

    stood = (min_torso_z_observed >= MIN_TORSO_Z_M
             and max_tilt <= MAX_TILT_DEG)
    if stood:
        lines += [
            "",
            "RESULT: PASS",
            "Humanoid absorbed the drop and held the standing pose without "
            "collapse or fall-over.",
        ]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 0

    reasons = []
    if min_torso_z_observed < MIN_TORSO_Z_M:
        reasons.append(
            f"torso collapsed to {min_torso_z_observed:.3f} m "
            f"(threshold {MIN_TORSO_Z_M:.3f})")
    if max_tilt > MAX_TILT_DEG:
        reasons.append(
            f"torso tilted {max_tilt:.2f}° "
            f"(limit {MAX_TILT_DEG:.1f}°)")
    lines += ["", "RESULT: FAIL", "Reason: " + "; ".join(reasons)]
    REPORT_PATH.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
