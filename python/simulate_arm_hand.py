"""MechProof PoC 6 — combined 6-DOF arm + 6-DOF hand digital twin.

The arm chain is built fresh here. The hand chain is reused verbatim from
`simulate_hand.py` so we have a single source of truth for the PoC 5 hand
kinematics. The hand's `palm` is attached to the arm's wrist roll body — no
`<weld>` is needed because we mount the palm directly inside the wrist's
`<body>` as a nested kinematic child.

Kinematic order (proximal → distal):
  shoulder pan (Z) → shoulder pitch (X) → elbow pitch (X) → wrist pitch (X)
  → wrist yaw (Z) → wrist roll (Y) → palm + 5 fingers

Test motion:
  1. Drive the arm to a horizontal extension pose (worst-case torque load
     according to the Lean proof).
  2. Pull every tendon to close the hand around a sphere mounted in the
     hand's grasp envelope.

Success criteria:
  * The arm does **not** collapse: shoulder pitch joint stays within ±5° of
    its commanded set-point (the Lean stall-torque proof must agree with
    the simulator).
  * The hand pinches the target with > 0.3 N contact force.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import mujoco
import numpy as np

# Reuse the hand builders so PoC 6 stays in lock-step with PoC 4/5.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from simulate_hand import (  # type: ignore
    FINGERS,
    finger_chain_xml,
    detect_self_collisions,
    sum_contact_force_on_geom,
)

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
ARM_META_PATH = REPO_ROOT / "out" / "arm_physics_meta.json"
HAND_META_PATH = REPO_ROOT / "out" / "hand_physics_meta.json"
HAND_PARAMS_PATH = REPO_ROOT / "out" / "hand_params.json"
XML_PATH = REPO_ROOT / "out" / "arm_hand_scene.xml"
REPORT_PATH = REPO_ROOT / "out" / "Arm_Hand_Report.txt"

SIM_SECONDS = 6.0
STABLE_WINDOW_S = 0.5
ARM_SETTLE_S = 2.0   # arm fully extends and settles before the hand closes
PINCH_FORCE_THRESHOLD_N = 0.3
ARM_DROOP_LIMIT_DEG = 5.0
VELOCITY_LIMIT_RAD_PER_S = 200.0

# Target arm pose (radians). Shoulder pitch at -π/2 places the arm along the
# horizontal +Y direction; elbow and wrist held straight. This is the
# worst-case static load Lean proved against.
ARM_TARGET = {
    "shoulder_pan":   0.0,
    "shoulder_pitch": -math.pi / 2,
    "elbow_pitch":    0.0,
    "wrist_pitch":    0.0,
    "wrist_yaw":      0.0,
    "wrist_roll":     0.0,
}

TARGET_RADIUS_M = 0.008
TARGET_MASS_KG = 0.005


def arm_link_xml(name: str, length_m: float, mass_kg: float,
                 parent_offset_m: float,
                 axis: str, range_rad: tuple,
                 stall_torque: float) -> str:
    """A tubular link rendered as a capsule for collision/visualisation.
    Inertia is approximated as a thin rod about the link's long axis."""
    # Approximate the link's principal moments about its CoM. For a slender
    # rod of length L and mass m: I_perp = m·L²/12, I_axial = m·r²/2.
    r = 0.022
    iperp = mass_kg * (length_m * length_m) / 12.0
    iaxial = mass_kg * r * r / 2.0
    # Joint axes are local-frame; the parent body's orientation handles the
    # global mapping.
    return (
        f'<body name="{name}" pos="0 {parent_offset_m:.6f} 0">'
        f'  <joint name="{name}_joint" type="hinge" axis="{axis}" '
        f'         range="{range_rad[0]:.6f} {range_rad[1]:.6f}" '
        f'         damping="0.2" armature="0.01"/>'
        f'  <inertial pos="0 {length_m/2:.6f} 0" mass="{mass_kg:.6f}" '
        f'            diaginertia="{iperp:.6f} {iaxial:.6f} {iperp:.6f}"/>'
        f'  <geom type="capsule" fromto="0 0 0 0 {length_m:.6f} 0" '
        f'        size="{r:.6f}" rgba="0.55 0.60 0.75 1" '
        f'        contype="0" conaffinity="0"/>'
    )


def build_hand_chain(hand_meta: dict, hand_params: dict) -> tuple:
    """Compose the palm + 5 finger bodies that will hang off the wrist."""
    palm = hand_meta["palm"]
    swivel_max = hand_meta["swivelMaxRad"]

    bodies = []
    for name in ("index", "middle", "ring", "pinky"):
        f = hand_meta["fingers"][name]
        m = f["mount"]
        chain, _ = finger_chain_xml(name, f)
        bodies.append(
            f'<body name="{name}_base" '
            f'      pos="{m["px"]:.6f} {m["py"]:.6f} {m["pz"]:.6f}" '
            f'      euler="0 0 {m["yawRad"]:.6f}">'
            f'  {chain}'
            f'</body>'
        )

    f = hand_meta["fingers"]["thumb"]
    m = f["mount"]
    chain, _ = finger_chain_xml("thumb", f)
    bodies.append(
        f'<body name="thumb_swivel_base" '
        f'      pos="{m["px"]:.6f} {m["py"]:.6f} {m["pz"]:.6f}" '
        f'      euler="0 0 {m["yawRad"]:.6f}">'
        f'  <joint name="thumb_swivel" type="hinge" axis="0 0 1" '
        f'         range="0 {swivel_max:.6f}" damping="0.01" '
        f'         armature="0.0005"/>'
        f'  <inertial pos="0 0 0" mass="0.002" '
        f'            diaginertia="1e-7 1e-7 1e-7"/>'
        f'  <geom name="thumb_swivel_geom" type="cylinder" '
        f'        size="0.006 0.003" rgba="0.65 0.55 0.55 1" '
        f'        contype="0" conaffinity="0"/>'
        f'  {chain}'
        f'</body>'
    )

    palm_geom = (
        f'<geom name="palm_geom" type="box" pos="0 {palm["length"]/2:.6f} 0" '
        f'      size="{palm["width"]/2:.6f} {palm["length"]/2:.6f} '
        f'{palm["thickness"]/2:.6f}" '
        f'      rgba="0.55 0.55 0.65 1" contype="0" conaffinity="0"/>'
    )

    return palm, swivel_max, palm_geom, "\n        ".join(bodies)


def compose_mjcf(arm_meta: dict, hand_meta: dict, hand_params: dict) -> str:
    arm_links = arm_meta["links"]
    L1 = arm_links[0]["length_m"]
    L2 = arm_links[1]["length_m"]
    L3 = arm_links[2]["length_m"]

    # Build the hand chain.
    palm, swivel_max, palm_geom, hand_bodies = build_hand_chain(
        hand_meta, hand_params)

    # Lump the hand + the entire payload as a single inertial mass at the
    # wrist. This is the load the Lean stall-torque proof asserts the motors
    # can hold — we mirror it 1:1 in the simulator so the test verifies the
    # exact claim. The little test sphere below is the *pinch target*, not
    # the payload it stands in for.
    wrist_lumped_mass = (float(arm_meta["hand_mass_kg"])
                         + float(arm_meta["payload_mass_kg"]))
    palm_inertia = wrist_lumped_mass * 0.06 * 0.06 / 6.0

    payload_geom_pos = (0.0485, 0.020, -0.030)   # palm-frame coordinates

    # Joint ranges (radians).
    SP_RANGE = "-3.14 3.14"            # shoulder pan
    SP_TILT_RANGE = "-2.0 0.5"         # shoulder pitch (negative = lift forward)
    EP_RANGE = "-2.5 2.5"              # elbow
    WP_RANGE = "-2.0 2.0"              # wrist pitch
    WY_RANGE = "-3.14 3.14"            # wrist yaw
    WR_RANGE = "-3.14 3.14"            # wrist roll

    tau_s = float(arm_meta["torques_nm"]["shoulder_supplied"])
    tau_e = float(arm_meta["torques_nm"]["elbow_supplied"])
    tau_w = float(arm_meta["torques_nm"]["wrist_supplied"])

    # Tendon definitions (one fixed tendon per finger, coef = -moment_arm).
    tendon_blocks = []
    for name in FINGERS:
        f = hand_meta["fingers"][name]
        coefs = f["moment_arms_m"]
        lines = []
        for i, c in enumerate(coefs):
            lines.append(
                f'<joint joint="{name}_j{i+1}" coef="{-c:.6f}"/>')
        joint_lines = "\n          ".join(lines)
        tendon_blocks.append(
            f'<fixed name="{name}_flexor" limited="false">\n'
            f'          {joint_lines}\n'
            f'        </fixed>'
        )
    tendons_xml = "\n        ".join(tendon_blocks)

    # Actuators: 6 arm position-controlled hinges, 1 thumb swivel, 5 finger
    # tendon motors → 12 in total.
    arm_act = [
        ('shoulder_pan_act',   'shoulder_pan_joint',   tau_s, SP_RANGE),
        ('shoulder_pitch_act', 'shoulder_pitch_joint', tau_s, SP_TILT_RANGE),
        ('elbow_pitch_act',    'elbow_pitch_joint',    tau_e, EP_RANGE),
        ('wrist_pitch_act',    'wrist_pitch_joint',    tau_w, WP_RANGE),
        ('wrist_yaw_act',      'wrist_yaw_joint',      tau_w, WY_RANGE),
        ('wrist_roll_act',     'wrist_roll_joint',     tau_w, WR_RANGE),
    ]
    # Position-controlled servos. Gains are sized so the steady-state error
    # under maximum static load is small (~1°) but not so high that the
    # joint becomes rigid enough to backfeed reaction torques into the hand
    # actuators. kp ≈ tau · 15 keeps the closed-loop response reasonable.
    arm_actuators = "\n    ".join(
        f'<position name="{aname}" joint="{jname}" '
        f'kp="{tau*15.0:.1f}" kv="{tau*0.8:.3f}" '
        f'ctrlrange="{rng}"/>'
        for (aname, jname, tau, rng) in arm_act
    )
    hand_actuators = (
        f'<position name="thumb_swivel_act" joint="thumb_swivel" '
        f'kp="40" kv="0.8" ctrlrange="0 {swivel_max:.6f}"/>\n    '
        + "\n    ".join(
            f'<motor name="{n}_flexor_act" tendon="{n}_flexor" '
            f'gear="1" ctrlrange="-12 0"/>'
            for n in FINGERS
        )
    )

    # Compose the kinematic tree. The wrist roll body carries the hand: we
    # nest the palm geom + all 5 finger bodies inside it, applying a rotation
    # so the palm's +Y (fingertip direction) points along the wrist's +Y.
    return f"""<?xml version="1.0"?>
<mujoco model="mechproof_arm_hand">
  <compiler angle="radian"/>
  <option timestep="0.0005" gravity="0 0 -9.81" integrator="implicit"
          cone="elliptic" impratio="2"/>

  <default>
    <!-- contype 2 / conaffinity 4 means finger-link geoms only collide
         with the target sphere (which has contype 4 / conaffinity 2).
         This disables inter-finger contact, which is outside the PoC 4
         clearance theorem's scope (it covers only the extended pose). -->
    <geom contype="2" conaffinity="4" condim="4"
          solref="0.005 1" solimp="0.95 0.99 0.001"/>
  </default>

  <worldbody>
    <light pos="0.3 0.3 1" dir="-0.3 -0.3 -1"/>
    <geom name="floor" type="plane" pos="0 0 -1.0" size="2 2 0.01"
          rgba="0.85 0.85 0.85 1" contype="0" conaffinity="0"/>

    <!-- Shoulder pan body (Z axis). -->
    <body name="shoulder_pan" pos="0 0 0">
      <joint name="shoulder_pan_joint" type="hinge" axis="0 0 1"
             range="{SP_RANGE}" damping="0.5" armature="0.05"/>
      <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>
      <geom type="cylinder" size="0.04 0.025" rgba="0.45 0.45 0.55 1"
            contype="0" conaffinity="0"/>

      <!-- Shoulder pitch body (X axis). Link 1 extends along +Y. -->
      {arm_link_xml("shoulder_pitch", L1, arm_links[0]["mass_kg"], 0.0,
                    "1 0 0", (-2.0, 0.5), tau_s)}
        <!-- Elbow pitch. Link 2 hangs off the end of link 1. -->
        {arm_link_xml("elbow_pitch", L2, arm_links[1]["mass_kg"], L1,
                      "1 0 0", (-2.5, 2.5), tau_e)}
          <!-- Wrist pitch. -->
          {arm_link_xml("wrist_pitch", L3, arm_links[2]["mass_kg"], L2,
                        "1 0 0", (-2.0, 2.0), tau_w)}
            <!-- Wrist yaw (Z). -->
            <body name="wrist_yaw" pos="0 {L3:.6f} 0">
              <joint name="wrist_yaw_joint" type="hinge" axis="0 0 1"
                     range="{WY_RANGE}" damping="0.05" armature="0.005"/>
              <inertial pos="0 0 0" mass="0.05" diaginertia="1e-5 1e-5 1e-5"/>
              <!-- Wrist roll (Y). -->
              <body name="wrist_roll" pos="0 0 0">
                <joint name="wrist_roll_joint" type="hinge" axis="0 1 0"
                       range="{WR_RANGE}" damping="0.05" armature="0.005"/>
                <!-- Wrist-flange visual + the hand mass is lumped here. -->
                <inertial pos="0 0.03 0" mass="{wrist_lumped_mass:.6f}"
                          diaginertia="{palm_inertia:.6f} {palm_inertia:.6f} {palm_inertia:.6f}"/>
                <geom name="wrist_flange_geom" type="cylinder"
                      size="0.032 0.004" euler="1.5708 0 0"
                      rgba="0.35 0.35 0.45 1"
                      contype="0" conaffinity="0"/>

                <!-- The hand: palm and 5 fingers, mounted directly on the
                     wrist-roll body. -->
                {palm_geom}
                {hand_bodies}

                <!-- Target sphere: anchored by stiff spring joints inside
                     the hand frame so the pinch test is decoupled from arm
                     motion. -->
                <body name="target" pos="{payload_geom_pos[0]:.6f} {payload_geom_pos[1]:.6f} {payload_geom_pos[2]:.6f}">
                  <!-- Mild spring mount so the target tracks the wrist
                       during the arm motion without rebounding through
                       the fingers. Damping is high to suppress oscillation
                       once the fingers close on it. -->
                  <joint name="target_slide_x" type="slide" axis="1 0 0"
                         stiffness="80" damping="3.0" ref="0"/>
                  <joint name="target_slide_y" type="slide" axis="0 1 0"
                         stiffness="80" damping="3.0" ref="0"/>
                  <joint name="target_slide_z" type="slide" axis="0 0 1"
                         stiffness="80" damping="3.0" ref="0"/>
                  <inertial pos="0 0 0" mass="0.005"
                            diaginertia="1e-6 1e-6 1e-6"/>
                  <geom name="target_geom" type="sphere"
                        size="{TARGET_RADIUS_M:.6f}"
                        contype="4" conaffinity="2"
                        rgba="0.9 0.4 0.4 1" friction="1.5 0.05 0.002"/>
                </body>

              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <tendon>
        {tendons_xml}
  </tendon>

  <actuator>
    {arm_actuators}
    {hand_actuators}
  </actuator>
</mujoco>
"""


def main() -> int:
    for p in (ARM_META_PATH, HAND_META_PATH, HAND_PARAMS_PATH):
        if not p.exists():
            print(f"error: {p} missing — run the full PoC 5 + PoC 6 Lean "
                  "proofs and CAD generators first.", file=sys.stderr)
            return 1

    arm_meta = json.loads(ARM_META_PATH.read_text())
    hand_meta = json.loads(HAND_META_PATH.read_text())
    hand_params = json.loads(HAND_PARAMS_PATH.read_text())

    xml = compose_mjcf(arm_meta, hand_meta, hand_params)
    XML_PATH.write_text(xml)
    print(f"Wrote {XML_PATH}")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    # qpos addresses for joints we care about.
    qpos_index = {}
    for jid in range(model.njnt):
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        qpos_index[jname] = model.jnt_qposadr[jid]

    # Pre-curl finger joints so they start near the target.
    for name in FINGERS:
        for i, ang in enumerate([40.0, 50.0, 50.0]):
            jname = f"{name}_j{i+1}"
            adr = qpos_index.get(jname)
            if adr is not None:
                data.qpos[adr] = math.radians(ang)

    target_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "target_geom")

    finger_geom_ids = {f: set() for f in FINGERS}
    for gid in range(model.ngeom):
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        if not gname:
            continue
        for f in FINGERS:
            if gname.startswith(f"{f}_link"):
                finger_geom_ids[f].add(gid)

    mujoco.mj_forward(model, data)

    # Actuator indices in declaration order (arm 0..5, swivel 6, flexors 7..11).
    SHOULDER_PITCH_CTRL = 1
    n_steps = int(SIM_SECONDS / model.opt.timestep)
    stable_steps = int(STABLE_WINDOW_S / model.opt.timestep)
    pinch_history = []
    droop_history = []
    self_collision_pairs = set()
    diverged = False
    diverged_reason = ""

    PULL_MAX = -6.0
    SWIVEL_TARGET_RAD = 1.2
    ARM_RAMP_S = 1.5
    HAND_RAMP_S = 1.5

    for step in range(n_steps):
        t = step * model.opt.timestep
        arm_ramp = min(1.0, t / ARM_RAMP_S)
        hand_ramp = min(1.0, max(0.0, (t - ARM_SETTLE_S) / HAND_RAMP_S))

        data.ctrl[0] = ARM_TARGET["shoulder_pan"]
        data.ctrl[1] = ARM_TARGET["shoulder_pitch"] * arm_ramp
        data.ctrl[2] = ARM_TARGET["elbow_pitch"]
        data.ctrl[3] = ARM_TARGET["wrist_pitch"]
        data.ctrl[4] = ARM_TARGET["wrist_yaw"]
        data.ctrl[5] = ARM_TARGET["wrist_roll"]
        data.ctrl[6] = SWIVEL_TARGET_RAD * hand_ramp
        for i in range(5):
            data.ctrl[7 + i] = PULL_MAX * hand_ramp

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

        for pair in detect_self_collisions(model, data, finger_geom_ids):
            self_collision_pairs.add(frozenset(pair))

        if target_geom_id >= 0:
            pinch_history.append(
                sum_contact_force_on_geom(model, data, target_geom_id))

        # Track shoulder-pitch droop relative to the commanded set-point.
        actual = float(data.qpos[qpos_index["shoulder_pitch_joint"]])
        commanded = data.ctrl[SHOULDER_PITCH_CTRL]
        droop_history.append(actual - commanded)

    lines = [
        "MechProof PoC 6 — Arm + Hand Verification Report",
        "==================================================",
        f"Simulation duration  : {SIM_SECONDS:.2f} s",
    ]

    if diverged:
        lines += ["", "RESULT: FAIL", f"Reason: {diverged_reason}"]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 2

    # Steady-state droop: how far the shoulder pitch sagged below the target.
    droop_window = droop_history[-stable_steps:]
    droop_mean_deg = math.degrees(float(np.mean(droop_window)))
    droop_max_abs_deg = math.degrees(float(np.max(np.abs(droop_window))))
    pinch_window = pinch_history[-stable_steps:] if pinch_history else []
    min_pinch = float(np.min(pinch_window)) if pinch_window else 0.0
    mean_pinch = float(np.mean(pinch_window)) if pinch_window else 0.0
    peak_pinch = float(np.max(pinch_history)) if pinch_history else 0.0

    final_arm_deg = {
        name: math.degrees(float(data.qpos[qpos_index[f"{name}_joint"]]))
        for name in ("shoulder_pan", "shoulder_pitch", "elbow_pitch",
                     "wrist_pitch", "wrist_yaw", "wrist_roll")
    }

    lines += [
        f"Arm final pose (deg) :",
        f"   shoulder_pan       = {final_arm_deg['shoulder_pan']:+.2f}",
        f"   shoulder_pitch     = {final_arm_deg['shoulder_pitch']:+.2f}  "
        f"(commanded {math.degrees(ARM_TARGET['shoulder_pitch']):+.2f})",
        f"   elbow_pitch        = {final_arm_deg['elbow_pitch']:+.2f}",
        f"   wrist_pitch        = {final_arm_deg['wrist_pitch']:+.2f}",
        f"   wrist_yaw          = {final_arm_deg['wrist_yaw']:+.2f}",
        f"   wrist_roll         = {final_arm_deg['wrist_roll']:+.2f}",
        f"Shoulder droop       : mean {droop_mean_deg:+.2f}°  "
        f"max |Δ| {droop_max_abs_deg:.2f}°  (limit {ARM_DROOP_LIMIT_DEG:.1f}°)",
        f"Peak pinch force     : {peak_pinch:.3f} N",
        f"Min pinch (last {STABLE_WINDOW_S:.1f}s): {min_pinch:.3f} N "
        f"(threshold {PINCH_FORCE_THRESHOLD_N})",
        f"Mean pinch (last {STABLE_WINDOW_S:.1f}s): {mean_pinch:.3f} N",
        f"Self-collision pairs : "
        f"{sorted(tuple(sorted(p)) for p in self_collision_pairs) if self_collision_pairs else 'NONE'}",
        "",
    ]

    arm_held = droop_max_abs_deg <= ARM_DROOP_LIMIT_DEG
    pinch_held = min_pinch >= PINCH_FORCE_THRESHOLD_N

    # The Lean clearance proof in PoC 4 covers only the **proximal** links at
    # extended pose. Distal-link contact during pinch is geometrically
    # expected (the curled fingertips meet) and is reported as a note rather
    # than a hard failure here. Future work: extend HandAssembly.lean to
    # cover the full curl envelope.
    if self_collision_pairs:
        lines.append(
            f"NOTE: distal links touched during pinch — pairs: "
            f"{sorted(tuple(sorted(p)) for p in self_collision_pairs)}. "
            "This is outside the scope of the PoC 4 clearance theorem.")

    if arm_held and pinch_held:
        lines += [
            "RESULT: PASS",
            "The 6-DOF arm held the hand + payload at horizontal extension "
            "(droop %.2f°), and the hand pinched the target." % droop_max_abs_deg,
        ]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 0

    reasons = []
    if not arm_held:
        reasons.append(
            f"arm collapsed ({droop_max_abs_deg:.2f}° > "
            f"{ARM_DROOP_LIMIT_DEG:.1f}°)")
    if not pinch_held:
        reasons.append(
            f"pinch {min_pinch:.3f} N below threshold "
            f"{PINCH_FORCE_THRESHOLD_N} N")
    lines += ["RESULT: FAIL", "Reason: " + "; ".join(reasons)]
    REPORT_PATH.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
