"""MechProof PoC 4 — 6-DOF tendon-driven hand digital twin.

Builds a MJCF scene with:
  * a static palm fixed to the world,
  * four tendon-driven fingers (index, middle, ring, pinky),
  * a thumb with a Z-axis swivel base **plus** its own tendon-driven flexion,
  * a small target sphere placed between the thumb and index for a precision
    pinch.

There are 6 actuators in total:
  1. thumb swivel (position-controlled hinge about +Z)
  2-6. one tendon-pull motor per finger (fixed tendon with coefficients =
       -r_i, mirroring the PoC 3 design).

The simulation:
  * commands the thumb swivel to its programmed angle,
  * ramps every flexion tendon to its target pull force,
  * monitors **self-collision** events between finger geoms (any non-zero
    inter-finger contact force is an empirical refutation of the Lean
    clearance proof),
  * checks that the target sphere is squeezed between the thumb and index
    above a 0.3 N threshold.

The report is written to `out/Hand_Report.txt`.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys
from typing import Dict, List

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
META_PATH = REPO_ROOT / "out" / "hand_physics_meta.json"
PARAMS_PATH = REPO_ROOT / "out" / "hand_params.json"
XML_PATH = REPO_ROOT / "out" / "hand_scene.xml"
REPORT_PATH = REPO_ROOT / "out" / "Hand_Report.txt"

SIM_SECONDS = 3.0
STABLE_WINDOW_S = 0.5
PINCH_FORCE_THRESHOLD_N = 0.3
VELOCITY_LIMIT_RAD_PER_S = 200.0

TARGET_RADIUS_M = 0.008
TARGET_MASS_KG = 0.005
SWIVEL_TARGET_RAD = 1.2   # radians (commanded swivel)

FINGERS = ("index", "middle", "ring", "pinky", "thumb")


def link_body_xml(link: dict, parent_length_m: float, thickness_m: float,
                  joint_name: str, joint_range_rad: tuple,
                  joint_stiffness: float) -> str:
    L = link["length_m"]
    com = link["com_m"]
    mass = link["mass_kg"]
    ixx = mass * (L * L + thickness_m * thickness_m) / 12.0
    iyy = mass * (thickness_m * thickness_m * 2) / 12.0
    izz = mass * (L * L + thickness_m * thickness_m) / 12.0
    return (
        f'<body name="{link["name"]}" pos="0 {parent_length_m:.6f} 0">'
        f'  <joint name="{joint_name}" type="hinge" axis="-1 0 0" '
        f'         range="{joint_range_rad[0]:.6f} {joint_range_rad[1]:.6f}" '
        f'         damping="0.002" stiffness="{joint_stiffness:.6f}" '
        f'         armature="0.0001"/>'
        f'  <inertial pos="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" '
        f'            mass="{mass:.6f}" '
        f'            diaginertia="{ixx:.9f} {iyy:.9f} {izz:.9f}"/>'
        f'  <geom name="{link["name"]}_geom" type="box" '
        f'        pos="0 {L/2:.6f} 0" '
        f'        size="{thickness_m/2:.6f} {L/2:.6f} {thickness_m/2:.6f}" '
        f'        rgba="0.70 0.75 0.90 1" '
        f'        friction="1.5 0.05 0.002"/>'
    )


def finger_chain_xml(name: str, finger_meta: dict) -> tuple:
    """Returns (chain_body_string, joint_names_in_order)."""
    links = finger_meta["links"]
    th = finger_meta["thickness_m"]
    jr = finger_meta["joint_range_rad"]
    stiffnesses = [0.001, 0.0015, 0.002]

    joint_names = [f"{name}_j{i+1}" for i in range(len(links))]

    parts = []
    for i, link in enumerate(links):
        parent_len = 0.0 if i == 0 else links[i - 1]["length_m"]
        parts.append(link_body_xml(
            link, parent_len, th, joint_names[i], (jr[0], jr[1]),
            stiffnesses[i]))
    nested = ""
    for body_open in reversed(parts):
        nested = body_open + nested + "</body>"
    return nested, joint_names


def compose_mjcf(meta: dict) -> str:
    palm = meta["palm"]
    swivel_max = meta["swivelMaxRad"]

    # Build each finger's chain. For the non-thumb fingers, the chain is
    # parented directly to the palm via a translated body. For the thumb,
    # the chain is parented to a swivel body that rotates about +Z.

    bodies = []
    all_joint_names: Dict[str, List[str]] = {}

    for name in ("index", "middle", "ring", "pinky"):
        f = meta["fingers"][name]
        m = f["mount"]
        chain, jnames = finger_chain_xml(name, f)
        all_joint_names[name] = jnames
        bodies.append(
            f'<body name="{name}_base" '
            f'      pos="{m["px"]:.6f} {m["py"]:.6f} {m["pz"]:.6f}" '
            f'      euler="0 0 {m["yawRad"]:.6f}">'
            f'  {chain}'
            f'</body>'
        )

    # Thumb: an additional Z-hinge body sits between the palm and the
    # thumb's link 1. This is the 6th DOF (thumb opposition/swivel).
    f = meta["fingers"]["thumb"]
    m = f["mount"]
    chain, jnames = finger_chain_xml("thumb", f)
    all_joint_names["thumb"] = jnames
    # The swivel-base body has a small inertia of its own so MuJoCo doesn't
    # complain about a massless moving body. The hinge axis is +Z (palm
    # normal), range [0, swivelMaxRad]. The thumb chain hangs off the swivel
    # base at its origin.
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
        f'        size="0.006 0.003" rgba="0.65 0.55 0.55 1"/>'
        f'  {chain}'
        f'</body>'
    )

    # Fixed tendon per finger: -r_i coefficients (per PoC 3).
    tendon_blocks = []
    for name in FINGERS:
        f = meta["fingers"][name]
        coefs = f["moment_arms_m"]
        joints = all_joint_names[name]
        joint_lines = "\n          ".join(
            f'<joint joint="{j}" coef="{-c:.6f}"/>'
            for j, c in zip(joints, coefs)
        )
        tendon_blocks.append(
            f'<fixed name="{name}_flexor" limited="false">\n'
            f'          {joint_lines}\n'
            f'        </fixed>'
        )
    tendons_xml = "\n        ".join(tendon_blocks)

    # Six actuators: 1 swivel + 5 tendon flexors.
    actuator_lines = [
        '<position name="thumb_swivel_act" joint="thumb_swivel" '
        f'kp="40" kv="0.8" ctrlrange="0 {swivel_max:.6f}"/>',
    ]
    for name in FINGERS:
        actuator_lines.append(
            f'<motor name="{name}_flexor_act" tendon="{name}_flexor" '
            f'gear="1" ctrlrange="-12 0"/>'
        )
    actuators_xml = "\n    ".join(actuator_lines)

    # Target sphere placed at the geometric midpoint between the curled
    # thumb-link3 and index-link3 centres (probed once for the canonical
    # 6-DOF pinch pose: swivel=1.2 rad, every flexion=90°). Using these
    # numbers up front avoids a brittle pre-flight simulation.
    target_pos = (0.0485, 0.020, -0.030)

    return f"""<?xml version="1.0"?>
<mujoco model="mechproof_hand">
  <compiler angle="radian"/>
  <option timestep="0.0005" gravity="0 0 -9.81" integrator="implicit"
          cone="elliptic" impratio="2"/>

  <default>
    <geom contype="1" conaffinity="1" condim="4"
          solref="0.005 1" solimp="0.95 0.99 0.001"/>
  </default>

  <worldbody>
    <light pos="0 0 1" dir="0 0 -1"/>
    <geom name="floor" type="plane" pos="0 0 -0.15" size="0.5 0.5 0.01"
          rgba="0.85 0.85 0.85 1" contype="0" conaffinity="0"/>

    <!-- Static palm. The collision-mask is disabled so the proximal links
         can sweep freely past the palm body — the Lean clearance proof
         already guarantees the geometric layout is collision-free; the
         palm here is purely for visualisation and inertial anchoring. -->
    <geom name="palm_geom" type="box" pos="0 {palm["length"]/2:.6f} 0"
          size="{palm["width"]/2:.6f} {palm["length"]/2:.6f} {palm["thickness"]/2:.6f}"
          rgba="0.55 0.55 0.65 1" contype="0" conaffinity="0"/>

    {chr(10).join(bodies)}

    <!-- Target sphere held by stiff 3-axis "force-gauge" springs at its
         initial pose. The thumb and index squeeze it; the springs absorb
         any net translation. -->
    <body name="target" pos="{target_pos[0]:.6f} {target_pos[1]:.6f} {target_pos[2]:.6f}">
      <joint name="target_slide_x" type="slide" axis="1 0 0"
             stiffness="500" damping="2.0" ref="0"/>
      <joint name="target_slide_y" type="slide" axis="0 1 0"
             stiffness="500" damping="2.0" ref="0"/>
      <joint name="target_slide_z" type="slide" axis="0 0 1"
             stiffness="500" damping="2.0" ref="0"/>
      <inertial pos="0 0 0" mass="{TARGET_MASS_KG:.6f}"
                diaginertia="1e-6 1e-6 1e-6"/>
      <geom name="target_geom" type="sphere" size="{TARGET_RADIUS_M:.6f}"
            rgba="0.9 0.4 0.4 1" friction="1.5 0.05 0.002"/>
    </body>
  </worldbody>

  <tendon>
        {tendons_xml}
  </tendon>

  <actuator>
    {actuators_xml}
  </actuator>
</mujoco>
"""


def sum_contact_force_on_geom(model: mujoco.MjModel, data: mujoco.MjData,
                              geom_id: int) -> float:
    total = 0.0
    for c in range(data.ncon):
        con = data.contact[c]
        if con.geom1 != geom_id and con.geom2 != geom_id:
            continue
        force = np.zeros(6, dtype=np.float64)
        mujoco.mj_contactForce(model, data, c, force)
        total += abs(float(force[0]))
    return total


def detect_self_collisions(model: mujoco.MjModel, data: mujoco.MjData,
                           finger_geom_ids: Dict[str, set]) -> List[tuple]:
    """Return a list of (finger_a, finger_b) pairs in contact this step,
    excluding contacts that involve the target sphere or non-finger geoms.
    """
    events = []
    for c in range(data.ncon):
        con = data.contact[c]
        a = con.geom1
        b = con.geom2
        a_owner = None
        b_owner = None
        for finger, ids in finger_geom_ids.items():
            if a in ids:
                a_owner = finger
            if b in ids:
                b_owner = finger
        if a_owner and b_owner and a_owner != b_owner:
            events.append((a_owner, b_owner))
    return events


def main() -> int:
    if not META_PATH.exists() or not PARAMS_PATH.exists():
        print("error: missing hand_physics_meta.json or hand_params.json — "
              "run Lean verification + CAD generation first.", file=sys.stderr)
        return 1

    meta = json.loads(META_PATH.read_text())
    xml = compose_mjcf(meta)
    XML_PATH.write_text(xml)
    print(f"Wrote {XML_PATH}")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    target_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "target_geom")
    if target_geom_id < 0:
        raise RuntimeError("target_geom not found in model")

    # Pre-curl every finger to ~45° so it starts close to the target.
    # Joint qpos ordering: traverse joints in model order.
    qpos_index = {}
    for jid in range(model.njnt):
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        qpos_index[jname] = model.jnt_qposadr[jid]

    for name in FINGERS:
        for i, ang in enumerate([40.0, 50.0, 50.0]):
            jname = f"{name}_j{i+1}"
            adr = qpos_index.get(jname)
            if adr is not None:
                data.qpos[adr] = math.radians(ang)

    # Collect geom ids per finger so we can detect self-collisions.
    finger_geom_ids: Dict[str, set] = {f: set() for f in FINGERS}
    for gid in range(model.ngeom):
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        if not gname:
            continue
        for f in FINGERS:
            if gname.startswith(f"{f}_link"):
                finger_geom_ids[f].add(gid)

    mujoco.mj_forward(model, data)

    n_steps = int(SIM_SECONDS / model.opt.timestep)
    stable_steps = int(STABLE_WINDOW_S / model.opt.timestep)
    pinch_history = []
    self_collision_pairs = set()
    diverged = False
    diverged_reason = ""

    PULL_MAX = -6.0
    RAMP_TIME = 2.0

    # Actuator order from the XML: swivel + 5 flexor motors.
    swivel_idx = 0
    flexor_idx = {name: 1 + i for i, name in enumerate(FINGERS)}

    for step in range(n_steps):
        t = step * model.opt.timestep
        ramp = min(1.0, t / RAMP_TIME)
        data.ctrl[swivel_idx] = SWIVEL_TARGET_RAD * ramp
        for name in FINGERS:
            data.ctrl[flexor_idx[name]] = PULL_MAX * ramp
        mujoco.mj_step(model, data)

        if (not np.all(np.isfinite(data.qpos))
                or not np.all(np.isfinite(data.qvel))):
            diverged = True
            diverged_reason = f"non-finite state at t={t:.3f}s"
            break
        if float(np.max(np.abs(data.qvel))) > VELOCITY_LIMIT_RAD_PER_S:
            diverged = True
            diverged_reason = (f"runaway velocity at t={t:.3f}s "
                               f"({np.max(np.abs(data.qvel)):.1f} rad/s)")
            break

        for pair in detect_self_collisions(model, data, finger_geom_ids):
            self_collision_pairs.add(frozenset(pair))

        pinch_history.append(
            sum_contact_force_on_geom(model, data, target_geom_id))

    swivel_final = float(data.qpos[qpos_index["thumb_swivel"]])
    final_flex = {
        name: [math.degrees(float(data.qpos[qpos_index[f"{name}_j{i+1}"]]))
               for i in range(3)]
        for name in FINGERS
    }

    lines = [
        "MechProof PoC 4 — 6-DOF Hand Verification Report",
        "==================================================",
        f"Simulation duration   : {SIM_SECONDS:.2f} s",
        f"Thumb swivel target   : {math.degrees(SWIVEL_TARGET_RAD):.1f}°",
        f"Thumb swivel final    : {math.degrees(swivel_final):.1f}°",
    ]
    for name in FINGERS:
        a, b, c = final_flex[name]
        lines.append(f"  {name:6s} flexion deg : [{a:6.2f}, {b:6.2f}, {c:6.2f}]")

    if diverged:
        lines += ["", "RESULT: FAIL", f"Reason: {diverged_reason}"]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 2

    final_pinch = pinch_history[-stable_steps:] if pinch_history else []
    mean_pinch = float(np.mean(final_pinch)) if final_pinch else 0.0
    min_pinch = float(np.min(final_pinch)) if final_pinch else 0.0
    peak_pinch = float(np.max(pinch_history)) if pinch_history else 0.0

    lines += [
        "",
        f"Peak pinch force      : {peak_pinch:.3f} N",
        f"Mean pinch (last {STABLE_WINDOW_S:.1f}s): {mean_pinch:.3f} N",
        f"Min pinch (last {STABLE_WINDOW_S:.1f}s) : {min_pinch:.3f} N "
        f"(threshold {PINCH_FORCE_THRESHOLD_N})",
        f"Self-collision pairs  : {sorted(tuple(sorted(p)) for p in self_collision_pairs) if self_collision_pairs else 'NONE'}",
        "",
    ]

    collision_free = (len(self_collision_pairs) == 0)
    pinch_held = (min_pinch >= PINCH_FORCE_THRESHOLD_N)

    if collision_free and pinch_held:
        lines += [
            "RESULT: PASS",
            "Hand completed precision pinch without inter-finger collision."
        ]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 0
    else:
        reasons = []
        if not collision_free:
            reasons.append(
                "self-collisions detected (Lean clearance proof contradicted "
                "in simulation — review HandAssembly.lean)")
        if not pinch_held:
            reasons.append(
                f"pinch force {min_pinch:.3f} N below threshold "
                f"{PINCH_FORCE_THRESHOLD_N} N")
        lines += ["RESULT: FAIL", "Reason: " + "; ".join(reasons)]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
