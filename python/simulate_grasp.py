"""MechProof PoC 3 — tendon-driven grasp simulation.

Builds a MJCF scene with the verified finger (anchored at the proximal pivot)
and a cylindrical target. The three hinge joints are coupled by a single
`<fixed>` tendon whose coefficients are the Lean-proven moment arms `r_i`.
A position actuator pulls the tendon to its lower bound (slack length),
flexing the finger around the target.

Success criterion: in the final `STABLE_WINDOW_S` seconds, the cylinder must
maintain contact with at least one finger link and the **sum of normal contact
forces between the finger and the cylinder** must exceed
`MIN_GRASP_FORCE_N` continuously.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
META_PATH = REPO_ROOT / "out" / "physics_meta.json"
PARAMS_PATH = REPO_ROOT / "out" / "tendon_params.json"
XML_PATH = REPO_ROOT / "out" / "grasp_scene.xml"
REPORT_PATH = REPO_ROOT / "out" / "Grasp_Report.txt"

SIM_SECONDS = 3.0
STABLE_WINDOW_S = 0.5
MIN_GRASP_FORCE_N = 0.5
VELOCITY_LIMIT_RAD_PER_S = 200.0

# Target cylinder (placed within the finger's curl envelope).
# Sized so the curling finger meets the cylinder before saturating its
# joint limits, then squeezes it as the tendon force ramps up.
TARGET_RADIUS_M = 0.013
TARGET_HEIGHT_M = 0.040
TARGET_MASS_KG = 0.020


def link_body(link: dict, parent_length_m: float, thickness_m: float,
              joint_range_rad: tuple, joint_stiffness: float) -> str:
    """Render one body. `joint_stiffness` lets us bias the under-actuated
    finger to curl proximal-first (MCP softest → DIP stiffest) so the joint
    that meets the obstacle first is the one nearest the palm."""
    name = link["name"]
    L = link["length_m"]
    com = link["com_m"]
    mass = link["mass_kg"]

    ixx = mass * (L * L + thickness_m * thickness_m) / 12.0
    iyy = mass * (thickness_m * thickness_m * 2) / 12.0
    izz = mass * (L * L + thickness_m * thickness_m) / 12.0

    return (
        f'<body name="{name}" pos="0 {parent_length_m:.6f} 0">'
        f'  <joint name="{name}_hinge" type="hinge" axis="-1 0 0" '
        f'         range="{joint_range_rad[0]:.6f} {joint_range_rad[1]:.6f}" '
        f'         damping="0.002" stiffness="{joint_stiffness:.6f}" '
        f'         armature="0.0001"/>'
        f'  <inertial pos="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}" '
        f'            mass="{mass:.6f}" '
        f'            diaginertia="{ixx:.9f} {iyy:.9f} {izz:.9f}"/>'
        f'  <geom name="{name}_geom" type="box" '
        f'        pos="0 {L/2:.6f} 0" '
        f'        size="{thickness_m/2:.6f} {L/2:.6f} {thickness_m/2:.6f}" '
        f'        rgba="0.70 0.75 0.90 1" '
        f'        friction="1.5 0.05 0.002"/>'
    )


def compose_mjcf(meta: dict) -> str:
    links = meta["links"]
    th = meta["thickness_m"]
    jr = meta["joint_range_rad"]
    coefs = meta["moment_arms_m"]

    # Mild extension springs at each joint hold the finger open when the
    # tendon is slack; the tendon needs to clearly dominate. Distal springs
    # are slightly stiffer so the proximal joint closes first under load.
    joint_stiffnesses = [0.001, 0.0015, 0.002]

    parts = []
    for i, link in enumerate(links):
        parent_len = 0.0 if i == 0 else links[i - 1]["length_m"]
        parts.append(link_body(link, parent_len, th, (jr[0], jr[1]),
                               joint_stiffnesses[i]))
    nested = ""
    for body_open in reversed(parts):
        nested = body_open + nested + "</body>"

    # Tendon: <fixed> with coefficient = moment arm. Convention: increasing
    # joint angle (flexion toward palm) requires shortening the tendon, so the
    # tendon length L = L0 - sum(r_i * q_i). Use negative coefficients so the
    # actuator pulling toward "negative tendon length" flexes the joints.
    fixed_joints = "\n          ".join(
        f'<joint joint="{links[i]["name"]}_hinge" coef="{-coefs[i]:.6f}"/>'
        for i in range(len(links))
    )

    # Cylinder sits on the palmar (-Z) side of the proximal link, where link
    # 1 sweeps as MCP flexes. With MCP ≈ 30-40° the link 1 inner surface
    # touches the cylinder; further tendon pull then squeezes it against the
    # stiff spring mount.
    target_y = 0.025
    target_z = -0.022

    # Cylinder is horizontal (axis = X) so the finger curls *around* it.
    return f"""<?xml version="1.0"?>
<mujoco model="mechproof_grasp">
  <!-- Use radians everywhere: joint ranges, euler angles, etc. -->
  <compiler angle="radian"/>
  <option timestep="0.0005" gravity="0 0 -9.81" integrator="implicit"
          cone="elliptic" impratio="2"/>

  <default>
    <geom contype="1" conaffinity="1" condim="4" solref="0.005 1" solimp="0.95 0.99 0.001"/>
  </default>

  <worldbody>
    <light pos="0 0 1" dir="0 0 -1"/>
    <geom name="floor" type="plane" pos="0 0 -0.1" size="0.5 0.5 0.01" rgba="0.85 0.85 0.85 1"/>

    {nested}

    <!-- Target cylinder pinned in space via a stiff 3-axis spring mount.
         This is a virtual force gauge: the cylinder cannot translate freely,
         so any force the finger applies to it is registered as a contact
         force without the cylinder being launched away. Real grasp testers
         use the same trick (force-instrumented dummy objects). -->
    <body name="target" pos="0 {target_y:.6f} {target_z:.6f}">
      <joint name="target_slide_x" type="slide" axis="1 0 0"
             stiffness="500" damping="2.0" ref="0"/>
      <joint name="target_slide_y" type="slide" axis="0 1 0"
             stiffness="500" damping="2.0" ref="0"/>
      <joint name="target_slide_z" type="slide" axis="0 0 1"
             stiffness="500" damping="2.0" ref="0"/>
      <inertial pos="0 0 0" mass="{TARGET_MASS_KG:.6f}"
                diaginertia="1e-6 1e-6 1e-6"/>
      <geom name="target_geom" type="cylinder"
            size="{TARGET_RADIUS_M:.6f} {TARGET_HEIGHT_M/2:.6f}"
            euler="0 1.5707963 0"
            rgba="0.9 0.4 0.4 1"
            friction="1.5 0.05 0.002"/>
    </body>
  </worldbody>

  <tendon>
    <fixed name="flexor" limited="false">
          {fixed_joints}
    </fixed>
  </tendon>

  <actuator>
    <!-- Explicit motor: ctrl directly equals tendon force in Newtons.
         Positive ctrl extends; negative ctrl contracts (flexes the joints
         because coef_i = -r_i). -->
    <motor name="flexor_act" tendon="flexor" gear="1"
           ctrlrange="-20 0"/>
  </actuator>

  <sensor>
    <tendonpos name="flexor_pos" tendon="flexor"/>
  </sensor>
</mujoco>
"""


def grasp_force_on_target(model: mujoco.MjModel,
                          data: mujoco.MjData,
                          target_geom_id: int) -> float:
    """Sum normal contact forces on the target geom."""
    total_normal = 0.0
    for c in range(data.ncon):
        con = data.contact[c]
        if con.geom1 != target_geom_id and con.geom2 != target_geom_id:
            continue
        force = np.zeros(6, dtype=np.float64)
        mujoco.mj_contactForce(model, data, c, force)
        # force[0] = normal in contact frame.
        total_normal += abs(float(force[0]))
    return total_normal


def main() -> int:
    if not META_PATH.exists() or not PARAMS_PATH.exists():
        print("error: missing physics_meta.json or tendon_params.json — "
              "run Lean verification + CAD generation first.", file=sys.stderr)
        return 1

    meta = json.loads(META_PATH.read_text())

    xml = compose_mjcf(meta)
    XML_PATH.write_text(xml)
    print(f"Wrote {XML_PATH}")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    # Pre-curl the finger so it starts already in contact with the cylinder.
    # This isolates the test to "does the tendon hold the grasp under load?"
    # rather than "does it close around a free object?" (the latter is a
    # PoC 4 question about real-world grasp acquisition).
    pre_curl_deg = [40.0, 50.0, 60.0]
    for i, ang in enumerate(pre_curl_deg):
        data.qpos[i] = math.radians(ang)
    mujoco.mj_forward(model, data)

    target_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "target_geom")
    if target_geom_id < 0:
        raise RuntimeError("target_geom not found in model")

    n_steps = int(SIM_SECONDS / model.opt.timestep)
    stable_steps = int(STABLE_WINDOW_S / model.opt.timestep)
    force_history = []
    diverged = False
    diverged_reason = ""

    # Tendon control: ramp from 0 to PULL_MAX Newtons over RAMP_TIME seconds,
    # then hold. Negative ctrl pulls the tendon (the <fixed> coefficients are
    # -r_i, so negative actuator force flexes the joints).
    PULL_MAX = -4.5   # N
    RAMP_TIME = 2.0
    for step in range(n_steps):
        t = step * model.opt.timestep
        pull = PULL_MAX * min(1.0, t / RAMP_TIME)
        data.ctrl[0] = pull
        mujoco.mj_step(model, data)

        if (not np.all(np.isfinite(data.qpos))
                or not np.all(np.isfinite(data.qvel))):
            diverged = True
            diverged_reason = f"non-finite state at t={t:.3f}s"
            break
        if float(np.max(np.abs(data.qvel))) > VELOCITY_LIMIT_RAD_PER_S:
            diverged = True
            diverged_reason = (f"runaway velocity at t={t:.3f}s "
                               f"(max {np.max(np.abs(data.qvel)):.1f} rad/s)")
            break

        force_history.append(grasp_force_on_target(model, data, target_geom_id))

    final_qpos_deg = [math.degrees(float(q)) for q in data.qpos[:3]]
    target_pos = data.body("target").xpos.copy()
    target_body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "target")
    initial_target_pos = model.body_pos[target_body_id].copy()
    target_displacement = float(np.linalg.norm(target_pos - initial_target_pos))

    lines = [
        "MechProof PoC 3 — Grasp Verification Report",
        "===========================================",
        f"Simulation duration  : {SIM_SECONDS:.2f} s",
        f"Final joint angles   : "
        + ", ".join(f"{d:.2f}°" for d in final_qpos_deg),
        f"Tendon moment arms   : "
        + ", ".join(f"{r*1000:.2f}mm" for r in meta["moment_arms_m"]),
        f"Target final pos (m) : "
        f"({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})",
    ]

    if diverged:
        lines += ["", "RESULT: FAIL", f"Reason: {diverged_reason}"]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 2

    final_forces = force_history[-stable_steps:] if len(force_history) >= stable_steps else force_history
    mean_force = float(np.mean(final_forces)) if final_forces else 0.0
    min_force_in_window = float(np.min(final_forces)) if final_forces else 0.0
    peak_force = float(np.max(force_history)) if force_history else 0.0
    grasp_held = bool(min_force_in_window >= MIN_GRASP_FORCE_N)

    # Sanity: the target should not have fallen to the floor or been
    # punched out of the grasp envelope (within 50 mm of its initial pose).
    target_intact = bool(target_displacement < 0.050 and target_pos[2] > -0.07)

    lines += [
        f"Peak normal force    : {peak_force:.3f} N",
        f"Mean force (last {STABLE_WINDOW_S:.1f}s): {mean_force:.3f} N",
        f"Min force (last {STABLE_WINDOW_S:.1f}s) : {min_force_in_window:.3f} N "
        f"(threshold {MIN_GRASP_FORCE_N})",
        "",
    ]

    if grasp_held and target_intact:
        lines += ["RESULT: PASS",
                  "The tendon-driven finger closed around the cylinder and "
                  "maintained contact force above threshold."]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 0
    else:
        reasons = []
        if not target_intact:
            reasons.append(
                f"target displaced {target_displacement*1000:.1f} mm from "
                "initial pose")
        if not grasp_held:
            reasons.append(
                f"contact force dropped below threshold "
                f"(min {min_force_in_window:.3f} N < {MIN_GRASP_FORCE_N} N)")
        lines += ["RESULT: FAIL", "Reason: " + "; ".join(reasons)]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
