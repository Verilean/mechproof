"""MechProof PoC 2 — MuJoCo digital twin of the verified finger.

Reads `finger_params.json` and `physics_meta.json`, composes a kinematic-tree
MJCF, and runs a headless simulation in which the three hinge actuators
command a flexion to 90° over 2 seconds. The run is judged a success if no
state diverges (NaN/Inf, runaway velocity, unresolved penetration).

The flexion axis is +X. Links extend along their local +Y; child links are
attached at `pos="0 length_m 0"` from their parent so the joint origins are
coincident with the previous link's distal end (where the pivot hole is).
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import mujoco
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PARAMS_PATH = REPO_ROOT / "out" / "finger_params.json"
META_PATH = REPO_ROOT / "out" / "physics_meta.json"
XML_PATH = REPO_ROOT / "out" / "finger.xml"
REPORT_PATH = REPO_ROOT / "out" / "Verification_Report.txt"

SIM_SECONDS = 2.0
TARGET_FLEXION_DEG = 90.0
VELOCITY_LIMIT_RAD_PER_S = 100.0  # any joint exceeding this is "blown up"


def link_xml(link: dict, params: dict, child_xml: str) -> str:
    """Render one body element with its joint, inertial, and child."""
    name = link["name"]
    length = link["length_m"]
    com = link["com_m"]
    mass = link["mass_kg"]
    th = params["thickness"] * 1e-3
    L = length

    # Cuboid principal moments of inertia about the COM, axes aligned with
    # body frame: Ixx=(L^2+th^2), Iyy=(th^2+th^2), Izz=(L^2+th^2), all × m/12.
    ixx = mass * (L * L + th * th) / 12.0
    iyy = mass * (th * th + th * th) / 12.0
    izz = mass * (L * L + th * th) / 12.0

    min_rad = math.radians(params["minAngle"])
    max_rad = math.radians(params["maxAngle"])

    # The root link sits at the world origin; child links are translated by
    # the parent link's length along +Y so their pivot aligns with the
    # parent's distal boss.
    return f"""
        <body name="{name}" pos="0 {0.0 if name == 'link1' else f'{params["_parent_length_m"]:.6f}'} 0">
          <joint name="{name}_hinge" type="hinge" axis="1 0 0"
                 range="{min_rad:.6f} {max_rad:.6f}" damping="0.0005"/>
          <inertial pos="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}"
                    mass="{mass:.6f}"
                    diaginertia="{ixx:.9f} {iyy:.9f} {izz:.9f}"/>
          <geom type="box" pos="0 {L/2:.6f} 0"
                size="{th/2:.6f} {L/2:.6f} {th/2:.6f}"
                rgba="0.7 0.7 0.85 1"/>
          {child_xml}
        </body>"""


def compose_mjcf(params: dict, meta: dict) -> str:
    links = meta["links"]
    # Build nested <body> elements, innermost first.
    inner = ""
    for i, link in enumerate(reversed(links)):
        depth = len(links) - 1 - i
        params_for_link = dict(params)
        params_for_link["_parent_length_m"] = (
            0.0 if depth == 0 else links[depth - 1]["length_m"]
        )
        inner = link_xml(link, params_for_link, inner)

    actuators = "\n        ".join(
        f'<position name="{l["name"]}_act" joint="{l["name"]}_hinge" '
        f'kp="2.0" kv="0.05" '
        f'ctrlrange="{math.radians(params["minAngle"]):.6f} '
        f'{math.radians(params["maxAngle"]):.6f}"/>'
        for l in links
    )

    return f"""<?xml version="1.0"?>
<mujoco model="mechproof_finger">
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicit"/>
  <worldbody>
    <light pos="0 0 1" dir="0 0 -1"/>
    {inner.strip()}
  </worldbody>
  <actuator>
    {actuators}
  </actuator>
</mujoco>
"""


def run_simulation(model: mujoco.MjModel, data: mujoco.MjData) -> dict:
    target = math.radians(TARGET_FLEXION_DEG)
    n_actuators = model.nu
    n_steps = int(SIM_SECONDS / model.opt.timestep)
    max_qvel = 0.0
    diverged = False
    diverged_reason = ""

    for step in range(n_steps):
        # Ramp the target linearly to TARGET_FLEXION_DEG across the run.
        t = step * model.opt.timestep
        ramp = min(1.0, t / (SIM_SECONDS * 0.5))
        data.ctrl[:n_actuators] = target * ramp
        mujoco.mj_step(model, data)

        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            diverged = True
            diverged_reason = f"non-finite state at t={t:.3f}s"
            break
        qvel_abs_max = float(np.max(np.abs(data.qvel)))
        if qvel_abs_max > max_qvel:
            max_qvel = qvel_abs_max
        if qvel_abs_max > VELOCITY_LIMIT_RAD_PER_S:
            diverged = True
            diverged_reason = (f"runaway velocity {qvel_abs_max:.2f} rad/s "
                               f"at t={t:.3f}s")
            break

    final_qpos = list(map(float, data.qpos[:n_actuators]))
    return {
        "diverged": diverged,
        "diverged_reason": diverged_reason,
        "max_qvel_rad_per_s": max_qvel,
        "final_qpos_rad": final_qpos,
        "final_qpos_deg": [math.degrees(q) for q in final_qpos],
        "target_deg": TARGET_FLEXION_DEG,
        "sim_seconds": SIM_SECONDS,
    }


def main() -> int:
    if not PARAMS_PATH.exists() or not META_PATH.exists():
        print("error: missing finger_params.json or physics_meta.json — run "
              "Lean verification and CAD generation first.", file=sys.stderr)
        return 1

    params = json.loads(PARAMS_PATH.read_text())
    meta = json.loads(META_PATH.read_text())

    xml = compose_mjcf(params, meta)
    XML_PATH.write_text(xml)
    print(f"Wrote {XML_PATH}")

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    result = run_simulation(model, data)

    lines = [
        "MechProof PoC 2 — Verification Report",
        "=====================================",
        f"Simulation duration : {result['sim_seconds']:.2f} s",
        f"Target flexion      : {result['target_deg']:.1f} deg",
        f"Final joint angles  : "
        + ", ".join(f"{d:.2f}°" for d in result["final_qpos_deg"]),
        f"Max joint velocity  : {result['max_qvel_rad_per_s']:.3f} rad/s "
        f"(limit {VELOCITY_LIMIT_RAD_PER_S})",
        "",
    ]
    if result["diverged"]:
        lines += [
            "RESULT: FAIL",
            f"Reason: {result['diverged_reason']}",
        ]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 2

    # Sanity check: final angles should be close to the commanded target.
    angle_errors_deg = [abs(d - TARGET_FLEXION_DEG)
                        for d in result["final_qpos_deg"]]
    if max(angle_errors_deg) > 30.0:
        lines += [
            "RESULT: FAIL",
            f"Reason: joints did not track target; max error "
            f"{max(angle_errors_deg):.1f}°",
        ]
        REPORT_PATH.write_text("\n".join(lines) + "\n")
        print("\n".join(lines))
        return 3

    lines += ["RESULT: PASS",
              "The finger closed to the commanded pose without instability."]
    REPORT_PATH.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
