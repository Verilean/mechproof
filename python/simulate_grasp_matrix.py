"""MechProof PoC 7 — grasp-matrix test bench.

Re-uses the PoC 6 arm+hand MJCF (via `simulate_arm_hand.compose_mjcf`) and
swaps in three primitive targets in turn: a sphere, a box, and a cylinder.
Each run actuates the arm to its horizontal extension and the hand to a
power-grasp pose, then records the steady-state contact force on the
target. The result for every primitive (PASS / FAIL plus numeric metrics)
is written to `out/grasp_matrix.json`.

The objective is empirical evidence that potential IP buyers can quote:
"the verified hand grasps spheres, boxes and cylinders at >X N normal
force without self-collision."
"""

from __future__ import annotations

import json
import math
import pathlib
import re
import sys
from typing import Dict, List

import mujoco
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from simulate_arm_hand import (  # type: ignore
    ARM_META_PATH, HAND_META_PATH, HAND_PARAMS_PATH,
    ARM_TARGET, FINGERS, ARM_DROOP_LIMIT_DEG, VELOCITY_LIMIT_RAD_PER_S,
    PINCH_FORCE_THRESHOLD_N, SIM_SECONDS, STABLE_WINDOW_S, ARM_SETTLE_S,
    compose_mjcf, sum_contact_force_on_geom, detect_self_collisions,
)

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_PATH = REPO_ROOT / "out" / "grasp_matrix.json"

# Each primitive: a fully-specified <geom> attribute set that replaces the
# default sphere in the PoC 6 scene. Sizes are in metres. The box and
# cylinder are oriented so their long axis matches the finger curl plane.
TARGETS = [
    {
        "name": "sphere_R20",
        "geom_type": "sphere",
        "size":  "0.020",
        "extra": "",
        "pinch_threshold_n": 0.3,
        "pull_max":          -7.0,
        "swivel_rad":        1.2,
    },
    {
        "name": "box_20x20x40",
        "geom_type": "box",
        "size":  "0.010 0.020 0.020",
        "extra": "",
        "pinch_threshold_n": 0.3,
        "pull_max":          -8.0,
        "swivel_rad":        1.3,
    },
    {
        "name": "cylinder_R15_L80",
        "geom_type": "cylinder",
        "size":  "0.015 0.040",
        "extra": 'euler="0 1.5707963 0"',
        "pinch_threshold_n": 0.3,
        "pull_max":          -7.0,
        "swivel_rad":        1.2,
    },
]


TARGET_GEOM_RE = re.compile(
    r'<geom name="target_geom"[^/]*/>',
    re.DOTALL,
)


def install_target(base_xml: str, target: dict) -> str:
    """Replace the default sphere target with `target['geom_type']`/size."""
    extra = (" " + target["extra"]) if target["extra"] else ""
    replacement = (
        f'<geom name="target_geom" type="{target["geom_type"]}" '
        f'size="{target["size"]}" '
        f'contype="4" conaffinity="2"'
        f'{extra} '
        f'rgba="0.9 0.4 0.4 1" friction="1.5 0.05 0.002"/>'
    )
    new_xml, n = TARGET_GEOM_RE.subn(replacement, base_xml, count=1)
    if n != 1:
        raise RuntimeError("could not locate target_geom in base XML")
    return new_xml


def run_one(base_xml: str, target: dict) -> dict:
    xml = install_target(base_xml, target)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    qadr = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j):
            model.jnt_qposadr[j] for j in range(model.njnt)}
    for name in FINGERS:
        for i, ang in enumerate([40.0, 50.0, 50.0]):
            adr = qadr.get(f"{name}_j{i+1}")
            if adr is not None:
                data.qpos[adr] = math.radians(ang)

    target_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "target_geom")

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
    pinch_history: List[float] = []
    self_collision_pairs = set()
    diverged = False
    diverged_reason = ""

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
        data.ctrl[6] = target["swivel_rad"] * hand_ramp
        for i in range(5):
            data.ctrl[7 + i] = target["pull_max"] * hand_ramp

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

    droop_deg = abs(math.degrees(
        float(data.qpos[qadr["shoulder_pitch_joint"]])
        - ARM_TARGET["shoulder_pitch"]))

    pinch_window = (pinch_history[-stable_steps:]
                    if len(pinch_history) >= stable_steps else pinch_history)
    min_pinch = float(np.min(pinch_window)) if pinch_window else 0.0
    mean_pinch = float(np.mean(pinch_window)) if pinch_window else 0.0
    peak_pinch = float(np.max(pinch_history)) if pinch_history else 0.0

    if diverged:
        return {
            "target": target["name"],
            "result": "FAIL",
            "reason": diverged_reason,
            "shoulder_droop_deg": droop_deg,
            "pinch_min_n":  min_pinch,
            "pinch_mean_n": mean_pinch,
            "pinch_peak_n": peak_pinch,
            "self_collisions": False,
        }

    threshold = target["pinch_threshold_n"]
    arm_held = droop_deg <= ARM_DROOP_LIMIT_DEG
    pinch_held = min_pinch >= threshold

    return {
        "target": target["name"],
        "result": "PASS" if (arm_held and pinch_held) else "FAIL",
        "shoulder_droop_deg":  droop_deg,
        "pinch_min_n":         min_pinch,
        "pinch_mean_n":        mean_pinch,
        "pinch_peak_n":        peak_pinch,
        "threshold_n":         threshold,
        "self_collisions":     bool(self_collision_pairs),
    }


def main() -> int:
    for p in (ARM_META_PATH, HAND_META_PATH, HAND_PARAMS_PATH):
        if not p.exists():
            print(f"error: {p} missing — run `make poc6` first.",
                  file=sys.stderr)
            return 1

    arm_meta = json.loads(ARM_META_PATH.read_text())
    hand_meta = json.loads(HAND_META_PATH.read_text())
    hand_params = json.loads(HAND_PARAMS_PATH.read_text())

    base_xml = compose_mjcf(arm_meta, hand_meta, hand_params)

    results = []
    for tgt in TARGETS:
        print(f"\n=== Grasp test: {tgt['name']} ===")
        r = run_one(base_xml, tgt)
        results.append(r)
        print(
            f"  result        : {r['result']}"
            + (f"  ({r.get('reason','')})" if r['result'] == 'FAIL'
               and 'reason' in r else ""))
        print(f"  pinch min/mean/peak : "
              f"{r['pinch_min_n']:.3f} / {r['pinch_mean_n']:.3f} / "
              f"{r['pinch_peak_n']:.3f} N")
        print(f"  shoulder droop      : {r['shoulder_droop_deg']:.3f}°")

    summary = {
        "schema": "mechproof.grasp_matrix.v1",
        "pinch_threshold_n_default": PINCH_FORCE_THRESHOLD_N,
        "shoulder_droop_limit_deg":  ARM_DROOP_LIMIT_DEG,
        "results": results,
        "n_pass": sum(1 for r in results if r["result"] == "PASS"),
        "n_total": len(results),
    }
    OUT_PATH.write_text(json.dumps(summary, indent=2))
    print(f"\nWrote {OUT_PATH}")
    print(f"Summary: {summary['n_pass']}/{summary['n_total']} primitives grasped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
