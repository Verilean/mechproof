"""MechProof PoC 11 — URDF exporter.

Builds `out/mechproof_humanoid.urdf` from the same Lean-emitted JSONs the
MuJoCo scene consumes (`leg_params.json`, `arm_params.json`,
`hand_params.json`, `hand_physics_meta.json`). The output URDF is a
top-down kinematic chain compatible with ROS 2 / RViz / Gazebo / IsaacSim:

  pelvis (free-floating in URDF — typically attached to `world` by the
          loader as a `base_link`)
    ├── left_thigh → left_shin → left_foot     (6 leg DOFs)
    ├── right_thigh → right_shin → right_foot
    ├── right_shoulder → ... → right_wrist_roll  (6 arm DOFs)
    └── right_palm → 5 finger chains             (6 hand DOFs)

Visual / collision meshes reference the STEP files in the same directory
(loaders typically prefer STL/OBJ but accept STEP; consumers can convert
once at install time). Inertials are computed analytically from the
masses Lean asserted, mirroring `simulate_stand.py`'s inertia model so
the URDF stays consistent with the proof-verified physics.
"""

from __future__ import annotations

import json
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
LEG_PARAMS    = REPO_ROOT / "out" / "leg_params.json"
ARM_PARAMS    = REPO_ROOT / "out" / "arm_params.json"
HAND_PARAMS   = REPO_ROOT / "out" / "hand_params.json"
HAND_META     = REPO_ROOT / "out" / "hand_physics_meta.json"
URDF_PATH     = REPO_ROOT / "out" / "mechproof_humanoid.urdf"


def require(p: pathlib.Path) -> None:
    if not p.exists():
        print(f"error: {p} missing — run `make poc8` first.", file=sys.stderr)
        sys.exit(1)


def add_inertial(link: ET.Element, mass: float, ixx: float, iyy: float,
                 izz: float, com=(0.0, 0.0, 0.0)) -> None:
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin",
                  xyz=f"{com[0]} {com[1]} {com[2]}", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.6f}")
    ET.SubElement(inertial, "inertia",
                  ixx=f"{ixx:.9f}", iyy=f"{iyy:.9f}", izz=f"{izz:.9f}",
                  ixy="0", ixz="0", iyz="0")


def add_box_geom(parent: ET.Element, tag: str,
                 size: tuple, origin=(0.0, 0.0, 0.0)) -> None:
    g = ET.SubElement(parent, tag)
    ET.SubElement(g, "origin",
                  xyz=f"{origin[0]} {origin[1]} {origin[2]}", rpy="0 0 0")
    geom = ET.SubElement(g, "geometry")
    ET.SubElement(geom, "box", size=f"{size[0]} {size[1]} {size[2]}")


def add_cylinder_geom(parent: ET.Element, tag: str,
                      radius: float, length: float,
                      origin=(0.0, 0.0, 0.0),
                      rpy=(0.0, 0.0, 0.0)) -> None:
    g = ET.SubElement(parent, tag)
    ET.SubElement(g, "origin",
                  xyz=f"{origin[0]} {origin[1]} {origin[2]}",
                  rpy=f"{rpy[0]} {rpy[1]} {rpy[2]}")
    geom = ET.SubElement(g, "geometry")
    ET.SubElement(geom, "cylinder",
                  radius=f"{radius}", length=f"{length}")


def add_mesh_geom(parent: ET.Element, tag: str, mesh_path: str,
                  origin=(0.0, 0.0, 0.0)) -> None:
    g = ET.SubElement(parent, tag)
    ET.SubElement(g, "origin",
                  xyz=f"{origin[0]} {origin[1]} {origin[2]}", rpy="0 0 0")
    geom = ET.SubElement(g, "geometry")
    ET.SubElement(geom, "mesh", filename=mesh_path)


def add_revolute_joint(robot: ET.Element, name: str,
                       parent: str, child: str,
                       axis=(1.0, 0.0, 0.0),
                       origin=(0.0, 0.0, 0.0),
                       lower=-math.pi, upper=math.pi,
                       effort=50.0, velocity=5.0,
                       rpy=(0.0, 0.0, 0.0)) -> None:
    j = ET.SubElement(robot, "joint", name=name, type="revolute")
    ET.SubElement(j, "parent", link=parent)
    ET.SubElement(j, "child", link=child)
    ET.SubElement(j, "origin",
                  xyz=f"{origin[0]} {origin[1]} {origin[2]}",
                  rpy=f"{rpy[0]} {rpy[1]} {rpy[2]}")
    ET.SubElement(j, "axis", xyz=f"{axis[0]} {axis[1]} {axis[2]}")
    ET.SubElement(j, "limit", lower=f"{lower}", upper=f"{upper}",
                  effort=f"{effort}", velocity=f"{velocity}")


def build_link_solid_cylinder(robot: ET.Element, name: str,
                              mass: float, length: float,
                              radius: float, mesh_basename: str) -> None:
    """A capsule-like link whose long axis is -Z (URDF child frame)."""
    link = ET.SubElement(robot, "link", name=name)
    iperp = mass * (length * length / 12.0 + radius * radius / 4.0)
    iaxial = mass * radius * radius / 2.0
    add_inertial(link, mass, iperp, iaxial, iperp,
                 com=(0.0, 0.0, -length / 2.0))
    add_cylinder_geom(link, "visual", radius, length,
                      origin=(0.0, 0.0, -length / 2.0))
    add_mesh_geom(link, "visual", mesh_basename,
                  origin=(0.0, 0.0, -length / 2.0))
    add_cylinder_geom(link, "collision", radius, length,
                      origin=(0.0, 0.0, -length / 2.0))


def build_link_box(robot: ET.Element, name: str,
                   mass: float, size: tuple, mesh_basename: str,
                   origin=(0.0, 0.0, 0.0)) -> None:
    link = ET.SubElement(robot, "link", name=name)
    sx, sy, sz = size
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    add_inertial(link, mass, ixx, iyy, izz, com=origin)
    add_box_geom(link, "visual", size, origin=origin)
    add_mesh_geom(link, "visual", mesh_basename, origin=origin)
    add_box_geom(link, "collision", size, origin=origin)


def main() -> int:
    for p in (LEG_PARAMS, ARM_PARAMS, HAND_PARAMS, HAND_META):
        require(p)

    leg_p = json.loads(LEG_PARAMS.read_text())
    arm_p = json.loads(ARM_PARAMS.read_text())
    hand_p = json.loads(HAND_PARAMS.read_text())
    hand_meta = json.loads(HAND_META.read_text())

    torso = leg_p["torso"]
    leg = leg_p["leg"]

    robot = ET.Element("robot", name="mechproof_humanoid")
    ET.SubElement(robot, "material", name="metal_blue")
    metal_blue = robot.find("material[@name='metal_blue']")
    ET.SubElement(metal_blue, "color", rgba="0.55 0.60 0.75 1")

    # --- Pelvis / torso ---------------------------------------------------
    torso_mass = float(torso["mass"]) + float(leg_p["upperBodyMass"])
    torso_size = (float(torso["width"]),
                  float(torso["depth"]),
                  float(torso["height"]))
    build_link_box(robot, "torso", torso_mass, torso_size, "torso.step",
                   origin=(0.0, 0.0, 0.0))

    # --- Legs (mirrored) -------------------------------------------------
    thigh_mass = float(leg["thighMass"])
    shin_mass  = float(leg["shinMass"])
    foot_mass  = float(leg["footMass"])
    thigh_len  = float(leg["thighLen"])
    shin_len   = float(leg["shinLen"])
    foot_len   = float(leg["footLen"])
    foot_width = float(leg["footWidth"])
    hip_off    = float(leg["hipOffsetX"])

    for side, sign in (("left", -1), ("right", +1)):
        build_link_solid_cylinder(robot, f"{side}_thigh",
                                  thigh_mass, thigh_len, 0.028, "thigh.step")
        build_link_solid_cylinder(robot, f"{side}_shin",
                                  shin_mass, shin_len, 0.026, "shin.step")
        build_link_box(robot, f"{side}_foot",
                       foot_mass, (foot_width, foot_len, 0.020),
                       "foot.step",
                       origin=(0.0, 0.0, -0.010))

        # Hip yaw (Z) at the bottom of the torso, offset laterally.
        add_revolute_joint(robot, f"{side}_hip_yaw",
                           "torso", f"{side}_thigh",
                           axis=(0, 0, 1),
                           origin=(sign * hip_off, 0.0, -torso_size[2] / 2.0),
                           lower=-1.0, upper=1.0,
                           effort=float(leg["hipPitchTau"]), velocity=5.0)
        # Knee at the bottom of the thigh.
        add_revolute_joint(robot, f"{side}_knee",
                           f"{side}_thigh", f"{side}_shin",
                           axis=(1, 0, 0),
                           origin=(0.0, 0.0, -thigh_len),
                           lower=0.0, upper=2.5,
                           effort=float(leg["kneeTau"]), velocity=5.0)
        # Ankle pitch + roll at the bottom of the shin. Two joints back-
        # to-back; URDF expresses them as separate links with massless dummies.
        ankle_dummy = ET.SubElement(robot, "link",
                                    name=f"{side}_ankle_dummy")
        add_inertial(ankle_dummy, 1e-4, 1e-7, 1e-7, 1e-7)
        add_revolute_joint(robot, f"{side}_ankle_pitch",
                           f"{side}_shin", f"{side}_ankle_dummy",
                           axis=(1, 0, 0),
                           origin=(0.0, 0.0, -shin_len),
                           lower=-1.0, upper=1.0,
                           effort=float(leg["ankleTau"]), velocity=5.0)
        add_revolute_joint(robot, f"{side}_ankle_roll",
                           f"{side}_ankle_dummy", f"{side}_foot",
                           axis=(0, 1, 0),
                           origin=(0.0, 0.0, 0.0),
                           lower=-0.5, upper=0.5,
                           effort=float(leg["ankleTau"]), velocity=5.0)

    # --- Arm (right side only, like in PoC 6) ----------------------------
    arm_links = [
        ("shoulder_pitch_link", float(arm_p["l1"]), 0.022, "arm_link1.step"),
        ("elbow_link",          float(arm_p["l2"]), 0.022, "arm_link2.step"),
        ("wrist_link",          float(arm_p["l3"]), 0.022, "arm_link3.step"),
    ]
    masses = [float(arm_p["m1"]), float(arm_p["m2"]), float(arm_p["m3"])]
    for (name, length, radius, mesh), mass in zip(arm_links, masses):
        build_link_solid_cylinder(robot, name, mass, length, radius, mesh)

    # Shoulder mounted on the upper +X edge of the torso. Joint chain:
    #   torso -> shoulder_pan (Z) -> shoulder_pitch (X) -> elbow (X)
    #          -> wrist (X)  (we collapse wrist pitch/yaw/roll into one revolute)
    shoulder_dummy = ET.SubElement(robot, "link", name="shoulder_dummy")
    add_inertial(shoulder_dummy, 1e-4, 1e-7, 1e-7, 1e-7)

    add_revolute_joint(robot, "shoulder_pan",
                       "torso", "shoulder_dummy",
                       axis=(0, 0, 1),
                       origin=(torso_size[0] / 2.0, 0.0, torso_size[2] / 2.0),
                       lower=-3.14, upper=3.14,
                       effort=float(arm_p["tauShoulder"]), velocity=5.0)
    add_revolute_joint(robot, "shoulder_pitch",
                       "shoulder_dummy", "shoulder_pitch_link",
                       axis=(1, 0, 0),
                       origin=(0, 0, 0),
                       lower=-2.0, upper=0.5,
                       effort=float(arm_p["tauShoulder"]), velocity=5.0)
    add_revolute_joint(robot, "elbow",
                       "shoulder_pitch_link", "elbow_link",
                       axis=(1, 0, 0),
                       origin=(0, 0, -float(arm_p["l1"])),
                       lower=-2.5, upper=2.5,
                       effort=float(arm_p["tauElbow"]), velocity=5.0)
    add_revolute_joint(robot, "wrist",
                       "elbow_link", "wrist_link",
                       axis=(1, 0, 0),
                       origin=(0, 0, -float(arm_p["l2"])),
                       lower=-2.0, upper=2.0,
                       effort=float(arm_p["tauWrist"]), velocity=5.0)

    # --- Palm + fingers (single tendon-driven flexion per finger) --------
    palm_geom = hand_p["palm"]
    palm_mass = float(hand_meta["palm"]["mass_kg"])
    build_link_box(robot, "palm", palm_mass,
                   (float(palm_geom["width"]),
                    float(palm_geom["length"]),
                    float(palm_geom["thickness"])),
                   "palm.step")
    add_revolute_joint(robot, "wrist_to_palm",
                       "wrist_link", "palm",
                       axis=(0, 1, 0),
                       origin=(0, 0, -float(arm_p["l3"])),
                       lower=-3.14, upper=3.14,
                       effort=4.0, velocity=5.0)

    for fname in ("index", "middle", "ring", "pinky", "thumb"):
        finger = hand_meta["fingers"][fname]
        mount = finger["mount"]
        # We model each finger as a single revolute "flexion" joint at
        # the proximal mount, with the three links lumped to keep the
        # URDF compact. Future revisions can break them out.
        L = sum(link["length_m"] for link in finger["links"])
        m = sum(link["mass_kg"] for link in finger["links"])
        link_name = f"{fname}_finger"
        build_link_solid_cylinder(robot, link_name, m, L, 0.006,
                                  f"{fname}_link1.step")
        add_revolute_joint(robot, f"{fname}_flexion",
                           "palm", link_name,
                           axis=(-1, 0, 0),
                           origin=(float(mount["px"]),
                                   float(mount["py"]),
                                   float(mount["pz"])),
                           lower=0.0, upper=1.57,
                           effort=6.0, velocity=2.0)

    # --- Pretty-print and write ------------------------------------------
    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ")
    URDF_PATH.parent.mkdir(parents=True, exist_ok=True)
    tree.write(URDF_PATH, encoding="utf-8", xml_declaration=True)

    n_links = len(robot.findall("link"))
    n_joints = len(robot.findall("joint"))
    print(f"Wrote {URDF_PATH}")
    print(f"  links  : {n_links}")
    print(f"  joints : {n_joints}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
