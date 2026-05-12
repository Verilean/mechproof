import MechProof.HandAssembly

namespace MechProof

/-! ## 6-DOF arm: static-torque proof at horizontal extension

    Joint order (proximal → distal):
      1. shoulderPan   — about world +Z, no gravity-induced static torque
      2. shoulderPitch — about local +X, **carries the entire arm**
      3. elbowPitch    — about local +X, carries link 2 + link 3 + payload
      4. wristPitch    — about local +X, carries link 3 + payload
      5. wristYaw      — about local +Z, no static torque
      6. wristRoll     — about local +Y, no static torque

    All distances in metres, masses in kg, torques in N·m.
-/

/-- Earth gravity used by the static torque check (m/s²). -/
def GRAVITY_M_PER_S2 : Float := 9.81

/-- Combined payload + hand mass at the wrist flange (kg).
    Hand mass is the PoC 5 estimate; payload is the design spec. -/
def PAYLOAD_MASS_KG : Float := 2.0
def HAND_MASS_KG    : Float := 0.5

/-- The three link lengths and masses, plus the three motor stall torques on
    the pitch axes (the only axes that resist gravity at horizontal pose). -/
structure ArmParams where
  l1      : Float   -- m, shoulder → elbow
  l2      : Float   -- m, elbow → wrist
  l3      : Float   -- m, wrist → hand mount
  m1      : Float   -- kg
  m2      : Float
  m3      : Float
  tauShoulder : Float   -- N·m, motor stall torque at shoulder pitch
  tauElbow    : Float
  tauWrist    : Float
  deriving Repr

/-- Static torque required at the **shoulder pitch** joint when the arm is
    fully extended horizontally. Each link contributes its weight times the
    horizontal distance from the shoulder to its CoM (assumed at link's
    midpoint). The wrist mass (hand + payload) contributes its full weight
    at the wrist-tip distance `l1 + l2 + l3`. -/
def ArmParams.requiredShoulderTorque (a : ArmParams) : Float :=
  let d1 := a.l1 / 2.0
  let d2 := a.l1 + a.l2 / 2.0
  let d3 := a.l1 + a.l2 + a.l3 / 2.0
  let dWrist := a.l1 + a.l2 + a.l3
  GRAVITY_M_PER_S2 *
    (a.m1 * d1 + a.m2 * d2 + a.m3 * d3 +
     (HAND_MASS_KG + PAYLOAD_MASS_KG) * dWrist)

/-- Static torque required at the **elbow pitch** joint at horizontal pose:
    only link 2, link 3, the hand and the payload sit distal to the elbow. -/
def ArmParams.requiredElbowTorque (a : ArmParams) : Float :=
  let d2 := a.l2 / 2.0
  let d3 := a.l2 + a.l3 / 2.0
  let dWrist := a.l2 + a.l3
  GRAVITY_M_PER_S2 *
    (a.m2 * d2 + a.m3 * d3 +
     (HAND_MASS_KG + PAYLOAD_MASS_KG) * dWrist)

/-- Static torque required at the **wrist pitch** joint at horizontal pose. -/
def ArmParams.requiredWristTorque (a : ArmParams) : Float :=
  let d3 := a.l3 / 2.0
  let dWrist := a.l3
  GRAVITY_M_PER_S2 *
    (a.m3 * d3 + (HAND_MASS_KG + PAYLOAD_MASS_KG) * dWrist)

/-- Dimensional / physical sanity. -/
def ArmParams.WellFormed (a : ArmParams) : Prop :=
  0 < a.l1 ∧ 0 < a.l2 ∧ 0 < a.l3 ∧
  0 < a.m1 ∧ 0 < a.m2 ∧ 0 < a.m3 ∧
  0 < a.tauShoulder ∧ 0 < a.tauElbow ∧ 0 < a.tauWrist

/-- **Stall-Torque-Sufficient Theorem.** Every pitch motor's stall torque
    strictly exceeds the static torque demand at horizontal full extension.
    `abbrev` lets `native_decide` unfold the inequality directly. -/
abbrev ArmParams.StallSufficient (a : ArmParams) : Prop :=
  a.requiredShoulderTorque < a.tauShoulder ∧
  a.requiredElbowTorque    < a.tauElbow ∧
  a.requiredWristTorque    < a.tauWrist

end MechProof
