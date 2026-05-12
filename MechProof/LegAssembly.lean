import MechProof.ArmAssembly

namespace MechProof

/-! ## Humanoid lower body: balance + squat-torque proof.

    World convention (also used by `simulate_stand.py`):
      * origin on the ground midway between the two feet,
      * +Z up,
      * +Y forward (the direction the robot faces),
      * +X to the robot's right.

    The two legs are mirror images of each other about the X = 0 plane.
    Each leg has 6 DOF (hip yaw / roll / pitch, knee pitch, ankle pitch /
    roll). For PoC 8 we only check static feasibility in the upright
    standing pose and the worst-case 90° knee squat — not the full pose
    space. The proof intentionally uses simple axis-aligned bounding
    geometry so `native_decide` can finish in O(1).
-/

structure TorsoParams where
  width      : Float   -- m, X (lateral)
  depth      : Float   -- m, Y (forward)
  height     : Float   -- m, Z
  mass       : Float   -- kg, structural torso only
  deriving Repr

structure LegParams where
  thighLen     : Float
  shinLen      : Float
  footLen      : Float    -- foot length along +Y
  footWidth    : Float    -- foot width along +X
  thighMass    : Float
  shinMass     : Float
  footMass     : Float
  -- Lateral offset from the body midline (X) where the hip pivot sits.
  hipOffsetX   : Float
  -- Stall torques at the load-bearing pitch joints.
  hipPitchTau  : Float
  kneeTau      : Float
  ankleTau     : Float
  deriving Repr

/-- The full lower body: a torso + two mirrored legs + everything mounted
    on the torso (arm + hand + payload), lumped into `upperBodyMass`. -/
structure LowerBody where
  torso          : TorsoParams
  leg            : LegParams
  upperBodyMass  : Float   -- kg, everything mounted above the pelvis CoM
  deriving Repr

/-- Total mass of the assembled humanoid. -/
def LowerBody.totalMass (lb : LowerBody) : Float :=
  lb.torso.mass + lb.upperBodyMass +
    2.0 * (lb.leg.thighMass + lb.leg.shinMass + lb.leg.footMass)

/-! ### Static balance

    In a symmetric standing pose (feet flat, hips at ±hipOffsetX), every
    body's CoM lies on the X = 0 plane and the Y = 0 plane (assuming the
    torso CoM is centred). The systemic CoM is therefore (0, 0, z), which
    projects onto the ground at (0, 0).

    The support polygon (union of the two feet) is the axis-aligned
    rectangle
      X ∈ [-hipOffsetX - footWidth/2, +hipOffsetX + footWidth/2]
      Y ∈ [-footLen/2, +footLen/2].

    We require the projected CoM to lie inside the polygon with a strict
    safety margin. -/
def staticBalanceMarginM : Float := 0.02

/-- The 2-D extent of the support polygon (half-widths along X and Y). -/
def LowerBody.supportHalfX (lb : LowerBody) : Float :=
  lb.leg.hipOffsetX + lb.leg.footWidth / 2

def LowerBody.supportHalfY (lb : LowerBody) : Float :=
  lb.leg.footLen / 2

/-- For our symmetric standing pose the CoM projects to (0, 0). The
    check therefore reduces to `0 + margin < supportHalfX` and the same
    for Y. Marked `abbrev` so `native_decide` can unfold it directly. -/
abbrev LowerBody.Balanced (lb : LowerBody) : Prop :=
  staticBalanceMarginM < lb.supportHalfX ∧
  staticBalanceMarginM < lb.supportHalfY

/-! ### Squat-torque check

    The worst-case static load on the knee occurs at 90° flexion, when the
    thigh is horizontal. The mass above the knee (one shin, foot, half of
    thigh, and the entire upper-body+torso) hangs off the knee axis at a
    horizontal moment arm equal to the thigh length. We require:

      knee stall torque  >  M_above_knee · g · thighLen.

    The mass above the knee is the sum, per leg, of half the thigh (the
    half above the knee), plus the upper body lumped at the hip. We
    apportion the torso + upperBodyMass evenly across the two knees
    because in a symmetric squat each knee carries half the body weight. -/
def LowerBody.massAboveKnee (lb : LowerBody) : Float :=
  lb.leg.thighMass / 2.0 +
    (lb.torso.mass + lb.upperBodyMass) / 2.0

def LowerBody.requiredKneeTorque (lb : LowerBody) : Float :=
  GRAVITY_M_PER_S2 * lb.massAboveKnee * lb.leg.thighLen

abbrev LowerBody.SquatTorqueSufficient (lb : LowerBody) : Prop :=
  lb.requiredKneeTorque < lb.leg.kneeTau

/-- Dimensional sanity. -/
def LowerBody.WellFormed (lb : LowerBody) : Prop :=
  0 < lb.torso.width ∧ 0 < lb.torso.depth ∧ 0 < lb.torso.height ∧
  0 < lb.torso.mass ∧
  0 < lb.leg.thighLen ∧ 0 < lb.leg.shinLen ∧
  0 < lb.leg.footLen ∧ 0 < lb.leg.footWidth ∧
  0 < lb.leg.thighMass ∧ 0 < lb.leg.shinMass ∧ 0 < lb.leg.footMass ∧
  0 < lb.leg.hipOffsetX ∧
  0 < lb.leg.hipPitchTau ∧ 0 < lb.leg.kneeTau ∧ 0 < lb.leg.ankleTau ∧
  0 < lb.upperBodyMass

end MechProof
