import MechProof.LegAssembly

namespace MechProof

/-! ## Dynamic walking — ZMP stability proof.

    Linear Inverted Pendulum Model (LIPM):

      x_zmp = x_com − (H / g) · ẍ_com
      y_zmp = y_com − (H / g) · ÿ_com

    where `H` is the CoM height above the ankle plane and `g` is gravity.
    A walking gait is "ZMP-stable" iff at every keyframe the computed
    (x_zmp, y_zmp) lies strictly inside an axis-aligned **support polygon**
    centred on the supporting foot (or the convex hull of both feet during
    double support). We discretise the trajectory into a fixed-length
    `List TrajectoryStep` and prove stability point-wise.

    Coordinate convention (matches PoC 8 / simulate_stand.py):
      * +X right, +Y forward, +Z up,
      * origin midway between the feet at the start of the gait.
-/

/-- A single point in the planned CoM trajectory. -/
structure TrajectoryStep where
  comX     : Float
  comY     : Float
  accX     : Float
  accY     : Float
  -- Axis-aligned support polygon for this keyframe. The two `lo`/`hi`
  -- fields delimit the bounding box of the foot (or feet) on the ground.
  loX : Float
  hiX : Float
  loY : Float
  hiY : Float
  deriving Repr

structure WalkingParams where
  comHeight : Float   -- m, H in the LIPM
  gravity   : Float   -- m/s²; reused from ArmAssembly's GRAVITY_M_PER_S2
  -- Strict margin (m) by which the ZMP must sit inside the polygon.
  zmpMargin : Float
  deriving Repr

/-- Project the CoM to its ZMP under the LIPM assumption. -/
def TrajectoryStep.zmpX (w : WalkingParams) (s : TrajectoryStep) : Float :=
  s.comX - (w.comHeight / w.gravity) * s.accX

def TrajectoryStep.zmpY (w : WalkingParams) (s : TrajectoryStep) : Float :=
  s.comY - (w.comHeight / w.gravity) * s.accY

/-- One keyframe is ZMP-stable when the projected ZMP lies inside the
    support polygon with the configured safety margin. -/
abbrev TrajectoryStep.Stable
    (w : WalkingParams) (s : TrajectoryStep) : Prop :=
  s.loX + w.zmpMargin < s.zmpX w  ∧  s.zmpX w < s.hiX - w.zmpMargin  ∧
  s.loY + w.zmpMargin < s.zmpY w  ∧  s.zmpY w < s.hiY - w.zmpMargin

/-- Decidable boolean form of a single keyframe's stability. We compute on
    `Float` directly so `native_decide` can evaluate it. -/
def TrajectoryStep.stableB
    (w : WalkingParams) (s : TrajectoryStep) : Bool :=
  decide (s.loX + w.zmpMargin < s.zmpX w) &&
  decide (s.zmpX w < s.hiX - w.zmpMargin) &&
  decide (s.loY + w.zmpMargin < s.zmpY w) &&
  decide (s.zmpY w < s.hiY - w.zmpMargin)

/-- Every keyframe in a trajectory is ZMP-stable. Decidable because it is
    a `Bool`-valued fold over a concrete list of `Float`s — exactly what
    `native_decide` can crunch. -/
abbrev allStable (w : WalkingParams) (xs : List TrajectoryStep) : Prop :=
  xs.all (TrajectoryStep.stableB w) = true

end MechProof
