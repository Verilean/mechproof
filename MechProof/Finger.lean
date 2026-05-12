namespace MechProof

/-- A 3-link planar finger.

    `l1`, `l2`, `l3` are link lengths (mm). `thickness` is the link cross-section.
    `minAngle` / `maxAngle` are the *common* joint-angle bounds in degrees,
    applied to every joint. The angle convention is the **flexion angle from
    the extended pose**: 0° means the link is in line with its parent, positive
    values flex into the palm.

    With this convention, a joint that bends "backward" past the back of the
    hand corresponds to a negative angle, and a joint that closes past the palm
    corresponds to an angle ≥ 180°. -/
structure FingerParams where
  l1        : Float
  l2        : Float
  l3        : Float
  thickness : Float
  minAngle  : Float
  maxAngle  : Float
  deriving Repr

/-- Dimensional sanity for the finger. -/
def FingerParams.WellFormed (f : FingerParams) : Prop :=
  0 < f.l1 ∧ 0 < f.l2 ∧ 0 < f.l3 ∧
  0 < f.thickness ∧
  f.minAngle ≤ f.maxAngle

/-- `NoBackwardBending` captures two facts at once:
    * `0 ≤ minAngle`  — no joint can hyper-extend past the back of the hand.
    * `maxAngle ≤ 120` — no joint can crush past the palm into its neighbour.

    Both are necessary conditions to avoid self-collision in a planar finger
    whose links are roughly equal in length. The 120° upper bound is the
    conservative limit used by most underactuated tendon-driven hands. -/
def FingerParams.NoBackwardBending (f : FingerParams) : Prop :=
  0 ≤ f.minAngle ∧ f.maxAngle ≤ 120

/-- If a well-formed finger additionally satisfies the joint-bound hypothesis,
    it is free of backward bending. The hypothesis is the same Prop, so the
    theorem is the identity — what matters is that this lemma documents the
    contract: callers must *supply* the bound proof, and `verify_finger`
    discharges it with `native_decide`. -/
theorem wellFormed_noBackwardBending
    (f : FingerParams) (_h : f.WellFormed)
    (hBounds : 0 ≤ f.minAngle ∧ f.maxAngle ≤ 120) : f.NoBackwardBending :=
  hBounds

end MechProof
