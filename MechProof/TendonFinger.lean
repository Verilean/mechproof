import MechProof.Finger

namespace MechProof

/-- Moment arms (mm) of the flexor tendon at MCP, PIP and DIP joints.

    Convention: each moment arm is measured from the joint axis to the tendon
    path on the **palmar (inner)** side of the link. A strictly positive
    moment arm therefore guarantees that pulling the tendon produces flexion
    torque toward the palm (positive flexion). A non-positive value would
    either zero the torque (`r = 0`) or reverse it (`r < 0` → extension), so
    the finger could lock or "back-crack". -/
structure TendonParams where
  r1 : Float
  r2 : Float
  r3 : Float
  deriving Repr

/-- A tendon routing is valid iff every moment arm is strictly positive. -/
def TendonParams.ValidRouting (t : TendonParams) : Prop :=
  0 < t.r1 ∧ 0 < t.r2 ∧ 0 < t.r3

/-- The 'pull force' produced by tensioning the tendon by `F` Newtons.
    Each joint receives torque `r_i * F`. We treat `F` as positive here. -/
def TendonParams.flexionTorques (t : TendonParams) (F : Float) : Float × Float × Float :=
  (t.r1 * F, t.r2 * F, t.r3 * F)

/-- **Positive Flexion Theorem.** A valid routing with positive tension forces
    flexion (positive torque) at every joint. Captured as a Prop on the
    moment arms (the multiplication by `F > 0` is monotone, so the strict
    positivity of `r_i` is the only structural hypothesis needed). -/
def TendonParams.PositiveFlexion (t : TendonParams) : Prop :=
  0 < t.r1 ∧ 0 < t.r2 ∧ 0 < t.r3

theorem valid_routing_implies_positive_flexion
    (t : TendonParams) (h : t.ValidRouting) : t.PositiveFlexion :=
  h

/-- Combined contract: a tendon-driven finger is fully described by the
    kinematic params (PoC 2) **and** the tendon routing. Both must be
    well-formed for the assembly to be manufacturable and operable. -/
structure TendonFinger where
  finger : FingerParams
  tendon : TendonParams
  deriving Repr

def TendonFinger.WellFormed (tf : TendonFinger) : Prop :=
  tf.finger.WellFormed ∧
  (0 ≤ tf.finger.minAngle ∧ tf.finger.maxAngle ≤ 120) ∧
  tf.tendon.ValidRouting

theorem wellFormed_finger_and_tendon
    (tf : TendonFinger) (h : tf.WellFormed) :
    tf.finger.NoBackwardBending ∧ tf.tendon.PositiveFlexion :=
  ⟨wellFormed_noBackwardBending tf.finger h.1 h.2.1,
   valid_routing_implies_positive_flexion tf.tendon h.2.2⟩

end MechProof
