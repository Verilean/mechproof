import MechProof.Walking

namespace MechProof

/-! ## Capture-Point stability proof.

    Pratt et al. (2006): for a bipedal robot modelled by the LIPM, the
    instantaneous capture point is

      x_cp = x_com + ẋ_com · √(H / g),
      y_cp = y_com + ẏ_com · √(H / g).

    If the **next** footstep's support polygon contains the capture point,
    the robot can dissipate its kinetic energy and come to a complete
    stop without falling. The orbital energy

      E = ½ ẋ² − g/(2 H) · x²

    is conserved along the LIPM dynamics and stays non-positive while
    the capture point is inside the polygon — which is the conventional
    "captureability" guarantee.

    We discretise the gait into keyframes whose *next* support polygon
    is supplied with each frame, then verify the capture point lies in
    that polygon (with a margin). The Lean theorem is exactly the same
    style as PoC 10's ZMP check: a Bool fold that `native_decide` can
    evaluate.
-/

/-- Square-root via Newton's method (10 iterations). Float-only, so
    `native_decide` can evaluate it. Accurate to ~1e-12 in the range we
    care about. -/
def floatSqrt (x : Float) : Float :=
  if x ≤ 0.0 then 0.0
  else
    let init := if x < 1.0 then x else 1.0 + x / 2.0
    let step (y : Float) := 0.5 * (y + x / y)
    let y₁  := step init
    let y₂  := step y₁
    let y₃  := step y₂
    let y₄  := step y₃
    let y₅  := step y₄
    let y₆  := step y₅
    let y₇  := step y₆
    let y₈  := step y₇
    let y₉  := step y₈
    step y₉

/-- A capture-point keyframe: the current CoM (x, y), the current CoM
    velocity (ẋ, ẏ), and the support polygon of the **next** footstep. -/
structure CaptureStep where
  comX     : Float
  comY     : Float
  velX     : Float
  velY     : Float
  loX : Float
  hiX : Float
  loY : Float
  hiY : Float
  deriving Repr

def CaptureStep.cpX (w : WalkingParams) (s : CaptureStep) : Float :=
  s.comX + s.velX * floatSqrt (w.comHeight / w.gravity)

def CaptureStep.cpY (w : WalkingParams) (s : CaptureStep) : Float :=
  s.comY + s.velY * floatSqrt (w.comHeight / w.gravity)

/-- Orbital energy under the LIPM (1-D form, used as a sanity-check
    quantity in the JSON output). -/
def CaptureStep.orbitalEnergy
    (w : WalkingParams) (s : CaptureStep) : Float :=
  let g := w.gravity
  let h := w.comHeight
  0.5 * (s.velX * s.velX) - g / (2.0 * h) * (s.comX * s.comX)

def CaptureStep.captureB
    (w : WalkingParams) (s : CaptureStep) : Bool :=
  decide (s.loX + w.zmpMargin < s.cpX w) &&
  decide (s.cpX w < s.hiX - w.zmpMargin) &&
  decide (s.loY + w.zmpMargin < s.cpY w) &&
  decide (s.cpY w < s.hiY - w.zmpMargin)

abbrev allCapturable
    (w : WalkingParams) (xs : List CaptureStep) : Prop :=
  xs.all (CaptureStep.captureB w) = true

end MechProof
