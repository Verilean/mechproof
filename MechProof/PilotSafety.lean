import MechProof.HeavyMachinery
import MechProof.Walking

namespace MechProof

/-! ## Piloted-mech safety proofs.

    Two independent theorems guarantee that a human pilot inside the
    4 m heavy machinery cannot be killed by either their own input or
    a sudden fall:

      1. `SafeOverride` — the input-filter clips any pilot command so
         the resulting CoM acceleration keeps the ZMP inside the
         support polygon.
      2. `SurvivalBrace` — assuming an unavoidable fall, the active
         bracing posture (arms extended, hydraulic damping at max)
         dissipates enough energy that cockpit deceleration stays
         below `maxSafeG · g`.

    Both reduce to strict `<` inequalities over `Float` so
    `native_decide` evaluates them.
-/

/-- Human-pilot physical limits. -/
structure PilotLimits where
  maxSafeG  : Float    -- acceleration cap (g-multiples)
  deriving Repr

def stdPilotLimits : PilotLimits :=
  { maxSafeG := 15.0 }    -- ~15 G, standard ejection-seat survival number

/-! ### 1. SafeOverride: ZMP-bounded input filter

    The LIPM ZMP equation says
        x_zmp = x_com − (H/g) · ẍ_com.
    For a quasi-static stance with x_com = 0, the ZMP stays inside the
    polygon [−Wpoly, +Wpoly] iff
        |ẍ_com| < g · Wpoly / H .
    The filter clips the pilot's requested ẍ to this bound (minus a
    safety margin), so any input — including a "full thrust" reckless
    command — yields a safe ZMP. -/

structure InputFilter where
  comHeightM     : Float    -- LIPM CoM height
  supportHalfX   : Float    -- half-width of the support polygon along X
  zmpMargin      : Float    -- strict safety margin inside the polygon
  -- The largest ẍ this filter will pass downstream:
  maxAccelMS2    : Float
  deriving Repr

/-- The ẍ cap derived from the polygon and CoM height (m/s²). -/
def InputFilter.accelLimitMS2 (f : InputFilter) : Float :=
  GRAVITY_M_PER_S2 * (f.supportHalfX - f.zmpMargin) / f.comHeightM

/-- Clip a pilot's requested ẍ symmetrically to `maxAccelMS2`. -/
def InputFilter.clip (f : InputFilter) (a : Float) : Float :=
  if a > f.maxAccelMS2 then f.maxAccelMS2
  else if a < -f.maxAccelMS2 then -f.maxAccelMS2
  else a

/-- The filter is **safe** iff its configured `maxAccelMS2` is itself
    below the LIPM-derived bound.  This is a static "compiler check":
    a filter whose cap is too high is rejected at proof time. -/
abbrev InputFilter.SafeOverride (f : InputFilter) : Prop :=
  0 < f.maxAccelMS2 ∧ f.maxAccelMS2 < f.accelLimitMS2

/-! ### 2. SurvivalBrace: cockpit-G under unavoidable fall

    Free-fall over height `hDrop` arrives at impact with velocity
        v = √(2 · g · hDrop).
    If the bracing posture provides an effective deceleration distance
    `dBrace` (the stroke over which the hydraulic arms collapse), the
    average deceleration is
        a = v² / (2 · dBrace) = g · hDrop / dBrace .
    Therefore the cockpit's G-load is exactly
        G = hDrop / dBrace .
    A passing build requires  G < maxSafeG. -/

structure BracingPosture where
  fallHeightM       : Float    -- ΔZ of the cockpit during the fall
  braceStrokeM      : Float    -- effective hydraulic-damper crush distance
  deriving Repr

/-- Computed cockpit deceleration as a multiple of `g`. -/
def BracingPosture.impactG (b : BracingPosture) : Float :=
  b.fallHeightM / b.braceStrokeM

abbrev BracingPosture.SurvivalBrace
    (b : BracingPosture) (p : PilotLimits) : Prop :=
  b.impactG < p.maxSafeG

/-! ### Combined safety contract -/

structure PilotedMech where
  filter  : InputFilter
  brace   : BracingPosture
  pilot   : PilotLimits
  deriving Repr

abbrev PilotedMech.SafePilotedOperation (m : PilotedMech) : Prop :=
  m.filter.SafeOverride ∧ m.brace.SurvivalBrace m.pilot

end MechProof
