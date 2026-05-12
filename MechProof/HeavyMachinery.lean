import MechProof.LegAssembly

namespace MechProof

/-! ## Heavy-machinery scale-up.

    The PoC 8 humanoid is a 1.55 m / 21 kg appliance.  PoC 15 retargets
    that same kinematic specification at a 4 m / multi-tonne
    construction-class build.  The Square-Cube Law immediately invalidates
    every torque/stress proof that was discharged at the small scale —
    forces grow as s⁴ (mass × moment arm), while available cross-section
    grows only as s².  Two independent upgrades close the gap:

      * heavier actuators (`Actuator.HeavyDutyHydraulic`) with stall
        torques in the kN·m range,
      * thicker structural tubing with higher yield strength (steel
        instead of PLA / aluminium).

    The two new theorems re-express PoC 8's `SquatTorqueSufficient` and
    add a fresh `YieldStrengthSafe` check at the scaled-up dimensions.

    Notation:
      `s` is the linear scale factor (height ratio versus the 1.55 m
      baseline).  Mass scales by `s³ · ρ_ratio`, where `ρ_ratio` is the
      density of the build material divided by the baseline PLA density.
-/

/-- Build-material density ratio relative to the PoC 8 PLA baseline (1240 kg/m³).
    For steel construction (~7800 kg/m³) the ratio is ~6.3. -/
structure RobotScale where
  linearScale       : Float    -- s = height_new / height_baseline
  densityRatio      : Float    -- ρ_new / ρ_PLA
  thighDiameterM    : Float    -- outer diameter of the scaled leg tube
  thighWallM        : Float    -- wall thickness (hollow tube)
  yieldStressPa     : Float    -- material yield strength (Pa)
  baselineMassKg    : Float    -- the PoC 8 total mass (~21.2 kg)
  baselineThighLenM : Float    -- 0.30 m at the baseline
  baselineUpperKg   : Float    -- 5 + 10 + 2·0.4 ≈ 17 kg above the knees
  deriving Repr

/-- Total mass of the scaled robot:  m₀ · s³ · ρ_ratio. -/
def RobotScale.totalMassKg (r : RobotScale) : Float :=
  r.baselineMassKg * r.linearScale * r.linearScale * r.linearScale
    * r.densityRatio

/-- Scaled thigh length. -/
def RobotScale.thighLenM (r : RobotScale) : Float :=
  r.baselineThighLenM * r.linearScale

/-- Upper-body mass at the new scale (kg). -/
def RobotScale.upperBodyKg (r : RobotScale) : Float :=
  r.baselineUpperKg * r.linearScale * r.linearScale * r.linearScale
    * r.densityRatio

/-- Required knee torque under PoC 8's 90° squat geometry, scaled to the
    new robot.  Mirrors `LowerBody.requiredKneeTorque`. -/
def RobotScale.requiredKneeTorqueNm (r : RobotScale) : Float :=
  GRAVITY_M_PER_S2 * (r.upperBodyKg / 2.0) * r.thighLenM

/-- Bending stress in the (hollow-tube) thigh under cantilever from the
    upper body at full horizontal extension:
        σ = M · c / I,
        M  = m_upper · g · L_thigh,
        c  = D / 2,
        I  = π · (c⁴ − (c − wall)⁴) / 4.
    We use a polynomial-friendly expansion of `c⁴ − (c−w)⁴` to keep
    `native_decide` happy. -/
def RobotScale.thighBendingStressPa (r : RobotScale) : Float :=
  let m := r.upperBodyKg
  let L := r.thighLenM
  let c := r.thighDiameterM / 2.0
  let w := r.thighWallM
  let c2 := c * c
  let c4 := c2 * c2
  let ci := c - w
  let ci2 := ci * ci
  let ci4 := ci2 * ci2
  let area_moment := 3.14159265358979 * (c4 - ci4) / 4.0
  m * GRAVITY_M_PER_S2 * L * c / area_moment

/-! ### Actuator class -/

structure Actuator where
  name            : String
  stallTorqueNm   : Float
  -- A coarse "rated" torque for continuous operation.  Used by the
  -- catalog text but not by the proof itself.
  ratedTorqueNm   : Float
  deriving Repr, Inhabited

namespace Actuator

/-- The PoC 8 small electric servo (40 N·m at the knee).  Provided here
    so the negative test `test-square-cube` can swap it back in. -/
def smallElectric : Actuator :=
  { name := "small_electric_servo", stallTorqueNm := 40.0,
    ratedTorqueNm := 25.0 }

/-- A heavy-duty hydraulic / harmonic-drive actuator suitable for 4 m
    construction-class humanoids.  15 kN·m stall is in line with
    excavator-arm joints. -/
def heavyDutyHydraulic : Actuator :=
  { name := "heavy_hydraulic_15kNm",
    stallTorqueNm := 15000.0,
    ratedTorqueNm := 9000.0 }

end Actuator

/-- A complete heavy-machinery build descriptor: scale × actuator. -/
structure HeavyBuild where
  scale       : RobotScale
  knee        : Actuator
  -- Hip-pitch + ankle-pitch actuators are also part of the leg chain;
  -- we conservatively require *all* pitch joints to use the same
  -- actuator class so the proof is a single inequality.
  -- Required margin (fraction) on top of the static demand — a hydraulic
  -- proof we discharge with strict `<` after applying this margin.
  margin      : Float
  deriving Repr

/-- The scaled robot's torque-balance theorem.  We require that the
    knee actuator's stall torque exceeds `(1 + margin) · required`. -/
abbrev HeavyBuild.HeavyTorqueSufficient (h : HeavyBuild) : Prop :=
  h.scale.requiredKneeTorqueNm * (1.0 + h.margin) < h.knee.stallTorqueNm

/-- The bending stress in the scaled thigh stays below the material's
    yield strength with a strict margin. -/
abbrev HeavyBuild.YieldStrengthSafe (h : HeavyBuild) : Prop :=
  h.scale.thighBendingStressPa * (1.0 + h.margin) < h.scale.yieldStressPa

/-- Both load-bearing checks combined.  This is the headline gate the
    `test-square-cube` negative test exercises. -/
abbrev HeavyBuild.HeavyStandStable (h : HeavyBuild) : Prop :=
  h.HeavyTorqueSufficient ∧ h.YieldStrengthSafe

end MechProof
