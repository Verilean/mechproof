import MechProof.CapturePoint

namespace MechProof

/-! ## Energy / endurance proof.

    Per-motor instantaneous power under the conservative model

      P_i(t) = |τ_i(t) · ω_i(t)|   +   R_i · (τ_i / K_t,i)²

    where the first term is mechanical work and the second is resistive
    (copper) loss. Driver loss is modelled as an efficiency factor `η`
    applied to the input bus: `P_bus = P_motor / η`.

    For a 30-DOF humanoid we group joints into 4 buckets with shared
    motor constants — leg, arm, hand, plus a "trunk" bucket for the
    freejoint's housekeeping draws (controllers, fans, sensors). Each
    bucket has an average torque / velocity per mode; the integrated
    energy over a mission duration is

      E = (duration_s / 3600)  ×  Σ_b  n_b · P_b   [Wh]

    where n_b is the number of joints in the bucket.

    The "Mission-Possible" theorem requires the integrated mission
    energy to stay below 80% of the rated battery capacity (a 20%
    safety margin for thermals, ageing and headroom).
-/

structure MotorConstants where
  -- Coil resistance (Ω) and torque constant (N·m / A).
  resistance     : Float
  torqueConstant : Float
  driverEff      : Float    -- 0..1; inverter + gearbox lumped efficiency
  deriving Repr

/-- A bucket of identically-spec'd joints operating at the same average
    torque / velocity for a chosen mission mode. -/
structure JointBucket where
  nJoints     : Float       -- count (as Float so arithmetic stays in ℝ)
  motor       : MotorConstants
  avgTorqueNm : Float       -- |τ| averaged over the mission
  avgOmegaRps : Float       -- |ω| averaged over the mission, rad/s
  deriving Repr

/-- Battery and mission descriptor. -/
structure MissionParams where
  batteryWh        : Float    -- nameplate capacity (Watt-hours)
  safetyFraction   : Float    -- 0..1 (fraction of nameplate that may be drawn)
  missionDurationS : Float    -- seconds
  modeLabel        : String   -- "standing" / "walking" / "10-step gait" / …
  deriving Repr

/-- Per-joint instantaneous bus-side power draw, in Watts. -/
def JointBucket.powerW (b : JointBucket) : Float :=
  let mech := b.avgTorqueNm * b.avgOmegaRps             -- |τ| · |ω| > 0
  let copper :=
    b.motor.resistance *
      (b.avgTorqueNm / b.motor.torqueConstant) *
      (b.avgTorqueNm / b.motor.torqueConstant)          -- R · I²
  (mech + copper) / b.motor.driverEff

/-- Total mission energy (Watt-hours) summed across the bucket list. -/
def missionEnergyWh
    (buckets : List JointBucket) (m : MissionParams) : Float :=
  let totalW := (buckets.map (fun b => b.nJoints * b.powerW)).foldl (· + ·) 0.0
  totalW * (m.missionDurationS / 3600.0)

/-- The mission is "possible" iff the integrated energy is strictly less
    than the battery's *usable* capacity. -/
abbrev MissionPossible
    (buckets : List JointBucket) (m : MissionParams) : Prop :=
  missionEnergyWh buckets m < m.batteryWh * m.safetyFraction

/-- A simple sanity check on the motor constants. -/
def MotorConstants.WellFormed (mc : MotorConstants) : Prop :=
  0 < mc.resistance ∧ 0 < mc.torqueConstant ∧
  0 < mc.driverEff ∧ mc.driverEff ≤ 1

/-- A bucket is well-formed iff its motor is and its averaged quantities
    are sensible. -/
abbrev JointBucket.WellFormed (b : JointBucket) : Prop :=
  0 < b.nJoints ∧ b.motor.WellFormed ∧
  0 ≤ b.avgTorqueNm ∧ 0 ≤ b.avgOmegaRps

def MissionParams.WellFormed (m : MissionParams) : Prop :=
  0 < m.batteryWh ∧
  0 < m.safetyFraction ∧ m.safetyFraction ≤ 1 ∧
  0 < m.missionDurationS

end MechProof
