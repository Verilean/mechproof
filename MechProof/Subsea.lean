import MechProof.Environment

namespace MechProof

/-! ## Subsea integrity proofs.

    Three theorems parameterised by `EnvironmentParams` so the same
    statements apply unchanged to other fluid environments (lake-bed
    inspection, contaminated tank cleanup, …). Each theorem reduces to
    a strict `Float <` inequality that `native_decide` evaluates.

    1. **PressureClearance** — joint gaps survive hydrostatic compression.
    2. **HydrostaticBalanced** — buoyancy magnitude matches gravity AND
       the centre of buoyancy sits above the centre of mass (passive
       righting moment).
    3. **CurrentStable** — leg motors hold the pose against fluid drag.
-/

/-- Per-component material + clearance descriptors.

    `bulkModulus` is the volumetric modulus K (Pa).  `nominalGapM` is the
    designed joint gap (m).  `minGapM` is the strict lower bound the
    proof must preserve. -/
structure ComponentMaterial where
  name         : String
  bulkModulus  : Float        -- Pa
  linkLengthM  : Float        -- m (representative dimension)
  nominalGapM  : Float        -- m, designed clearance
  minGapM      : Float        -- m, required after compression
  deriving Repr, Inhabited

/-- Linear compressive strain under hydrostatic pressure P:  ε = P / (3·K).
    The corresponding length shrinkage of a dimension L is  ΔL = L · ε. -/
def ComponentMaterial.shrinkageM
    (env : EnvironmentParams) (c : ComponentMaterial) : Float :=
  c.linkLengthM * env.pressure / (3.0 * c.bulkModulus)

/-- The gap the component still presents after compression. -/
def ComponentMaterial.gapAfterPressureM
    (env : EnvironmentParams) (c : ComponentMaterial) : Float :=
  c.nominalGapM - c.shrinkageM env

/-- **PressureClearance theorem.** Every material's post-pressure gap
    stays above its required minimum (`minGapM`). -/
abbrev ComponentMaterial.PressureSafe
    (env : EnvironmentParams) (c : ComponentMaterial) : Prop :=
  c.minGapM < c.gapAfterPressureM env

def ComponentMaterial.pressureSafeB
    (env : EnvironmentParams) (c : ComponentMaterial) : Bool :=
  decide (c.minGapM < c.gapAfterPressureM env)

abbrev allPressureSafe
    (env : EnvironmentParams) (xs : List ComponentMaterial) : Prop :=
  xs.all (ComponentMaterial.pressureSafeB env) = true

/-- Hydrostatic stability characterisation of the submerged robot. We
    treat the body as a rigid solid with a known displaced volume
    (`displacedVolumeM3`), a CoM height above the soles (`comHeightM`),
    and a CoB height (`cobHeightM`). Standard ship-stability rule:
    CoB **above** CoM gives an automatic righting moment for a fully
    submerged body.  -/
structure HydroBody where
  totalMassKg       : Float
  displacedVolumeM3 : Float
  comHeightM        : Float
  cobHeightM        : Float
  -- Tolerated buoyancy-vs-weight imbalance (fraction of weight).
  buoyancyTolerance : Float
  -- Required CoB-above-CoM separation (m).
  rightingMarginM   : Float
  deriving Repr

def HydroBody.weightN (env : EnvironmentParams) (b : HydroBody) : Float :=
  b.totalMassKg * env.gravity

def HydroBody.buoyancyN
    (env : EnvironmentParams) (b : HydroBody) : Float :=
  env.density * env.gravity * b.displacedVolumeM3

/-- Buoyancy magnitude is within `buoyancyTolerance` of the gravity force. -/
def HydroBody.buoyancyBalancedB
    (env : EnvironmentParams) (b : HydroBody) : Bool :=
  let w := b.weightN env
  let f := b.buoyancyN env
  let tol := w * b.buoyancyTolerance
  let diff := if f > w then f - w else w - f
  decide (diff < tol)

/-- CoB lies above CoM by ≥ `rightingMarginM`. -/
def HydroBody.cobAboveComB (b : HydroBody) : Bool :=
  decide (b.comHeightM + b.rightingMarginM < b.cobHeightM)

abbrev HydroBody.HydrostaticBalanced
    (env : EnvironmentParams) (b : HydroBody) : Prop :=
  (b.buoyancyBalancedB env && b.cobAboveComB) = true

/-- Drag model for a quasi-rigid silhouette.

    `F = ½ · ρ · v² · Cd · A`   (drag force, Newtons)

    `dragMomentN_m` is the resulting moment about the ankle pitch axis
    at moment arm `momentArmM`. -/
structure DragModel where
  projectedAreaM2 : Float
  dragCoeff       : Float
  momentArmM      : Float
  -- Sum of all motor stall torques that resist the current (per the
  -- robot's posture — typically `2 · (hipPitchTau + ankleTau)`).
  motorTorqueNm   : Float
  deriving Repr

def DragModel.dragForceN
    (env : EnvironmentParams) (d : DragModel) : Float :=
  let v := env.currentVel
  0.5 * env.density * v * v * d.dragCoeff * d.projectedAreaM2

def DragModel.dragMomentNm
    (env : EnvironmentParams) (d : DragModel) : Float :=
  d.dragForceN env * d.momentArmM

abbrev DragModel.CurrentStable
    (env : EnvironmentParams) (d : DragModel) : Prop :=
  d.dragMomentNm env < d.motorTorqueNm

end MechProof
