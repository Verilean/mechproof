import MechProof
open MechProof

/-- A typical robotic-servo bucket: ~1 N·m/A torque constant, 0.1 Ω coil,
    80% combined inverter+gearbox efficiency. -/
def standardServo : MotorConstants :=
  { resistance := 0.10, torqueConstant := 1.0, driverEff := 0.8 }

/-- A tiny hobby-grade servo for the hand (lower current capacity but a
    weaker torque constant; still 80% efficient). -/
def microServo : MotorConstants :=
  { resistance := 0.30, torqueConstant := 0.6, driverEff := 0.8 }

/-! ### Standing mode (1 hour)

    Legs hold a static squat-stand pose (small joint torques against
    gravity, zero velocity). Arms and hand are passive. The "trunk"
    bucket carries the electronics baseline (compute board + sensors). -/

def legsStanding : JointBucket :=
  { nJoints := 12.0, motor := standardServo,
    avgTorqueNm := 2.0, avgOmegaRps := 0.0 }

def armsStanding : JointBucket :=
  { nJoints := 6.0, motor := standardServo,
    avgTorqueNm := 1.0, avgOmegaRps := 0.0 }

def handStanding : JointBucket :=
  { nJoints := 6.0, motor := microServo,
    avgTorqueNm := 0.5, avgOmegaRps := 0.0 }

def trunkBaseline : JointBucket :=
  { nJoints := 1.0, motor := standardServo,
    avgTorqueNm := 20.0, avgOmegaRps := 0.0 }

def standingBuckets : List JointBucket :=
  [legsStanding, armsStanding, handStanding, trunkBaseline]

/-! ### Walking mode (1 hour of continuous gait) -/

def legsWalking : JointBucket :=
  { nJoints := 12.0, motor := standardServo,
    avgTorqueNm := 10.0, avgOmegaRps := 1.5 }

def armsWalking : JointBucket :=
  { nJoints := 6.0, motor := standardServo,
    avgTorqueNm := 2.0, avgOmegaRps := 0.3 }

def handWalking : JointBucket :=
  { nJoints := 6.0, motor := microServo,
    avgTorqueNm := 0.5, avgOmegaRps := 0.1 }

def walkingBuckets : List JointBucket :=
  [legsWalking, armsWalking, handWalking, trunkBaseline]

/-! ### Battery and mission specs -/

def BATTERY_WH       : Float := 800.0
def SAFETY_FRACTION  : Float := 0.80      -- 20% reserve
def ONE_HOUR_S       : Float := 3600.0

def standingMission : MissionParams :=
  { batteryWh := BATTERY_WH, safetyFraction := SAFETY_FRACTION,
    missionDurationS := ONE_HOUR_S, modeLabel := "Standing 1 h" }

def walkingMission : MissionParams :=
  { batteryWh := BATTERY_WH, safetyFraction := SAFETY_FRACTION,
    missionDurationS := ONE_HOUR_S, modeLabel := "Walking 1 h" }

/-- A 10-step quasi-static gait at the PoC 10 keyframe cadence
    (~0.4 s per keyframe × ~3 keyframes per step ≈ 1.2 s/step → 12 s). -/
def tenStepMission : MissionParams :=
  { batteryWh := BATTERY_WH, safetyFraction := SAFETY_FRACTION,
    missionDurationS := 12.0, modeLabel := "10-step gait" }

/-- The headline missions, in the order we report them. -/
def allMissions : List (MissionParams × List JointBucket) := [
  (standingMission, standingBuckets),
  (walkingMission,  walkingBuckets),
  (tenStepMission,  walkingBuckets)
]

/-- **Compile-time energy proofs.** Each of the three missions stays
    inside 80% of the battery nameplate. If any one fails, the build
    aborts and no energy reports / battery cert / teleop sim runs. -/
def standingPossible :
    MissionPossible standingBuckets standingMission := by native_decide
def walkingPossible :
    MissionPossible walkingBuckets walkingMission := by native_decide
def tenStepPossible :
    MissionPossible walkingBuckets tenStepMission := by native_decide

def renderBucket (b : JointBucket) : String :=
  "{\"nJoints\":" ++ toString b.nJoints ++
  ",\"resistance\":" ++ toString b.motor.resistance ++
  ",\"torqueConstant\":" ++ toString b.motor.torqueConstant ++
  ",\"driverEff\":" ++ toString b.motor.driverEff ++
  ",\"avgTorqueNm\":" ++ toString b.avgTorqueNm ++
  ",\"avgOmegaRps\":" ++ toString b.avgOmegaRps ++
  ",\"perJointPowerW\":" ++ toString b.powerW ++ "}"

def renderBuckets : List JointBucket → List String
  | [] => []
  | b :: rest => renderBucket b :: renderBuckets rest

def renderMission (m : MissionParams) (bs : List JointBucket) : String :=
  let totalW := (bs.map (fun b => b.nJoints * b.powerW)).foldl (· + ·) 0.0
  let energy := missionEnergyWh bs m
  "{\"mode\":\"" ++ m.modeLabel ++ "\"" ++
  ",\"durationS\":" ++ toString m.missionDurationS ++
  ",\"batteryWh\":" ++ toString m.batteryWh ++
  ",\"safetyFraction\":" ++ toString m.safetyFraction ++
  ",\"usableWh\":" ++ toString (m.batteryWh * m.safetyFraction) ++
  ",\"totalBusPowerW\":" ++ toString totalW ++
  ",\"missionEnergyWh\":" ++ toString energy ++
  ",\"buckets\":[" ++ String.intercalate "," (renderBuckets bs) ++ "]}"

def main : IO Unit := do
  let _ : MissionPossible standingBuckets standingMission := standingPossible
  let _ : MissionPossible walkingBuckets walkingMission := walkingPossible
  let _ : MissionPossible walkingBuckets tenStepMission := tenStepPossible
  IO.FS.createDirAll "out"
  let json :=
    "{\"missions\":[" ++
    String.intercalate ","
      ((allMissions.map (fun pr => renderMission pr.1 pr.2))) ++
    "]}"
  IO.FS.writeFile "out/energy_proof.json" json
  IO.println "Lean energy proofs passed: every mission fits within 80% of battery."
  IO.println s!"  battery        = {BATTERY_WH} Wh"
  IO.println s!"  usable         = {BATTERY_WH * SAFETY_FRACTION} Wh"
  IO.println s!"  standing 1 h   = {missionEnergyWh standingBuckets standingMission} Wh"
  IO.println s!"  walking  1 h   = {missionEnergyWh walkingBuckets walkingMission} Wh"
  IO.println s!"  10-step gait   = {missionEnergyWh walkingBuckets tenStepMission} Wh"
  IO.println "Wrote ../out/energy_proof.json"
