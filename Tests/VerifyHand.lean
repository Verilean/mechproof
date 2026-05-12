import MechProof
open MechProof

/-- Standard finger geometry. Thickness is 10 mm so the tendon channel
    plus its surrounding 1.2 mm wall fits inside the link cross-section
    even at the proximal joint where the moment arm is largest. -/
def stdFinger : FingerParams :=
  { l1 := 40.0, l2 := 30.0, l3 := 20.0,
    thickness := 10.0, minAngle := 0.0, maxAngle := 90.0 }

/-- Convert millimetre finger length fields to metres for the assembly math.
    The proof works entirely in metres so the clearance check has the same
    units as the mount offsets. -/
def stdFingerMetric : FingerParams :=
  { stdFinger with
    l1 := stdFinger.l1 * 0.001,
    l2 := stdFinger.l2 * 0.001,
    l3 := stdFinger.l3 * 0.001,
    thickness := stdFinger.thickness * 0.001 }

/-- DFM-compliant tendon channels.

    Per-joint moment arms taper distally (2.5 / 2.0 / 1.5 mm); the channel
    radius is 0.6 mm = 1.2 mm diameter (above the 1.0 mm MIN_TENDON_HOLE_DIA).
    Wall margin at the proximal joint:
      thickness/2 − arm − rChannel = 5.0 − 2.5 − 0.6 = 1.9 mm ≥ 1.2 mm ✓ -/
def stdChannels : FingerChannels :=
  { ch1 := { channelRadius := 0.0006, momentArm := 0.0025 },
    ch2 := { channelRadius := 0.0006, momentArm := 0.0020 },
    ch3 := { channelRadius := 0.0006, momentArm := 0.0015 } }

/-- Hand layout, metres throughout. The four straight fingers sit on the
    distal edge of the palm (y = palm.length), 16 mm apart along X. The
    thumb's base is at (x = 0.045, y = 0.020) on the +X side, and its
    rest yaw is 0 (parallel to the others); swivel rotates it about +Z. -/
def candidate : HandAssembly :=
  { palm := { width := 0.080, length := 0.060, thickness := 0.008 },
    index  := stdFingerMetric,
    middle := stdFingerMetric,
    ring   := stdFingerMetric,
    pinky  := stdFingerMetric,
    thumb  := stdFingerMetric,
    indexMount  := { px :=  0.024, py := 0.060, pz := 0.0, yawRad := 0.0 },
    middleMount := { px :=  0.008, py := 0.060, pz := 0.0, yawRad := 0.0 },
    ringMount   := { px := -0.008, py := 0.060, pz := 0.0, yawRad := 0.0 },
    pinkyMount  := { px := -0.024, py := 0.060, pz := 0.0, yawRad := 0.0 },
    thumbMount  := { px :=  0.045, py := 0.020, pz := 0.0, yawRad := 0.0 },
    swivelMaxRad := 1.4 }

/-- Tendon-channel geometry, one set per finger (identical across the 5
    fingers here). Carried alongside `candidate` so the DFM proof can
    consume it. -/
def candidateChannels : HandChannels :=
  { index  := stdChannels,
    middle := stdChannels,
    ring   := stdChannels,
    pinky  := stdChannels,
    thumb  := stdChannels }

/-- Compile-time well-formedness of the assembly. -/
def candidateWellFormed : candidate.WellFormed := by
  refine ⟨?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_, ?_⟩ <;>
    first
      | native_decide
      | (refine ⟨?_, ?_, ?_, ?_, ?_⟩ <;> native_decide)

/-- **Compile-time collision proof.** The thumb-vs-index clearance is
    strictly positive (with the 3 mm safety margin baked in) at the
    extreme of the thumb's swivel range. If this `native_decide` fails,
    the build aborts and no CAD or simulation is produced. -/
def candidateThumbIndexClear : candidate.ThumbIndexClear := by
  native_decide

/-- **Compile-time DFM proof.** Every finger's tendon channel satisfies
    the MIN_TENDON_HOLE_DIA / MIN_WALL_THICKNESS rules. Failure here means
    the part is provably non-manufacturable; the build aborts. -/
def candidateManufacturable :
    candidate.Manufacturable candidateChannels := by
  native_decide

def renderMount (m : FingerMount) : String :=
  "{\"px\":" ++ toString m.px ++
  ",\"py\":" ++ toString m.py ++
  ",\"pz\":" ++ toString m.pz ++
  ",\"yawRad\":" ++ toString m.yawRad ++ "}"

def renderFinger (f : FingerParams) : String :=
  "{\"l1\":" ++ toString f.l1 ++
  ",\"l2\":" ++ toString f.l2 ++
  ",\"l3\":" ++ toString f.l3 ++
  ",\"thickness\":" ++ toString f.thickness ++
  ",\"minAngle\":" ++ toString f.minAngle ++
  ",\"maxAngle\":" ++ toString f.maxAngle ++ "}"

def renderChannel (g : TendonChannelGeom) (linkThickness : Float) : String :=
  "{\"channelRadius\":" ++ toString g.channelRadius ++
  ",\"momentArm\":" ++ toString g.momentArm ++
  ",\"wallMargin\":" ++ toString (wallThicknessMargin linkThickness g) ++ "}"

def renderFingerChannels (fc : FingerChannels) (linkThickness : Float) : String :=
  "{\"ch1\":" ++ renderChannel fc.ch1 linkThickness ++
  ",\"ch2\":" ++ renderChannel fc.ch2 linkThickness ++
  ",\"ch3\":" ++ renderChannel fc.ch3 linkThickness ++ "}"

def renderJson (h : HandAssembly) (ch : HandChannels) : String :=
  "{\"palm\":{\"width\":" ++ toString h.palm.width ++
  ",\"length\":" ++ toString h.palm.length ++
  ",\"thickness\":" ++ toString h.palm.thickness ++ "}" ++
  ",\"index\":" ++ renderFinger h.index ++
  ",\"middle\":" ++ renderFinger h.middle ++
  ",\"ring\":" ++ renderFinger h.ring ++
  ",\"pinky\":" ++ renderFinger h.pinky ++
  ",\"thumb\":" ++ renderFinger h.thumb ++
  ",\"indexMount\":" ++ renderMount h.indexMount ++
  ",\"middleMount\":" ++ renderMount h.middleMount ++
  ",\"ringMount\":" ++ renderMount h.ringMount ++
  ",\"pinkyMount\":" ++ renderMount h.pinkyMount ++
  ",\"thumbMount\":" ++ renderMount h.thumbMount ++
  ",\"swivelMaxRad\":" ++ toString h.swivelMaxRad ++
  ",\"thumbIndexMinDistSq\":" ++ toString h.thumbIndexMinDistSq ++
  ",\"requiredClearanceSq\":" ++ toString h.requiredClearanceSq ++
  ",\"dfm\":{" ++
    "\"minWallThicknessM\":" ++ toString MIN_WALL_THICKNESS_M ++
    ",\"minTendonHoleDiaM\":" ++ toString MIN_TENDON_HOLE_DIA_M ++
    ",\"indexChannels\":" ++ renderFingerChannels ch.index h.index.thickness ++
    ",\"middleChannels\":" ++ renderFingerChannels ch.middle h.middle.thickness ++
    ",\"ringChannels\":" ++ renderFingerChannels ch.ring h.ring.thickness ++
    ",\"pinkyChannels\":" ++ renderFingerChannels ch.pinky h.pinky.thickness ++
    ",\"thumbChannels\":" ++ renderFingerChannels ch.thumb h.thumb.thickness ++
  "}}"

def main : IO Unit := do
  let _ : candidate.WellFormed := candidateWellFormed
  let _ : candidate.ThumbIndexClear := candidateThumbIndexClear
  let _ : candidate.Manufacturable candidateChannels := candidateManufacturable
  IO.FS.createDirAll "out"
  IO.FS.writeFile "out/hand_params.json"
    (renderJson candidate candidateChannels)
  IO.println "Lean proofs passed:"
  IO.println "  • Hand is well-formed."
  IO.println "  • Thumb-index clearance is provably positive at full swivel."
  IO.println s!"      min distance² = {candidate.thumbIndexMinDistSq} m²"
  IO.println s!"      required²     = {candidate.requiredClearanceSq} m²"
  IO.println "  • Every tendon channel satisfies the DFM rules."
  IO.println s!"      MIN_WALL_THICKNESS_M  = {MIN_WALL_THICKNESS_M} m"
  IO.println s!"      MIN_TENDON_HOLE_DIA_M = {MIN_TENDON_HOLE_DIA_M} m"
  IO.println "Wrote ../out/hand_params.json"
