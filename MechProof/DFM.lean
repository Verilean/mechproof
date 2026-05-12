import MechProof.HandAssembly
import MechProof.TendonFinger

namespace MechProof

/-! ## Design-for-Manufacturing (DFM) rules

    All distances in **metres**. These thresholds reflect the conservative
    intersection of injection-moulding and SLA-3D-printing capabilities for
    PLA / nylon / similar engineering plastics. If your fab process is more
    capable, lower the numbers here (and the proofs of dependent assemblies
    must be re-discharged automatically — that's the whole point).
-/

/-- 1.2 mm: the global minimum wall thickness required by injection moulding
    and conservative SLA printing. Below this, parts deform, warp, or fail
    to fill. -/
def MIN_WALL_THICKNESS_M : Float := 0.0012

/-- 1.0 mm: the minimum through-hole diameter for a tendon channel.
    Below this, SLA resins clog and injection cores break. -/
def MIN_TENDON_HOLE_DIA_M : Float := 0.0010

/-- DFM parameters carried alongside the Lean-proven kinematics: the
    physical tendon-channel radius and the channel's offset from the joint
    axis (= the moment arm). These are what the CadQuery generator actually
    drills; tying them into Lean makes the manufacturing claim load-bearing. -/
structure TendonChannelGeom where
  channelRadius : Float
  momentArm     : Float
  deriving Repr

/-- A single link is manufacturable iff:
      * the tendon hole is at least `MIN_TENDON_HOLE_DIA_M / 2` in radius,
        AND
      * the wall *between the hole and the outer surface of the link* is at
        least `MIN_WALL_THICKNESS_M` thick.

    The wall thickness is computed from the link cross-section as:
      `linkThickness/2 - momentArm - channelRadius`
    (half the link thickness is the distance from the joint axis to the
    outer surface; the channel sits at `-momentArm` along the palmar axis,
    and the channel has its own radius). -/
abbrev linkManufacturable
    (linkThickness : Float) (g : TendonChannelGeom) : Prop :=
  MIN_TENDON_HOLE_DIA_M / 2 ≤ g.channelRadius ∧
  MIN_WALL_THICKNESS_M ≤
    linkThickness / 2 - g.momentArm - g.channelRadius

/-- The wall thickness left between the tendon channel and the outer
    surface (m). Exposed so the certificate can quote a number. -/
def wallThicknessMargin
    (linkThickness : Float) (g : TendonChannelGeom) : Float :=
  linkThickness / 2 - g.momentArm - g.channelRadius

/-- Tendon-channel geometry for one finger: the three per-joint channel
    descriptions (the wall test is run independently per link, because the
    moment arms taper distally). -/
structure FingerChannels where
  ch1 : TendonChannelGeom
  ch2 : TendonChannelGeom
  ch3 : TendonChannelGeom
  deriving Repr

/-- A finger is manufacturable iff each of its three links is. -/
abbrev fingerManufacturable
    (linkThickness : Float) (fc : FingerChannels) : Prop :=
  linkManufacturable linkThickness fc.ch1 ∧
  linkManufacturable linkThickness fc.ch2 ∧
  linkManufacturable linkThickness fc.ch3

/-- Full-hand DFM bundle. -/
structure HandChannels where
  index  : FingerChannels
  middle : FingerChannels
  ring   : FingerChannels
  pinky  : FingerChannels
  thumb  : FingerChannels
  deriving Repr

/-- A hand is manufacturable iff every finger is, using its own link
    thickness. -/
abbrev HandAssembly.Manufacturable
    (h : HandAssembly) (ch : HandChannels) : Prop :=
  fingerManufacturable h.index.thickness  ch.index  ∧
  fingerManufacturable h.middle.thickness ch.middle ∧
  fingerManufacturable h.ring.thickness   ch.ring   ∧
  fingerManufacturable h.pinky.thickness  ch.pinky  ∧
  fingerManufacturable h.thumb.thickness  ch.thumb

end MechProof
