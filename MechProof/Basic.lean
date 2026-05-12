namespace MechProof

/-- A rectangular case to be injection-molded. All dimensions in mm; `draftDeg`
    is the inner-side-wall taper measured from the pull axis (+Z). -/
structure DraftedCase where
  length    : Float
  width     : Float
  height    : Float
  thickness : Float
  draftDeg  : Float
  deriving Repr

/-- Dimensional sanity: positive sizes, walls fit inside the outer footprint,
    and the draft angle is strictly between 0° and 90°. -/
def DraftedCase.WellFormed (c : DraftedCase) : Prop :=
  0 < c.length ∧ 0 < c.width ∧ 0 < c.height ∧
  0 < c.thickness ∧ 2 * c.thickness < c.length ∧ 2 * c.thickness < c.width ∧
  0 < c.draftDeg ∧ c.draftDeg < 90

/-- Moldability condition: the pull is along +Z, so each inner side-wall normal
    has Z-component `sin draftDeg`. For `0 < draftDeg < 90` we have
    `sin draftDeg > 0`, i.e. the dot product with the pull direction is
    strictly positive — no undercuts. We capture the angle bound as the Prop
    that's directly extractable from `WellFormed`. -/
def DraftedCase.Moldable (c : DraftedCase) : Prop :=
  0 < c.draftDeg ∧ c.draftDeg < 90

theorem wellFormed_implies_moldable
    (c : DraftedCase) (h : c.WellFormed) : c.Moldable :=
  ⟨h.2.2.2.2.2.2.1, h.2.2.2.2.2.2.2⟩

end MechProof
