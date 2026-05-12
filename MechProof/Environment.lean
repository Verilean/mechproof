import MechProof.LegAssembly

namespace MechProof

/-! ## Operating environment.

    A reusable description of the fluid / pressure / gravity context the
    humanoid must function in. Future operating profiles (lunar, factory
    floor, deep space, contaminated cleanup) plug into the same struct
    and benefit from every theorem written against `EnvironmentParams`
    without forking the codebase.

    Units: SI throughout — density (kg/m³), gravity (m/s²), pressure (Pa),
    fluid velocity (m/s), viscosity (Pa·s), temperature (K).
-/

structure EnvironmentParams where
  name           : String      -- "air_surface" / "subsea_500m" / "lunar" / …
  density        : Float       -- ambient fluid density
  gravity        : Float       -- local gravitational acceleration
  pressure       : Float       -- ambient hydrostatic / gas pressure
  currentVel     : Float       -- ambient fluid velocity (still air = 0)
  viscosity      : Float       -- dynamic viscosity (Pa·s)
  temperature    : Float       -- ambient (K), informational
  buoyancyEnabled : Bool       -- false in air/vacuum, true in dense fluid
  deriving Repr

namespace EnvironmentParams

/-- Standard surface air (the baseline used by PoC 1–12). -/
def airSurface : EnvironmentParams :=
  { name := "air_surface",
    density := 1.225,
    gravity := 9.81,
    pressure := 101325.0,
    currentVel := 0.0,
    viscosity := 1.81e-5,
    temperature := 293.15,
    buoyancyEnabled := false }

/-- 500 m deep ocean: PoC 13's headline workload. -/
def subsea500m : EnvironmentParams :=
  { name := "subsea_500m",
    density := 1025.0,
    gravity := 9.81,
    pressure := 101325.0 + 1025.0 * 9.81 * 500.0,  -- ≈ 5.13 MPa
    currentVel := 1.5,
    viscosity := 1.4e-3,
    temperature := 277.0,                          -- ~4 °C at depth
    buoyancyEnabled := true }

/-- Mariana-trench-scale (~11 000 m). Used as the canonical *crush* case
    for the negative test. -/
def marianaTrench : EnvironmentParams :=
  { subsea500m with
    name := "mariana_trench",
    pressure := 101325.0 + 1025.0 * 9.81 * 11000.0,  -- ≈ 111 MPa
    temperature := 275.0 }

/-- Lunar surface — forward-looking environment for non-aquatic PoCs. -/
def lunar : EnvironmentParams :=
  { name := "lunar",
    density := 0.0,
    gravity := 1.62,
    pressure := 1.0e-9,           -- effectively vacuum
    currentVel := 0.0,
    viscosity := 0.0,
    temperature := 250.0,
    buoyancyEnabled := false }

/-- Mars surface — thin CO₂ atmosphere (~600 Pa), modest dust-driven
    winds (we treat the average as a 5 m/s "current"). -/
def mars : EnvironmentParams :=
  { name := "mars",
    density := 0.020,             -- kg/m³ CO₂ at Mars surface
    gravity := 3.72,
    pressure := 600.0,            -- Pa
    currentVel := 5.0,            -- conservative wind/gust
    viscosity := 1.07e-5,
    temperature := 210.0,
    buoyancyEnabled := false }

/-- Convenience list of every shipping environment.  Used by the env-
    matrix tooling to fan the standard proofs across all targets. -/
def all : List EnvironmentParams :=
  [airSurface, subsea500m, lunar, mars]

end EnvironmentParams

end MechProof
