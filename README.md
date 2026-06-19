# Formula-1-Dream-

A **parametric Formula 1 car generator** written as a single-file script for the
**Autodesk Fusion 360 Python API**. Running the script builds a complete,
stylised open-wheel race car — body, wings, wheels, floor, and suspension —
entirely from code, driven by a table of named parameters.

The target silhouette is the **ground-effect era (2022–2025) car**, proportioned
after the **McLaren MCL39** (2025 Constructors' and Drivers' champion).

---

## Why this project exists

This is a portfolio project built to demonstrate **cross-domain complexity** —
combining software engineering with motorsport engineering in one artifact:

- **Software side:** programmatic CAD, a parametric design system, robust error
  handling, idempotent generation, version-specific API work, and a disciplined
  commit history used as living documentation.
- **Motorsport side:** real F1 geometry and proportions — airfoil cross-sections,
  wing incidence and DRS, ground-effect-era dimensions, double-wishbone
  suspension, and an underbody floor.

It grew out of hands-on motorsport experience with **Formula Student** (see
*Formula Student / South Bank Racing* below), translating an understanding of
real race-car components into a generative model.

---

## What it builds

The script generates nine geometry components plus two helpers, each using a
specific Fusion modelling technique:

| Component | What it is | Fusion API technique |
|---|---|---|
| **Monocoque** | Nose + survival cell + body, full car length | Multi-section **Loft** through rounded "eight-curve box" cross-sections on offset YZ planes, parametric dimensions |
| **Halo** | Cockpit safety hoop | **PipeFeatures** swept along a sketch-line path (real-seeded `sectionSize`, then a live diameter expression) |
| **Engine cover** | Airbox + engine/gearbox cowl | **Loft** from an intake profile through cover sections, stations spaced as fractions of body length |
| **Rear wing** | Mainplane + DRS flap, full span | Airfoil **Lofts**, **mirrored** across the centreline, with live incidence via **Move→Rotate** |
| **Front wing** | Mainplane + 3-element flap cascade + endplates | Extruded airfoil cascade with trailing-edge chaining, mirrored, live incidence |
| **Wheels** | Four tyres at the corners | One **revolved** master section (about a sketch-line axis) **instanced** to four corners via Occurrences |
| **Sidepods** | Sculpted side bodywork with undercut | Twin **Loft** (mirrored), sections spaced as fractions of body length |
| **Floor** | Flat underbody plate | Single thin **Extrude** of a rectangle on an offset XY plane |
| **Suspension** | Double-wishbone arms at all 4 corners | 16 thin planar **Extrudes** forming upper + lower wishbones (fore + aft arms) |
| *airfoil_section* | NACA-style profile generator | Fitted-spline helper used by both wings |
| *finalize()* | Post-build health check | Timeline sweep reporting any unhealthy feature by component + state |

The `run()` harness ties these together with **idempotent clear-and-rebuild**
(each run wipes the previous car and builds exactly one), **guarded independent
components** (one failure doesn't lose the whole car), and a loud summary dialog.

---

## Current build status

**Tested on Fusion 360 build 2702.1.58 (April 2026), Windows.**

**Working — confirmed in Fusion:**
- Monocoque, halo, engine cover, rear wing, front wing, wheels, sidepods all build.
- **Live wing angles** apply to every wing element (mainplanes, DRS flap, front-wing flap cascade).
- **MCL39-like proportions:** full-length body (~5.4 m), wheels at the corners, rear wing hung behind the rear axle.
- **One clean car per run** — re-running replaces the car instead of stacking duplicates.

**Added and logic-verified, pending first in-Fusion confirmation:**
- **Floor** and **suspension** (compile + lint + mock-API run all pass; first real-Fusion run is the next check).
- Engine cover loft fillet/taper fix (to clear a health-sweep warning) and the upgraded health sweep.

---

## How to run

1. Use **Fusion 360 build 2702.1.x** (the script targets API changes introduced
   around 2702; older builds may differ).
2. Copy `F1_Design.py` **and** `F1_Design.manifest` into:
   `C:\Users\<you>\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\Scripts\F1_Design\`
   *(The `.manifest` is required — Fusion silently ignores a script without it.)*
3. Open Fusion with a **new, empty design** that has **Design History enabled**
   (parametric mode). The script also forces `ParametricDesignType` itself, and
   it clears any prior generated geometry on each run, so re-running is safe.
4. **Utilities → Add-Ins → Scripts and Add-Ins** (`Shift+S`) → **Scripts** tab →
   select **F1_Design** → **Run**.
5. Read the summary dialog: it lists each component as it completes, reports
   whether live angles applied, and runs a final health sweep.

To debug a single component, set `BUILD_ONLY` at the top of `run()` to one of
`"monocoque"`, `"halo"`, `"engine_cover"`, `"rear_wing"`, `"front_wing"`,
`"wheels"`, `"sidepods"`, `"floor"`, `"suspension"`, or `"all"`.

---

## Parametric map — what drags live vs what needs a re-run

Geometry **sizes and angles** are driven by user-parameter *expressions*, so
editing them in the Parameters dialog updates the model on recompute. Component
**positions** that are read once at build time are baked in, so changing those
parameters needs a **re-run** of the script. The reliable workflow is always:
*change parameters → re-run.*

| Live (edit parameter → recompute) | Frozen at build (edit parameter → re-run script) |
|---|---|
| Body cross-section sizes | Wheel positions (`front_axle_x`, `wheelbase`, `front_track`, `rear_track`) |
| Wing chord / thickness / span | Rear-wing longitudinal station |
| Wing incidence + DRS (`rw_mainplane_angle`, `drs_flap_angle`, `fw_mainplane_angle`, `fw_flap_angle_step`) | Floor extent + suspension attachment points |
| Wheel / tyre sizes (`tyre_outer_diameter`, `wheel_rim_diameter`, `tyre_width`) | Floor ride height + thickness, suspension arm thickness (code constants) |
| Engine-cover / sidepod section sizes + stations | |
| Halo pipe diameter | |

---

## Key Fusion 2702 API notes

Version-specific fixes that were required to make the script run on 2702:

- `adsk.fusion.DesignTypes.ParametricDesignType` — the enum is **plural** (`DesignTypes`).
- Paths use `Features.createPath(curve, isChain)` rather than `Path.create()`.
- Move features use `MoveFeatures.createInput2(...)`; the old `createInput` is retired.
- `PipeFeatureInput.sectionSize` must be **seeded with `ValueInput.createByReal`** (a string `ValueInput` throws "Value does not contain a real"); the live diameter is then set via the feature's `sectionSize` `ModelParameter.expression`.
- Rotation/revolve **axes must be sketch lines** in parametric mode — a raw `InfiniteLine3D` construction axis is rejected ("Environment is not supported").
- Geometry inside a sub-component (placed via an occurrence) must be converted with `createForAssemblyContext(occurrence)` before a Move→Rotate, or `defineAsRotate` throws "Invalid entity".
- `addDistanceDimension` takes **SketchPoints**, not SketchLines.
- The `.manifest` product id must be **`Fusion`** (not `AutoCAD`), or the script won't load.

---

## Known issues & next steps

**Known / pending:**
- Floor and suspension await their first confirmed in-Fusion build.
- The model is a **stylised lofted approximation** — it reads as "a modern F1
  car," not a photoreal MCL39, and intentionally carries no team livery or
  branded surfaces.

**Next steps (polish, not missing parts):**
- Rear **diffuser** (floor kicking up at the back).
- Suspension **pushrods** (the diagonal link).
- Raised, narrower **nose** and a lower overall **stance**.
- Coke-bottle sidepod refinement toward the MCL39 profile.

---

## Formula Student / South Bank Racing

The domain knowledge behind this model comes from involvement with **Formula
Student** through **South Bank Racing**. Working on a real competition car —
understanding how a monocoque, wings, suspension, and aero surfaces actually fit
together — is what made it possible to translate those components into a
parametric generator rather than just modelling a generic "car shape." This
project is, in effect, that hands-on engineering encoded as software.

---

## Tech

- **Autodesk Fusion 360** build 2702.1.58, Python API (`adsk.core`, `adsk.fusion`)
- Single-file script: `F1_Design.py` (+ required `F1_Design.manifest`)
- Coordinate convention: **X** longitudinal (nose tip at 0, increasing rearward), **Y** lateral, **Z** up; internal units in centimetres
- Documentation: this README + `CHANGELOG.md` + the Git commit history
