# Changelog — Formula-1-Dream-

Parametric Formula 1 car generator for the **Autodesk Fusion Python API** (build 2702.x, 2026).
Coordinate convention: **X** longitudinal (nose tip at X=0, increasing rearward), **Y** lateral, **Z** up; internal units in centimetres; angles as string expressions (e.g. `"16 deg"`). All version‑sensitive API calls were checked against the live Autodesk API reference.

---

## [Unreleased] — Feature-complete car: floor, suspension, one clean build per run

### Added
- **Floor.** A flat underbody plate running from just behind the front wheels to just past the rear axle (~1040 mm wide, 40 mm ride height, 25 mm thick). New independent component.
- **Suspension.** Double-wishbone arms at all four corners: an upper and a lower wishbone (each a fore + aft arm) reach from the body out to each wheel — the open-wheel look. Built as flat horizontal extrudes (planar sketch + extrude, robust in parametric mode); per-arm guarded so one bad arm can't lose the set.
- **Diffuser.** Rear underbody ramp that climbs from floor level toward the exit behind the rear axle. Side profile sketched in X-Z and extruded symmetrically across the width (centred, no left/right ambiguity). Independent, guarded component.

### Fixed
- **One clean car per run (idempotent rebuild).** `run()` now wipes all previously generated components before building, so re-running REPLACES the car instead of stacking duplicates (`EngineCover (1)`, `Halo (1)`, …). User parameters live on the design and survive, refreshed idempotently.
- **Engine cover loft warning.** The fixed 3 cm section fillet was too large for the tapered tail section (Fusion flagged the loft yellow). Fillet reduced to 2 cm and the tail section made less degenerate, so the loft is healthy.

### Changed
- **Health sweep** now reports the owning component, the health state (WARNING/ERROR), and a trimmed message — e.g. `[WARNING] EngineCover/Loft1: …` instead of a bare `Loft1`.

### Verification
- `py_compile` + `pyflakes` clean; full mock-API run executes monocoque, front wing, sidepods, floor, and suspension end-to-end.
- Floor and suspension are logic-verified but **pending first in-Fusion confirmation** (new geometry). Both are isolated and guarded — if either fails on a given build, the rest of the car still builds and the run report names the failure.

---

## [Unreleased] — Fusion 2702 build fixes + live wing angles

### Fixed
- **Halo pipe — `RuntimeError: Value does not contain a real`.** `PipeFeatureInput.sectionSize` is read as a real by `PipeFeatures.add()`, so a *string* `ValueInput` failed. Now seeded with `ValueInput.createByReal(diameter_cm)` (centimetres); the live link is restored afterward via the feature's `sectionSize` `ModelParameter.expression`.
- **`Environment is not supported` on wheels and all wing incidence rotations.** Construction axes built from a raw `InfiniteLine3D` are rejected in parametric mode. Both the wheel revolve axis and the incidence rotation axis are now **sketch lines** (entity‑based), which are valid in parametric mode.
- **Live angles `Invalid entity`.** The wings live in sub‑components placed via occurrences, so the Move feature runs in the assembly context; the body and axis are now converted with `createForAssemblyContext(occurrence)` before `defineAsRotate`.
- **Stray fifth wheel at the origin.** The master wheel is now built directly at the first station and the other three are occurrences of it, so exactly four wheels exist with no ghost at (0,0,0).
- **Rear wing was only half‑span.** The mainplane and flap are now mirrored across the car centreline (XZ) for full span; previously they were one‑sided and mismatched the endplates.
- **`DesignType` → `DesignTypes`** (plural) `ParametricDesignType` — a latent crash on startup.
- **`Path.create` → `Features.createPath(curve, isChain)`** for the halo paths.
- **`addDistanceDimension`** now receives `SketchPoint`s, not `SketchLine`s (the wheel section dimensions).
- **Manifest** product id `AutoCAD` → `Fusion` — was silently preventing the script from loading.

### Added
- **Live wing incidence / DRS.** Each wing element is rotated about its quarter‑chord spanwise (Y) axis by a parameter‑driven Move→Rotate (`MoveFeatures.createInput2` + `defineAsRotate`; the old `createInput` is retired). `rw_mainplane_angle`, `drs_flap_angle`, `fw_mainplane_angle` and `fw_flap_angle_step` now drive the geometry. `LIVE_ANGLES` master toggle; each element degrades gracefully (stays flat and is reported) if a rotation can't be applied.
- **Halo via `PipeFeatures`** — replaces the fragile manual profile+sweep pipeline (perpendicular plane → profile circle → sweep) that failed silently. Sets `creationOccurrence` (required for a path in a non‑root component) and raises loud, located errors on no‑body / error states. `_HALO_CLOSED_HOOP` toggle gives an open‑path fallback.
- **Wheels via occurrence instancing** (`addExistingComponent`) instead of `copyToComponent` + `moveFeatures`, which crashed on a zero transform. All four wheels share one editable master section.
- **Loud `run()` harness** — a start heartbeat, per‑step progress, and a full traceback naming the failing component on error, so a silent "Run does nothing" is impossible.
- **`BUILD_ONLY`** single‑component isolation mode for debugging.
- **`CONTINUE_ON_INDEPENDENT_FAILURE`** — independent components log failures and continue; the dependency chain (monocoque → halo → engine cover → rear wing) skips dependents with a clear note.
- **Idempotent parameters** — re‑running updates expressions instead of erroring on "already exists".
- **`finalize()`** timeline health sweep that reports any unhealthy feature by name.

### Verification
- `py_compile` and `pyflakes`: clean (valid syntax, no undefined names).
- Mock Fusion API execution: full build of all eight components, every `BUILD_ONLY` isolation mode, and the `LIVE_ANGLES=False` fallback — all pass.
- **Confirmed building in real Fusion 2702: ALL components** — monocoque, halo, engine cover, rear wing, front wing, wheels, sidepods. The full car assembles.
- **Fix applied, pending in‑Fusion confirmation:** live wing‑angle rotation (the `createForAssemblyContext` change for the "Invalid entity" error). If a rotation still can't apply, that element stays flat and is named in the run report.
- **Known open item:** if the closed‑hoop pipe produces no body on a given build, set `_HALO_CLOSED_HOOP = False` for an open‑path hoop.

### Tuning (not bugs — adjust the parameters)
- Wheelbase (3600 mm) is longer than the current body length, so the rear wheels sit behind the bodywork. Lengthen `tub_length` or shorten `wheelbase` to close the gap.
- Proportions (tub size, wheel size, wing positions) are placeholder values — now fully parametric, so drag them to taste.

---

## Earlier
- v1: monocoque, front wing, sidepods confirmed building in Fusion 2702.
