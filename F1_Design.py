################################################################################
# f1_car.py  --  Parametric F1 car for Autodesk Fusion (single runnable file)
#
# Target build: Fusion 2702.x (April 2026). Verified against the live Autodesk
# API reference (help.autodesk.com, "April 2026 API Help") in May 2026.
#
# EIGHT components, each in its own component/occurrence:
#   monocoque, halo, engine cover, rear wing, front wing, wheels, sidepods.
# Plus finalize() (timeline health sweep + optional export) and a LOUD run()
# that reports progress at every step so a silent no-op is impossible.
#
# ---------------------------------------------------------------------------
# HOW TO RUN (read this if "Run" seems to do nothing):
#   1. Open or create a Fusion DESIGN document first (File > New Design). Do NOT
#      reuse a document that has half-built geometry from a failed run.
#   2. In Utilities > Scripts and Add-Ins, ideally use "Create New > Script" so
#      Fusion generates a correct manifest for your build, then paste this code
#      into the generated .py. (A bad/missing manifest makes Fusion silently
#      refuse to load a script.)
#   3. Run. A "build starting" box appears immediately -- if it does NOT, the
#      problem is loading/manifest, not this code (run a hello-world stub to
#      confirm the Python engine). If it DOES, you'll get a step-by-step report.
#
# CONVENTION: X longitudinal (nose tip at X=0, increasing rearward +);
#             Y lateral; Z up. Internal units are CM; angles via "<n> deg".
#
# VERIFIED API (live docs, signatures shown):
#   adsk.fusion.DesignTypes.ParametricDesignType                      [confirmed]
#   Features.createPath(curve, isChain=True)                          [confirmed]
#   ConstructionPlaneInput.setByOffset(planarEntity, ValueInput)      [confirmed]
#   ConstructionPlaneInput.setByDistanceOnPath(path, ValueInput 0..1) [confirmed]
#   PipeFeatures.createInput(path, operation)                         [confirmed]
#   PipeFeatureInput: .sectionSize .isHollow .creationOccurrence      [confirmed]
#   PipeFeature.sectionSize -> ModelParameter (set .expression=live)  [confirmed]
#   Occurrences.addNewComponent(Matrix3D)                             [confirmed]
#   Occurrences.addExistingComponent(component, Matrix3D)             [confirmed]
#   MoveFeatures.createInput2(entities)  (createInput is retired)      [confirmed]
#   MoveFeatureInput.defineAsRotate(axisEntity, angle ValueInput)      [confirmed]
#   SweepFeatures.createInput(profile, path, operation)               [confirmed]
#   MoveFeatures.add() throws on a ZERO transform -> avoided here     [confirmed]
#
# PARAMETRIC-MODE LESSONS (learned from real 2702 runtime errors):
#   - PipeFeatureInput.sectionSize needs a REAL ValueInput (cm); a string
#     expression throws "Value does not contain a real". Live link is set after
#     creation via the feature's sectionSize ModelParameter .expression.
#   - Construction axes/points from RAW geometry (InfiniteLine3D / Point3D) are
#     rejected in parametric mode ("Environment is not supported"). Revolve and
#     Move>Rotate axes are therefore SKETCH LINES (entity-based), which work.
#
# STILL "# VERIFY" (stable/long-standing, but not re-fetched this pass; confirm
# if a line throws): addDistanceDimension orientation enum, addFillet,
# sketchEllipses.add, sketchFittedSplines.add + isClosed, addByThreePoints,
# revolve setAngleExtent, mirrorFeatures.createInput, FeatureHealthStates enum.
####################################################################################

import math
import adsk.core
import adsk.fusion
import adsk.cam
import traceback


# ===========================================================================
# 0. SHARED UTILITIES
# ===========================================================================
def _get_param_value(params, name, default_cm):
    """Read a user parameter's value in internal units (cm), with a fallback."""
    try:
        p = params.get(name)
        if p is not None:
            return p.value
    except Exception:
        pass
    return default_cm


def _err(feature):
    """Return True if a feature is in an error health state (silent-fail guard)."""
    try:
        return feature.healthState == adsk.fusion.FeatureHealthStates.ErrorFeatureHealthState
    except Exception:
        return False


def setup_design(app):
    """Cast the active product to a Design and force PARAMETRIC mode.

    Uses app.activeProduct (NOT documents.add) -- requires an OPEN design doc.
    If the active product isn't a Design, returns None and the caller reports it
    loudly. DesignTypes is PLURAL (confirmed against the live docs).
    """
    design = adsk.fusion.Design.cast(app.activeProduct)
    if design:
        try:
            design.designType = adsk.fusion.DesignTypes.ParametricDesignType
        except Exception:
            # Already in the right mode, or a doc type that rejects the set --
            # don't hard-crash before any geometry is built.
            pass
    return design


# ===========================================================================
# 1. PARAMETER TABLE  (defined BEFORE any geometry; idempotent on re-run)
# ===========================================================================
def create_parameters(design):
    """Create every user parameter; return {name: UserParameter}."""
    up = design.userParameters
    VI = adsk.core.ValueInput

    def add(name, expression, units, comment):
        existing = up.itemByName(name)
        if existing:
            existing.expression = expression
            return existing
        return up.add(name, VI.createByString(expression), units, comment)

    p = {}
    # --- monocoque ---  (MCL39-like proportions: long body that runs the full
    # length of the car so the rear wheels sit IN the bodywork, not behind it.)
    p["nose_length"]    = add("nose_length",    "1000 mm", "mm", "Nose cone length (tip to front bulkhead)")
    p["tub_length"]     = add("tub_length",     "3600 mm", "mm", "Body length: survival cell + engine + gearbox bay")
    p["tub_max_width"]  = add("tub_max_width",  "600 mm",  "mm", "Max tub width")
    p["tub_height"]     = add("tub_height",     "550 mm",  "mm", "Max tub height")
    p["nose_tip_width"] = add("nose_tip_width", "120 mm",  "mm", "Nose-tip ellipse width")
    # --- halo ---
    p["halo_height"]        = add("halo_height",        "850 mm",  "mm", "Hoop apex height above z=0")
    p["halo_tube_diameter"] = add("halo_tube_diameter", "100 mm",  "mm", "Round halo tube diameter")
    p["halo_pylon_x"]       = add("halo_pylon_x",       "1150 mm", "mm", "Pylon foot / hoop front apex X")
    # --- engine cover ---
    p["airbox_halo_clearance"]   = add("airbox_halo_clearance",   "40 mm",  "mm", "Halo->intake gap")
    p["cover_taper_length"]      = add("cover_taper_length",      "600 mm", "mm", "Cover taper length")
    p["intake_width"]            = add("intake_width",            "250 mm", "mm", "Intake mouth width (shared)")
    p["intake_height"]           = add("intake_height",           "280 mm", "mm", "Intake mouth height (shared)")
    p["airbox_cover_max_width"]  = add("airbox_cover_max_width",  "320 mm", "mm", "Cover max width")
    p["airbox_cover_max_height"] = add("airbox_cover_max_height", "350 mm", "mm", "Cover max height")
    # --- rear wing ---
    p["rw_span"]            = add("rw_span",            "1000 mm", "mm", "Full rear wing span")
    p["rw_chord_root"]      = add("rw_chord_root",      "280 mm",  "mm", "Mainplane chord root")
    p["rw_chord_tip"]       = add("rw_chord_tip",       "250 mm",  "mm", "Mainplane chord tip")
    p["rw_mainplane_angle"] = add("rw_mainplane_angle", "16 deg",  "deg", "Mainplane AoA (see angle note)")
    p["rw_thickness"]       = add("rw_thickness",       "9 mm",    "mm", "RW foil thickness (mm scratch)")
    p["rw_camber"]          = add("rw_camber",          "5 mm",    "mm", "RW foil camber (mm scratch)")
    p["rw_height"]          = add("rw_height",          "350 mm",  "mm", "Wing Z above cover tail")
    p["rw_overhang"]        = add("rw_overhang",        "800 mm",  "mm", "Wing X aft of body rear (hangs behind rear axle)")
    p["drs_slot_gap_x"]     = add("drs_slot_gap_x",     "20 mm",   "mm", "DRS slot gap X")
    p["drs_slot_gap_z"]     = add("drs_slot_gap_z",     "35 mm",   "mm", "DRS slot gap Z")
    p["drs_flap_angle"]     = add("drs_flap_angle",     "28 deg",  "deg", "DRS flap angle (see angle note)")
    p["rw_endplate_height"] = add("rw_endplate_height", "350 mm",  "mm", "RW endplate height")
    p["rw_endplate_length"] = add("rw_endplate_length", "320 mm",  "mm", "RW endplate length")
    p["endplate_thickness"] = add("endplate_thickness", "10 mm",   "mm", "Endplate thickness (shared)")
    p["strut_diameter"]     = add("strut_diameter",     "40 mm",   "mm", "Swan-neck strut diameter")
    # --- front wing ---
    p["fw_span"]              = add("fw_span",              "1900 mm", "mm", "Full front wing span (near max width)")
    p["fw_chord"]             = add("fw_chord",             "330 mm",  "mm", "Main plane chord")
    p["fw_flap_chord"]        = add("fw_flap_chord",        "220 mm",  "mm", "Flap chord")
    p["fw_thickness"]         = add("fw_thickness",         "0.10",    "",   "FW foil thickness fraction")
    p["fw_camber"]            = add("fw_camber",            "0.03",    "",   "FW foil camber fraction")
    p["fw_mainplane_angle"]   = add("fw_mainplane_angle",   "3 deg",   "deg", "Main plane AoA (see angle note)")
    p["fw_flap_angle_step"]   = add("fw_flap_angle_step",   "8 deg",   "deg", "AoA added per flap (see angle note)")
    p["fw_height_off_ground"] = add("fw_height_off_ground", "75 mm",   "mm", "Main plane Z (ride height)")
    p["fw_x_offset"]          = add("fw_x_offset",          "-200 mm", "mm", "Wing X (negative = ahead of nose tip)")
    p["fw_slot_gap_x"]        = add("fw_slot_gap_x",        "20 mm",   "mm", "Flap LE aft of prev TE")
    p["fw_slot_gap_z"]        = add("fw_slot_gap_z",        "15 mm",   "mm", "Flap LE above prev TE")
    p["fw_endplate_height"]   = add("fw_endplate_height",   "280 mm",  "mm", "FW endplate height")
    p["fw_endplate_length"]   = add("fw_endplate_length",   "350 mm",  "mm", "FW endplate length")
    p["fw_endplate_curl"]     = add("fw_endplate_curl",     "40 mm",   "mm", "FW endplate curl Y-depth")
    p["fw_pylon_diameter"]    = add("fw_pylon_diameter",    "20 mm",   "mm", "Nose strut diameter")
    # --- wheels ---
    p["wheelbase"]           = add("wheelbase",           "3600 mm", "mm", "Front axle to rear axle")
    p["front_track"]         = add("front_track",         "1700 mm", "mm", "Front wheel centre-to-centre")
    p["rear_track"]          = add("rear_track",          "1550 mm", "mm", "Rear wheel centre-to-centre")
    p["tyre_outer_diameter"] = add("tyre_outer_diameter", "720 mm",  "mm", "Tyre outer diameter")
    p["wheel_rim_diameter"]  = add("wheel_rim_diameter",  "457 mm",  "mm", "Rim diameter (18 in)")
    p["tyre_width"]          = add("tyre_width",          "305 mm",  "mm", "Tyre width")
    p["front_axle_x"]        = add("front_axle_x",        "950 mm",  "mm", "X of the front axle line (nose overhangs ahead)")
    # --- sidepods ---
    p["sidepod_front_x"]      = add("sidepod_front_x",      "nose_length + tub_length * 0.45", "mm",
                                    "Intake-face X, expressed so it tracks the tub")
    p["sidepod_length"]       = add("sidepod_length",       "1400 mm", "mm", "Sidepod length")
    p["sidepod_height"]       = add("sidepod_height",       "200 mm", "mm", "Sidepod Z above floor")
    p["sidepod_inner_offset"] = add("sidepod_inner_offset", "50 mm",  "mm", "Gap between tub side and pod")
    p["sidepod_max_width"]    = add("sidepod_max_width",    "280 mm", "mm", "Sidepod max width")
    p["sidepod_max_height"]   = add("sidepod_max_height",   "320 mm", "mm", "Sidepod max height")
    p["sidepod_curl_depth"]   = add("sidepod_curl_depth",   "30 mm",  "mm", "Undercut depth (coke-bottle pinch)")
    return p


# ===========================================================================
# 2. AIRFOIL HELPER  (pure-Python NACA 4-digit -> one closed spline profile)
# ===========================================================================
def _naca4_points(chord, thickness, camber, camber_pos, n_points):
    """Return (upper, lower) lists of (x, y) cm tuples for a NACA 4-digit foil."""
    m, p, t = camber, camber_pos, thickness
    n_per = max(3, int(n_points // 2))

    def yt_of(xn):
        return 5.0 * t * (0.2969 * math.sqrt(xn) - 0.1260 * xn - 0.3516 * xn**2
                          + 0.2843 * xn**3 - 0.1015 * xn**4)

    def yc_and_theta(xn):
        if m == 0.0 or p == 0.0:
            return 0.0, 0.0
        if xn < p:
            yc = (m / p**2) * (2 * p * xn - xn**2); dyc = (2 * m / p**2) * (p - xn)
        else:
            yc = (m / (1 - p)**2) * ((1 - 2 * p) + 2 * p * xn - xn**2); dyc = (2 * m / (1 - p)**2) * (p - xn)
        return yc, math.atan(dyc)

    upper, lower = [], []
    for i in range(n_per):
        beta = math.pi * i / (n_per - 1)
        xn = (1.0 - math.cos(beta)) / 2.0   # cosine spacing: dense at LE/TE
        yt = yt_of(xn)
        yc, theta = yc_and_theta(xn)
        upper.append(((xn - yt * math.sin(theta)) * chord, (yc + yt * math.cos(theta)) * chord))
        lower.append(((xn + yt * math.sin(theta)) * chord, (yc - yt * math.cos(theta)) * chord))
    return upper, lower


def airfoil_section(sketch, chord, thickness, camber, origin,
                    camber_pos=0.4, n_points=40, blunt_te=True):
    """Draw one NACA airfoil as a single closed loop; return a dict of handles.

    FROZEN vs LIVE: thickness/camber/chord are baked into the points (re-run to
    reshape). aoa is the caller's job (tilt the plane); origin position follows
    the caller's (possibly parametric) plane.
    """
    upper_pts, lower_pts = _naca4_points(chord, thickness, camber, camber_pos, n_points)
    # One ordered perimeter: TE(top) -> upper -> LE -> lower -> TE(bottom).
    perimeter = list(reversed(upper_pts)) + lower_pts[1:]
    P = adsk.core.Point3D.create
    pts = adsk.core.ObjectCollection.create()
    for (x, y) in perimeter:
        pts.add(P(origin.x + x, origin.y + y, 0))
    spline = sketch.sketchCurves.sketchFittedSplines.add(pts)  # VERIFY
    if blunt_te:
        # Close the TE with one short line -> one closed loop, robust for loft.
        te_u = P(origin.x + upper_pts[-1][0], origin.y + upper_pts[-1][1], 0)
        te_l = P(origin.x + lower_pts[-1][0], origin.y + lower_pts[-1][1], 0)
        sketch.sketchCurves.sketchLines.addByTwoPoints(te_u, te_l)  # VERIFY
    else:
        spline.isClosed = True  # VERIFY closure mechanism
    profile = sketch.profiles.item(0)
    le_point = P(origin.x, origin.y, 0)
    te_x = (upper_pts[-1][0] + lower_pts[-1][0]) / 2.0
    te_y = (upper_pts[-1][1] + lower_pts[-1][1]) / 2.0
    te_point = P(origin.x + te_x, origin.y + te_y, 0)
    qc_point = P(origin.x + 0.25 * chord, origin.y, 0)
    return {"profile": profile, "le_point": le_point,
            "te_point": te_point, "quarter_chord_point": qc_point}


# ===========================================================================
# 3. SHARED SKETCH / PATH HELPERS
# ===========================================================================
def _draw_eight_curve_box(sketch, half_w_expr, half_h_expr, top_fillet_cm, bottom_fillet_cm):
    """Centred, axis-aligned rounded box -> one closed profile (8 curves always).

    Parametric size via ORIGIN->edge-midpoint distance dimensions (immune to the
    corner fillets). Top/bottom fillet radii differ to make intake/undercut
    silhouettes WITHOUT changing the curve count -- the loft topology contract.
    """
    lines = sketch.sketchCurves.sketchLines
    arcs  = sketch.sketchCurves.sketchArcs
    geo   = sketch.geometricConstraints
    dims  = sketch.sketchDimensions
    pts   = sketch.sketchPoints
    HORIZ = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
    VERT  = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
    P = adsk.core.Point3D.create
    hw, hh = 5.0, 3.0
    ln_b = lines.addByTwoPoints(P(-hw, -hh, 0), P(hw, -hh, 0))
    ln_r = lines.addByTwoPoints(ln_b.endSketchPoint, P(hw, hh, 0))
    ln_t = lines.addByTwoPoints(ln_r.endSketchPoint, P(-hw, hh, 0))
    ln_l = lines.addByTwoPoints(ln_t.endSketchPoint, ln_b.startSketchPoint)
    geo.addHorizontal(ln_b); geo.addHorizontal(ln_t); geo.addVertical(ln_l); geo.addVertical(ln_r)
    mp_r = pts.add(P(hw, 0, 0));  geo.addMidPoint(mp_r, ln_r)
    mp_l = pts.add(P(-hw, 0, 0)); geo.addMidPoint(mp_l, ln_l)
    mp_t = pts.add(P(0, hh, 0));  geo.addMidPoint(mp_t, ln_t)
    mp_b = pts.add(P(0, -hh, 0)); geo.addMidPoint(mp_b, ln_b)
    txt = P(hw * 1.6, hh * 1.6, 0)
    d_r = dims.addDistanceDimension(sketch.originPoint, mp_r, HORIZ, txt)  # VERIFY orientation enum
    d_l = dims.addDistanceDimension(sketch.originPoint, mp_l, HORIZ, txt)
    d_t = dims.addDistanceDimension(sketch.originPoint, mp_t, VERT,  txt)
    d_b = dims.addDistanceDimension(sketch.originPoint, mp_b, VERT,  txt)
    d_r.parameter.expression = half_w_expr; d_l.parameter.expression = half_w_expr
    d_t.parameter.expression = half_h_expr; d_b.parameter.expression = half_h_expr
    c_br = ln_b.endSketchPoint.geometry; c_tr = ln_r.endSketchPoint.geometry
    c_tl = ln_t.endSketchPoint.geometry; c_bl = ln_b.startSketchPoint.geometry
    arcs.addFillet(ln_b, c_br, ln_r, c_br, bottom_fillet_cm)  # VERIFY
    arcs.addFillet(ln_r, c_tr, ln_t, c_tr, top_fillet_cm)
    arcs.addFillet(ln_t, c_tl, ln_l, c_tl, top_fillet_cm)
    arcs.addFillet(ln_l, c_bl, ln_b, c_bl, bottom_fillet_cm)
    return sketch.profiles.item(0)


def _draw_rounded_rect_profile(sketch, half_w_expr, half_h_expr, fillet_cm):
    return _draw_eight_curve_box(sketch, half_w_expr, half_h_expr, fillet_cm, fillet_cm)


def _draw_nose_tip_ellipse_profile(sketch, width_cm, height_cm):
    P = adsk.core.Point3D.create
    sketch.sketchCurves.sketchEllipses.add(P(0, 0, 0), P(width_cm/2.0, 0, 0), P(0, height_cm/2.0, 0))  # VERIFY
    return sketch.profiles.item(0)


def _build_span_plane(comp, y_expr, name):
    """XZ-offset construction plane (span along Y). Used by both wings.
    VERIFY the chord lands along world X (streamwise), not vertical, by eyeing
    one section before trusting the whole wing.
    """
    planes = comp.constructionPlanes
    pin = planes.createInput()
    pin.setByOffset(comp.xZConstructionPlane, adsk.core.ValueInput.createByString(y_expr))  # confirmed
    pl = planes.add(pin); pl.name = name
    return pl


# ===========================================================================
# 4. MONOCOQUE  (lofted tub)
# ===========================================================================
def create_monocoque(root_comp, params, design):
    occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())  # confirmed
    comp = occ.component; comp.name = "Monocoque"
    planes = comp.constructionPlanes; sketches = comp.sketches; VI = adsk.core.ValueInput

    def offset_plane(x_expr, name):
        pin = planes.createInput()
        pin.setByOffset(comp.yZConstructionPlane, VI.createByString(x_expr))  # confirmed
        pl = planes.add(pin); pl.name = name; return pl

    plane = [comp.yZConstructionPlane,
             offset_plane("nose_length * 0.5", "X_S1"),
             offset_plane("nose_length", "X_S2"),
             offset_plane("nose_length + tub_length * 0.30", "X_S3"),
             offset_plane("nose_length + tub_length * 0.65", "X_S4"),
             offset_plane("nose_length + tub_length", "X_S5")]
    profiles = [None] * 6
    ntw = (params.get("nose_tip_width") or design.userParameters.itemByName("nose_tip_width")).value
    sk0 = sketches.add(plane[0]); sk0.name = "Sec_S0"
    profiles[0] = _draw_nose_tip_ellipse_profile(sk0, ntw, ntw * 0.6)
    for idx, hw_e, hh_e in [(1, "tub_max_width*0.30/2", "tub_height*0.35/2"),
                            (2, "tub_max_width*0.55/2", "tub_height*0.55/2"),
                            (3, "tub_max_width*0.90/2", "tub_height*0.90/2"),
                            (4, "tub_max_width/2",      "tub_height/2"),
                            (5, "tub_max_width*0.75/2", "tub_height*0.70/2")]:
        sk = sketches.add(plane[idx]); sk.name = "Sec_S%d" % idx
        profiles[idx] = _draw_rounded_rect_profile(sk, hw_e, hh_e, 1.0)
    lf = comp.features.loftFeatures
    li = lf.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    for i in range(6):
        li.loftSections.add(profiles[i])
    li.isSolid = True
    loft = lf.add(li)
    if _err(loft):
        raise RuntimeError("Monocoque loft: %s" % loft.errorOrWarningMessage)
    body = loft.bodies.item(0); body.name = "Monocoque_Tub"
    return {"component": comp, "body": body, "front_plane": plane[0],
            "rear_plane": plane[5], "bbox": body.boundingBox}


# ===========================================================================
# 5. HALO  (PipeFeatures: native round tube along the hoop + pylon paths)
# ===========================================================================
# If the closed hoop pipe yields no body, flip this to False for an open path
# (tiny seam at the apex, hidden by the pylon).
_HALO_CLOSED_HOOP = True


def _build_hoop_path(comp, params):
    h = _get_param_value(params, "halo_height", 85.0)
    px = _get_param_value(params, "halo_pylon_x", 115.0)
    factors = [(0.00, 0.00, 1.00), (0.12, 0.16, 0.97), (0.45, 0.30, 0.90),
               (0.85, 0.20, 0.85), (0.85, -0.20, 0.85), (0.45, -0.30, 0.90),
               (0.12, -0.16, 0.97)]
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = "Halo_hoop_path"
    pts = adsk.core.ObjectCollection.create()
    for fx, fy, fz in factors:
        # World coords -> sketch space (identity on XY; corrective on any other
        # plane). Guarantees correct 3D placement of the closed spline.
        world_pt = adsk.core.Point3D.create(px + fx * h, fy * h, fz * h)
        pts.add(sk.modelToSketchSpace(world_pt))  # VERIFY modelToSketchSpace
    spline = sk.sketchCurves.sketchFittedSplines.add(pts)  # VERIFY
    spline.isClosed = _HALO_CLOSED_HOOP                    # VERIFY closure behaviour
    return comp.features.createPath(spline, True)          # confirmed: createPath(curve, isChain)


def _build_pylon_path(comp, params):
    h = _get_param_value(params, "halo_height", 85.0)
    px = _get_param_value(params, "halo_pylon_x", 115.0)
    base_z = _get_param_value(params, "tub_height", 55.0) / 2.0
    P = adsk.core.Point3D.create
    sk = comp.sketches.add(comp.xZConstructionPlane); sk.name = "Halo_pylon_path"
    a = sk.sketchCurves.sketchArcs.addByThreePoints(  # VERIFY
        P(px + 0.06 * h, base_z, 0), P(px + 0.12 * h, (base_z + h) / 2.0, 0), P(px, h, 0))
    return comp.features.createPath(a, True)          # confirmed


def _pipe_tube(comp, occ, path, diameter_cm, diameter_expr, operation):
    """Native round Pipe along `path`. Returns the feature.

    Replaces the old manual profile+sweep. sectionSize MUST be seeded with a
    REAL ValueInput (cm) -- PipeFeatures.add() reads it as a real and a string
    ValueInput throws "Value does not contain a real". The live link to the
    parameter is set AFTER creation via the feature's sectionSize ModelParameter.
    """
    pipes = comp.features.pipeFeatures              # confirmed
    pin = pipes.createInput(path, operation)        # confirmed: createInput(path, operation)
    pin.isHollow = False                            # confirmed property
    # NOTE: circular is the DEFAULT section, so we deliberately do NOT set
    # sectionType (avoids any PipeSectionTypes enum-name risk on 2702).
    # sectionSize needs a REAL (cm), NOT a string expression (confirmed by the
    # "Value does not contain a real" failure on 2702).
    pin.sectionSize = adsk.core.ValueInput.createByReal(diameter_cm)  # real cm value

    # REQUIRED when the path is in a non-root component (our halo occurrence),
    # or the pipe geometry can transform to the wrong place / misbuild silently.
    try:
        pin.creationOccurrence = occ                # confirmed property
    except Exception:
        pass

    pipe = pipes.add(pin)

    # Make the diameter GENUINELY LIVE: PipeFeature.sectionSize is a
    # ModelParameter -> set its expression so dragging halo_tube_diameter updates
    # the tube. (Best-effort; guarded in case the property is read-only here.)
    try:
        if pipe.sectionSize is not None:
            pipe.sectionSize.expression = diameter_expr  # confirmed: sectionSize -> ModelParameter
    except Exception:
        pass

    # LOUD GUARDS -- kill the silent-failure mode.
    if _err(pipe):
        raise RuntimeError("Halo pipe ERROR: %s" % pipe.errorOrWarningMessage)
    if pipe.bodies.count == 0:
        raise RuntimeError(
            "Halo pipe created NO BODY. If this is the closed hoop, set "
            "_HALO_CLOSED_HOOP=False (open path). For a closed pipe, distanceOne "
            "+ distanceTwo must sum to <= 1.0 (PipeFeatureInput docs).")
    return pipe


def create_halo(root_comp, params, design, mono):
    occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component; comp.name = "Halo"
    NB = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    dia_cm = _get_param_value(params, "halo_tube_diameter", 10.0)  # cm (real seed)

    hoop = _pipe_tube(comp, occ, _build_hoop_path(comp, params),
                      dia_cm, "halo_tube_diameter", NB).bodies.item(0)
    hoop.name = "Halo_hoop"
    pylon = _pipe_tube(comp, occ, _build_pylon_path(comp, params),
                       dia_cm, "halo_tube_diameter", NB).bodies.item(0)
    pylon.name = "Halo_pylon"

    tools = adsk.core.ObjectCollection.create(); tools.add(pylon)
    ci = comp.features.combineFeatures.createInput(hoop, tools)
    ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    ci.isKeepToolBodies = False
    ci.isNewComponent = False
    cmb = comp.features.combineFeatures.add(ci)
    if _err(cmb):
        raise RuntimeError("Halo union: %s" % cmb.errorOrWarningMessage)

    unioned = hoop
    if (not unioned.isValid) or comp.bRepBodies.count < 1:
        raise RuntimeError("Halo union produced no valid body.")
    unioned.name = "Halo"
    return {"component": comp, "body": unioned, "hoop_body": hoop, "pylon_body": pylon,
            "rear_extent_x": "halo_pylon_x + halo_height*0.85"}


# ===========================================================================
# 6. ENGINE COVER  (single loft; intake S0 + body S1..S3 + tail S4)
# ===========================================================================
def _draw_intake_profile(sketch, w_expr, h_expr):
    return _draw_eight_curve_box(sketch, w_expr, h_expr, 1.0, 6.0)   # inverted-D


def _draw_cover_section(sketch, w_expr, h_expr):
    return _draw_eight_curve_box(sketch, w_expr, h_expr, 2.0, 2.0)   # smaller fillet fits small tail sections


def create_engine_cover(root_comp, params, design, mono, halo):
    occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component; comp.name = "EngineCover"
    planes = comp.constructionPlanes; sketches = comp.sketches; VI = adsk.core.ValueInput
    front_expr = "(%s) + airbox_halo_clearance" % halo["rear_extent_x"]
    tail_x_expr = "nose_length + tub_length - 100 mm"
    # Span the cover's intermediate sections across the FULL front->tail length
    # (as fractions), not a fixed cover_taper_length. Otherwise, when the body is
    # long, S0..S3 bunch up at the front and the loft must bridge a huge gap to
    # the tail -> LOFT_NO_TOOLBODY. Fractions keep the sections evenly spread.
    span_expr = "((%s) - (%s))" % (tail_x_expr, front_expr)

    def offset_plane(x_expr, name):
        pin = planes.createInput()
        pin.setByOffset(comp.yZConstructionPlane, VI.createByString(x_expr))
        pl = planes.add(pin); pl.name = name; return pl

    def at(frac):
        return "(%s) + %s*%s" % (front_expr, span_expr, frac)

    plane = [offset_plane(front_expr, "EC_S0"),
             offset_plane(at("0.22"), "EC_S1"),
             offset_plane(at("0.48"), "EC_S2"),
             offset_plane(at("0.80"), "EC_S3"),
             offset_plane(tail_x_expr, "EC_S4")]
    profiles = [None] * 5
    sk0 = sketches.add(plane[0]); sk0.name = "EC_S0"
    profiles[0] = _draw_intake_profile(sk0, "intake_width/2", "intake_height/2")
    for idx, hw_e, hh_e in [(1, "airbox_cover_max_width*0.75/2", "airbox_cover_max_height*0.80/2"),
                            (2, "airbox_cover_max_width/2",      "airbox_cover_max_height/2"),
                            (3, "airbox_cover_max_width*0.55/2", "airbox_cover_max_height*0.45/2"),
                            (4, "airbox_cover_max_width*0.28/2", "airbox_cover_max_height*0.22/2")]:
        sk = sketches.add(plane[idx]); sk.name = "EC_S%d" % idx
        profiles[idx] = _draw_cover_section(sk, hw_e, hh_e)
    lf = comp.features.loftFeatures
    li = lf.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    for i in range(5):
        li.loftSections.add(profiles[i])
    li.isSolid = True
    loft = lf.add(li)
    if _err(loft):
        raise RuntimeError("Engine cover loft: %s" % loft.errorOrWarningMessage)
    body = loft.bodies.item(0); body.name = "EngineCover"
    return {"component": comp, "body": body, "rear_plane": mono["rear_plane"],
            "tail_top_z": "airbox_cover_max_height*0.22/2", "rear_station_x": tail_x_expr}


# ===========================================================================
# 6b. LIVE WING INCIDENCE  (parametric Move>Rotate per element)
# ===========================================================================
# Master toggle. When True, each wing element is rotated about its spanwise
# (Y) axis by a parameter-driven angle, so AoA / DRS are LIVE drags. When False,
# wings stay flat (the angle params are ignored) -- a safe fallback if the
# parametric rotation misbehaves on a given build.
LIVE_ANGLES = True

# Running tally of elements whose incidence rotation could NOT be applied
# (so the run report can surface a flat-wing situation honestly). Cleared at
# the start of every run().
_INCIDENCE_SKIPS = []


def _make_y_axis_line(comp, x_cm, z_cm, tag):
    """Return a SketchLine parallel to world +Y, passing through (x_cm, *, z_cm).

    Built as a construction line on an XY-offset plane (offset = z_cm). Both the
    plane (setByOffset of a real plane) and the sketch line are entity-based, so
    this works in PARAMETRIC mode -- unlike a construction axis from a raw
    InfiniteLine3D, which Fusion rejects with "Environment is not supported".
    """
    planes = comp.constructionPlanes
    pin = planes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(z_cm))
    pl = planes.add(pin); pl.name = "AoA_plane_" + tag
    sk = comp.sketches.add(pl); sk.name = "AoA_axis_" + tag
    P = adsk.core.Point3D.create
    # On an XY-offset plane, sketch (u,v) -> world (u, v, z_cm). A line at u=x_cm
    # spanning v is therefore parallel to world Y at (x_cm, *, z_cm).
    ln = sk.sketchCurves.sketchLines.addByTwoPoints(P(x_cm, -50.0, 0), P(x_cm, 50.0, 0))
    try:
        ln.isConstruction = True
    except Exception:
        pass
    return ln


def _apply_incidence(comp, occ, body, qc_x, qc_z, angle_expr, axis_name):
    """Rotate `body` about a spanwise (+Y) axis through (qc_x, *, qc_z) by
    `angle_expr` -> LIVE angle of attack.

    Axis is a SketchLine (parametric-safe). The wing lives in a sub-component
    placed via occurrence `occ`, so the Move feature runs in the ASSEMBLY
    context: the body and axis must be converted with createForAssemblyContext
    or defineAsRotate throws "Invalid entity" (confirmed: Autodesk forum). Uses
    the current Move API (createInput2 + defineAsRotate; old createInput is
    retired). Guarded: on any failure the element is left flat and the skip is
    recorded rather than aborting the wing. Sign follows the right-hand rule
    about +Y; if a wing tilts the wrong way, negate the angle parameter.
    """
    if not LIVE_ANGLES:
        return False
    try:
        axis_line = _make_y_axis_line(comp, qc_x, qc_z, axis_name)
        # Convert native entities into the occurrence's assembly context so the
        # Move feature accepts them (the "Invalid entity" fix).
        try:
            axis_ctx = axis_line.createForAssemblyContext(occ)
            body_ctx = body.createForAssemblyContext(occ)
        except Exception:
            # Fallback: if proxies aren't needed/available, use natives.
            axis_ctx, body_ctx = axis_line, body
        ents = adsk.core.ObjectCollection.create(); ents.add(body_ctx)
        mf = comp.features.moveFeatures
        mi = mf.createInput2(ents)              # confirmed: createInput is retired -> createInput2
        ok = mi.defineAsRotate(axis_ctx, adsk.core.ValueInput.createByString(angle_expr))  # confirmed
        if ok is False:
            raise RuntimeError("defineAsRotate returned False")
        feat = mf.add(mi)
        if _err(feat):
            raise RuntimeError(feat.errorOrWarningMessage)
        return True
    except Exception as exc:
        _INCIDENCE_SKIPS.append("%s (%s): %s" % (axis_name, angle_expr, exc))
        return False


# ===========================================================================
# 7. REAR WING  (5 bodies, zero booleans here)
# ===========================================================================
def _foil_fractions(params, chord_param_name):
    chord_cm = _get_param_value(params, chord_param_name, 28.0)
    return (_get_param_value(params, "rw_thickness", 0.9) / chord_cm,
            _get_param_value(params, "rw_camber", 0.5) / chord_cm)


def create_rear_wing(root_comp, params, design, mono, engine_cover):
    occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component; comp.name = "RearWing"
    P = adsk.core.Point3D.create
    rear_station_cm = _get_param_value(params, "tub_length", 200.0) + _get_param_value(params, "nose_length", 60.0) - 10.0
    tail_top_cm = _get_param_value(params, "airbox_cover_max_height", 35.0) * 0.15 / 2.0
    x_wing = rear_station_cm + _get_param_value(params, "rw_overhang", 15.0)
    z_wing = tail_top_cm + _get_param_value(params, "rw_height", 35.0)
    cr = _get_param_value(params, "rw_chord_root", 28.0); ct = _get_param_value(params, "rw_chord_tip", 25.0)
    t_r, m_r = _foil_fractions(params, "rw_chord_root"); t_t, m_t = _foil_fractions(params, "rw_chord_tip")

    sk_r = comp.sketches.add(_build_span_plane(comp, "0 mm", "RW_root_plane")); sk_r.name = "RW_mp_root"
    sk_t = comp.sketches.add(_build_span_plane(comp, "rw_span/2", "RW_tip_plane")); sk_t.name = "RW_mp_tip"
    root_af = airfoil_section(sk_r, cr, t_r, m_r, P(x_wing, z_wing, 0))
    tip_af  = airfoil_section(sk_t, ct, t_t, m_t, P(x_wing, z_wing, 0))
    lf = comp.features.loftFeatures
    li = lf.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    li.loftSections.add(root_af["profile"]); li.loftSections.add(tip_af["profile"]); li.isSolid = True
    mp = lf.add(li)
    if _err(mp):
        raise RuntimeError("RW mainplane: %s" % mp.errorOrWarningMessage)
    mp_body = mp.bodies.item(0); mp_body.name = "RW_mainplane"
    # LIVE AoA: tilt the mainplane about its quarter-chord spanwise (Y) axis.
    _apply_incidence(comp, occ, mp_body, x_wing + 0.25 * cr, z_wing,
                     "rw_mainplane_angle", "RW_mp_AoA")

    gx = _get_param_value(params, "drs_slot_gap_x", 2.0); gz = _get_param_value(params, "drs_slot_gap_z", 3.5)
    fr_o = P(root_af["te_point"].x + gx, root_af["te_point"].y + gz, 0)
    ft_o = P(tip_af["te_point"].x + gx, tip_af["te_point"].y + gz, 0)
    sk_fr = comp.sketches.add(_build_span_plane(comp, "0 mm", "RW_flap_root_plane")); sk_fr.name = "RW_flap_root"
    sk_ft = comp.sketches.add(_build_span_plane(comp, "rw_span/2", "RW_flap_tip_plane")); sk_ft.name = "RW_flap_tip"
    fr = airfoil_section(sk_fr, cr * 0.55, t_r, m_r, fr_o); ft = airfoil_section(sk_ft, ct * 0.55, t_t, m_t, ft_o)
    li2 = lf.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    li2.loftSections.add(fr["profile"]); li2.loftSections.add(ft["profile"]); li2.isSolid = True
    fl = lf.add(li2)
    if _err(fl):
        raise RuntimeError("RW flap: %s" % fl.errorOrWarningMessage)
    fl_body = fl.bodies.item(0); fl_body.name = "RW_flap"
    # LIVE DRS: tilt the flap about its own quarter-chord spanwise axis.
    _apply_incidence(comp, occ, fl_body, fr_o.x + 0.25 * cr * 0.55, fr_o.y,
                     "drs_flap_angle", "RW_flap_AoA")

    # FULL SPAN: the loft above only spans 0..+span/2. Mirror the mainplane +
    # flap across the car centreline (XZ) so the wing fills -span/2..+span/2 (the
    # endplates already sit at both tips). Done AFTER incidence so the mirrored
    # half inherits the same tilt -- a Y-reflection preserves the X-Z chord angle.
    wing_mt = adsk.core.ObjectCollection.create()
    wing_mt.add(mp_body); wing_mt.add(fl_body)
    mwf = comp.features.mirrorFeatures.add(
        comp.features.mirrorFeatures.createInput(wing_mt, comp.xZConstructionPlane))
    if _err(mwf):
        raise RuntimeError("RW full-span mirror: %s" % mwf.errorOrWarningMessage)

    sk_ep = comp.sketches.add(_build_span_plane(comp, "rw_span/2", "RW_endplate_plane")); sk_ep.name = "RW_endplate"
    ep_prof = _draw_rounded_rect_profile(sk_ep, "rw_endplate_length/2", "rw_endplate_height/2", 4.0)
    ext = comp.features.extrudeFeatures.createInput(ep_prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext.setDistanceExtent(False, adsk.core.ValueInput.createByString("endplate_thickness"))  # VERIFY
    ep_L = comp.features.extrudeFeatures.add(ext).bodies.item(0); ep_L.name = "RW_endplate_L"
    mt = adsk.core.ObjectCollection.create(); mt.add(ep_L)
    comp.features.mirrorFeatures.add(comp.features.mirrorFeatures.createInput(mt, comp.xZConstructionPlane))  # VERIFY
    ep_R = comp.bRepBodies.item(comp.bRepBodies.count - 1); ep_R.name = "RW_endplate_R"
    return {"component": comp, "mainplane_body": mp_body, "flap_body": fl_body,
            "endplate_bodies": [ep_L, ep_R], "strut_body": None,
            "tip_plane": comp.constructionPlanes.itemByName("RW_tip_plane"), "span_y": "rw_span/2"}


# ===========================================================================
# 8. FRONT WING  (constant-chord extrude + flap cascade + curl endplate)
# ===========================================================================
def _build_fw_element(comp, params, sketch_plane, chord_cm, origin):
    sk = comp.sketches.add(sketch_plane); sk.name = "FW_elem_%d" % int(round(chord_cm * 10))
    t_frac = _get_param_value(params, "fw_thickness", 0.10); m_frac = _get_param_value(params, "fw_camber", 0.03)
    af = airfoil_section(sk, chord_cm, t_frac, m_frac, origin)
    ext = comp.features.extrudeFeatures.createInput(af["profile"], adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext.setDistanceExtent(False, adsk.core.ValueInput.createByString("fw_span/2"))  # VERIFY
    ext_feat = comp.features.extrudeFeatures.add(ext)
    if _err(ext_feat):
        raise RuntimeError("FW element: %s" % ext_feat.errorOrWarningMessage)
    return ext_feat, af


def create_front_wing(root_comp, params, design, mono, airfoil_section_fn):
    occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component; comp.name = "FrontWing"
    P = adsk.core.Point3D.create
    centre_plane = _build_span_plane(comp, "0 mm", "FW_centre_plane")
    x_off = _get_param_value(params, "fw_x_offset", -20.0); z_off = _get_param_value(params, "fw_height_off_ground", 7.5)
    chord_main = _get_param_value(params, "fw_chord", 33.0); chord_flap = _get_param_value(params, "fw_flap_chord", 22.0)
    gap_x = _get_param_value(params, "fw_slot_gap_x", 2.0); gap_z = _get_param_value(params, "fw_slot_gap_z", 1.5)

    main_ext, main_af = _build_fw_element(comp, params, centre_plane, chord_main, P(x_off, z_off, 0))
    main_body = main_ext.bodies.item(0); main_body.name = "FW_mainplane"
    # LIVE AoA: tilt the main plane about its quarter-chord spanwise (Y) axis.
    # Done BEFORE the wing mirror so the mirrored (right) side inherits the tilt.
    _apply_incidence(comp, occ, main_body,
                     x_off + 0.25 * chord_main, z_off, "fw_mainplane_angle", "FW_mp_AoA")
    flap_features = []; flap_bodies = []; prev_te = main_af["te_point"]
    for i in range(1, 4):
        flap_le_x = prev_te.x + gap_x
        flap_le_z = prev_te.y + gap_z   # te_point.y carries the vertical (world Z) value
        ext_i, af_i = _build_fw_element(comp, params, centre_plane, chord_flap, P(flap_le_x, flap_le_z, 0))
        fb = ext_i.bodies.item(0); fb.name = "FW_flap_%d" % i
        # Cumulative AoA: main plane angle + i flap steps (positions stay frozen).
        _apply_incidence(comp, occ, fb,
                         flap_le_x + 0.25 * chord_flap, flap_le_z,
                         "fw_mainplane_angle + %d*fw_flap_angle_step" % i, "FW_flap%d_AoA" % i)
        flap_features.append((ext_i, af_i)); flap_bodies.append(fb); prev_te = af_i["te_point"]

    plane_outer = _build_span_plane(comp, "fw_span/2", "FW_ep_outer_plane")
    plane_inner = _build_span_plane(comp, "fw_span/2 - fw_endplate_curl", "FW_ep_inner_plane")
    sko = comp.sketches.add(plane_outer); sko.name = "FW_ep_outer"
    ski = comp.sketches.add(plane_inner); ski.name = "FW_ep_inner"
    op = _draw_rounded_rect_profile(sko, "fw_endplate_length/2", "fw_endplate_height/2", 4.0)
    ip = _draw_rounded_rect_profile(ski, "fw_endplate_length/2", "fw_endplate_height/2", 4.0)
    lf = comp.features.loftFeatures
    li = lf.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    li.loftSections.add(op); li.loftSections.add(ip); li.isSolid = True
    epl = lf.add(li)
    if _err(epl):
        raise RuntimeError("FW endplate: %s" % epl.errorOrWarningMessage)
    ep_L = epl.bodies.item(0); ep_L.name = "FW_endplate_L"
    et = adsk.core.ObjectCollection.create(); et.add(ep_L)
    comp.features.mirrorFeatures.add(comp.features.mirrorFeatures.createInput(et, comp.xZConstructionPlane))  # VERIFY
    ep_R = comp.bRepBodies.item(comp.bRepBodies.count - 1); ep_R.name = "FW_endplate_R"

    wing_left = [main_body] + flap_bodies
    mt = adsk.core.ObjectCollection.create()
    for b in wing_left:
        mt.add(b)
    mwf = comp.features.mirrorFeatures.add(comp.features.mirrorFeatures.createInput(mt, comp.xZConstructionPlane))
    if _err(mwf):
        raise RuntimeError("FW full-span mirror: %s" % mwf.errorOrWarningMessage)
    return {"component": comp, "mainplane_body": main_body,
            "flap_bodies": flap_bodies,
            "endplate_bodies": [ep_L, ep_R], "pylon_bodies": [],
            "span_y": "fw_span/2", "tip_plane": comp.constructionPlanes.itemByName("FW_ep_outer_plane")}


# ===========================================================================
# 9. WHEELS  (one master wheel, 4 occurrences -- no body-copy, no zero-move)
# ===========================================================================
def _build_one_wheel(comp):
    """Revolve a single tyre+rim cross-section in `comp`. Cross-section dims are
    LIVE; the four placements (occurrences) are frozen build-time positions."""
    P = adsk.core.Point3D.create
    sk = comp.sketches.add(comp.xYConstructionPlane); sk.name = "Wheel_section"
    geo = sk.geometricConstraints; dims = sk.sketchDimensions
    lines = sk.sketchCurves.sketchLines
    r_rim, r_out, half_w = 9.0, 14.0, 6.0   # cm guesses; dims drive real size
    l_left  = lines.addByTwoPoints(P(r_rim, -half_w, 0), P(r_rim,  half_w, 0))
    l_top   = lines.addByTwoPoints(l_left.endSketchPoint, P(r_out,  half_w, 0))
    l_right = lines.addByTwoPoints(l_top.endSketchPoint,  P(r_out, -half_w, 0))
    l_bot   = lines.addByTwoPoints(l_right.endSketchPoint, l_left.startSketchPoint)
    geo.addVertical(l_left); geo.addVertical(l_right); geo.addHorizontal(l_top); geo.addHorizontal(l_bot)
    HORIZ = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
    VERT  = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
    txt = P(r_out + 3, 0, 0)
    # addDistanceDimension takes POINTS (confirmed against docs). Use the lines'
    # sketch endpoints; HORIZ/VERT orientation isolates the axis component.
    #   l_left.startSketchPoint  = (r_rim, -half_w) -> horiz dist = rim radius
    #   l_right.startSketchPoint = (r_out, +half_w) -> horiz dist = tyre radius
    #   l_top.startSketchPoint   y=+half_w ; l_bot.startSketchPoint y=-half_w -> vert = width
    d_rim = dims.addDistanceDimension(sk.originPoint, l_left.startSketchPoint, HORIZ, txt)
    d_out = dims.addDistanceDimension(sk.originPoint, l_right.startSketchPoint, HORIZ, txt)
    d_wid = dims.addDistanceDimension(l_top.startSketchPoint, l_bot.startSketchPoint, VERT, txt)
    d_rim.parameter.expression = "wheel_rim_diameter/2"
    d_out.parameter.expression = "tyre_outer_diameter/2"
    d_wid.parameter.expression = "tyre_width"
    # Revolve axis as a SKETCH construction LINE along Y at X=0 (parametric-safe).
    # A construction axis from a raw InfiniteLine3D is rejected in parametric mode
    # ("Environment is not supported"); a sketch line is an entity, so it works.
    # The profile sits at X=r_rim..r_out (positive X), so it never crosses this
    # axis. Construction => it adds no profile region. Added BEFORE capturing the
    # profile so adding a curve can't invalidate the profile reference.
    axis_line = lines.addByTwoPoints(P(0, -half_w * 3.0, 0), P(0, half_w * 3.0, 0))
    try:
        axis_line.isConstruction = True
    except Exception:
        pass
    profile = sk.profiles.item(0)
    revolves = comp.features.revolveFeatures
    ri = revolves.createInput(profile, axis_line, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByString("360 deg"))  # VERIFY
    wheel = revolves.add(ri)
    if _err(wheel):
        raise RuntimeError("Wheel revolve: %s" % wheel.errorOrWarningMessage)
    body = wheel.bodies.item(0); body.name = "Wheel"
    return body


def create_wheels(root_comp, params, design, mono):
    parent_occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    parent = parent_occ.component; parent.name = "Wheels"

    wb  = _get_param_value(params, "wheelbase", 360.0)
    ft  = _get_param_value(params, "front_track", 170.0)
    rt  = _get_param_value(params, "rear_track", 155.0)
    fax = _get_param_value(params, "front_axle_x", 30.0)
    z   = _get_param_value(params, "tyre_outer_diameter", 72.0) / 2.0  # axle height = tyre radius
    rear_axle_x = fax + wb

    # The stylised wheel section is symmetric in Y, so NO mirror is needed --
    # all four are pure translations (this also sidesteps the zero-transform
    # MoveFeatures bug, since occurrences don't use MoveFeatures).
    stations = [
        (fax,         +ft / 2.0, "Wheel_FR"),
        (fax,         -ft / 2.0, "Wheel_FL"),
        (rear_axle_x, +rt / 2.0, "Wheel_RR"),
        (rear_axle_x, -rt / 2.0, "Wheel_RL"),
    ]

    def _xform(x_cm, y_cm):
        m = adsk.core.Matrix3D.create()
        m.translation = adsk.core.Vector3D.create(x_cm, y_cm, z)
        return m

    # Build the master wheel ONCE, placed DIRECTLY at the first station (not at
    # the origin -- placing it at identity would leave a stray 5th wheel at the
    # centreline). The other three are occurrences of the same master, so all
    # four share one editable wheel section.
    x0, y0, n0 = stations[0]
    master_occ = parent.occurrences.addNewComponent(_xform(x0, y0))  # confirmed: addNewComponent(Matrix3D)
    master = master_occ.component; master.name = "Wheel_master"
    _build_one_wheel(master)
    try:
        master_occ.component.name = n0
    except Exception:
        pass

    wheel_occs = [master_occ]
    for x_cm, y_cm, name in stations[1:]:
        oc = parent.occurrences.addExistingComponent(master, _xform(x_cm, y_cm))  # confirmed
        try:
            oc.component.name = name
        except Exception:
            pass
        wheel_occs.append(oc)

    wheel_bodies = []
    for oc in wheel_occs:
        try:
            wheel_bodies.append(oc.bRepBodies.item(0))
        except Exception:
            pass
    return {"component": parent, "wheel_bodies": wheel_bodies,
            "wheel_master": master, "wheel_outer_d": "tyre_outer_diameter"}


# ===========================================================================
# 10. SIDEPODS  (one loft + mirror; undercut baked into the profile)
# ===========================================================================
def _draw_sidepod_section(sketch, width_expr, height_expr, curl_depth_cm, is_intake):
    if is_intake:
        return _draw_eight_curve_box(sketch, width_expr, height_expr, 2.0, 2.0)
    # Heavy bottom fillet = the undercut (coke-bottle pinch); same 8-curve topology.
    return _draw_eight_curve_box(sketch, width_expr, height_expr, 2.0, curl_depth_cm)


def create_sidepods(root_comp, params, design, mono, engine_cover):
    occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component; comp.name = "Sidepods"
    planes = comp.constructionPlanes; sketches = comp.sketches; VI = adsk.core.ValueInput
    front = "sidepod_front_x"

    def offset_plane(x_expr, name):
        pin = planes.createInput()
        pin.setByOffset(comp.yZConstructionPlane, VI.createByString(x_expr))
        pl = planes.add(pin); pl.name = name; return pl

    # Spread sidepod sections across the full front->rear-exit span (fractions),
    # same reasoning as the engine cover: keeps the loft well-conditioned when
    # the body is long instead of leaving a big gap before the rear-exit section.
    rear_exit_expr = "nose_length + tub_length - 200 mm"
    sp_span = "((%s) - (%s))" % (rear_exit_expr, front)
    plane = [offset_plane(front, "SP_S0"),
             offset_plane("(%s) + %s*0.35" % (front, sp_span), "SP_S1"),
             offset_plane("(%s) + %s*0.70" % (front, sp_span), "SP_S2"),
             offset_plane(rear_exit_expr, "SP_S3")]
    curl = _get_param_value(params, "sidepod_curl_depth", 3.0)
    specs = [
        (0, "intake_width/2",            "intake_height/2",           True),
        (1, "sidepod_max_width*0.85/2",  "sidepod_max_height*0.90/2", False),
        (2, "sidepod_max_width/2",       "sidepod_max_height/2",      False),
        (3, "sidepod_max_width*0.40/2",  "sidepod_max_height*0.45/2", False),
    ]
    profiles = []
    for idx, w_e, h_e, is_in in specs:
        sk = sketches.add(plane[idx]); sk.name = "SP_S%d" % idx
        profiles.append(_draw_sidepod_section(sk, w_e, h_e, curl, is_in))
    lf = comp.features.loftFeatures
    li = lf.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    for pr in profiles:
        li.loftSections.add(pr)
    li.isSolid = True
    loft = lf.add(li)
    if _err(loft):
        raise RuntimeError("Sidepod loft: %s" % loft.errorOrWarningMessage)
    pod_R = loft.bodies.item(0); pod_R.name = "Sidepod_R"
    mt = adsk.core.ObjectCollection.create(); mt.add(pod_R)
    mf = comp.features.mirrorFeatures.add(comp.features.mirrorFeatures.createInput(mt, comp.xZConstructionPlane))  # VERIFY
    if _err(mf):
        raise RuntimeError("Sidepod mirror: %s" % mf.errorOrWarningMessage)
    pod_L = comp.bRepBodies.item(comp.bRepBodies.count - 1); pod_L.name = "Sidepod_L"
    return {"component": comp, "pod_bodies": [pod_R, pod_L],
            "front_x": "sidepod_front_x", "rear_x": rear_exit_expr}


# ===========================================================================
# 11. FINALIZE  (timeline health sweep + documented join policy + optional export)
# ===========================================================================
def finalize(root_comp, design, all_dicts):
    """Sweep the timeline for unhealthy features and report them. Joins nothing
    by default -- a multi-body design (each component its own body) is the
    correct parametric end state. Optional STEP export is shown, commented."""
    app = adsk.core.Application.get()
    ui = app.userInterface
    problems = []
    try:
        HS = adsk.fusion.FeatureHealthStates
        state_name = {
            getattr(HS, "WarningFeatureHealthState", -1): "WARNING",
            getattr(HS, "ErrorFeatureHealthState", -2): "ERROR",
            getattr(HS, "SuppressedFeatureHealthState", -3): "suppressed",
            getattr(HS, "RolledBackFeatureHealthState", -4): "rolled-back",
        }
        timeline = design.timeline
        for i in range(timeline.count):
            try:
                entity = timeline.item(i).entity
                health = getattr(entity, "healthState", None)
                if (health is not None and
                        health != HS.HealthyFeatureHealthState):
                    nm = getattr(entity, "name", "timeline[%d]" % i)
                    # Owning component so a bare "Loft1" becomes "RearWing/Loft1".
                    try:
                        pc = getattr(entity, "parentComponent", None)
                        if pc is not None and pc.name:
                            nm = pc.name + "/" + nm
                    except Exception:
                        pass
                    tag = state_name.get(health, "unhealthy")
                    msg = (getattr(entity, "errorOrWarningMessage", "") or "")
                    msg = " ".join(msg.split())          # collapse whitespace/newlines
                    if len(msg) > 140:
                        msg = msg[:140] + "..."
                    problems.append("[%s] %s: %s" % (tag, nm, msg))
            except Exception:
                continue
    except Exception:
        pass
    if problems:
        ui.messageBox("Health sweep found %d issue(s):\n- %s" % (len(problems), "\n- ".join(problems)))

    # JOIN POLICY (documented, NOT executed):
    #   Permanently separate: wheels, DRS flap, halo, wing elements, endplates.
    #   Optional export-only join (on COPIES): monocoque + engine cover + sidepods.
    # STEP export (uncomment + set a real path; STEP needs no watertightness):
    #   em = design.exportManager
    #   opts = em.createSTEPExportOptions("/path/to/f1_car.step")  # VERIFY method name
    #   em.execute(opts)
    return problems


# ===========================================================================
# 12. RUN  (loud, dependency-ordered, per-component error isolation)
# ===========================================================================
# ===========================================================================
# 9. FLOOR  (flat underbody plate -- the biggest single 'it's an F1 car' cue)
# ===========================================================================
def create_floor(root_comp, params, design):
    """Flat underbody floor: a thin plate from just behind the front wheels to
    just past the rear axle, low to the ground. Independent component (reads
    only parameter VALUES, no geometry from other parts). Positions are read at
    build time; the whole car rebuilds each run, so they always track the params.
    """
    occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component; comp.name = "Floor"
    VI = adsk.core.ValueInput; P = adsk.core.Point3D.create
    g = lambda k, d: _get_param_value(params, k, d)
    fax = g("front_axle_x", 95.0); wb = g("wheelbase", 360.0)
    front_x = fax + 22.0                 # just behind the front wheels
    rear_x = fax + wb + 6.0              # to just past the rear axle
    half_w = 52.0                        # ~1040 mm wide
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, VI.createByString("40 mm"))  # ride height
    pl = comp.constructionPlanes.add(pin); pl.name = "Floor_plane"
    sk = comp.sketches.add(pl); sk.name = "Floor_sketch"
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        P(front_x, -half_w, 0), P(rear_x, half_w, 0))
    prof = sk.profiles.item(0)
    ext = comp.features.extrudeFeatures
    ei = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ei.setDistanceExtent(False, VI.createByString("25 mm"))  # plate thickness
    feat = ext.add(ei)
    if _err(feat):
        raise RuntimeError("Floor extrude: %s" % feat.errorOrWarningMessage)
    body = feat.bodies.item(0); body.name = "Floor"
    return {"component": comp, "body": body}


# ===========================================================================
# 10. SUSPENSION  (double-wishbone arms linking each wheel to the body)
# ===========================================================================
def _susp_arm(comp, z_cm, x1, y1, x2, y2, hw, thick_expr, name):
    """One thin, flat suspension arm lying in a horizontal plane at height
    z_cm, running from an inboard point (x1,y1) to an outboard point (x2,y2),
    extruded `thick_expr` upward. A planar sketch + extrude -> robust in
    parametric mode (no 3D-sketch / construction-axis pitfalls)."""
    import math
    VI = adsk.core.ValueInput; P = adsk.core.Point3D.create
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, VI.createByString("%.4f cm" % z_cm))
    pl = comp.constructionPlanes.add(pin); pl.name = name + "_pl"
    sk = comp.sketches.add(pl); sk.name = name + "_sk"
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy) or 1.0
    px, py = -dy / L * hw, dx / L * hw     # in-plane perpendicular * half-width
    a = P(x1 + px, y1 + py, 0); b = P(x2 + px, y2 + py, 0)
    c = P(x2 - px, y2 - py, 0); d = P(x1 - px, y1 - py, 0)
    lines = sk.sketchCurves.sketchLines
    lines.addByTwoPoints(a, b); lines.addByTwoPoints(b, c)
    lines.addByTwoPoints(c, d); lines.addByTwoPoints(d, a)
    prof = sk.profiles.item(0)
    ext = comp.features.extrudeFeatures
    ei = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ei.setDistanceExtent(False, VI.createByString(thick_expr))
    feat = ext.add(ei)
    if _err(feat):
        raise RuntimeError("arm %s: %s" % (name, feat.errorOrWarningMessage))
    return feat.bodies.item(0)


def _susp_strut(comp, x_cm, ya, za, yb, zb, hw, thick_cm, name):
    """One thin strut (a pushrod) lying in a TRANSVERSE plane at X = x_cm,
    running from (ya, za) to (yb, zb) in Y-Z (a=Y, b=Z, the same convention the
    monocoque cross-sections use), extruded SYMMETRICALLY across X so it is
    centred on the plane -> no +X/-X direction ambiguity. Planar sketch +
    extrude, so it stays robust in parametric mode."""
    import math
    VI = adsk.core.ValueInput; P = adsk.core.Point3D.create
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.yZConstructionPlane, VI.createByString("%.4f cm" % x_cm))
    pl = comp.constructionPlanes.add(pin); pl.name = name + "_pl"
    sk = comp.sketches.add(pl); sk.name = name + "_sk"
    dy, dz = yb - ya, zb - za
    L = math.hypot(dy, dz) or 1.0
    py, pz = -dz / L * hw, dy / L * hw          # in-plane perpendicular * half-width
    a = P(ya + py, za + pz, 0); b = P(yb + py, zb + pz, 0)
    c = P(yb - py, zb - pz, 0); d = P(ya - py, za - pz, 0)
    lines = sk.sketchCurves.sketchLines
    lines.addByTwoPoints(a, b); lines.addByTwoPoints(b, c)
    lines.addByTwoPoints(c, d); lines.addByTwoPoints(d, a)
    prof = sk.profiles.item(0)
    ext = comp.features.extrudeFeatures
    ei = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ei.setDistanceExtent(True, VI.createByString("%.4f cm" % (thick_cm / 2.0)))   # symmetric -> centred
    feat = ext.add(ei)
    if _err(feat):
        raise RuntimeError("strut %s: %s" % (name, feat.errorOrWarningMessage))
    return feat.bodies.item(0)


def create_suspension(root_comp, params, design):
    """Double-wishbone suspension: at each of the four corners, an upper and a
    lower wishbone (each a fore + aft arm) plus a pushrod reach from the body out
    to the wheel -- the open-wheel 'arms to the tyres' look. Per-element guarded
    so one bad element can't lose the whole set. Independent component."""
    occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component; comp.name = "Suspension"
    g = lambda k, d: _get_param_value(params, k, d)
    fax = g("front_axle_x", 95.0); wb = g("wheelbase", 360.0)
    ft = g("front_track", 170.0); rt = g("rear_track", 155.0)
    hub_z = g("tyre_outer_diameter", 72.0) / 2.0
    tw = g("tyre_width", 30.5)
    body_half = g("tub_max_width", 60.0) / 2.0 * 0.85     # inboard attach near the body side
    corners = [(fax, +ft / 2.0), (fax, -ft / 2.0),
               (fax + wb, +rt / 2.0), (fax + wb, -rt / 2.0)]
    spread = 16.0          # fore/aft separation of the inboard pivots (cm)
    up_z = hub_z + 6.0     # upper wishbone height
    lo_z = hub_z - 6.0     # lower wishbone height
    hw = 1.6               # arm half-width (cm) -> ~32 mm arms
    thick = "30 mm"
    bodies = []
    skips = []
    n = 0
    for (ax, yo) in corners:
        s = 1.0 if yo > 0 else -1.0
        y_in = s * body_half                       # inboard (body side)
        y_out = yo - s * (tw / 2.0 + 1.0)          # outboard, just inside the wheel inner face
        plan = [(up_z, ax - spread, "UWB_f"), (up_z, ax + spread, "UWB_a"),
                (lo_z, ax - spread, "LWB_f"), (lo_z, ax + spread, "LWB_a")]
        for (z, x_in, tag) in plan:
            nm = "%s%d" % (tag, n)
            try:
                bodies.append(_susp_arm(comp, z, x_in, y_in, ax, y_out, hw, thick, nm))
            except Exception as exc:
                skips.append("%s: %s" % (nm, exc))
        # pushrod: from the lower-outboard corner (near the wheel, low) up to an
        # inboard-high rocker point on the chassis -> the diagonal F1 strut.
        prnm = "PR%d" % n
        try:
            bodies.append(_susp_strut(comp, ax, y_out, lo_z, y_in, up_z + 8.0, 1.2, 2.4, prnm))
        except Exception as exc:
            skips.append("%s: %s" % (prnm, exc))
        n += 1
    if not bodies:
        raise RuntimeError("Suspension: no arms built (%s)" % "; ".join(skips))
    return {"component": comp, "bodies": bodies, "skipped": skips}


# ===========================================================================
# 11. DIFFUSER  (rear underbody ramp -- rises toward the back, F1 signature)
# ===========================================================================
def create_diffuser(root_comp, params, design):
    """Rear diffuser: a ramped plate under the back of the car that climbs from
    floor level up toward the rear exit. Side profile sketched in X-Z on the
    centre (XZ) plane, then extruded symmetrically across the width -> centred,
    no left/right direction ambiguity. Independent, guarded component."""
    occ = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component; comp.name = "Diffuser"
    VI = adsk.core.ValueInput; P = adsk.core.Point3D.create
    g = lambda k, d: _get_param_value(params, k, d)
    rax = g("front_axle_x", 95.0) + g("wheelbase", 360.0)   # rear axle X
    x0 = rax - 35.0       # ramp starts ~35 cm ahead of the rear axle
    x1 = rax + 45.0       # exits ~45 cm behind it
    z0 = 4.0              # floor underside / ride height
    rise = 16.0           # how high the underside climbs to the exit
    pth = 3.0             # plate thickness
    half_w = 48.0         # ~960 mm wide (symmetric extrude -> centred)
    sk = comp.sketches.add(comp.xZConstructionPlane); sk.name = "Diff_sketch"
    L = sk.sketchCurves.sketchLines
    a = P(x0, z0, 0); b = P(x1, z0 + rise, 0)
    c = P(x1, z0 + rise + pth, 0); d = P(x0, z0 + pth, 0)   # (a=X, b=Z)
    L.addByTwoPoints(a, b); L.addByTwoPoints(b, c)
    L.addByTwoPoints(c, d); L.addByTwoPoints(d, a)
    prof = sk.profiles.item(0)
    ext = comp.features.extrudeFeatures
    ei = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ei.setDistanceExtent(True, VI.createByString("%.4f cm" % half_w))   # symmetric -> full width centred
    feat = ext.add(ei)
    if _err(feat):
        raise RuntimeError("Diffuser extrude: %s" % feat.errorOrWarningMessage)
    body = feat.bodies.item(0); body.name = "Diffuser"
    return {"component": comp, "body": body}


def _clear_previous(root):
    """Delete all top-level occurrences so each run rebuilds ONE clean car
    instead of stacking duplicates (EngineCover, EngineCover (1), ...).

    Every component this script makes is a top-level occurrence of the root
    (create_* all call root.occurrences.addNewComponent), and the wheel instances
    are nested under the Wheels occurrence -- so deleting the top-level
    occurrences clears the whole car. User parameters live on the design, not a
    component, so they survive and are refreshed idempotently by
    create_parameters (the count stays stable, e.g. 58, instead of growing).

    A snapshot list is taken first because deleteMe() mutates the collection;
    holding object references lets us delete each safely. Fully guarded: a delete
    that fails is skipped rather than aborting the rebuild.
    """
    removed = 0
    try:
        occ_list = [root.occurrences.item(i) for i in range(root.occurrences.count)]
    except Exception:
        occ_list = []
    for occ in occ_list:
        try:
            occ.deleteMe(); removed += 1
        except Exception:
            pass
    # Defensive: clear any stray root-level geometry left by an older/aborted run
    # (normally none -- all geometry lives in sub-components).
    for coll_name in ("bRepBodies", "sketches", "constructionPlanes", "constructionAxes"):
        try:
            coll = getattr(root, coll_name)
            for item in [coll.item(i) for i in range(coll.count)]:
                try:
                    item.deleteMe()
                except Exception:
                    pass
        except Exception:
            pass
    return removed


def run(context):
    # ISOLATION: build one component at a time for first-run debugging.
    #   "all" | "monocoque" | "halo" | "engine_cover" | "rear_wing" |
    #   "front_wing" | "wheels" | "sidepods"
    BUILD_ONLY = "all"

    # If True, an independent component's failure is logged and the build
    # continues (so one bad wing doesn't lose the whole car). Dependencies
    # (monocoque, and halo->cover->rear_wing) still abort their dependents.
    CONTINUE_ON_INDEPENDENT_FAILURE = True

    # If True, wipe any previously generated car at the start of each run so a
    # re-run REPLACES the car instead of stacking a duplicate on top of it
    # (which is what produced EngineCover (1), Halo (1), ... and the doubled
    # health warnings). Turn off only if you want to keep prior runs in the doc.
    CLEAR_PREVIOUS = True

    ui = None
    progress = []
    errors = []
    del _INCIDENCE_SKIPS[:]   # reset live-angle tally (module persists across runs)
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        ui.messageBox("F1 build starting (BUILD_ONLY=%s)." % BUILD_ONLY)

        design = setup_design(app)
        if not design:
            ui.messageBox(
                "No active Fusion Design.\n\nFix: File > New Design (a fresh, empty "
                "parametric design), then Run again. Do NOT reuse a document that "
                "already has half-built geometry from a failed run.")
            return
        progress.append("design OK")

        root = design.rootComponent
        if CLEAR_PREVIOUS:
            n = _clear_previous(root)
            progress.append("cleared previous (%d component(s) removed)" % n)

        params = create_parameters(design)
        progress.append("parameters OK (%d defined)" % design.userParameters.count)

        def want(name):
            return BUILD_ONLY in ("all", name)

        # --- monocoque: hard dependency for everything ---
        mono = None
        if want("monocoque") or BUILD_ONLY != "all":
            mono = create_monocoque(root, params, design)
            progress.append("monocoque OK")

        # --- halo -> engine cover -> rear wing chain ---
        # In "all" mode each link is guarded; if a link fails, its dependents are
        # SKIPPED with a logged note (no confusing double-traceback). In isolation
        # mode the dependency builds are left UNGUARDED so a dep failure surfaces
        # as a clear fatal traceback naming exactly what broke.
        halo = ec = None
        # halo
        if want("halo") or BUILD_ONLY in ("engine_cover", "rear_wing"):
            try:
                halo = create_halo(root, params, design, mono)
                progress.append("halo OK")
            except Exception:
                errors.append("halo:\n" + traceback.format_exc())
                if not CONTINUE_ON_INDEPENDENT_FAILURE:
                    raise
        # engine cover (HARD dependency on halo)
        if want("engine_cover") or BUILD_ONLY == "rear_wing":
            if halo is None and BUILD_ONLY in ("engine_cover", "rear_wing"):
                halo = create_halo(root, params, design, mono)  # isolation dep (unguarded)
            if halo is not None:
                ec = create_engine_cover(root, params, design, mono, halo)
                progress.append("engine_cover OK")
            else:
                errors.append("engine_cover: SKIPPED (halo dependency failed)")
        # rear wing (HARD dependency on engine cover)
        if want("rear_wing"):
            if ec is None and BUILD_ONLY == "rear_wing":
                if halo is None:
                    halo = create_halo(root, params, design, mono)
                ec = create_engine_cover(root, params, design, mono, halo)  # isolation dep
            if ec is not None:
                try:
                    create_rear_wing(root, params, design, mono, ec)
                    progress.append("rear_wing OK")
                except Exception:
                    errors.append("rear_wing:\n" + traceback.format_exc())
                    if not CONTINUE_ON_INDEPENDENT_FAILURE:
                        raise
            else:
                errors.append("rear_wing: SKIPPED (engine cover dependency failed)")

        # --- independent components (only need monocoque) ---
        for name, fn in (
            ("front_wing", lambda: create_front_wing(root, params, design, mono, airfoil_section)),
            ("wheels",     lambda: create_wheels(root, params, design, mono)),
            ("sidepods",   lambda: create_sidepods(root, params, design, mono, ec)),
            ("floor",      lambda: create_floor(root, params, design)),
            ("suspension", lambda: create_suspension(root, params, design)),
            ("diffuser",   lambda: create_diffuser(root, params, design)),
        ):
            if not want(name):
                continue
            try:
                fn()
                progress.append("%s OK" % name)
            except Exception:
                errors.append("%s:\n%s" % (name, traceback.format_exc()))
                if not CONTINUE_ON_INDEPENDENT_FAILURE:
                    raise

        if BUILD_ONLY == "all":
            finalize(root, design, {})

        # Final report.
        msg = "DONE.\nCompleted:\n- " + "\n- ".join(progress)
        if LIVE_ANGLES:
            if _INCIDENCE_SKIPS:
                msg += ("\n\nLIVE ANGLES: %d element(s) could NOT be tilted (left "
                        "flat):\n- %s" % (len(_INCIDENCE_SKIPS), "\n- ".join(_INCIDENCE_SKIPS)))
            else:
                msg += "\n\nLIVE ANGLES: applied to all wing elements."
        if errors:
            msg += "\n\nNON-FATAL FAILURES (%d):\n\n%s" % (len(errors), "\n\n".join(errors))
        ui.messageBox(msg)

    except:  # noqa: E722
        report = "FAILED.\n\nReached:\n- " + ("\n- ".join(progress) if progress else "(nothing)") \
                 + "\n\nTraceback:\n" + traceback.format_exc()
        if ui:
            ui.messageBox(report)


# ---------------------------------------------------------------------------
# FIRST-RUN DEBUG ORDER (set BUILD_ONLY and run each in a FRESH design):
#   1. (params populate -- automatic at the start of any run)
#   2. "monocoque"   -> validates setup + loft pipeline
#   3. "wheels"      -> validates revolve + Y-axis (wheel must stand upright)
#   4. "sidepods"    -> validates twin loft + mirror
#   5. "halo"        -> the pipe-based hoop; if NO body, set _HALO_CLOSED_HOOP=False
#   6. "engine_cover"-> validates the cross-component (halo) expression
#   7. "front_wing"  -> validates the extrude cascade + te_point chain
#   8. "rear_wing"   -> validates multi-body + DRS slot
#   9. "all"         -> full car (independent failures are reported, not fatal)
#
# ANGLE NOTE (updated -- angles are now LIVE): with LIVE_ANGLES=True each wing
# element is rotated about its quarter-chord spanwise (Y) axis by a
# parameter-driven Move>Rotate, so rw_mainplane_angle, drs_flap_angle,
# fw_mainplane_angle and fw_flap_angle_step DRIVE the geometry -- drag them and
# the wings re-tilt. Honest caveats: (1) rotation SIGN follows the right-hand
# rule about +Y; if a wing tilts the wrong way, negate that parameter. (2) Slot
# GAP positions are computed from the un-rotated sections (positions stay frozen
# as before), so at large angles the slot geometry is approximate. (3) If a
# rotation can't be applied on a given build, that element is left flat and the
# run report lists it -- set LIVE_ANGLES=False to disable the feature entirely.
# ---------------------------------------------------------------------------
