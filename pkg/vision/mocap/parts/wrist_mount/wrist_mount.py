"""Two-part mocap mount.

base  : wristband top. Webbing runs under it (skin side), up through a slot at each end,
        over the end bar, and back down around the limb. Top has a riser + dovetail tongue.
plate : curved marker plate. Arc about Y (bends along the strap, follows the limb).
        Dovetail groove in the bottom slides onto the tongue (along Y).
        One M4 screw at (0, 0) locks plate to base (nut is captive in the base bottom).
        Grid of M4 holes normal to the arc. Each hole has a hex nut pocket from the inside,
        ending SKIN below the outer surface. A commercial M4 M-F standoff screws (male end)
        into the nut; the marker's M4 stud screws into the standoff.

Axes: X = along the strap, Y = across the strap, Z = up. All units mm.
Print base flat. Print plate on one of its flat Y-end faces (no supports).
"""
import math
import cadquery as cq

# --- webbing ---
WEB_W, WEB_T = 38.1, 2.0      # 1.5" webbing width, thickness
WEB_CLR = 1.5                 # added to webbing width and thickness in slots

# --- base ---
BASE_T = 4.0                  # main body thickness
BAR_L = 5.0                   # end bar length (X), webbing wraps over it
SIDE_WALL = 4.0               # material each side of the slot (Y)
RISE = WEB_T + 1.0            # riser height so the plate clears webbing on the end bars
EDGE_R = 3.0                  # vertical corner fillet

# --- dovetail (tongue on base, groove in plate) ---
DT_ROOT = 18.0                # tongue width at its root (X)
DT_H = 3.5                    # tongue height
DT_ANG = 30.0                 # flank angle from vertical (deg)
FIT = 0.2                     # clearance per side, groove vs tongue

# --- plate ---
PAD_WALL = 4.0                # plate material each side of the groove (X)
ARC_R = 45.0                  # inner radius of the arc
SHELL_T = 5.5                 # arc shell thickness
GROOVE_ROOF = 1.0             # gap from groove top to arc inner surface at the apex
N_COLS, COL_PITCH = 7, 12.0   # hole columns along the arc, pitch on the outer surface
N_ROWS, ROW_PITCH = 4, 12.0   # hole rows along Y
EDGE = 7.0                    # outer surface margin from last hole to plate edge
SKIN = 2.0                    # material above the nut (standoff male thread goes through it)
LOCK_CLR = 9.0                # holes closer than this to the lock screw are left out

# --- M4 hardware ---
HOLE_D = 4.4                  # M4 clearance
NUT_AF, NUT_CLR, NUT_H = 7.0, 0.4, 3.5   # nut across flats, clearance, pocket depth (base)

# --- derived ---
SLOT_W = WEB_W + WEB_CLR                      # slot size Y
SLOT_L = WEB_T + WEB_CLR                      # slot size X
BASE_W = SLOT_W + 2 * SIDE_WALL               # base size Y
DT_TOP = DT_ROOT + 2 * DT_H * math.tan(math.radians(DT_ANG))
PAD_L = DT_TOP + 2 * FIT + 2 * PAD_WALL       # riser / plate pad size X
BASE_L = PAD_L + 2 * (SLOT_L + BAR_L)         # base size X
NUT_D = 2 * (NUT_AF + NUT_CLR) / math.sqrt(3) # hex circumscribed diameter
GD = DT_H + FIT                               # groove depth
R_OUT = ARC_R + SHELL_T
ARC_HALF = ((N_COLS - 1) * COL_PITCH / 2 + EDGE) / R_OUT     # half arc angle (rad)
PLATE_W = max(BASE_W, (N_ROWS - 1) * ROW_PITCH + 2 * EDGE)  # plate size Y
ZC = GD + GROOVE_ROOF - ARC_R                 # arc center Z (plate coords, plate bottom = 0)


def prism_y(pts, length):
    """Closed XZ polygon extruded symmetric along Y."""
    return cq.Workplane("XZ").polyline(pts).close().extrude(length / 2, both=True)


def build_base():
    body = cq.Workplane("XY").rect(BASE_L, BASE_W).extrude(BASE_T).edges("|Z").fillet(EDGE_R)
    body = body.union(cq.Workplane("XY").rect(PAD_L, BASE_W).extrude(BASE_T + RISE))
    z0 = BASE_T + RISE
    tongue = [(-DT_ROOT / 2, z0), (DT_ROOT / 2, z0), (DT_TOP / 2, z0 + DT_H), (-DT_TOP / 2, z0 + DT_H)]
    body = body.union(prism_y(tongue, BASE_W))
    for sx in (-1, 1):
        x = sx * (PAD_L / 2 + SLOT_L / 2)
        body = body.cut(cq.Workplane("XY").center(x, 0).rect(SLOT_L, SLOT_W).extrude(BASE_T))
    top = z0 + DT_H
    body = body.cut(cq.Workplane("XY").circle(HOLE_D / 2).extrude(top))
    body = body.cut(cq.Workplane("XY").polygon(6, NUT_D).extrude(NUT_H))  # lock nut, from bottom
    return body


def arc_pt(r, a):
    """XZ point at radius r, angle a from +Z (positive toward +X)."""
    return (r * math.sin(a), ZC + r * math.cos(a))


def holes():
    """(angle, y) of each grid hole, lock screw area left out."""
    out = []
    for i in range(N_COLS):
        a = (i - (N_COLS - 1) / 2) * COL_PITCH / R_OUT
        for j in range(N_ROWS):
            y = (j - (N_ROWS - 1) / 2) * ROW_PITCH
            if math.hypot(a * R_OUT, y) >= LOCK_CLR:
                out.append((a, y))
    return out


def build_plate():
    assert ARC_R - math.sqrt(ARC_R**2 - (PAD_L / 2) ** 2) < SHELL_T, "hub sticks out of shell, raise ARC_R"
    assert min(COL_PITCH * (R_OUT - SKIN - NUT_H) / R_OUT, ROW_PITCH) > NUT_D + 1, "nut pockets too close"

    a = ARC_HALF
    shell = (cq.Workplane("XZ").moveTo(*arc_pt(ARC_R, -a)).lineTo(*arc_pt(R_OUT, -a))
             .threePointArc(arc_pt(R_OUT, 0), arc_pt(R_OUT, a)).lineTo(*arc_pt(ARC_R, a))
             .threePointArc(arc_pt(ARC_R, 0), arc_pt(ARC_R, -a)).close()
             .extrude(PLATE_W / 2, both=True))
    hub = cq.Workplane("XY").rect(PAD_L, PLATE_W).extrude(GD + GROOVE_ROOF)  # flat bottom on the riser
    body = shell.union(hub)

    gw0, gw1 = DT_ROOT + 2 * FIT, DT_TOP + 2 * FIT
    body = body.cut(prism_y([(-gw0 / 2, 0), (gw0 / 2, 0), (gw1 / 2, GD), (-gw1 / 2, GD)], 2 * PLATE_W))
    body = body.cut(cq.Workplane("XY").circle(HOLE_D / 2).extrude(GD + GROOVE_ROOF + SHELL_T))  # lock screw

    for ang, y in holes():
        inward = cq.Vector(-math.sin(ang), 0, -math.cos(ang))
        def plane(r):
            x, z = arc_pt(r, ang)
            # local x = Y, so one hex corner points along Y (up when printed on the Y end face)
            return cq.Plane(origin=(x, y, z), xDir=(0, 1, 0), normal=inward)
        body = body.cut(cq.Workplane(plane(R_OUT + 1)).circle(HOLE_D / 2).extrude(SHELL_T + 20))
        body = body.cut(cq.Workplane(plane(R_OUT - SKIN)).polygon(6, NUT_D).extrude(20))
    return body


def export(wp, name):
    solid = wp.val()
    cq.exporters.export(solid, f"{name}.step")
    cq.exporters.export(solid, f"{name}.stl")
    bb = solid.BoundingBox()
    print(f"{name}: valid={solid.isValid()} solids={len(solid.Solids())} "
          f"size={bb.xlen:.1f}x{bb.ylen:.1f}x{bb.zlen:.1f}")


if __name__ == "__main__":
    export(build_base(), "wrist_base")
    export(build_plate(), "plate_grid")
    print(f"holes={len(holes())} arc={math.degrees(2 * ARC_HALF):.0f} deg "
          f"arc end inner z vs skin={arc_pt(ARC_R, ARC_HALF)[1] + BASE_T + RISE + WEB_T:.1f} mm (flat skin)")
