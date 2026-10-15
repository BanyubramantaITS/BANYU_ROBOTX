#!/usr/bin/env python3
"""
RobotX 2026 - Underwater Magnetically Activated Light Node
CAD Assembly Script for FreeCAD
Modeled Competition Unit (Task 2: Underwater Pipeline)

Features:
- Polycase ML-22F*1508 Enclosure with clean flanged base (as before)
- Transparent polycarbonate lid with smooth rounded top shoulder fillet (not boxy)
- Adafruit TLV493D Magnetometer Board oriented orthogonally (as before, long axis along X)
- Adafruit 16-LED WS2812B RGB Ring (44.45mm OD, 31.75mm ID)
- Dual Blue Robotics M10 WetLink Penetrators on opposing side bulkheads
- Sensor elevated to precisely 0.20" (5.0mm) below outer top cover
- 4x M3 Stainless Steel corner machine screws
"""

import math
import FreeCAD as App
import Part
from FreeCAD import Vector, Matrix, Rotation, Placement

def make_rounded_rect_wire(dx, dy, r, z=0.0):
    """Creates a planar closed Wire of a rectangle centered at (0, 0, z) with corner radius r."""
    hx, hy = dx / 2.0, dy / 2.0
    r = min(r, hx - 0.1, hy - 0.1)
    
    e1 = Part.makeLine(Vector(-hx + r, hy, z), Vector(hx - r, hy, z))
    arc1 = Part.Arc(
        Vector(hx - r, hy, z),
        Vector(hx - r + r * 0.7071068, hy - r + r * 0.7071068, z),
        Vector(hx, hy - r, z)
    )
    e2 = Part.makeLine(Vector(hx, hy - r, z), Vector(hx, -hy + r, z))
    arc2 = Part.Arc(
        Vector(hx, -hy + r, z),
        Vector(hx - r + r * 0.7071068, -hy + r - r * 0.7071068, z),
        Vector(hx - r, -hy, z)
    )
    e3 = Part.makeLine(Vector(hx - r, -hy, z), Vector(-hx + r, -hy, z))
    arc3 = Part.Arc(
        Vector(-hx + r, -hy, z),
        Vector(-hx + r - r * 0.7071068, -hy + r - r * 0.7071068, z),
        Vector(-hx, -hy + r, z)
    )
    e4 = Part.makeLine(Vector(-hx, -hy + r, z), Vector(-hx, hy - r, z))
    arc4 = Part.Arc(
        Vector(-hx, hy - r, z),
        Vector(-hx + r - r * 0.7071068, hy - r + r * 0.7071068, z),
        Vector(-hx + r, hy, z)
    )
    
    return Part.Wire([e1, arc1.toShape(), e2, arc2.toShape(), e3, arc3.toShape(), e4, arc4.toShape()])

def make_hex_prism(across_flats, height, z_start=0.0):
    """Creates a regular hexagonal prism centered at (0, 0, z_start) along Z."""
    r = (across_flats / 2.0) / math.cos(math.radians(30))
    pts = []
    for i in range(6):
        ang = math.radians(i * 60 + 30)
        pts.append(Vector(r * math.cos(ang), r * math.sin(ang), z_start))
    edges = [Part.makeLine(pts[i], pts[(i + 1) % 6]) for i in range(6)]
    wire = Part.Wire(edges)
    face = Part.Face(wire)
    return face.extrude(Vector(0, 0, height))

def build_competition_node(doc_name="RobotX_Light_Node"):
    if doc_name in App.listDocuments():
        doc = App.getDocument(doc_name)
        for obj in list(doc.Objects):
            doc.removeObject(obj.Name)
    else:
        doc = App.newDocument(doc_name)

    # -------------------------------------------------------------
    # Dimensions (Polycase ML-22F Nominal)
    # -------------------------------------------------------------
    body_dx = 66.8
    body_dy = 66.8
    base_h = 26.0
    lid_h = 15.4
    total_h = base_h + lid_h  # 41.4 mm
    wall_t = 2.5
    corner_r = 4.0

    # -------------------------------------------------------------
    # 1. Enclosure Base Housing (Standard Polycase ML-22F as before)
    # -------------------------------------------------------------
    # Main outer base shell
    base_wire = make_rounded_rect_wire(body_dx, body_dy, corner_r, z=0.0)
    base_outer = Part.Face(base_wire).extrude(Vector(0, 0, base_h))

    # Inner cavity
    cavity_wire = make_rounded_rect_wire(body_dx - 2 * wall_t, body_dy - 2 * wall_t, max(1.0, corner_r - wall_t), z=wall_t)
    cavity = Part.Face(cavity_wire).extrude(Vector(0, 0, base_h - wall_t + 1.0))
    base_shell = base_outer.cut(cavity)

    # 4 Corner internal bosses for lid screws
    boss_pos = [(27.0, 27.0), (-27.0, 27.0), (-27.0, -27.0), (27.0, -27.0)]
    for bx, by in boss_pos:
        boss = Part.makeCylinder(4.2, base_h - wall_t, Vector(bx, by, wall_t), Vector(0, 0, 1))
        hole = Part.makeCylinder(1.5, base_h - wall_t + 1.0, Vector(bx, by, wall_t), Vector(0, 0, 1))
        boss = boss.cut(hole)
        base_shell = base_shell.fuse(boss)

    # Base Mounting Flange (Clean flat plate as before: 85.0 x 78.0 x 3.5 mm, R = 7.0 mm)
    flange_dx = 85.0
    flange_dy = 78.0
    flange_r = 7.0
    flange_wire = make_rounded_rect_wire(flange_dx, flange_dy, flange_r, z=0.0)
    flange_solid = Part.Face(flange_wire).extrude(Vector(0, 0, 3.5))

    # 4 Mounting slots in flange: X = +-35.5, Y = +-25.0
    flange_holes = [(35.5, 25.0), (35.5, -25.0), (-35.5, 25.0), (-35.5, -25.0)]
    for hx, hy in flange_holes:
        slot_hole = Part.makeCylinder(2.2, 5.0, Vector(hx, hy, -0.5), Vector(0, 0, 1))
        flange_solid = flange_solid.cut(slot_hole)

    base_shell = base_shell.fuse(flange_solid)

    # Penetrator mounting holes through left and right bulkheads (M10, Z = 13.0, Y = 0.0)
    hole_left = Part.makeCylinder(5.1, 15.0, Vector(-35.0, 0.0, 13.0), Vector(1, 0, 0))
    hole_right = Part.makeCylinder(5.1, 15.0, Vector(25.0, 0.0, 13.0), Vector(1, 0, 0))
    base_shell = base_shell.cut(hole_left).cut(hole_right)
    base_shell = base_shell.removeSplitter()

    obj_base = doc.addObject("Part::Feature", "Enclosure_Base")
    obj_base.Shape = base_shell
    obj_base.Label = "Enclosure Base (Polycase ML-22F)"
    if hasattr(obj_base, "ViewObject") and obj_base.ViewObject:
        obj_base.ViewObject.ShapeColor = (0.76, 0.78, 0.80)  # Matte light gray polycarbonate
        obj_base.ViewObject.DisplayMode = "Shaded"

    # -------------------------------------------------------------
    # 2. Dual Blue Robotics M10 WetLink Penetrators
    # -------------------------------------------------------------
    def make_wetlink_penetrator(center_pos, dir_vector):
        rot_q = Rotation(Vector(0, 0, 1), dir_vector)
        bulkhead = Part.makeCylinder(5.0, 10.0, Vector(0, 0, -8.0), Vector(0, 0, 1))
        hex_collar = make_hex_prism(16.0, 4.0, z_start=0.0)
        neck = Part.makeCylinder(6.5, 3.0, Vector(0, 0, 4.0), Vector(0, 0, 1))
        hex_nut = make_hex_prism(18.0, 9.0, z_start=7.0)
        cone_tip = Part.makeCone(7.5, 5.0, 3.0, Vector(0, 0, 16.0), Vector(0, 0, 1))
        bore = Part.makeCylinder(3.1, 35.0, Vector(0, 0, -10.0), Vector(0, 0, 1))
        cable = Part.makeCylinder(3.0, 16.0, Vector(0, 0, 16.0), Vector(0, 0, 1))

        pen_gland = bulkhead.fuse(hex_collar).fuse(neck).fuse(hex_nut).fuse(cone_tip).cut(bore).fuse(cable)
        pen_gland = pen_gland.removeSplitter()
        pen_gland.Placement = Placement(center_pos, rot_q)
        return pen_gland

    pen_left = make_wetlink_penetrator(Vector(-body_dx / 2.0, 0.0, 13.0), Vector(-1, 0, 0))
    obj_pen_l = doc.addObject("Part::Feature", "WetLink_Penetrator_Left")
    obj_pen_l.Shape = pen_left
    obj_pen_l.Label = "WetLink Penetrator (Left Port)"
    if hasattr(obj_pen_l, "ViewObject") and obj_pen_l.ViewObject:
        obj_pen_l.ViewObject.ShapeColor = (0.15, 0.15, 0.15)  # Black acetal
        obj_pen_l.ViewObject.DisplayMode = "Shaded"

    pen_right = make_wetlink_penetrator(Vector(body_dx / 2.0, 0.0, 13.0), Vector(1, 0, 0))
    obj_pen_r = doc.addObject("Part::Feature", "WetLink_Penetrator_Right")
    obj_pen_r.Shape = pen_right
    obj_pen_r.Label = "WetLink Penetrator (Right Port)"
    if hasattr(obj_pen_r, "ViewObject") and obj_pen_r.ViewObject:
        obj_pen_r.ViewObject.ShapeColor = (0.15, 0.15, 0.15)
        obj_pen_r.ViewObject.DisplayMode = "Shaded"

    # -------------------------------------------------------------
    # 3. Internal Carrier Standoff
    # -------------------------------------------------------------
    carrier_plate_w = make_rounded_rect_wire(52.0, 52.0, 3.0, z=2.5)
    carrier_plate = Part.Face(carrier_plate_w).extrude(Vector(0, 0, 2.0))

    pillar_pos = [(18.0, 18.0), (-18.0, 18.0), (-18.0, -18.0), (18.0, -18.0)]
    standoff_solids = []
    for px, py in pillar_pos:
        pil = Part.makeCylinder(2.5, 28.7, Vector(px, py, 4.5), Vector(0, 0, 1))
        standoff_solids.append(pil)
    
    seat_ring = Part.makeCylinder(22.2, 1.2, Vector(0, 0, 32.0), Vector(0, 0, 1))
    seat_hole = Part.makeCylinder(16.0, 2.0, Vector(0, 0, 31.5), Vector(0, 0, 1))
    seat = seat_ring.cut(seat_hole)

    carrier = carrier_plate
    for s in standoff_solids:
        carrier = carrier.fuse(s)
    carrier = carrier.fuse(seat)
    carrier = carrier.removeSplitter()

    obj_carrier = doc.addObject("Part::Feature", "Internal_Carrier")
    obj_carrier.Shape = carrier
    obj_carrier.Label = "Sensor/LED Carrier Standoff"
    if hasattr(obj_carrier, "ViewObject") and obj_carrier.ViewObject:
        obj_carrier.ViewObject.ShapeColor = (0.35, 0.38, 0.42)
        obj_carrier.ViewObject.DisplayMode = "Shaded"

    # -------------------------------------------------------------
    # 4. Adafruit 16-LED WS2812B RGB Ring
    # -------------------------------------------------------------
    ring_outer = Part.makeCylinder(44.45 / 2.0, 1.6, Vector(0, 0, 33.2), Vector(0, 0, 1))
    ring_inner = Part.makeCylinder(31.75 / 2.0, 2.0, Vector(0, 0, 33.0), Vector(0, 0, 1))
    ring_pcb = ring_outer.cut(ring_inner)

    obj_ring_pcb = doc.addObject("Part::Feature", "LED_Ring_PCB")
    obj_ring_pcb.Shape = ring_pcb
    obj_ring_pcb.Label = "Adafruit 16-LED Ring PCB"
    if hasattr(obj_ring_pcb, "ViewObject") and obj_ring_pcb.ViewObject:
        obj_ring_pcb.ViewObject.ShapeColor = (0.08, 0.08, 0.08)  # Black PCB
        obj_ring_pcb.ViewObject.DisplayMode = "Shaded"

    r_pcd = 38.10 / 2.0
    led_packages = []
    led_lenses = []
    for i in range(16):
        ang_deg = i * 22.5
        ang_rad = math.radians(ang_deg)
        lx = r_pcd * math.cos(ang_rad)
        ly = r_pcd * math.sin(ang_rad)
        
        pkg = Part.makeBox(5.0, 5.0, 1.4, Vector(-2.5, -2.5, 34.8))
        pkg.rotate(Vector(0, 0, 0), Vector(0, 0, 1), ang_deg)
        pkg.translate(Vector(lx, ly, 0))
        led_packages.append(pkg)

        lens = Part.makeCylinder(1.6, 0.25, Vector(0, 0, 36.2), Vector(0, 0, 1))
        lens.translate(Vector(lx, ly, 0))
        led_lenses.append(lens)

    all_pkgs = led_packages[0]
    for p in led_packages[1:]:
        all_pkgs = all_pkgs.fuse(p)
    all_pkgs = all_pkgs.removeSplitter()

    obj_led_pkgs = doc.addObject("Part::Feature", "WS2812_LED_Packages")
    obj_led_pkgs.Shape = all_pkgs
    obj_led_pkgs.Label = "WS2812 5050 LED Packages"
    if hasattr(obj_led_pkgs, "ViewObject") and obj_led_pkgs.ViewObject:
        obj_led_pkgs.ViewObject.ShapeColor = (0.95, 0.95, 0.95)
        obj_led_pkgs.ViewObject.DisplayMode = "Shaded"

    all_lenses = led_lenses[0]
    for l in led_lenses[1:]:
        all_lenses = all_lenses.fuse(l)
    all_lenses = all_lenses.removeSplitter()

    obj_led_lenses = doc.addObject("Part::Feature", "WS2812_LED_Lenses")
    obj_led_lenses.Shape = all_lenses
    obj_led_lenses.Label = "WS2812 LED Emitting Lenses"
    if hasattr(obj_led_lenses, "ViewObject") and obj_led_lenses.ViewObject:
        obj_led_lenses.ViewObject.ShapeColor = (0.75, 0.92, 0.96)  # Frosted cyan / mint blue
        obj_led_lenses.ViewObject.DisplayMode = "Shaded"

    # -------------------------------------------------------------
    # 5. Adafruit TLV493D Magnetometer Board (90 deg / Orthogonal as before)
    # -------------------------------------------------------------
    # Dimensions: 25.40 mm (along X) x 17.78 mm (along Y), thickness = 1.6 mm
    mag_wire = make_rounded_rect_wire(25.4, 17.78, 2.54, z=33.7)
    mag_pcb = Part.Face(mag_wire).extrude(Vector(0, 0, 1.6))

    mag_holes = [
        (10.16, 6.35),
        (-10.16, 6.35),
        (-10.16, -6.35),
        (10.16, -6.35)
    ]
    for mx, my in mag_holes:
        m_hole = Part.makeCylinder(1.25, 2.0, Vector(mx, my, 33.5), Vector(0, 0, 1))
        mag_pcb = mag_pcb.cut(m_hole)

    gold_pads = []
    # 5 Pin Header gold pads at bottom (Y = -6.35)
    for px_idx in range(5):
        h_x = -5.08 + px_idx * 2.54
        pad_cyl = Part.makeCylinder(0.85, 0.1, Vector(h_x, -6.35, 35.3), Vector(0, 0, 1))
        gold_pads.append(pad_cyl)

    # 4 Gold annular rings at mounting holes
    for mx, my in mag_holes:
        pad_ring = Part.makeCylinder(1.7, 0.08, Vector(mx, my, 35.3), Vector(0, 0, 1))
        h_cut = Part.makeCylinder(1.25, 0.2, Vector(mx, my, 35.2), Vector(0, 0, 1))
        pad_ring = pad_ring.cut(h_cut)
        gold_pads.append(pad_ring)

    all_pads = gold_pads[0]
    for gp in gold_pads[1:]:
        all_pads = all_pads.fuse(gp)
    all_pads = all_pads.removeSplitter()

    # Central Sensor IC: TLV493D (2.9 x 1.6 x 1.1 mm) at (0, 0, 35.3) -> Top at Z = 36.4 mm
    # Distance to outer top cover (Z = 41.4): 41.4 - 36.4 = 5.0 mm (0.20 in)
    ic_body = Part.makeBox(2.9, 1.6, 1.1, Vector(-1.45, -0.8, 35.3))

    # White silkscreen markings on PCB
    silk_elements = []
    # Adafruit flower logo
    silk_logo = Part.makeCylinder(1.2, 0.05, Vector(0, 3.8, 35.3), Vector(0, 0, 1))
    silk_elements.append(silk_logo)
    # Coordinate axes cross
    cross_x = Part.makeBox(3.0, 0.4, 0.05, Vector(-1.5, -2.7, 35.3))
    cross_y = Part.makeBox(0.4, 3.0, 0.05, Vector(-0.2, -4.0, 35.3))
    silk_elements.append(cross_x)
    silk_elements.append(cross_y)

    all_silk = silk_elements[0]
    for se in silk_elements[1:]:
        all_silk = all_silk.fuse(se)
    all_silk = all_silk.removeSplitter()

    # Dual STEMMA QT Connectors on left and right edges
    conn_left = Part.makeBox(4.2, 3.2, 1.7, Vector(-11.5, -1.6, 35.3))
    conn_right = Part.makeBox(4.2, 3.2, 1.7, Vector(7.3, -1.6, 35.3))
    both_conns = conn_left.fuse(conn_right).removeSplitter()

    obj_mag_pcb = doc.addObject("Part::Feature", "Magnetometer_PCB")
    obj_mag_pcb.Shape = mag_pcb
    obj_mag_pcb.Label = "Adafruit TLV493D PCB"
    if hasattr(obj_mag_pcb, "ViewObject") and obj_mag_pcb.ViewObject:
        obj_mag_pcb.ViewObject.ShapeColor = (0.12, 0.52, 0.24)  # Vibrant Adafruit Green
        obj_mag_pcb.ViewObject.DisplayMode = "Shaded"

    obj_gold_pads = doc.addObject("Part::Feature", "Magnetometer_Gold_Pads")
    obj_gold_pads.Shape = all_pads
    obj_gold_pads.Label = "Magnetometer Gold Pads & Rings"
    if hasattr(obj_gold_pads, "ViewObject") and obj_gold_pads.ViewObject:
        obj_gold_pads.ViewObject.ShapeColor = (0.85, 0.70, 0.20)  # ENIG Gold
        obj_gold_pads.ViewObject.DisplayMode = "Shaded"

    obj_ic = doc.addObject("Part::Feature", "TLV493D_Sensor_IC")
    obj_ic.Shape = ic_body
    obj_ic.Label = "Infineon TLV493D 3D Sensor IC"
    if hasattr(obj_ic, "ViewObject") and obj_ic.ViewObject:
        obj_ic.ViewObject.ShapeColor = (0.05, 0.05, 0.05)
        obj_ic.ViewObject.DisplayMode = "Shaded"

    obj_silk = doc.addObject("Part::Feature", "Magnetometer_Silkscreen")
    obj_silk.Shape = all_silk
    obj_silk.Label = "Magnetometer Silkscreen Graphics"
    if hasattr(obj_silk, "ViewObject") and obj_silk.ViewObject:
        obj_silk.ViewObject.ShapeColor = (0.95, 0.95, 0.95)
        obj_silk.ViewObject.DisplayMode = "Shaded"

    obj_conns = doc.addObject("Part::Feature", "STEMMA_QT_Connectors")
    obj_conns.Shape = both_conns
    obj_conns.Label = "STEMMA QT JST-SH Connectors"
    if hasattr(obj_conns, "ViewObject") and obj_conns.ViewObject:
        obj_conns.ViewObject.ShapeColor = (0.90, 0.90, 0.88)
        obj_conns.ViewObject.DisplayMode = "Shaded"

    # -------------------------------------------------------------
    # 6. Transparent Polycarbonate Cover (With smooth rounded shoulder fillet)
    # -------------------------------------------------------------
    # Sits from Z = 26.0 to Z = 41.4 mm (Height: 15.4 mm)
    lid_wire_outer = make_rounded_rect_wire(body_dx, body_dy, corner_r, z=26.0)
    lid_outer = Part.Face(lid_wire_outer).extrude(Vector(0, 0, lid_h))

    # Molded rounded shoulder radius on top edge (radius 3.0 mm)
    top_edges = [e for e in lid_outer.Edges if abs(e.BoundBox.ZMin - total_h) < 0.01 and abs(e.BoundBox.ZMax - total_h) < 0.01]
    if len(top_edges) == 8:
        lid_outer = lid_outer.makeFillet(3.0, top_edges)

    # Internal hollow of lid (Z = 25.5 to Z = 38.9 mm, wall thickness 2.5 mm)
    lid_wire_inner = make_rounded_rect_wire(body_dx - 2 * wall_t, body_dy - 2 * wall_t, max(1.0, corner_r - wall_t), z=25.5)
    lid_cavity = Part.Face(lid_wire_inner).extrude(Vector(0, 0, lid_h - wall_t + 0.5))
    lid_shell = lid_outer.cut(lid_cavity)

    # 4 Corner Screw Boss Pillars in Lid
    for bx, by in boss_pos:
        lboss = Part.makeCylinder(4.2, lid_h - wall_t + 0.5, Vector(bx, by, 25.5), Vector(0, 0, 1))
        lid_shell = lid_shell.fuse(lboss)

    # 4 Corner Screw Counterbore Pockets & Holes from top surface (Z = 41.4 mm)
    for bx, by in boss_pos:
        cbore = Part.makeCylinder(3.25, 4.0, Vector(bx, by, total_h - 3.5), Vector(0, 0, 1))
        shole = Part.makeCylinder(1.7, 18.0, Vector(bx, by, 25.0), Vector(0, 0, 1))
        lid_shell = lid_shell.cut(cbore).cut(shole)

    lid_shell = lid_shell.removeSplitter()

    obj_lid = doc.addObject("Part::Feature", "Enclosure_Lid")
    obj_lid.Shape = lid_shell
    obj_lid.Label = "Transparent Polycarbonate Cover"
    if hasattr(obj_lid, "ViewObject") and obj_lid.ViewObject:
        obj_lid.ViewObject.ShapeColor = (0.92, 0.96, 1.0)
        obj_lid.ViewObject.Transparency = 75
        obj_lid.ViewObject.DisplayMode = "Shaded"

    # -------------------------------------------------------------
    # 7. 4x M3 Stainless Steel Corner Machine Screws
    # -------------------------------------------------------------
    screws = []
    for bx, by in boss_pos:
        head = Part.makeCylinder(2.75, 2.2, Vector(bx, by, total_h - 3.5), Vector(0, 0, 1))
        shank = Part.makeCylinder(1.5, 14.0, Vector(bx, by, total_h - 17.5), Vector(0, 0, 1))
        slot1 = Part.makeBox(4.0, 0.7, 1.0, Vector(bx - 2.0, by - 0.35, total_h - 2.3))
        slot2 = Part.makeBox(0.7, 4.0, 1.0, Vector(bx - 0.35, by - 2.0, total_h - 2.3))
        screw = head.fuse(shank).cut(slot1).cut(slot2)
        screws.append(screw)

    all_screws = screws[0]
    for s in screws[1:]:
        all_screws = all_screws.fuse(s)
    all_screws = all_screws.removeSplitter()

    obj_screws = doc.addObject("Part::Feature", "Cover_Screws")
    obj_screws.Shape = all_screws
    obj_screws.Label = "M3 Stainless Steel Lid Screws (x4)"
    if hasattr(obj_screws, "ViewObject") and obj_screws.ViewObject:
        obj_screws.ViewObject.ShapeColor = (0.86, 0.88, 0.90)
        obj_screws.ViewObject.DisplayMode = "Shaded"

    doc.recompute()
    return doc

if __name__ == "__main__":
    doc = build_competition_node()
    print("Build complete! Document:", doc.Name, "Objects:", len(doc.Objects))
