#!/usr/bin/env python3
"""
Bed_guides.py - CNC Plywood Bed Cutting-Mat Grid & Alignment Guide Generator
=============================================================================
Generates CNC G-code and an SVG blueprint for engraving a precision 
"cutting-mat" style alignment guide on a 3/4" (19.05 mm) plywood bed (275 x 170 mm).

Features:
  - 10 x 10 mm grid lines across the entire 275 x 170 mm bed.
  - Major grid lines highlighted every 50 mm.
  - Millimeter ruler tick marks (5mm minor, 10mm major) along borders.
  - Coordinate measurement numbers (10, 20, 30... 270 on X; 10, 20, 30... 160 on Y)
    using single-stroke CNC vector fonts for crisp, rapid V-bit engraving.
  - Angle guide lines for 30°, 45°, and 60° (with engraved degree labels).
  - Four-corner datum brackets and center crosshairs for effortless removal
    and precise repositioning / relocation of the plywood bed.
  - Limit Switch Pocket marker at the probed coordinates:
    Corner 1: (275.002, 160.003) | Corner 2: (269.502, 140.003)
    (19.5 x 5.9 mm nominal microswitch, 10mm depth).
  - High-resolution SVG preview styled like a classic self-healing cutting mat.

Usage:
  python Bed_guides.py
  python Bed_guides.py --grid-step 10.0 --depth 0.25
  python Bed_guides.py --output-dir ./output
"""

import os
import sys
import math
import argparse
from datetime import datetime


# ---------------------------------------------------------------------------
# Single-Stroke Vector Font for CNC G-Code Engraving
# Normalized 0..1 scale (width=1.0, height=1.0)
# ---------------------------------------------------------------------------

STROKE_FONT = {
    '0': [[(0.1, 0.0), (0.9, 0.0), (0.9, 1.0), (0.1, 1.0), (0.1, 0.0)]],
    '1': [[(0.3, 0.8), (0.5, 1.0), (0.5, 0.0)], [(0.2, 0.0), (0.8, 0.0)]],
    '2': [[(0.1, 0.8), (0.3, 1.0), (0.7, 1.0), (0.9, 0.8), (0.9, 0.5), (0.1, 0.0), (0.9, 0.0)]],
    '3': [[(0.1, 0.9), (0.3, 1.0), (0.7, 1.0), (0.9, 0.8), (0.5, 0.5), (0.9, 0.5), (0.9, 0.2), (0.7, 0.0), (0.3, 0.0), (0.1, 0.1)]],
    '4': [[(0.8, 0.0), (0.8, 1.0), (0.1, 0.35), (0.9, 0.35)]],
    '5': [[(0.9, 1.0), (0.1, 1.0), (0.1, 0.55), (0.7, 0.55), (0.9, 0.35), (0.9, 0.2), (0.7, 0.0), (0.1, 0.0)]],
    '6': [[(0.8, 0.9), (0.6, 1.0), (0.3, 1.0), (0.1, 0.8), (0.1, 0.2), (0.3, 0.0), (0.7, 0.0), (0.9, 0.2), (0.9, 0.5), (0.1, 0.5)]],
    '7': [[(0.1, 1.0), (0.9, 1.0), (0.4, 0.0)]],
    '8': [[(0.3, 0.5), (0.1, 0.7), (0.3, 1.0), (0.7, 1.0), (0.9, 0.7), (0.7, 0.5), (0.3, 0.5), (0.1, 0.3), (0.3, 0.0), (0.7, 0.0), (0.9, 0.3), (0.7, 0.5)]],
    '9': [[(0.1, 0.5), (0.9, 0.5), (0.9, 0.8), (0.7, 1.0), (0.3, 1.0), (0.1, 0.8), (0.1, 0.5), (0.9, 0.5), (0.9, 0.2), (0.7, 0.0), (0.3, 0.0), (0.1, 0.1)]],
    '°': [[(0.3, 0.7), (0.7, 0.7), (0.7, 1.0), (0.3, 1.0), (0.3, 0.7)]],
    'X': [[(0.1, 0.0), (0.9, 1.0)], [(0.1, 1.0), (0.9, 0.0)]],
    'Y': [[(0.1, 1.0), (0.5, 0.5), (0.9, 1.0)], [(0.5, 0.5), (0.5, 0.0)]],
    '+': [[(0.5, 0.1), (0.5, 0.9)], [(0.1, 0.5), (0.9, 0.5)]],
    '-': [[(0.1, 0.5), (0.9, 0.5)]],
    '.': [[(0.4, 0.0), (0.6, 0.0), (0.6, 0.2), (0.4, 0.2), (0.4, 0.0)]],
}


def get_text_polylines(text, x, y, char_h=3.0, char_w=2.0, char_spacing=0.8):
    """
    Convert a text string into a list of 2D polylines for CNC G-code engraving.
    """
    polylines = []
    curr_x = x
    for ch in str(text):
        if ch in STROKE_FONT:
            for stroke in STROKE_FONT[ch]:
                poly = [(curr_x + px * char_w, y + py * char_h) for (px, py) in stroke]
                polylines.append(poly)
            curr_x += char_w + char_spacing
        elif ch == ' ':
            curr_x += char_w + char_spacing
        else:
            curr_x += char_w + char_spacing
    return polylines


def generate_bed_geometry(bed_w=275.0, bed_h=170.0, grid_step=10.0,
                          switch_p1=(275.002, 160.003), switch_p2=(269.502, 140.003)):
    """
    Generate all geometric entities for the cutting-mat style bed guide:
      - Border rectangle
      - 10mm Grid lines (serpentine ordered)
      - 50mm Major lines
      - Ruler tick marks
      - Coordinate number polylines
      - 30°, 45°, 60° angle lines and labels
      - Relocation datum corner brackets
      - Center crosshair
      - Limit switch pocket marker
    """
    data = {
        "border": [],
        "minor_grid": [],
        "major_grid": [],
        "ticks": [],
        "angles": [],
        "angle_labels": [],
        "text_strokes": [],
        "datums": [],
        "center_cross": [],
        "switch_box": {},
    }

    # 1. Border
    data["border"] = [
        [(0.0, 0.0), (bed_w, 0.0), (bed_w, bed_h), (0.0, bed_h), (0.0, 0.0)]
    ]

    # 2. Grid lines (serpentine / bi-directional for fast cutting)
    # Horizontal lines (Y = grid_step to bed_h - grid_step)
    y_vals = []
    y = grid_step
    while y < bed_h - 1e-4:
        y_vals.append(round(y, 3))
        y += grid_step

    for i, y_coord in enumerate(y_vals):
        is_major = (int(round(y_coord)) % 50 == 0)
        p1 = (0.0, y_coord) if (i % 2 == 0) else (bed_w, y_coord)
        p2 = (bed_w, y_coord) if (i % 2 == 0) else (0.0, y_coord)
        if is_major:
            data["major_grid"].append([p1, p2])
        else:
            data["minor_grid"].append([p1, p2])

    # Vertical lines (X = grid_step to bed_w - grid_step)
    x_vals = []
    x = grid_step
    while x < bed_w - 1e-4:
        x_vals.append(round(x, 3))
        x += grid_step

    for j, x_coord in enumerate(x_vals):
        is_major = (int(round(x_coord)) % 50 == 0)
        p1 = (x_coord, 0.0) if (j % 2 == 0) else (x_coord, bed_h)
        p2 = (x_coord, bed_h) if (j % 2 == 0) else (x_coord, 0.0)
        if is_major:
            data["major_grid"].append([p1, p2])
        else:
            data["minor_grid"].append([p1, p2])

    # 3. Millimeter Ruler Ticks along borders
    # X ticks (every 5mm and 10mm)
    tx = 5.0
    while tx < bed_w - 1e-4:
        is_10 = (int(round(tx)) % 10 == 0)
        t_len = 3.5 if is_10 else 1.8
        # Bottom edge
        data["ticks"].append([(tx, 0.0), (tx, t_len)])
        # Top edge
        data["ticks"].append([(tx, bed_h), (tx, bed_h - t_len)])
        tx += 5.0

    # Y ticks (every 5mm and 10mm)
    ty = 5.0
    while ty < bed_h - 1e-4:
        is_10 = (int(round(ty)) % 10 == 0)
        t_len = 3.5 if is_10 else 1.8
        # Left edge
        data["ticks"].append([(0.0, ty), (t_len, ty)])
        # Right edge
        data["ticks"].append([(bed_w, ty), (bed_w - t_len, ty)])
        ty += 5.0

    # 4. Numeric Measurement Labels along X and Y axes
    # X numbers (every 20mm or 50mm, with 10mm markers)
    for x_c in range(10, int(bed_w), 20):
        # Engrave numbers just above bottom ticks (Y = 4.5 mm)
        num_str = str(x_c)
        str_w = len(num_str) * 2.0 + (len(num_str) - 1) * 0.8
        px = x_c - str_w / 2.0
        py = 4.5
        polys = get_text_polylines(num_str, px, py, char_h=2.8, char_w=1.8, char_spacing=0.6)
        data["text_strokes"].extend(polys)

    # Y numbers
    for y_c in range(10, int(bed_h), 20):
        num_str = str(y_c)
        px = 4.5
        py = y_c - 1.4
        polys = get_text_polylines(num_str, px, py, char_h=2.8, char_w=1.8, char_spacing=0.6)
        data["text_strokes"].extend(polys)

    # 5. Angle Guide Lines: 30°, 45°, 60°
    # Primary rays from (0, 0):
    # 30°: tan(30) = 0.57735. At X=bed_w: Y = bed_w * tan(30)
    y_30 = bed_w * math.tan(math.radians(30))
    if y_30 <= bed_h:
        end_30 = (bed_w, y_30)
    else:
        end_30 = (bed_h / math.tan(math.radians(30)), bed_h)
    data["angles"].append([(0.0, 0.0), end_30])
    lbl_30 = get_text_polylines("30°", 65.0, 65.0 * math.tan(math.radians(30)) + 1.2,
                                char_h=3.2, char_w=2.0)
    data["angle_labels"].extend(lbl_30)

    # 45°: from (0, 0)
    x_45 = min(bed_w, bed_h)
    end_45 = (x_45, x_45)
    data["angles"].append([(0.0, 0.0), end_45])
    lbl_45 = get_text_polylines("45°", 65.0, 65.0 + 1.2, char_h=3.2, char_w=2.0)
    data["angle_labels"].extend(lbl_45)

    # 60°: tan(60) = 1.73205.
    x_60 = bed_h / math.tan(math.radians(60))
    end_60 = (x_60, bed_h)
    data["angles"].append([(0.0, 0.0), end_60])
    lbl_60 = get_text_polylines("60°", 40.0, 40.0 * math.tan(math.radians(60)) + 1.2,
                                char_h=3.2, char_w=2.0)
    data["angle_labels"].extend(lbl_60)

    # Secondary complementary 45° line from (105, 0) to (275, 170)
    # (Since 275 - 105 = 170, this forms a perfect diagonal across the top-right!)
    data["angles"].append([(105.0, 0.0), (bed_w, bed_h)])
    lbl_45_sec = get_text_polylines("45°", 175.0, 70.0 + 1.2, char_h=3.2, char_w=2.0)
    data["angle_labels"].extend(lbl_45_sec)

    # Secondary 30° line from bottom-right (275, 0) to (0, 158.77)
    data["angles"].append([(bed_w, 0.0), (0.0, y_30)])
    lbl_30_sec = get_text_polylines("30°", 200.0, (bed_w - 200.0) * math.tan(math.radians(30)) + 1.2,
                                    char_h=3.2, char_w=2.0)
    data["angle_labels"].extend(lbl_30_sec)

    # 6. Relocation Datum Brackets (L-brackets at the 4 corners: 8x8mm)
    b_len = 8.0
    data["datums"].append([(0.0, b_len), (0.0, 0.0), (b_len, 0.0)])
    data["datums"].append([(bed_w - b_len, 0.0), (bed_w, 0.0), (bed_w, b_len)])
    data["datums"].append([(bed_w, bed_h - b_len), (bed_w, bed_h), (bed_w - b_len, bed_h)])
    data["datums"].append([(b_len, bed_h), (0.0, bed_h), (0.0, bed_h - b_len)])

    # Center crosshair at (bed_w/2, bed_h/2)
    cx, cy = bed_w / 2.0, bed_h / 2.0
    data["center_cross"].append([(cx - 7.0, cy), (cx + 7.0, cy)])
    data["center_cross"].append([(cx, cy - 7.0), (cx, cy + 7.0)])

    # 7. Limit Switch Pocket Reference (clamped within bed travel limits)
    sw_xmax = min(max(switch_p1[0], switch_p2[0]), bed_w - 0.1)
    sw_xmin = max(0.0, min(switch_p1[0], switch_p2[0]))
    sw_ymax = min(max(switch_p1[1], switch_p2[1]), bed_h - 0.1)
    sw_ymin = max(0.0, min(switch_p1[1], switch_p2[1]))
    data["switch_box"] = {
        "xmin": sw_xmin,
        "xmax": sw_xmax,
        "ymin": sw_ymin,
        "ymax": sw_ymax,
        "width": sw_xmax - sw_xmin,
        "length": sw_ymax - sw_ymin,
        "p1": switch_p1,
        "p2": switch_p2,
        "polyline": [
            (sw_xmin, sw_ymin),
            (sw_xmax, sw_ymin),
            (sw_xmax, sw_ymax),
            (sw_xmin, sw_ymax),
            (sw_xmin, sw_ymin),
        ]
    }

    return data


def generate_cutting_mat_gcode(data, output_path, cut_depth=0.25, feed_rate=450.0,
                               plunge_rate=120.0, spindle_speed=10000, safe_z=3.0,
                               engrave_switch=True):
    """
    Generate optimized GRBL G-code for V-bit engraving of the cutting mat bed guides.
    """
    lines = []
    lines.append("( ========================================================== )")
    lines.append("(  CNC Plywood Bed Guides - Cutting-Mat Engraving            )")
    lines.append(f"(  Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} )")
    lines.append("(  Bed Size : 275.0 mm x 170.0 mm (3/4\" Plywood)          )")
    lines.append(f"(  Engrave Depth: -{cut_depth:.2f} mm with V-Bit             )")
    lines.append(f"(  Feed Rate    : {feed_rate:.1f} mm/min, Spindle: {spindle_speed} RPM )")
    lines.append("(  Includes 10mm Grid, Major 50mm, 30/45/60 Deg Angles, Ticks )")
    lines.append("( ========================================================== )")
    lines.append("")
    lines.append("G21          (Units in Millimeters)")
    lines.append("G90          (Absolute Coordinates)")
    lines.append("G94          (Feed Rate per Minute)")
    lines.append("G17          (XY Work Plane Selection)")
    lines.append("G54          (Work Coordinate System 1 / Active Work Offset)")
    lines.append(f"G00 Z{safe_z:.4f} (Rapid to Safe Z)")
    lines.append(f"G00 X0.0000 Y0.0000 (Establish Initial Workpiece Position)")
    lines.append(f"M03 S{spindle_speed} (Spindle Start CW)")
    lines.append("G04 P2.0     (Dwell 2s for spindle to reach full speed)")
    lines.append("")

    def write_polylines(polyline_list, section_name):
        lines.append(f"( --- Section: {section_name} ({len(polyline_list)} items) --- )")
        for poly in polyline_list:
            if not poly or len(poly) < 2:
                continue
            start = poly[0]
            lines.append(f"G00 X{start[0]:.4f} Y{start[1]:.4f}")
            lines.append(f"G01 Z-{cut_depth:.4f} F{plunge_rate:.1f}")
            for pt in poly[1:]:
                lines.append(f"G01 X{pt[0]:.4f} Y{pt[1]:.4f} F{feed_rate:.1f}")
            lines.append(f"G00 Z{safe_z:.4f}")
        lines.append("")

    # 1. Border
    write_polylines(data["border"], "Outer Bed Boundary")

    # 2. Minor Grid (10mm)
    write_polylines(data["minor_grid"], "10mm Minor Grid Lines")

    # 3. Major Grid (50mm)
    write_polylines(data["major_grid"], "50mm Major Grid Lines")

    # 4. Ruler Ticks
    write_polylines(data["ticks"], "Millimeter Ruler Ticks")

    # 5. Angle Guides (30°, 45°, 60°)
    write_polylines(data["angles"], "Angle Guides 30-45-60 deg")

    # 6. Angle Labels & Coordinate Numbers
    write_polylines(data["angle_labels"], "Angle Text Labels")
    write_polylines(data["text_strokes"], "Coordinate Measurement Numbers")

    # 7. Datum Brackets & Center Cross
    write_polylines(data["datums"], "Relocation Datum Brackets")
    write_polylines(data["center_cross"], "Bed Center Crosshair")

    # 8. Limit Switch Pocket Outline
    if engrave_switch and data.get("switch_box"):
        write_polylines([data["switch_box"]["polyline"]], "Limit Switch Pocket Outline")

    # Footer
    lines.append("M05          (Spindle Stop)")
    lines.append("G00 Z20.0000 (Clearance Height)")
    lines.append("G00 X0.0000 Y0.0000 (Return to Origin)")
    lines.append("M30          (Program End and Reset)")

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"  [+] Saved Cutting-Mat G-code: {output_path}")


def generate_cutting_mat_svg(data, bed_w=275.0, bed_h=170.0, output_path="bed_guides_cutting_mat.svg"):
    """
    Generate a full-detail, scalable SVG file styled as a classic cutting mat.
    Includes the 275x170 bed, 10x10 grid, 30°/45°/60° angle lines, millimeter rulers,
    and the limit switch pocket location.
    """
    margin = 14.0
    view_w = bed_w + 2 * margin
    view_h = bed_h + 2 * margin

    # Pixels per mm for display sharpness
    ppm = 4.0
    canvas_w = view_w * ppm
    canvas_h = view_h * ppm

    def tx(x):
        return (x + margin) * ppm

    def ty(y):
        # Invert Y so (0,0) is at bottom-left in standard CNC coords
        return (bed_h - y + margin) * ppm

    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {canvas_w} {canvas_h}" width="{canvas_w}" height="{canvas_h}">')
    svg.append('  <defs>')
    # Subtle drop shadow
    svg.append('    <filter id="shadow" x="-5%" y="-5%" width="110%" height="110%">')
    svg.append('      <feDropShadow dx="3" dy="4" stdDeviation="4" flood-color="#000000" flood-opacity="0.5" />')
    svg.append('    </filter>')
    svg.append('  </defs>')

    # Outer canvas background (dark workshop surface)
    svg.append(f'  <rect width="100%" height="100%" fill="#1a1c1e" />')

    # Bed outline (Cutting Mat green style with rounded corners)
    bx, by = tx(0), ty(bed_h)
    bw, bh = bed_w * ppm, bed_h * ppm
    svg.append(f'  <!-- Plywood Bed / Cutting Mat Body -->')
    svg.append(f'  <rect x="{bx}" y="{by}" width="{bw}" height="{bh}" rx="6" fill="#143224" stroke="#22543d" stroke-width="2" filter="url(#shadow)" />')

    # 10mm Minor Grid Lines
    svg.append('  <!-- 10mm Minor Grid Lines -->')
    for poly in data["minor_grid"]:
        p1, p2 = poly[0], poly[1]
        svg.append(f'  <line x1="{tx(p1[0])}" y1="{ty(p1[1])}" x2="{tx(p2[0])}" y2="{ty(p2[1])}" stroke="#2d6a4f" stroke-width="1.0" opacity="0.85" />')

    # 50mm Major Grid Lines
    svg.append('  <!-- 50mm Major Grid Lines -->')
    for poly in data["major_grid"]:
        p1, p2 = poly[0], poly[1]
        svg.append(f'  <line x1="{tx(p1[0])}" y1="{ty(p1[1])}" x2="{tx(p2[0])}" y2="{ty(p2[1])}" stroke="#52b788" stroke-width="1.8" opacity="0.9" />')

    # Ruler Tick Marks (5mm and 10mm)
    svg.append('  <!-- Ruler Ticks -->')
    for poly in data["ticks"]:
        p1, p2 = poly[0], poly[1]
        svg.append(f'  <line x1="{tx(p1[0])}" y1="{ty(p1[1])}" x2="{tx(p2[0])}" y2="{ty(p2[1])}" stroke="#74c69d" stroke-width="1.2" />')

    # Angle Guide Lines (30°, 45°, 60°)
    svg.append('  <!-- Angle Guides (30, 45, 60 deg) -->')
    for poly in data["angles"]:
        p1, p2 = poly[0], poly[1]
        svg.append(f'  <line x1="{tx(p1[0])}" y1="{ty(p1[1])}" x2="{tx(p2[0])}" y2="{ty(p2[1])}" stroke="#f1c40f" stroke-width="1.8" stroke-dasharray="8,4" />')

    # Angle Labels
    svg.append('  <!-- Angle Labels -->')
    for poly in data["angle_labels"]:
        for i in range(len(poly) - 1):
            svg.append(f'  <line x1="{tx(poly[i][0])}" y1="{ty(poly[i][1])}" x2="{tx(poly[i+1][0])}" y2="{ty(poly[i+1][1])}" stroke="#f1c40f" stroke-width="1.8" />')

    # Numeric Measurement Labels (X & Y)
    svg.append('  <!-- Measurement Numbers -->')
    for poly in data["text_strokes"]:
        for i in range(len(poly) - 1):
            svg.append(f'  <line x1="{tx(poly[i][0])}" y1="{ty(poly[i][1])}" x2="{tx(poly[i+1][0])}" y2="{ty(poly[i+1][1])}" stroke="#d8f3dc" stroke-width="1.4" />')

    # Center Crosshair & Concentric Target Rings
    cx, cy = bed_w / 2.0, bed_h / 2.0
    c_px, c_py = tx(cx), ty(cy)
    svg.append('  <!-- Center Reference Mark -->')
    svg.append(f'  <circle cx="{c_px}" cy="{c_py}" r="{10 * ppm}" fill="none" stroke="#52b788" stroke-width="1.2" stroke-dasharray="4,3" opacity="0.7" />')
    svg.append(f'  <circle cx="{c_px}" cy="{c_py}" r="{20 * ppm}" fill="none" stroke="#52b788" stroke-width="1.2" stroke-dasharray="4,3" opacity="0.7" />')
    for poly in data["center_cross"]:
        p1, p2 = poly[0], poly[1]
        svg.append(f'  <line x1="{tx(p1[0])}" y1="{ty(p1[1])}" x2="{tx(p2[0])}" y2="{ty(p2[1])}" stroke="#52b788" stroke-width="1.5" />')
    svg.append(f'  <text x="{c_px + 8}" y="{c_py - 8}" fill="#95d5b2" font-size="11" font-family="monospace">CENTER ({cx:.1f}, {cy:.1f})</text>')

    # Corner Datum Relocation Brackets
    svg.append('  <!-- Relocation Datum Brackets -->')
    for poly in data["datums"]:
        for i in range(len(poly) - 1):
            svg.append(f'  <line x1="{tx(poly[i][0])}" y1="{ty(poly[i][1])}" x2="{tx(poly[i+1][0])}" y2="{ty(poly[i+1][1])}" stroke="#ffffff" stroke-width="2.5" />')

    # Bed Outer Boundary line
    for poly in data["border"]:
        for i in range(len(poly) - 1):
            svg.append(f'  <line x1="{tx(poly[i][0])}" y1="{ty(poly[i][1])}" x2="{tx(poly[i+1][0])}" y2="{ty(poly[i+1][1])}" stroke="#d8f3dc" stroke-width="2.2" />')

    # Machine Origin (0,0) Marker
    o_px, o_py = tx(0.0), ty(0.0)
    svg.append('  <!-- Machine Origin Indicator -->')
    svg.append(f'  <circle cx="{o_px}" cy="{o_py}" r="7" fill="#e74c3c" stroke="#ffffff" stroke-width="2" />')
    svg.append(f'  <line x1="{o_px}" y1="{o_py}" x2="{o_px + 28}" y2="{o_py}" stroke="#e74c3c" stroke-width="2.5" />')
    svg.append(f'  <polygon points="{o_px + 34},{o_py} {o_px + 26},{o_py - 4} {o_px + 26},{o_py + 4}" fill="#e74c3c" />')
    svg.append(f'  <text x="{o_px + 38}" y="{o_py + 4}" fill="#e74c3c" font-size="11" font-weight="bold" font-family="sans-serif">+X</text>')
    svg.append(f'  <line x1="{o_px}" y1="{o_py}" x2="{o_px}" y2="{o_py - 28}" stroke="#e74c3c" stroke-width="2.5" />')
    svg.append(f'  <polygon points="{o_px},{o_py - 34} {o_px - 4},{o_py - 26} {o_px + 4},{o_py - 26}" fill="#e74c3c" />')
    svg.append(f'  <text x="{o_px - 8}" y="{o_py - 38}" fill="#e74c3c" font-size="11" font-weight="bold" font-family="sans-serif">+Y</text>')
    svg.append(f'  <text x="{o_px + 8}" y="{o_py + 16}" fill="#ffffff" font-size="11" font-weight="bold" font-family="monospace">ORIGIN (0,0)</text>')

    # LIMIT SWITCH POCKET HIGHLIGHT
    sw = data.get("switch_box")
    if sw:
        sw_bx = tx(sw["xmin"])
        sw_by = ty(sw["ymax"])
        sw_w = sw["width"] * ppm
        sw_h = sw["length"] * ppm

        svg.append('  <!-- LIMIT SWITCH POCKET MARKER -->')
        # Red transparent fill with dashed border
        svg.append(f'  <rect x="{sw_bx}" y="{sw_by}" width="{sw_w}" height="{sw_h}" rx="2" fill="rgba(231, 76, 60, 0.4)" stroke="#e74c3c" stroke-width="2.5" stroke-dasharray="4,2" />')
        # Center cross
        sw_cx = (sw["xmin"] + sw["xmax"]) / 2.0
        sw_cy = (sw["ymin"] + sw["ymax"]) / 2.0
        svg.append(f'  <line x1="{tx(sw_cx) - 6}" y1="{ty(sw_cy)}" x2="{tx(sw_cx) + 6}" y2="{ty(sw_cy)}" stroke="#ffffff" stroke-width="1.2" />')
        svg.append(f'  <line x1="{tx(sw_cx)}" y1="{ty(sw_cy) - 6}" x2="{tx(sw_cx)}" y2="{ty(sw_cy) + 6}" stroke="#ffffff" stroke-width="1.2" />')

        # Callout card pointing to switch pocket
        card_x = sw_bx - 260
        card_y = sw_by - 20
        svg.append(f'  <g transform="translate({card_x}, {card_y})">')
        svg.append('    <rect width="250" height="75" rx="5" fill="rgba(20, 20, 20, 0.9)" stroke="#e74c3c" stroke-width="1.5" />')
        svg.append('    <text x="10" y="18" fill="#e74c3c" font-size="11" font-weight="bold" font-family="sans-serif">LIMIT SWITCH RECESS (10mm DEPTH)</text>')
        svg.append(f'    <text x="10" y="36" fill="#ffffff" font-size="10" font-family="monospace">Probed: ({sw["xmin"]:.1f}, {sw["ymin"]:.1f}) -> ({sw["xmax"]:.1f}, {sw["ymax"]:.1f})</text>')
        svg.append(f'    <text x="10" y="52" fill="#f1c40f" font-size="10" font-family="monospace">Span: {sw["width"]:.1f} x {sw["length"]:.1f} mm (Switch: 19.5x5.9mm)</text>')
        svg.append('    <text x="10" y="67" fill="#74c69d" font-size="9" font-family="monospace">Milling script: shpae_to_gcode.py</text>')
        # Arrow pointing to pocket
        svg.append(f'    <path d="M 250 35 L 260 35" stroke="#e74c3c" stroke-width="1.5" />')
        svg.append('  </g>')

    # Title & Specification Legend Block
    svg.append('  <!-- Title & Setup Info Block -->')
    leg_x = tx(12.0)
    leg_y = ty(bed_h - 10.0)
    svg.append(f'  <g transform="translate({leg_x}, {leg_y})">')
    svg.append('    <rect width="360" height="85" rx="5" fill="rgba(10, 25, 18, 0.92)" stroke="#52b788" stroke-width="1.5" />')
    svg.append('    <text x="12" y="20" fill="#52b788" font-size="13" font-weight="bold" font-family="sans-serif">CNC BED CUTTING-MAT &amp; ALIGNMENT SETUP</text>')
    svg.append(f'    <text x="12" y="38" fill="#d8f3dc" font-size="11" font-family="monospace">Bed Travel : {bed_w:.1f} x {bed_h:.1f} mm | Material: 3/4" Plywood</text>')
    svg.append('    <text x="12" y="54" fill="#f1c40f" font-size="11" font-family="monospace">Grid: 10x10 mm | Angles: 30°, 45°, 60°</text>')
    svg.append(f'    <text x="12" y="70" fill="#95d5b2" font-size="10" font-family="monospace">Relocation : 4 Corner Datum Marks &amp; Center Crosshair</text>')
    svg.append('  </g>')

    # Dimension Annotations along Bed Edges
    # Width annotation at bottom
    dim_y = ty(-7.0)
    svg.append(f'  <!-- Dimensions -->')
    svg.append(f'  <line x1="{tx(0)}" y1="{dim_y}" x2="{tx(bed_w)}" y2="{dim_y}" stroke="#888888" stroke-width="1" />')
    svg.append(f'  <polyline points="{tx(0) + 6},{dim_y - 3} {tx(0)},{dim_y} {tx(0) + 6},{dim_y + 3}" fill="none" stroke="#888888" stroke-width="1" />')
    svg.append(f'  <polyline points="{tx(bed_w) - 6},{dim_y - 3} {tx(bed_w)},{dim_y} {tx(bed_w) - 6},{dim_y + 3}" fill="none" stroke="#888888" stroke-width="1" />')
    svg.append(f'  <text x="{tx(bed_w / 2.0) - 40}" y="{dim_y - 4}" fill="#ffffff" font-size="12" font-family="sans-serif" font-weight="bold">BED WIDTH: {bed_w:.1f} mm</text>')

    # Height annotation at right
    dim_x = tx(bed_w + 7.0)
    svg.append(f'  <line x1="{dim_x}" y1="{ty(0)}" x2="{dim_x}" y2="{ty(bed_h)}" stroke="#888888" stroke-width="1" />')
    svg.append(f'  <polyline points="{dim_x - 3},{ty(0) - 6} {dim_x},{ty(0)} {dim_x + 3},{ty(0) - 6}" fill="none" stroke="#888888" stroke-width="1" />')
    svg.append(f'  <polyline points="{dim_x - 3},{ty(bed_h) + 6} {dim_x},{ty(bed_h)} {dim_x + 3},{ty(bed_h) + 6}" fill="none" stroke="#888888" stroke-width="1" />')
    svg.append(f'  <text x="{dim_x + 6}" y="{ty(bed_h / 2.0) + 4}" fill="#ffffff" font-size="12" font-family="sans-serif" font-weight="bold" transform="rotate(90 {dim_x + 6} {ty(bed_h / 2.0)})">BED HEIGHT: {bed_h:.1f} mm</text>')

    svg.append('</svg>')

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))
    print(f"  [+] Saved Cutting-Mat SVG blueprint: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate CNC Bed Cutting-Mat Guides (10mm grid, 30/45/60 angles, switch pocket) in G-code & SVG",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--bed-w", type=float, default=275.0, help="Machine bed width in X (mm)")
    parser.add_argument("--bed-h", type=float, default=170.0, help="Machine bed height in Y (mm)")
    parser.add_argument("--grid-step", type=float, default=10.0, help="Grid line spacing (mm)")
    parser.add_argument("--depth", type=float, default=0.25, help="V-bit engraving depth in plywood (mm)")
    parser.add_argument("--feed", type=float, default=450.0, help="XY feed rate (mm/min)")
    parser.add_argument("--plunge", type=float, default=120.0, help="Plunge rate (mm/min)")
    parser.add_argument("--spindle", type=int, default=10000, help="Spindle speed (RPM)")
    parser.add_argument("--safe-z", type=float, default=3.0, help="Safe Z height (mm)")

    # Probed switch coordinates (shifted 10 mm towards front: Y1=150.003, Y2=130.003)
    parser.add_argument("--switch-x1", type=float, default=275.002, help="Probed switch X1")
    parser.add_argument("--switch-y1", type=float, default=150.003, help="Switch Y1 (shifted 10mm front from 160.003)")
    parser.add_argument("--switch-x2", type=float, default=269.502, help="Probed switch X2")
    parser.add_argument("--switch-y2", type=float, default=130.003, help="Switch Y2 (shifted 10mm front from 140.003)")
    parser.add_argument("--no-switch-box", action="store_true", help="Omit switch pocket outline from bed G-code")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument("--output-dir", type=str, default=script_dir, help="Output directory")

    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    print("=================================================================")
    print("  CNC Plywood Bed Cutting-Mat Guides Generator")
    print("=================================================================")
    print(f"  Bed Dimensions    : {args.bed_w:.1f} mm (X) x {args.bed_h:.1f} mm (Y)")
    print(f"  Grid Spacing      : {args.grid_step:.1f} mm (Major every 50mm)")
    print(f"  Angle Guides      : 30°, 45°, 60°")
    print(f"  V-Bit Engrave Depth: -{args.depth:.2f} mm (Feed: {args.feed:.0f} mm/min, {args.spindle} RPM)")
    print(f"  Limit Switch Loc  : ({args.switch_x1:.3f}, {args.switch_y1:.3f}) to ({args.switch_x2:.3f}, {args.switch_y2:.3f})")
    print("-----------------------------------------------------------------")

    # Generate geometric paths
    data = generate_bed_geometry(
        bed_w=args.bed_w, bed_h=args.bed_h, grid_step=args.grid_step,
        switch_p1=(args.switch_x1, args.switch_y1),
        switch_p2=(args.switch_x2, args.switch_y2)
    )

    # 1. Output G-code
    gcode_path = os.path.join(args.output_dir, "bed_guides_cutting_mat.gcode")
    generate_cutting_mat_gcode(
        data, gcode_path, cut_depth=args.depth, feed_rate=args.feed,
        plunge_rate=args.plunge, spindle_speed=args.spindle, safe_z=args.safe_z,
        engrave_switch=(not args.no_switch_box)
    )

    # 2. Output SVG Blueprint
    svg_path = os.path.join(args.output_dir, "bed_guides_cutting_mat.svg")
    generate_cutting_mat_svg(data, bed_w=args.bed_w, bed_h=args.bed_h, output_path=svg_path)

    print("-----------------------------------------------------------------")
    print("  Generated Bed Guides Files:")
    print(f"    G-Code : {gcode_path}")
    print(f"    SVG    : {svg_path}")
    print("  Done! Perfect for CNC bed alignment and repositioning.")
    print("=================================================================")


if __name__ == "__main__":
    main()
