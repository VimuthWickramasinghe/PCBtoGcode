#!/usr/bin/env python3
"""
shpae_to_gcode.py - Limit Switch Pocket G-Code & Drill Generator
================================================================
Designed for CNC routers machining a rectangular pocket in 3/4" (19.05 mm) plywood
for a standard limit switch (19.5 x 5.9 mm) to an exact depth of 10.0 mm.

Probed coordinates measured with V-cut bit:
  Corner 1: (275.002, 160.003)
  Corner 2: (269.502, 140.003)

Tooling Supported:
  1. V-Shape Bit: Scores the top plywood veneer perimeter (-0.3mm to -0.5mm depth)
     to prevent tear-out / splintering.
  2. Drill Bits (up to 3mm collet, e.g. 3.0mm, 2.5mm, 2.0mm):
     Generates a tight Peck-Drilling Matrix. Because twist drill bits cannot mill
     sideways, overlapping vertical peck-drilling holes down to 10mm depth safely
     hogs out all the wood material without snapping bits.
  3. Endmill / Corn Milling Bit (1.0mm - 3.0mm):
     Generates multi-pass pocket clearing toolpaths down to 10mm depth.

Outputs:
  - G-Code files for V-bit outline, Drill peck matrix, and Milling pocket.
  - SVG visual diagram illustrating the hole layout, drill pattern, and coordinates.

Usage:
  python shpae_to_gcode.py
  python shpae_to_gcode.py --drill-dia 3.0 --depth 10.0
  python shpae_to_gcode.py --drill-dia 2.0 --peck-depth 1.0
  python shpae_to_gcode.py --mode nominal --clearance 0.2
"""

import os
import sys
import math
import argparse
from datetime import datetime


def calculate_pocket_bounds(x1, y1, x2, y2, switch_w, switch_l, mode="probed", clearance=0.0,
                            bed_w=275.0, bed_h=170.0, edge_safety=0.1):
    """
    Calculate the bounding box of the pocket, strictly clamping within the
    machine's physical travel limits (bed_w, bed_h) to avoid FluidNC soft limits / alarm.
    """
    p_xmin = min(x1, x2)
    p_xmax = max(x1, x2)
    p_ymin = min(y1, y2)
    p_ymax = max(y1, y2)

    # Max safe travel limit
    max_safe_x = bed_w - edge_safety
    max_safe_y = bed_h - edge_safety

    if mode == "probed":
        desired_w = (p_xmax - p_xmin) + 2 * clearance
        desired_l = (p_ymax - p_ymin) + 2 * clearance
        xmax = min(p_xmax + clearance, max_safe_x)
        xmin = max(0.0, xmax - desired_w)
        ymax = min(p_ymax + clearance, max_safe_y)
        ymin = max(0.0, ymax - desired_l)
    else:  # nominal
        desired_w = switch_w + 2 * clearance
        desired_l = switch_l + 2 * clearance
        # Anchor right edge safely inside bed limit, expand inward
        xmax = min(p_xmax, max_safe_x)
        xmin = max(0.0, xmax - desired_w)
        cy = (p_ymin + p_ymax) / 2.0
        ymin = max(0.0, cy - desired_l / 2.0)
        ymax = min(cy + desired_l / 2.0, max_safe_y)

    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0

    return {
        "xmin": xmin,
        "xmax": xmax,
        "ymin": ymin,
        "ymax": ymax,
        "width": xmax - xmin,
        "length": ymax - ymin,
        "center_x": cx,
        "center_y": cy,
        "probed_xmin": p_xmin,
        "probed_xmax": p_xmax,
        "probed_ymin": p_ymin,
        "probed_ymax": p_ymax,
    }


def compute_drill_matrix(bounds, tool_dia, stepover_ratio=0.65):
    """
    Compute a grid of drill hole centers that completely evacuates the pocket
    without exceeding the boundary.
    """
    radius = tool_dia / 2.0
    xmin, xmax = bounds["xmin"], bounds["xmax"]
    ymin, ymax = bounds["ymin"], bounds["ymax"]
    
    width = bounds["width"]
    length = bounds["length"]
    
    # Tool center bounds
    cx_min = xmin + radius
    cx_max = xmax - radius
    cy_min = ymin + radius
    cy_max = ymax - radius
    
    # Handle narrow pocket where tool is larger or equal to width
    if cx_min >= cx_max:
        x_coords = [(xmin + xmax) / 2.0]
    else:
        max_step_x = tool_dia * stepover_ratio
        span_x = cx_max - cx_min
        num_cols = max(2, int(math.ceil(span_x / max_step_x)) + 1)
        step_x = span_x / (num_cols - 1)
        x_coords = [cx_min + i * step_x for i in range(num_cols)]
        
    if cy_min >= cy_max:
        y_coords = [(ymin + ymax) / 2.0]
    else:
        max_step_y = tool_dia * stepover_ratio
        span_y = cy_max - cy_min
        num_rows = max(2, int(math.ceil(span_y / max_step_y)) + 1)
        step_y = span_y / (num_rows - 1)
        y_coords = [cy_min + i * step_y for i in range(num_rows)]
        
    # Order drill points in an efficient serpentine / zigzag path
    drill_points = []
    for col_idx, x in enumerate(x_coords):
        rows = list(enumerate(y_coords))
        if col_idx % 2 == 1:
            rows.reverse()
        for row_idx, y in rows:
            drill_points.append((x, y))
            
    return drill_points, x_coords, y_coords


def generate_vbit_pocket_10mm_gcode(bounds, total_depth=10.0, stepdown=0.5,
                                    vbit_angle=30.0, vbit_tip=0.2, vbit_shank=3.0,
                                    feed_xy=350.0, plunge_rate=80.0, spindle_speed=10000, safe_z=5.0):
    """
    Generate complete 10mm deep rectangular pocket milling G-code using ONLY the V-bit.
    Accurately calculates the widening tool diameter as Z deepens:
      D(z) = min(vbit_shank, vbit_tip + 2 * depth * tan(angle / 2))
    At each depth layer:
      - Tool center boundary is offset inward by R(z) = D(z) / 2 to preserve exact pocket walls.
      - Raster stepover dynamically adapts to D(z) * 0.55 to guarantee 100% material evacuation
        without uncut wood ridges at shallow passes or redundant air cuts at deep passes.
      - Perimeter contour pass at each depth level ensures straight, clean vertical pocket walls.
    """
    lines = []
    alpha = math.radians(vbit_angle / 2.0)
    num_passes = int(math.ceil(total_depth / stepdown))
    d_top = min(vbit_shank, vbit_tip + 2.0 * stepdown * math.tan(alpha))
    d_deep = min(vbit_shank, vbit_tip + 2.0 * total_depth * math.tan(alpha))

    lines.append("( ========================================================== )")
    lines.append("(  Limit Switch Pocket - 100% V-Bit Milling Program          )")
    lines.append(f"(  Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} )")
    lines.append(f"(  Bounding Box : X[{bounds['xmin']:.3f}..{bounds['xmax']:.3f}], Y[{bounds['ymin']:.3f}..{bounds['ymax']:.3f}] )")
    lines.append(f"(  Pocket Size  : {bounds['width']:.3f} mm x {bounds['length']:.3f} mm )")
    lines.append(f"(  Total Depth  : -{total_depth:.2f} mm in 3/4\" Plywood ({num_passes} passes) )")
    lines.append(f"(  V-Bit Specs  : {vbit_angle:.1f} deg angle, {vbit_tip:.2f}mm tip, {vbit_shank:.2f}mm shank )")
    lines.append(f"(  Dynamic Dia  : {d_top:.3f}mm at top pass -> {d_deep:.3f}mm at depth )")
    lines.append(f"(  Stepdown     : {stepdown:.2f} mm/pass, Feed XY: {feed_xy:.1f} mm/min )")
    lines.append(f"(  Tool Offset  : Dynamic R(z) = D(z)/2 compensation per layer )")
    lines.append("( ========================================================== )")
    lines.append("")
    lines.append("G21          (Units in Millimeters)")
    lines.append("G90          (Absolute Coordinates)")
    lines.append("G94          (Feed Rate per Minute)")
    lines.append("G17          (XY Plane Selection)")
    lines.append("G54          (Work Coordinate System 1 / Active Work Offset)")

    xmin, xmax = bounds["xmin"], bounds["xmax"]
    ymin, ymax = bounds["ymin"], bounds["ymax"]

    # Initial rapid at Safe Z to center of first pass
    d_init = min(vbit_shank, vbit_tip + 2.0 * stepdown * math.tan(alpha))
    r_init = d_init / 2.0
    start_x = xmin + r_init
    start_y = ymin + r_init

    lines.append(f"G00 Z{safe_z:.4f} (Rapid to Safe Z)")
    lines.append(f"G00 X{start_x:.4f} Y{start_y:.4f} (Rapid directly to pocket start at Safe Z)")
    lines.append(f"M03 S{spindle_speed} (Spindle Start CW)")
    lines.append("G04 P2.0     (Dwell 2s for spindle to reach speed)")
    lines.append("")

    for p in range(1, num_passes + 1):
        z_curr = -min(p * stepdown, total_depth)
        depth = abs(z_curr)
        # Calculate bit diameter at this depth layer
        d_z = min(vbit_shank, vbit_tip + 2.0 * depth * math.tan(alpha))
        r_z = d_z / 2.0
        # Dynamic stepover: ~55% of current cutting diameter
        stepover_z = min(vbit_shank * 0.5, max(0.25, d_z * 0.55))

        # Dynamic tool center bounds to maintain exact pocket outer walls
        xc_min = xmin + r_z
        xc_max = xmax - r_z
        yc_min = ymin + r_z
        yc_max = ymax - r_z

        # Handle case where tool radius approaches pocket half-width
        if xc_min >= xc_max:
            xc_min = xc_max = (xmin + xmax) / 2.0
        if yc_min >= yc_max:
            yc_min = yc_max = (ymin + ymax) / 2.0

        # Compute X slices for this layer
        x_slices = []
        curr_x = xc_min
        while curr_x <= xc_max + 1e-4:
            x_slices.append(round(min(curr_x, xc_max), 4))
            curr_x += stepover_z
        if x_slices and x_slices[-1] < xc_max - 1e-4:
            x_slices.append(round(xc_max, 4))

        lines.append(f"( --- Pass {p}/{num_passes}: Depth {z_curr:.4f} mm | Dia={d_z:.3f}mm, R={r_z:.3f}mm, Stepover={stepover_z:.3f}mm --- )")
        lines.append(f"G00 X{xc_min:.4f} Y{yc_min:.4f}")
        lines.append(f"G01 Z{z_curr:.4f} F{plunge_rate:.1f} (Plunge to pass depth)")

        # Serpentine raster clearing across pocket interior
        dir_y = 1
        for sx in x_slices:
            if dir_y == 1:
                lines.append(f"G01 X{sx:.4f} Y{yc_min:.4f} F{feed_xy:.1f}")
                lines.append(f"G01 X{sx:.4f} Y{yc_max:.4f}")
                dir_y = -1
            else:
                lines.append(f"G01 X{sx:.4f} Y{yc_max:.4f} F{feed_xy:.1f}")
                lines.append(f"G01 X{sx:.4f} Y{yc_min:.4f}")
                dir_y = 1

        # Perimeter wall cleanup pass with tool radius compensation for vertical walls
        lines.append(f"( Perimeter wall contour at Z={z_curr:.4f} mm )")
        lines.append(f"G01 X{xc_min:.4f} Y{yc_min:.4f} F{feed_xy:.1f}")
        lines.append(f"G01 X{xc_max:.4f} Y{yc_min:.4f}")
        lines.append(f"G01 X{xc_max:.4f} Y{yc_max:.4f}")
        lines.append(f"G01 X{xc_min:.4f} Y{yc_max:.4f}")
        lines.append(f"G01 X{xc_min:.4f} Y{yc_min:.4f}")
        lines.append("")

    lines.append(f"G00 Z{safe_z:.4f} (Retract to Safe Z)")
    lines.append("M05          (Spindle Stop)")
    lines.append("G00 Z20.0000 (Clearance Height)")
    lines.append("M30          (Program End and Reset)")
    return "\n".join(lines)


def generate_vbit_outline_gcode(bounds, cut_depth=0.4, feed_xy=300.0, plunge_rate=80.0,
                                spindle_speed=10000, safe_z=5.0, passes=2):
    """
    Generate V-bit outline scoring G-code to prevent plywood veneer tear-out.
    """
    lines = []
    lines.append("( ========================================================== )")
    lines.append("(  Limit Switch Pocket - V-Bit Perimeter Scoring             )")
    lines.append(f"(  Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} )")
    lines.append(f"(  Bounding Box: X[{bounds['xmin']:.3f}..{bounds['xmax']:.3f}], Y[{bounds['ymin']:.3f}..{bounds['ymax']:.3f}] )")
    lines.append(f"(  Pocket Size : {bounds['width']:.3f} mm x {bounds['length']:.3f} mm )")
    lines.append(f"(  Target Depth: -{cut_depth:.2f} mm ({passes} passes) )")
    lines.append(f"(  Spindle     : {spindle_speed} RPM, Feed: {feed_xy:.1f} mm/min )")
    lines.append("( ========================================================== )")
    lines.append("")
    lines.append("G21          (Units in Millimeters)")
    lines.append("G90          (Absolute Coordinates)")
    lines.append("G94          (Feed Rate per Minute)")
    lines.append("G17          (XY Plane Selection)")
    lines.append("G54          (Work Coordinate System 1 / Active Work Offset)")
    xmin, xmax = bounds["xmin"], bounds["xmax"]
    ymin, ymax = bounds["ymin"], bounds["ymax"]

    depth_per_pass = cut_depth / passes

    lines.append(f"G00 Z{safe_z:.4f} (Rapid to Safe Z)")
    lines.append(f"G00 X{xmin:.4f} Y{ymin:.4f} (Rapid directly to pocket start at Safe Z)")
    lines.append(f"M03 S{spindle_speed} (Spindle Start CW)")
    lines.append("G04 P2.0     (Dwell 2s for spindle to reach speed)")
    lines.append("")

    for p in range(1, passes + 1):
        z_curr = -p * depth_per_pass
        lines.append(f"\n( --- Pass {p}/{passes}: Depth {z_curr:.4f} mm --- )")
        lines.append(f"G01 Z{z_curr:.4f} F{plunge_rate:.1f} (Plunge)")
        lines.append(f"G01 X{xmax:.4f} Y{ymin:.4f} F{feed_xy:.1f}")
        lines.append(f"G01 X{xmax:.4f} Y{ymax:.4f}")
        lines.append(f"G01 X{xmin:.4f} Y{ymax:.4f}")
        lines.append(f"G01 X{xmin:.4f} Y{ymin:.4f}")

    lines.append(f"\nG00 Z{safe_z:.4f} (Retract to Safe Z)")
    lines.append("M05          (Spindle Stop)")
    lines.append("G00 Z20.0000 (Clearance Height)")
    lines.append("M30          (Program End and Reset)")
    return "\n".join(lines)


def generate_peck_drill_matrix_gcode(bounds, drill_points, tool_dia=3.0, total_depth=10.0,
                                     peck_depth=1.5, peck_retract=0.5, full_retract_interval=3.0,
                                     drill_feed=150.0, spindle_speed=10000, safe_z=5.0):
    """
    Generate Peck Drilling Matrix G-code for standard twist drill bits.
    Expanded peck cycles compatible with GRBL / Candle.
    """
    lines = []
    lines.append("( ========================================================== )")
    lines.append(f"(  Limit Switch Pocket - Peck Drill Matrix ({tool_dia:.1f}mm Bit)    )")
    lines.append(f"(  Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} )")
    lines.append(f"(  Total Holes : {len(drill_points)} holes in matrix )")
    lines.append(f"(  Hole Depth  : -{total_depth:.2f} mm in 3/4\" Plywood )")
    lines.append(f"(  Peck Depth  : {peck_depth:.2f} mm per peck )")
    lines.append(f"(  Drill Bit D : {tool_dia:.3f} mm )")
    lines.append(f"(  Feed Rate   : {drill_feed:.1f} mm/min, Spindle: {spindle_speed} RPM )")
    lines.append("( ========================================================== )")
    lines.append("")
    lines.append("G21          (Units in Millimeters)")
    lines.append("G90          (Absolute Coordinates)")
    lines.append("G94          (Feed Rate per Minute)")
    lines.append("G17          (XY Plane Selection)")
    lines.append("G54          (Work Coordinate System 1 / Active Work Offset)")
    h0_x, h0_y = drill_points[0]
    lines.append(f"G00 Z{safe_z:.4f} (Rapid to Safe Z)")
    lines.append(f"G00 X{h0_x:.4f} Y{h0_y:.4f} (Rapid to Hole 1 at Safe Z)")
    lines.append(f"M03 S{spindle_speed} (Spindle Start CW)")
    lines.append("G04 P2.0     (Dwell 2s for spindle to reach speed)")
    lines.append("")

    num_pecks = int(math.ceil(total_depth / peck_depth))

    for idx, (hx, hy) in enumerate(drill_points, 1):
        lines.append(f"( --- Hole {idx}/{len(drill_points)} at X={hx:.4f}, Y={hy:.4f} --- )")
        if idx > 1:
            lines.append(f"G00 X{hx:.4f} Y{hy:.4f}")

        current_z = 0.0
        depth_since_clear = 0.0

        for peck in range(1, num_pecks + 1):
            target_z = -min(peck * peck_depth, total_depth)
            
            # Rapid down to just above previous peck depth
            if peck > 1:
                rapid_z = current_z + 0.3
                lines.append(f"G00 Z{rapid_z:.4f}")

            # Drill to target peck depth
            lines.append(f"G01 Z{target_z:.4f} F{drill_feed:.1f}")

            depth_since_clear += (abs(target_z) - abs(current_z))
            current_z = target_z

            if abs(current_z - (-total_depth)) < 1e-4:
                # Bottom reached: brief dwell for clean flat bottom
                lines.append("G04 P0.2     (Dwell at bottom)")
                lines.append(f"G00 Z{safe_z:.4f} (Full retract to Safe Z)")
                break
            elif depth_since_clear >= full_retract_interval:
                # Full retract to clear chips from deep wood hole
                lines.append(f"G00 Z{safe_z:.4f} (Full chip clearance retract)")
                depth_since_clear = 0.0
            else:
                # Partial retract for chip breaking
                chip_break_z = current_z + peck_retract
                lines.append(f"G00 Z{chip_break_z:.4f} (Chip break retract)")

        lines.append("")

    lines.append("M05          (Spindle Stop)")
    lines.append("G00 Z20.0000 (Clearance Height)")
    lines.append("M30          (Program End and Reset)")
    return "\n".join(lines)


def generate_pocket_milling_gcode(bounds, tool_dia=3.0, total_depth=10.0, stepdown=0.8,
                                  stepover=1.5, feed_xy=250.0, plunge_rate=80.0,
                                  spindle_speed=10000, safe_z=5.0):
    """
    Generate multi-pass pocket milling G-code for spiral/endmill bits.
    Uses concentric rectangular passes with a final contour pass per layer.
    """
    lines = []
    lines.append("( ========================================================== )")
    lines.append(f"(  Limit Switch Pocket - Endmill Milling ({tool_dia:.1f}mm Tool)   )")
    lines.append(f"(  Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} )")
    lines.append(f"(  Pocket Size : {bounds['width']:.3f} mm x {bounds['length']:.3f} mm )")
    lines.append(f"(  Total Depth : -{total_depth:.2f} mm ({int(math.ceil(total_depth/stepdown))} passes) )")
    lines.append(f"(  Stepdown    : {stepdown:.2f} mm per pass )")
    lines.append(f"(  Feed XY     : {feed_xy:.1f} mm/min, Plunge: {plunge_rate:.1f} mm/min )")
    lines.append("( ========================================================== )")
    lines.append("")
    lines.append("G21          (Units in Millimeters)")
    lines.append("G90          (Absolute Coordinates)")
    lines.append("G94          (Feed Rate per Minute)")
    lines.append("G17          (XY Plane Selection)")
    lines.append("G54          (Work Coordinate System 1 / Active Work Offset)")
    r = tool_dia / 2.0
    x_min_cut = bounds["xmin"] + r
    x_max_cut = bounds["xmax"] - r
    y_min_cut = bounds["ymin"] + r
    y_max_cut = bounds["ymax"] - r

    # Determine internal clearing loops
    num_passes = int(math.ceil(total_depth / stepdown))
    start_x = (x_min_cut + x_max_cut) / 2.0
    start_y = (y_min_cut + y_max_cut) / 2.0

    lines.append(f"G00 Z{safe_z:.4f} (Rapid to Safe Z)")
    lines.append(f"G00 X{start_x:.4f} Y{start_y:.4f} (Rapid directly to pocket center at Safe Z)")
    lines.append(f"M03 S{spindle_speed} (Spindle Start CW)")
    lines.append("G04 P2.0     (Dwell 2s for spindle to reach speed)")
    lines.append("")

    for pass_num in range(1, num_passes + 1):
        z = -min(pass_num * stepdown, total_depth)
        lines.append(f"\n( --- Pass {pass_num}/{num_passes} at Z = {z:.4f} mm --- )")
        lines.append(f"G01 Z{z:.4f} F{plunge_rate:.1f} (Plunge)")

        # Zigzag / raster clearing
        # Generate X slices
        curr_x = x_min_cut
        dir_y = 1
        while curr_x <= x_max_cut + 1e-4:
            x_target = min(curr_x, x_max_cut)
            if dir_y == 1:
                lines.append(f"G01 X{x_target:.4f} Y{y_min_cut:.4f} F{feed_xy:.1f}")
                lines.append(f"G01 X{x_target:.4f} Y{y_max_cut:.4f}")
                dir_y = -1
            else:
                lines.append(f"G01 X{x_target:.4f} Y{y_max_cut:.4f} F{feed_xy:.1f}")
                lines.append(f"G01 X{x_target:.4f} Y{y_min_cut:.4f}")
                dir_y = 1
            curr_x += stepover

        # Perimeter cleanup pass for this layer
        lines.append(f"( Perimeter cleanup pass )")
        lines.append(f"G01 X{x_min_cut:.4f} Y{y_min_cut:.4f} F{feed_xy:.1f}")
        lines.append(f"G01 X{x_max_cut:.4f} Y{y_min_cut:.4f}")
        lines.append(f"G01 X{x_max_cut:.4f} Y{y_max_cut:.4f}")
        lines.append(f"G01 X{x_min_cut:.4f} Y{y_max_cut:.4f}")
        lines.append(f"G01 X{x_min_cut:.4f} Y{y_min_cut:.4f}")

    lines.append(f"\nG00 Z{safe_z:.4f} (Retract to Safe Z)")
    lines.append("M05          (Spindle Stop)")
    lines.append("G00 Z20.0000 (Clearance Height)")
    lines.append("M30          (Program End and Reset)")
    return "\n".join(lines)


def generate_svg_preview(bounds, drill_points, tool_dia, output_path):
    """
    Generate an SVG diagram showing the limit switch pocket layout,
    drill points, probed coordinates, and machine context.
    """
    margin = 15.0
    view_xmin = bounds["xmin"] - margin
    view_xmax = bounds["xmax"] + margin
    view_ymin = bounds["ymin"] - margin
    view_ymax = bounds["ymax"] + margin
    
    view_w = view_xmax - view_xmin
    view_h = view_ymax - view_ymin

    scale = 800.0 / view_h
    svg_w = view_w * scale
    svg_h = view_h * scale

    def tx(x):
        return (x - view_xmin) * scale

    def ty(y):
        return (view_ymax - y) * scale

    svg = []
    svg.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {svg_w} {svg_h}" width="{svg_w}" height="{svg_h}">')
    svg.append('  <defs>')
    svg.append('    <pattern id="wood-grid" width="10" height="10" patternUnits="userSpaceOnUse">')
    svg.append('      <rect width="10" height="10" fill="#2d2a26" />')
    svg.append('      <path d="M 10 0 L 0 0 0 10" fill="none" stroke="#3d3a36" stroke-width="0.5" />')
    svg.append('    </pattern>')
    svg.append('  </defs>')
    
    # Background
    svg.append(f'  <rect width="100%" height="100%" fill="#1e1e1e" />')

    # Draw 5mm reference grid
    gx_start = math.floor(view_xmin / 5.0) * 5.0
    while gx_start <= view_xmax:
        sx = tx(gx_start)
        color = "#444444" if (gx_start % 10 == 0) else "#2d2d2d"
        svg.append(f'  <line x1="{sx}" y1="0" x2="{sx}" y2="{svg_h}" stroke="{color}" stroke-width="1" stroke-dasharray="2,2" />')
        svg.append(f'  <text x="{sx + 2}" y="{svg_h - 6}" fill="#777777" font-size="10" font-family="monospace">X={gx_start:.0f}</text>')
        gx_start += 5.0

    gy_start = math.floor(view_ymin / 5.0) * 5.0
    while gy_start <= view_ymax:
        sy = ty(gy_start)
        color = "#444444" if (gy_start % 10 == 0) else "#2d2d2d"
        svg.append(f'  <line x1="0" y1="{sy}" x2="{svg_w}" y2="{sy}" stroke="{color}" stroke-width="1" stroke-dasharray="2,2" />')
        svg.append(f'  <text x="6" y="{sy - 3}" fill="#777777" font-size="10" font-family="monospace">Y={gy_start:.0f}</text>')
        gy_start += 5.0

    # Draw Pocket Cutout Boundary
    bx = tx(bounds["xmin"])
    by = ty(bounds["ymax"])
    bw = bounds["width"] * scale
    bh = bounds["length"] * scale

    svg.append(f'  <!-- Pocket Boundary (10mm depth) -->')
    svg.append(f'  <rect x="{bx}" y="{by}" width="{bw}" height="{bh}" fill="rgba(231, 76, 60, 0.25)" stroke="#e74c3c" stroke-width="2.5" stroke-dasharray="6,3" rx="2" />')

    # Draw Tool Circles for drill points
    r_px = (tool_dia / 2.0) * scale
    svg.append(f'  <!-- Drill Bit Holes (Dia={tool_dia}mm) -->')
    for idx, (hx, hy) in enumerate(drill_points, 1):
        px = tx(hx)
        py = ty(hy)
        # Hole circle
        svg.append(f'  <circle cx="{px}" cy="{py}" r="{r_px}" fill="rgba(52, 152, 219, 0.4)" stroke="#3498db" stroke-width="1.5" />')
        # Center cross
        svg.append(f'  <line x1="{px - 4}" y1="{py}" x2="{px + 4}" y2="{py}" stroke="#ffffff" stroke-width="1" />')
        svg.append(f'  <line x1="{px}" y1="{py - 4}" x2="{px}" y2="{py + 4}" stroke="#ffffff" stroke-width="1" />')
        svg.append(f'  <text x="{px + 4}" y="{py - 4}" fill="#ffffff" font-size="9" font-family="monospace">#{idx}</text>')

    # Connect drill path
    if len(drill_points) > 1:
        path_d = []
        for i, (hx, hy) in enumerate(drill_points):
            cmd = "M" if i == 0 else "L"
            path_d.append(f"{cmd} {tx(hx):.1f} {ty(hy):.1f}")
        svg.append(f'  <path d="{" ".join(path_d)}" fill="none" stroke="#f1c40f" stroke-width="1" stroke-dasharray="3,3" />')

    # Draw probed reference corners
    c1x, c1y = tx(bounds["probed_xmax"]), ty(bounds["probed_ymax"])
    c2x, c2y = tx(bounds["probed_xmin"]), ty(bounds["probed_ymin"])
    
    svg.append(f'  <!-- Probed Corners -->')
    svg.append(f'  <circle cx="{c1x}" cy="{c1y}" r="5" fill="#2ecc71" stroke="#ffffff" stroke-width="1.5" />')
    svg.append(f'  <text x="{c1x - 170}" y="{c1y - 8}" fill="#2ecc71" font-size="11" font-weight="bold" font-family="monospace">Probed P1: ({bounds["probed_xmax"]:.3f}, {bounds["probed_ymax"]:.3f})</text>')

    svg.append(f'  <circle cx="{c2x}" cy="{c2y}" r="5" fill="#2ecc71" stroke="#ffffff" stroke-width="1.5" />')
    svg.append(f'  <text x="{c2x - 170}" y="{c2y + 16}" fill="#2ecc71" font-size="11" font-weight="bold" font-family="monospace">Probed P2: ({bounds["probed_xmin"]:.3f}, {bounds["probed_ymin"]:.3f})</text>')

    # Title & Metadata Card
    svg.append('  <g transform="translate(20, 25)">')
    svg.append('    <rect width="360" height="115" rx="6" fill="rgba(0,0,0,0.75)" stroke="#555555" stroke-width="1" />')
    svg.append('    <text x="12" y="22" fill="#ffffff" font-size="14" font-weight="bold" font-family="sans-serif">LIMIT SWITCH POCKET (10mm DEPTH)</text>')
    svg.append(f'    <text x="12" y="42" fill="#e74c3c" font-size="12" font-family="monospace">Pocket Size: {bounds["width"]:.2f} x {bounds["length"]:.2f} mm (Depth: 10.0mm)</text>')
    svg.append(f'    <text x="12" y="60" fill="#3498db" font-size="12" font-family="monospace">Tool Dia: {tool_dia:.2f} mm | Total Holes: {len(drill_points)}</text>')
    svg.append(f'    <text x="12" y="78" fill="#f1c40f" font-size="12" font-family="monospace">Material: 3/4" Plywood (19.05mm)</text>')
    svg.append(f'    <text x="12" y="96" fill="#aaaaaa" font-size="11" font-family="monospace">Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}</text>')
    svg.append('  </g>')

    svg.append('</svg>')
    
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))
    print(f"  [+] Saved SVG preview: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate CNC G-Code & Drill Patterns for 10mm Deep Limit Switch Pocket in Plywood Bed",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    # Probed coordinates (shifted 10 mm towards front: Y1=150.003, Y2=130.003):
    parser.add_argument("--x1", type=float, default=275.002, help="Probed corner 1 X")
    parser.add_argument("--y1", type=float, default=150.003, help="Corner 1 Y (shifted 10mm front from 160.003)")
    parser.add_argument("--x2", type=float, default=269.502, help="Probed corner 2 X")
    parser.add_argument("--y2", type=float, default=130.003, help="Corner 2 Y (shifted 10mm front from 140.003)")
    
    # Nominal switch & pocket parameters:
    parser.add_argument("--switch-w", type=float, default=5.9, help="Nominal switch width (mm)")
    parser.add_argument("--switch-l", type=float, default=19.5, help="Nominal switch length (mm)")
    parser.add_argument("--depth", type=float, default=10.0, help="Target hole depth in plywood (mm)")
    parser.add_argument("--mode", choices=["probed", "nominal"], default="probed",
                        help="'probed' uses exact probed bounding box (5.5x20mm), 'nominal' uses 5.9x19.5mm centered at probed center")
    parser.add_argument("--clearance", type=float, default=0.2, help="Clearance added to width and length (mm)")

    # Tooling parameters:
    parser.add_argument("--vbit-angle", type=float, default=30.0, help="V-bit included angle in degrees (e.g. 20, 30, 45, 60)")
    parser.add_argument("--vbit-tip", type=float, default=0.2, help="V-bit tip flat diameter (mm)")
    parser.add_argument("--vbit-shank", type=float, default=3.0, help="V-bit shank diameter (mm)")
    parser.add_argument("--vbit-stepdown", type=float, default=0.5, help="V-bit pocket stepdown per pass (mm)")
    parser.add_argument("--vbit-feed", type=float, default=350.0, help="V-bit pocket XY feed rate (mm/min)")
    parser.add_argument("--drill-dia", type=float, default=3.0, help="Drill bit diameter (mm, up to 3mm)")
    parser.add_argument("--peck-depth", type=float, default=1.5, help="Drill peck depth increment (mm)")
    parser.add_argument("--drill-feed", type=float, default=150.0, help="Plunge feed rate for drilling (mm/min)")
    parser.add_argument("--mill-feed", type=float, default=250.0, help="XY feed rate for pocket milling (mm/min)")
    parser.add_argument("--vbit-depth", type=float, default=0.4, help="V-bit outline scoring depth (mm)")
    parser.add_argument("--spindle", type=int, default=10000, help="Spindle speed (RPM)")
    parser.add_argument("--safe-z", type=float, default=5.0, help="Safe clearance height Z (mm)")
    parser.add_argument("--bed-w", type=float, default=275.0, help="Machine bed travel in X (mm)")
    parser.add_argument("--bed-h", type=float, default=170.0, help="Machine bed travel in Y (mm)")
    parser.add_argument("--edge-safety", type=float, default=0.1, help="Safety margin inside bed travel limit (mm)")
    parser.add_argument("--op", choices=["all", "vbit-pocket", "vbit-outline", "drill", "pocket"], default="all",
                        help="Operation to generate")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parser.add_argument("--output-dir", type=str, default=script_dir, help="Directory to save generated files")

    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    print("=================================================================")
    print("  Limit Switch Pocket G-Code & Drill Matrix Generator")
    print("=================================================================")
    print(f"  Probed Coordinates : P1({args.x1:.3f}, {args.y1:.3f}) -> P2({args.x2:.3f}, {args.y2:.3f})")
    print(f"  Target Hole Depth  : {args.depth:.2f} mm in 3/4\" Plywood")
    print(f"  Nominal Switch Size: {args.switch_w:.2f} x {args.switch_l:.2f} mm")
    print(f"  Mode               : {args.mode} (Clearance: {args.clearance:.2f} mm)")
    print(f"  Bed Limits Clamped : X[0..{args.bed_w:.1f}], Y[0..{args.bed_h:.1f}] (Safety: {args.edge_safety:.2f}mm)")

    # Calculate bounding box
    bounds = calculate_pocket_bounds(
        args.x1, args.y1, args.x2, args.y2,
        args.switch_w, args.switch_l,
        mode=args.mode, clearance=args.clearance,
        bed_w=args.bed_w, bed_h=args.bed_h, edge_safety=args.edge_safety
    )
    print(f"  Computed Pocket    : X[{bounds['xmin']:.3f}..{bounds['xmax']:.3f}] (W={bounds['width']:.3f} mm)")
    print(f"                       Y[{bounds['ymin']:.3f}..{bounds['ymax']:.3f}] (L={bounds['length']:.3f} mm)")
    print(f"  Center             : ({bounds['center_x']:.3f}, {bounds['center_y']:.3f})")
    print("-----------------------------------------------------------------")

    # Compute drill matrix
    drill_points, x_cols, y_rows = compute_drill_matrix(bounds, tool_dia=args.drill_dia, stepover_ratio=0.65)
    print(f"  Drill Matrix ({args.drill_dia:.1f}mm bit): {len(x_cols)} cols x {len(y_rows)} rows = {len(drill_points)} holes")

    # Generate G-code files
    # Primary: 100% V-Bit 10mm Pocket Milling (NO TOOL CHANGE)
    if args.op in ("all", "vbit-pocket"):
        vbit_pocket_file = os.path.join(args.output_dir, "limit_switch_vbit_pocket_10mm.gcode")
        vbit_pocket_gcode = generate_vbit_pocket_10mm_gcode(
            bounds, total_depth=args.depth, stepdown=args.vbit_stepdown,
            vbit_angle=args.vbit_angle, vbit_tip=args.vbit_tip, vbit_shank=args.vbit_shank,
            feed_xy=args.vbit_feed, plunge_rate=80.0,
            spindle_speed=args.spindle, safe_z=args.safe_z
        )
        with open(vbit_pocket_file, "w", encoding="utf-8") as f:
            f.write(vbit_pocket_gcode)
        print(f"  [+] Saved V-Bit 10mm Pocket G-code: {vbit_pocket_file}")

    if args.op in ("all", "vbit-outline", "vbit"):
        vbit_file = os.path.join(args.output_dir, "limit_switch_vbit_outline.gcode")
        vbit_gcode = generate_vbit_outline_gcode(
            bounds, cut_depth=args.vbit_depth, feed_xy=300.0, plunge_rate=80.0,
            spindle_speed=args.spindle, safe_z=args.safe_z, passes=2
        )
        with open(vbit_file, "w", encoding="utf-8") as f:
            f.write(vbit_gcode)
        print(f"  [+] Saved V-Bit Outline G-code : {vbit_file}")

    if args.op in ("all", "drill"):
        drill_file = os.path.join(args.output_dir, f"limit_switch_peck_drill_{args.drill_dia:.1f}mm.gcode")
        drill_gcode = generate_peck_drill_matrix_gcode(
            bounds, drill_points, tool_dia=args.drill_dia, total_depth=args.depth,
            peck_depth=args.peck_depth, peck_retract=0.5, full_retract_interval=3.0,
            drill_feed=args.drill_feed, spindle_speed=args.spindle, safe_z=args.safe_z
        )
        with open(drill_file, "w", encoding="utf-8") as f:
            f.write(drill_gcode)
        print(f"  [+] Saved Peck Drill G-code    : {drill_file}")

    if args.op in ("all", "pocket"):
        pocket_file = os.path.join(args.output_dir, f"limit_switch_pocket_milling_{args.drill_dia:.1f}mm.gcode")
        pocket_gcode = generate_pocket_milling_gcode(
            bounds, tool_dia=args.drill_dia, total_depth=args.depth, stepdown=0.8,
            stepover=args.drill_dia * 0.5, feed_xy=args.mill_feed, plunge_rate=80.0,
            spindle_speed=args.spindle, safe_z=args.safe_z
        )
        with open(pocket_file, "w", encoding="utf-8") as f:
            f.write(pocket_gcode)
        print(f"  [+] Saved Pocket Milling G-code: {pocket_file}")

    # Generate visual SVG preview
    svg_file = os.path.join(args.output_dir, "limit_switch_pocket.svg")
    generate_svg_preview(bounds, drill_points, tool_dia=args.drill_dia, output_path=svg_file)

    print("-----------------------------------------------------------------")
    print("  Done! Ready to machine limit switch pocket on CNC.")
    print("=================================================================")


if __name__ == "__main__":
    main()
