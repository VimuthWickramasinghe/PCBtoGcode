#!/usr/bin/env python3
"""
Gerber to G-Code Converter for CNC PCB Isolation Milling
Built from the ground up without outdated or broken third-party packages.
Supports RS-274X, Gerber X2/X3, Aperture Macros (including KiCad RoundRect),
G36/G37 polygon regions, multi-pass isolation, path optimization, and mirroring.

Usage:
    python gerber_to_gcode.py input.gbr output.gcode [options]
"""

import sys
import os
import re
import math
import time
import argparse
import shapely
from shapely.geometry import Point, LineString, Polygon, MultiPolygon, box
from shapely.ops import unary_union
import shapely.affinity


# -----------------------------------------------------------------------------
# Utility Functions
# -----------------------------------------------------------------------------

def clean_geometry(geom):
    """Ensure geometry is valid and non-empty."""
    if geom is None or geom.is_empty:
        return None
    if not geom.is_valid:
        if hasattr(shapely, "make_valid"):
            try:
                geom = shapely.make_valid(geom)
            except Exception:
                geom = geom.buffer(0)
        else:
            geom = geom.buffer(0)
    return geom if (geom and not geom.is_empty) else None


def eval_macro_expr(expr, params):
    """Safely evaluate arithmetic expressions inside aperture macros."""
    def repl(match):
        idx = int(match.group(1)) - 1
        return str(params[idx]) if 0 <= idx < len(params) else "0.0"

    expr_str = re.sub(r"\$(\d+)", repl, expr.strip())
    # Sanitize characters to only numbers, operators, parens, spaces
    if not re.match(r"^[\d\.\+\-\*\/\(\)\s]+$", expr_str):
        return 0.0
    try:
        return float(eval(expr_str, {"__builtins__": None}, {}))
    except Exception:
        return 0.0


def arc_interpolate(xs, ys, xe, ye, i, j, is_cw, max_step=0.08):
    """Interpolate an arc from (xs, ys) to (xe, ye) with center offset (i, j)."""
    xc = xs + i
    yc = ys + j
    r = math.hypot(i, j)
    if r < 1e-6:
        return [(xs, ys), (xe, ye)]

    a_start = math.atan2(ys - yc, xs - xc)
    a_end = math.atan2(ye - yc, xe - xc)

    # 360-degree circle check (multi-quadrant G75)
    if abs(xs - xe) < 1e-6 and abs(ys - ye) < 1e-6:
        sweep = -2 * math.pi if is_cw else 2 * math.pi
    else:
        if is_cw:
            if a_end >= a_start:
                a_end -= 2 * math.pi
        else:
            if a_end <= a_start:
                a_end += 2 * math.pi
        sweep = a_end - a_start

    num_steps = max(4, int(math.ceil(abs(sweep) * r / max_step)))
    pts = []
    for step in range(num_steps + 1):
        a = a_start + sweep * (step / num_steps)
        pts.append((xc + r * math.cos(a), yc + r * math.sin(a)))
    return pts


# -----------------------------------------------------------------------------
# Gerber RS-274X Parser
# -----------------------------------------------------------------------------

class GerberParser:
    def __init__(self, filepath):
        self.filepath = filepath
        self.scale_x = 1e6
        self.scale_y = 1e6
        self.unit_scale = 1.0  # mm default
        self.macros = {}
        self.apertures = {}
        self.copper = None

    def _parse_macro_shape(self, macro_body, param_str):
        """Build Shapely shape from RS-274X aperture macro primitives."""
        params = [float(p) for p in param_str.split("X")] if param_str else []
        shapes = []

        for line in macro_body.strip().split("*"):
            line = line.strip()
            if not line or line.startswith("0"):
                continue  # Skip comments
            parts = [p.strip() for p in line.split(",")]
            if not parts or not parts[0]:
                continue

            p_code = int(eval_macro_expr(parts[0], params))

            if p_code == 1:  # Circle: 1, exposure, dia, cx, cy [, rot]
                exposure = int(eval_macro_expr(parts[1], params)) if len(parts) > 1 else 1
                dia = eval_macro_expr(parts[2], params)
                cx = eval_macro_expr(parts[3], params)
                cy = eval_macro_expr(parts[4], params)
                c_geom = Point(cx, cy).buffer(dia / 2.0)
                if exposure == 1:
                    shapes.append(c_geom)

            elif p_code in (2, 20):  # Vector Line: 20, exposure, width, xs, ys, xe, ye, rot
                exposure = int(eval_macro_expr(parts[1], params)) if len(parts) > 1 else 1
                w = eval_macro_expr(parts[2], params)
                xs = eval_macro_expr(parts[3], params)
                ys = eval_macro_expr(parts[4], params)
                xe = eval_macro_expr(parts[5], params)
                ye = eval_macro_expr(parts[6], params)
                vl_geom = LineString([(xs, ys), (xe, ye)]).buffer(w / 2.0, cap_style=2)
                if exposure == 1:
                    shapes.append(vl_geom)

            elif p_code == 4:  # Outline: 4, exposure, num_vertices, x1, y1, ...
                exposure = int(eval_macro_expr(parts[1], params)) if len(parts) > 1 else 1
                n = int(eval_macro_expr(parts[2], params))
                pts = []
                for i in range(n):
                    if 3 + 2 * i + 1 < len(parts):
                        px = eval_macro_expr(parts[3 + 2 * i], params)
                        py = eval_macro_expr(parts[4 + 2 * i], params)
                        pts.append((px, py))
                if len(pts) >= 3:
                    poly = Polygon(pts)
                    if exposure == 1:
                        shapes.append(poly)

            elif p_code == 21:  # Center line / rectangle: 21, exposure, w, h, cx, cy, rot
                exposure = int(eval_macro_expr(parts[1], params)) if len(parts) > 1 else 1
                w = eval_macro_expr(parts[2], params)
                h = eval_macro_expr(parts[3], params)
                cx = eval_macro_expr(parts[4], params)
                cy = eval_macro_expr(parts[5], params)
                rect_geom = box(cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)
                if exposure == 1:
                    shapes.append(rect_geom)

        return unary_union(shapes) if shapes else None

    def parse(self):
        """Parse Gerber file into a unified Shapely copper geometry."""
        with open(self.filepath, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read()

        # Tokenize into extended commands (%...%) and standard commands (...*)
        token_pattern = re.compile(r"%([^%]+)%|([^%*]+)\*")
        tokens = token_pattern.findall(content)

        # Regex matchers
        re_fs = re.compile(r"FSLA(?:[A-Z]*)X(\d)(\d)Y(\d)(\d)")
        re_add = re.compile(r"ADD(\d+)([A-Za-z0-9_]+)(?:,(.*))?")
        re_x = re.compile(r"X([\+\-]?[0-9]+)")
        re_y = re.compile(r"Y([\+\-]?[0-9]+)")
        re_i = re.compile(r"I([\+\-]?[0-9]+)")
        re_j = re.compile(r"J([\+\-]?[0-9]+)")
        re_d = re.compile(r"D([0-9]+)")
        re_g = re.compile(r"G([0-9]+)")

        # First pass: parse headers, format specification, units, macros, and apertures
        for ext, norm in tokens:
            if not ext:
                continue
            for sub in ext.split("*"):
                sub = sub.strip()
                if not sub:
                    continue

                if sub.startswith("FS"):
                    m_fs = re_fs.search(sub)
                    if m_fs:
                        self.scale_x = 10.0 ** int(m_fs.group(2))
                        self.scale_y = 10.0 ** int(m_fs.group(4))

                elif sub == "MOIN":
                    self.unit_scale = 25.4  # Inches -> mm
                elif sub == "MOMM":
                    self.unit_scale = 1.0   # Millimeters

                elif sub.startswith("AM"):
                    parts = ext.split("*", 1)
                    macro_name = parts[0][2:].strip()
                    self.macros[macro_name] = parts[1]

                elif sub.startswith("ADD"):
                    m_add = re_add.match(sub)
                    if m_add:
                        d_code = int(m_add.group(1))
                        ap_type = m_add.group(2)
                        ap_params = m_add.group(3) or ""

                        if ap_type == "C":  # Circle: dia [X hole]
                            dia = float(ap_params.split("X")[0]) * self.unit_scale
                            self.apertures[d_code] = ("C", dia)
                        elif ap_type == "R":  # Rectangle: w X h [X hole]
                            p_parts = ap_params.split("X")
                            w = float(p_parts[0]) * self.unit_scale
                            h = float(p_parts[1]) * self.unit_scale
                            self.apertures[d_code] = ("R", (w, h))
                        elif ap_type == "O":  # Obround: w X h [X hole]
                            p_parts = ap_params.split("X")
                            w = float(p_parts[0]) * self.unit_scale
                            h = float(p_parts[1]) * self.unit_scale
                            if w >= h:
                                ob_geom = LineString([(-(w - h) / 2.0, 0), ((w - h) / 2.0, 0)]).buffer(h / 2.0)
                            else:
                                ob_geom = LineString([(0, -(h - w) / 2.0), (0, (h - w) / 2.0)]).buffer(w / 2.0)
                            self.apertures[d_code] = ("O", ob_geom)
                        elif ap_type in self.macros:
                            macro_geom = self._parse_macro_shape(self.macros[ap_type], ap_params)
                            if self.unit_scale != 1.0 and macro_geom:
                                macro_geom = shapely.affinity.scale(
                                    macro_geom, xfact=self.unit_scale, yfact=self.unit_scale, origin=(0, 0)
                                )
                            self.apertures[d_code] = ("MACRO", macro_geom)

        # Second pass: execute commands and build geometric features with polarity
        layers = [{"polarity": "D", "shapes": []}]
        current_polarity = "D"

        cur_x, cur_y = 0.0, 0.0
        cur_aper = None
        g_mode = 1  # 1: Linear, 2: CW arc, 3: CCW arc
        in_region = False
        cur_contour = []

        for ext, norm in tokens:
            if ext:
                for sub in ext.split("*"):
                    sub = sub.strip()
                    if sub == "LPD" and current_polarity != "D":
                        current_polarity = "D"
                        layers.append({"polarity": "D", "shapes": []})
                    elif sub == "LPC" and current_polarity != "C":
                        current_polarity = "C"
                        layers.append({"polarity": "C", "shapes": []})
                continue

            cmd = norm.strip()
            if not cmd or cmd.startswith("G04"):
                continue

            # Process G-codes
            for g_str in re_g.findall(cmd):
                g_val = int(g_str)
                if g_val in (1, 2, 3):
                    g_mode = g_val
                elif g_val == 36:  # Start polygon region
                    in_region = True
                    cur_contour = []
                elif g_val == 37:  # End polygon region
                    in_region = False
                    if len(cur_contour) >= 3:
                        poly = Polygon(cur_contour)
                        poly = clean_geometry(poly)
                        if poly:
                            layers[-1]["shapes"].append(poly)
                    cur_contour = []

            # Process Coordinates
            mx = re_x.search(cmd)
            my = re_y.search(cmd)
            mi = re_i.search(cmd)
            mj = re_j.search(cmd)

            new_x = int(mx.group(1)) / self.scale_x * self.unit_scale if mx else cur_x
            new_y = int(my.group(1)) / self.scale_y * self.unit_scale if my else cur_y
            i_val = int(mi.group(1)) / self.scale_x * self.unit_scale if mi else 0.0
            j_val = int(mj.group(1)) / self.scale_y * self.unit_scale if mj else 0.0

            # Process D-codes
            md = re_d.search(cmd)
            if md:
                d_val = int(md.group(1))
                if d_val >= 10:
                    cur_aper = d_val
                elif d_val == 2:  # Move (rapid)
                    cur_x, cur_y = new_x, new_y
                    if in_region:
                        if len(cur_contour) >= 3:
                            p = Polygon(cur_contour)
                            p = clean_geometry(p)
                            if p:
                                layers[-1]["shapes"].append(p)
                        cur_contour = [(cur_x, cur_y)]
                elif d_val == 1:  # Interpolate / Stroke
                    if in_region:
                        if g_mode in (2, 3):
                            arc_pts = arc_interpolate(cur_x, cur_y, new_x, new_y, i_val, j_val, is_cw=(g_mode == 2))
                            cur_contour.extend(arc_pts[1:])
                        else:
                            cur_contour.append((new_x, new_y))
                    else:
                        ap = self.apertures.get(cur_aper)
                        if ap:
                            ap_type, ap_val = ap
                            if g_mode in (2, 3):
                                arc_pts = arc_interpolate(cur_x, cur_y, new_x, new_y, i_val, j_val, is_cw=(g_mode == 2))
                                if ap_type == "C":
                                    stroke = LineString(arc_pts).buffer(ap_val / 2.0)
                                    layers[-1]["shapes"].append(stroke)
                            else:
                                if ap_type == "C":
                                    stroke = LineString([(cur_x, cur_y), (new_x, new_y)]).buffer(ap_val / 2.0)
                                    layers[-1]["shapes"].append(stroke)
                    cur_x, cur_y = new_x, new_y
                elif d_val == 3:  # Flash aperture
                    cur_x, cur_y = new_x, new_y
                    ap = self.apertures.get(cur_aper)
                    if ap:
                        ap_type, ap_val = ap
                        flash_geom = None
                        if ap_type == "C":
                            flash_geom = Point(cur_x, cur_y).buffer(ap_val / 2.0)
                        elif ap_type == "R":
                            w, h = ap_val
                            flash_geom = box(cur_x - w / 2.0, cur_y - h / 2.0, cur_x + w / 2.0, cur_y + h / 2.0)
                        elif ap_type in ("O", "MACRO") and ap_val is not None:
                            flash_geom = shapely.affinity.translate(ap_val, cur_x, cur_y)

                        if flash_geom:
                            layers[-1]["shapes"].append(flash_geom)

        # Combine polarity layers
        copper_geom = None
        for layer in layers:
            shapes = layer["shapes"]
            if not shapes:
                continue
            merged = unary_union(shapes)
            merged = clean_geometry(merged)
            if not merged:
                continue

            if layer["polarity"] == "D":
                if copper_geom is None:
                    copper_geom = merged
                else:
                    copper_geom = copper_geom.union(merged)
            elif layer["polarity"] == "C":
                if copper_geom is not None:
                    copper_geom = copper_geom.difference(merged)

            copper_geom = clean_geometry(copper_geom)

        self.copper = copper_geom
        return self.copper


# -----------------------------------------------------------------------------
# Toolpath Generator & Path Optimizer
# -----------------------------------------------------------------------------

class ToolpathGenerator:
    def __init__(self, copper_geom, tool_dia=0.2, passes=1, stepover=0.7,
                 mirror_x=False, zero_origin=True, margin=2.0):
        self.copper = copper_geom
        self.tool_dia = tool_dia
        self.tool_rad = tool_dia / 2.0
        self.passes = max(1, passes)
        self.stepover = stepover
        self.mirror_x = mirror_x
        self.zero_origin = zero_origin
        self.margin = margin
        self.transformed_copper = None
        self.bounds = None

    def _extract_rings(self, geom):
        """Extract closed coordinate loops from Polygon or MultiPolygon."""
        rings = []
        if geom is None or geom.is_empty:
            return rings
        if geom.geom_type == "Polygon":
            rings.append(list(geom.exterior.coords))
            for interior in geom.interiors:
                rings.append(list(interior.coords))
        elif geom.geom_type == "MultiPolygon":
            for poly in geom.geoms:
                rings.append(list(poly.exterior.coords))
                for interior in poly.interiors:
                    rings.append(list(interior.coords))
        elif geom.geom_type == "GeometryCollection":
            for item in geom.geoms:
                rings.extend(self._extract_rings(item))
        return rings

    def generate(self):
        """Generate optimized isolation milling loops."""
        if self.copper is None or self.copper.is_empty:
            return []

        # 1. Coordinate transformations
        geom = self.copper
        minx, miny, maxx, maxy = geom.bounds

        if self.mirror_x:
            # Mirror horizontally around minx
            geom = shapely.affinity.scale(geom, xfact=-1.0, yfact=1.0, origin=(minx, 0))

        if self.zero_origin:
            t_minx, t_miny, t_maxx, t_maxy = geom.bounds
            geom = shapely.affinity.translate(geom, xoff=-t_minx + self.margin, yoff=-t_miny + self.margin)

        self.transformed_copper = geom
        self.bounds = geom.bounds

        # 2. Multi-pass isolation buffering
        raw_rings = []
        for pass_idx in range(self.passes):
            offset = self.tool_rad + pass_idx * (self.tool_dia * self.stepover)
            iso_geom = geom.buffer(offset)
            iso_geom = clean_geometry(iso_geom)
            if iso_geom:
                raw_rings.extend(self._extract_rings(iso_geom))

        # 3. Path optimization (Greedy Nearest Neighbor with loop start rotation)
        optimized_loops, total_rapid = self._optimize_paths(raw_rings)
        return optimized_loops, total_rapid

    def _optimize_paths(self, rings, start_pos=(0.0, 0.0)):
        """Sort loops and pick best start vertex to minimize rapid travel."""
        unvisited = [r for r in rings if len(r) >= 3]
        ordered = []
        cur_x, cur_y = start_pos
        total_rapid = 0.0

        while unvisited:
            best_idx = -1
            best_dist_sq = float("inf")
            best_rot_ring = None

            for i, ring in enumerate(unvisited):
                pts = ring[:-1] if ring[0] == ring[-1] else ring
                # Subsample vertices for large contours to keep sorting instantaneous
                step = max(1, len(pts) // 30)
                for v_idx in range(0, len(pts), step):
                    vx, vy = pts[v_idx]
                    d_sq = (vx - cur_x) ** 2 + (vy - cur_y) ** 2
                    if d_sq < best_dist_sq:
                        best_dist_sq = d_sq
                        best_idx = i
                        best_rot_ring = pts[v_idx:] + pts[:v_idx] + [pts[v_idx]]

            total_rapid += math.sqrt(best_dist_sq)
            ordered.append(best_rot_ring)
            cur_x, cur_y = best_rot_ring[-1]
            unvisited.pop(best_idx)

        return ordered, total_rapid


# -----------------------------------------------------------------------------
# G-Code Generator
# -----------------------------------------------------------------------------

class GcodeWriter:
    def __init__(self, output_path, loops, total_rapid_dist, bounds,
                 tool_dia=0.2, cut_depth=-0.04, safe_z=2.0, travel_z=1.0,
                 feed_rate=120.0, plunge_rate=60.0, spindle_speed=10000,
                 source_filename="", mirrored=False, zeroed=True):
        self.output_path = output_path
        self.loops = loops
        self.total_rapid_dist = total_rapid_dist
        self.bounds = bounds
        self.tool_dia = tool_dia
        self.cut_depth = cut_depth
        self.safe_z = safe_z
        self.travel_z = travel_z
        self.feed_rate = feed_rate
        self.plunge_rate = plunge_rate
        self.spindle_speed = spindle_speed
        self.source_filename = source_filename
        self.mirrored = mirrored
        self.zeroed = zeroed

    def write(self):
        """Write standards-compliant G-code file."""
        total_cut_dist = sum(
            sum(math.hypot(p2[0] - p1[0], p2[1] - p1[1]) for p1, p2 in zip(loop[:-1], loop[1:]))
            for loop in self.loops
        )
        total_plunges = len(self.loops)
        plunge_dist = total_plunges * (self.safe_z - self.cut_depth)
        retract_dist = total_plunges * (self.safe_z - self.cut_depth)

        # Estimate run time (minutes)
        rapid_rate = 1000.0  # mm/min standard rapid
        est_time_min = (
            (total_cut_dist / self.feed_rate) +
            (plunge_dist / self.plunge_rate) +
            (retract_dist / rapid_rate) +
            (self.total_rapid_dist / rapid_rate) +
            0.1  # spindle dwell
        )

        minx, miny, maxx, maxy = self.bounds if self.bounds else (0, 0, 0, 0)
        width = maxx - minx
        height = maxy - miny

        lines = [
            "( ========================================================== )",
            "(  Generated by Gerber-to-GCode PCB Isolation Router        )",
            f"(  Source Gerber    : {self.source_filename} )",
            f"(  Date             : {time.strftime('%Y-%m-%d %H:%M:%S')} )",
            f"(  Tool Tip Dia     : {self.tool_dia:.3f} mm )",
            f"(  Milling Cut Depth: {self.cut_depth:.4f} mm )",
            f"(  Safe Height (Z)  : {self.safe_z:.2f} mm )",
            f"(  Feed Rate (XY)   : {self.feed_rate:.1f} mm/min )",
            f"(  Plunge Rate (Z)  : {self.plunge_rate:.1f} mm/min )",
            f"(  Spindle Speed    : {self.spindle_speed} RPM )",
            f"(  Mirrored X       : {'YES (Bottom Copper)' if self.mirrored else 'NO'} )",
            f"(  Zeroed Origin    : {'YES (Workpiece 0,0)' if self.zeroed else 'NO'} )",
            f"(  Board Dimensions : {width:.2f} mm x {height:.2f} mm )",
            f"(  Bounding Box     : X [{minx:.2f}, {maxx:.2f}], Y [{miny:.2f}, {maxy:.2f}] )",
            f"(  Total Contours   : {len(self.loops)} )",
            f"(  Cut Distance     : {total_cut_dist:.2f} mm )",
            f"(  Rapid Distance   : {self.total_rapid_dist:.2f} mm )",
            f"(  Est. Cut Time    : {est_time_min:.1f} min )",
            "( ========================================================== )",
            "",
            "G21          (Units in Millimeters)",
            "G90          (Absolute Coordinates)",
            "G94          (Feed Rate per Minute)",
            "G17          (XY Work Plane Selection)",
            f"G00 Z{self.safe_z:.4f} (Rapid to Safe Z)",
            f"M03 S{self.spindle_speed} (Spindle Start CW)",
            "G04 P2.0     (Dwell 2 seconds for spindle to reach full speed)",
            ""
        ]

        for idx, loop in enumerate(self.loops, 1):
            if not loop:
                continue
            start_x, start_y = loop[0]
            lines.append(f"( --- Contour {idx}/{len(self.loops)} --- )")
            lines.append(f"G00 X{start_x:.4f} Y{start_y:.4f}")
            lines.append(f"G01 Z{self.cut_depth:.4f} F{self.plunge_rate:.1f}")

            for pt in loop[1:]:
                lines.append(f"G01 X{pt[0]:.4f} Y{pt[1]:.4f} F{self.feed_rate:.1f}")

            lines.append(f"G00 Z{self.safe_z:.4f}")
            lines.append("")

        lines.extend([
            "( --- Milling Completed --- )",
            f"G00 Z{self.safe_z:.4f}",
            "M05          (Spindle Stop)",
            "G00 X0.0000 Y0.0000 (Return to Workpiece Origin)",
            "M02          (End of Program)"
        ])

        with open(self.output_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

        return {
            "contours": len(self.loops),
            "cut_dist_mm": total_cut_dist,
            "rapid_dist_mm": self.total_rapid_dist,
            "est_time_min": est_time_min,
            "width_mm": width,
            "height_mm": height
        }


# -----------------------------------------------------------------------------
# Optional SVG Preview Generator
# -----------------------------------------------------------------------------

def export_svg_preview(svg_path, copper_geom, loops, bounds, margin=5.0):
    """Render an SVG preview showing copper areas and isolation toolpaths."""
    if not bounds:
        return
    minx, miny, maxx, maxy = bounds
    vb_x = minx - margin
    vb_y = miny - margin
    vb_w = (maxx - minx) + 2 * margin
    vb_h = (maxy - miny) + 2 * margin

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vb_x:.2f} {-maxy - margin:.2f} {vb_w:.2f} {vb_h:.2f}" width="1000" height="{1000 * vb_h / vb_w:.1f}" style="background:#131c19;">',
        '  <defs>',
        '    <pattern id="grid" width="10" height="10" patternUnits="userSpaceOnUse">',
        '      <path d="M 10 0 L 0 0 0 10" fill="none" stroke="#1f2d29" stroke-width="0.3"/>',
        '    </pattern>',
        '  </defs>',
        f'  <rect x="{vb_x:.2f}" y="{-maxy - margin:.2f}" width="{vb_w:.2f}" height="{vb_h:.2f}" fill="url(#grid)" />'
    ]

    # Render copper geometry in amber/copper color
    def poly_to_svg_path(poly):
        paths = []
        ext_d = "M " + " L ".join(f"{x:.3f},{-y:.3f}" for x, y in poly.exterior.coords) + " Z"
        paths.append(ext_d)
        for inter in poly.interiors:
            int_d = "M " + " L ".join(f"{x:.3f},{-y:.3f}" for x, y in inter.coords) + " Z"
            paths.append(int_d)
        return " ".join(paths)

    if copper_geom:
        if copper_geom.geom_type == "Polygon":
            d_str = poly_to_svg_path(copper_geom)
            svg.append(f'  <path d="{d_str}" fill="#d97706" fill-rule="evenodd" opacity="0.85" />')
        elif copper_geom.geom_type == "MultiPolygon":
            for p in copper_geom.geoms:
                d_str = poly_to_svg_path(p)
                svg.append(f'  <path d="{d_str}" fill="#d97706" fill-rule="evenodd" opacity="0.85" />')

    # Render CNC isolation toolpaths in cyan
    for loop in loops:
        if len(loop) < 2:
            continue
        d = "M " + " L ".join(f"{x:.3f},{-y:.3f}" for x, y in loop)
        svg.append(f'  <path d="{d}" fill="none" stroke="#22d3ee" stroke-width="0.25" opacity="0.9" />')
        # Mark plunge point
        svg.append(f'  <circle cx="{loop[0][0]:.3f}" cy="{-loop[0][1]:.3f}" r="0.2" fill="#ef4444" />')

    # Workpiece origin marker (0, 0)
    svg.append('  <circle cx="0" cy="0" r="1.0" fill="none" stroke="#22c55e" stroke-width="0.3" />')
    svg.append('  <line x1="-2" y1="0" x2="2" y2="0" stroke="#22c55e" stroke-width="0.2" />')
    svg.append('  <line x1="0" y1="-2" x2="0" y2="2" stroke="#22c55e" stroke-width="0.2" />')
    svg.append('</svg>')

    with open(svg_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg) + "\n")


# -----------------------------------------------------------------------------
# Main CLI
# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Ground-up Gerber RS-274X to CNC Isolation G-Code Converter"
    )
    parser.add_argument("input", help="Input Gerber file path (e.g., ./AirCooler-B_Cu.gbr)")
    parser.add_argument("output", help="Output G-code file path (e.g., output-B_Cu.gcode)")
    parser.add_argument("-t", "--tool-dia", type=float, default=0.2,
                        help="Milling bit tip diameter in mm (default: 0.2)")
    parser.add_argument("-z", "--cut-depth", type=float, default=-0.04,
                        help="Cut depth Z in mm (default: -0.04)")
    parser.add_argument("--safe-z", type=float, default=2.0,
                        help="Safe travel height Z in mm (default: 2.0)")
    parser.add_argument("-f", "--feed-rate", type=float, default=120.0,
                        help="XY milling feed rate in mm/min (default: 120.0)")
    parser.add_argument("--plunge-rate", type=float, default=60.0,
                        help="Z plunge feed rate in mm/min (default: 60.0)")
    parser.add_argument("-s", "--spindle-speed", type=int, default=10000,
                        help="Spindle speed in RPM (default: 10000)")
    parser.add_argument("-p", "--passes", type=int, default=1,
                        help="Number of isolation passes (default: 1)")
    parser.add_argument("--stepover", type=float, default=0.7,
                        help="Stepover ratio for multi-pass isolation (default: 0.7)")
    parser.add_argument("--mirror", dest="mirror", action="store_true",
                        help="Mirror X coordinates (essential for bottom copper when milled copper-side up)")
    parser.add_argument("--no-mirror", dest="mirror", action="store_false",
                        help="Do not mirror X coordinates")
    parser.set_defaults(mirror=None)
    parser.add_argument("--preserve-origin", action="store_true",
                        help="Preserve raw Gerber coordinates instead of shifting bottom-left to (0,0)")
    parser.add_argument("--margin", type=float, default=2.0,
                        help="Margin in mm from (0,0) when zeroing origin (default: 2.0)")
    parser.add_argument("--preview", type=str, default=None,
                        help="Optional SVG preview output path (e.g. preview.svg)")

    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"Error: Input file '{args.input}' not found.", file=sys.stderr)
        sys.exit(1)

    # Auto-detect mirroring for bottom layers if not explicitly specified
    filename = os.path.basename(args.input).lower()
    is_bottom = any(kw in filename for kw in ("b_cu", "bottom", "bot", "b.cu", "b_mask"))
    if args.mirror is None:
        args.mirror = is_bottom

    print("==========================================================")
    print(" Gerber to G-Code CNC Isolation Router")
    print("==========================================================")
    print(f" Input File      : {args.input}")
    print(f" Output G-Code   : {args.output}")
    print(f" Tool Diameter   : {args.tool_dia:.3f} mm (Radius: {args.tool_dia/2.0:.3f} mm)")
    print(f" Cut Depth (Z)   : {args.cut_depth:.4f} mm")
    print(f" Safe Z Height   : {args.safe_z:.2f} mm")
    print(f" Feed Rate (XY)  : {args.feed_rate:.1f} mm/min")
    print(f" Plunge Rate (Z) : {args.plunge_rate:.1f} mm/min")
    print(f" Spindle Speed   : {args.spindle_speed} RPM")
    print(f" Isolation Passes: {args.passes}")
    print(f" Mirror X Axis   : {args.mirror} {'(Auto-detected Bottom Layer)' if is_bottom else ''}")
    print(f" Zero Origin     : {not args.preserve_origin} (Margin: {args.margin} mm)")
    print("==========================================================")

    # 1. Parse Gerber
    t0 = time.time()
    print("[1/3] Parsing Gerber file and apertures...")
    parser = GerberParser(args.input)
    copper = parser.parse()
    if copper is None or copper.is_empty:
        print("Error: No copper geometry found in Gerber file.", file=sys.stderr)
        sys.exit(1)
    print(f"      Parsed in {time.time() - t0:.3f}s. Copper area: {copper.area:.2f} mm^2")

    # 2. Generate Toolpaths
    t1 = time.time()
    print("[2/3] Generating isolation toolpaths & optimizing travel...")
    generator = ToolpathGenerator(
        copper_geom=copper,
        tool_dia=args.tool_dia,
        passes=args.passes,
        stepover=args.stepover,
        mirror_x=args.mirror,
        zero_origin=(not args.preserve_origin),
        margin=args.margin
    )
    loops, rapid_dist = generator.generate()
    print(f"      Generated {len(loops)} closed milling loops in {time.time() - t1:.3f}s.")

    # 3. Write G-Code
    t2 = time.time()
    print("[3/3] Writing G-code file...")
    writer = GcodeWriter(
        output_path=args.output,
        loops=loops,
        total_rapid_dist=rapid_dist,
        bounds=generator.bounds,
        tool_dia=args.tool_dia,
        cut_depth=args.cut_depth,
        safe_z=args.safe_z,
        feed_rate=args.feed_rate,
        plunge_rate=args.plunge_rate,
        spindle_speed=args.spindle_speed,
        source_filename=os.path.basename(args.input),
        mirrored=args.mirror,
        zeroed=(not args.preserve_origin)
    )
    stats = writer.write()
    print(f"      G-code successfully written to '{args.output}'.")

    # Optional Preview
    if args.preview:
        print(f"      Exporting SVG preview to '{args.preview}'...")
        export_svg_preview(args.preview, generator.transformed_copper, loops, generator.bounds)

    print("\n------------------ Summary ------------------")
    print(f" Board Dimensions: {stats['width_mm']:.2f} mm x {stats['height_mm']:.2f} mm")
    print(f" Total Contours  : {stats['contours']}")
    print(f" Cut Distance    : {stats['cut_dist_mm']:.2f} mm ({stats['cut_dist_mm']/1000.0:.3f} m)")
    print(f" Rapid Distance  : {stats['rapid_dist_mm']:.2f} mm ({stats['rapid_dist_mm']/1000.0:.3f} m)")
    print(f" Estimated Time  : {stats['est_time_min']:.1f} minutes")
    print("---------------------------------------------")
    print("Conversion complete!\n")


if __name__ == "__main__":
    main()
