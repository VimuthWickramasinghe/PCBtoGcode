#!/usr/bin/env python3
"""
Gerber and Excellon to G-Code Converter for CNC PCB Isolation Routing and Drilling
Specialized for EasyEDA (and KiCad) PCB Project Exports.

Features:
- RS-274X Gerber parser supporting 4:5 format (%FSLAX45Y45*%), millimeter units (%MOMM*%),
  standard apertures (C, R, O/Obround), and aperture macros with arithmetic expression evaluation.
- Robust Excellon drill parser with automatic implicit decimal detection (e.g. METRIC,LZ,000.000 / 3:3)
  and explicit decimal handling.
- G85 canned cycle slot routing and M15/M16 slot parsing.
- Precise alignment: lower-left corner of the board outline (Gerber_BoardOutlineLayer.GKO) is mapped
  exactly to (0.0000, 0.0000) with configurable margin.
- Automatic X-axis mirroring for bottom copper, bottom drilling, and mirrored cutout.
- Board edge outline is engraved onto BOTH Front and Back copper passes at isolation depth (-0.04 mm).
- Individual G-code and visual SVG previews generated for every single layer and drill file.
- Unified deduplicated multi-tool drill file (Drill_Combined) with intelligent duplicate-hole removal
  (e.g. deduplicating via holes shared between Drill_PTH_Through and Drill_PTH_Through_Via).
- Multi-pass board cutout milling with step-down depth passes.
- Standard G-code compatible with GRBL, Candle, UGS, FluidNC (MKS-DLC32), LinuxCNC, and Mach3.

Usage:
    # Process an entire EasyEDA project folder:
    python gerber_to_gcode_easyeda.py ./project02

    # Process into a custom output folder:
    python gerber_to_gcode_easyeda.py ./project02 ./my_output_gcode

    # Convert a single Gerber or Drill file:
    python gerber_to_gcode_easyeda.py ./project02/Gerber_TopLayer.GTL output.gcode
"""

import sys
import os
import re
import math
import time
import glob
import argparse
import shapely
from shapely.geometry import Point, LineString, Polygon, MultiPolygon, box
from shapely.ops import unary_union
import shapely.affinity


# -----------------------------------------------------------------------------
# Geometric Utilities
# -----------------------------------------------------------------------------

def clean_geometry(geom):
    """Ensure geometry is topologically valid and non-empty."""
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
    """Safely evaluate arithmetic expressions inside aperture macros ($1, $2, etc.)."""
    def repl(match):
        idx = int(match.group(1)) - 1
        return str(params[idx]) if 0 <= idx < len(params) else "0.0"

    expr_str = re.sub(r"\$(\d+)", repl, expr.strip())
    if not re.match(r"^[\d\.\+\-\*\/\(\)\s]+$", expr_str):
        return 0.0
    try:
        return float(eval(expr_str, {"__builtins__": None}, {}))
    except Exception:
        return 0.0


def arc_interpolate(xs, ys, xe, ye, i, j, is_cw, max_step=0.08):
    """Interpolate a circular arc from (xs, ys) to (xe, ye) with center offset (i, j)."""
    xc = xs + i
    yc = ys + j
    r = math.hypot(i, j)
    if r < 1e-6:
        return [(xs, ys), (xe, ye)]

    a_start = math.atan2(ys - yc, xs - xc)
    a_end = math.atan2(ye - yc, xe - xc)

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
        self.scale_x = 1e5  # EasyEDA default is 4:5 -> 10^5
        self.scale_y = 1e5
        self.unit_scale = 1.0  # mm default
        self.macros = {}
        self.apertures = {}
        self.copper = None
        self.raw_paths = []
        self.raw_bounds = None

    def _parse_macro_shape(self, macro_body, param_str):
        params = [float(p) for p in param_str.split("X")] if param_str else []
        shapes = []

        for line in macro_body.strip().split("*"):
            line = line.strip()
            if not line or line.startswith("0"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if not parts or not parts[0]:
                continue

            p_code = int(eval_macro_expr(parts[0], params))
            if p_code == 1:
                # Circle: 1, exposure, dia, cx, cy
                dia = eval_macro_expr(parts[2], params) * self.unit_scale
                cx = eval_macro_expr(parts[3], params) * self.unit_scale
                cy = eval_macro_expr(parts[4], params) * self.unit_scale
                shapes.append(Point(cx, cy).buffer(dia / 2.0))
            elif p_code in (2, 20):
                # Vector line: 2/20, exposure, width, sx, sy, ex, ey, rot
                w = eval_macro_expr(parts[2], params) * self.unit_scale
                sx = eval_macro_expr(parts[3], params) * self.unit_scale
                sy = eval_macro_expr(parts[4], params) * self.unit_scale
                ex = eval_macro_expr(parts[5], params) * self.unit_scale
                ey = eval_macro_expr(parts[6], params) * self.unit_scale
                shapes.append(LineString([(sx, sy), (ex, ey)]).buffer(w / 2.0, cap_style=2))
            elif p_code == 21:
                # Center rectangle: 21, exposure, w, h, cx, cy, rot
                w = eval_macro_expr(parts[2], params) * self.unit_scale
                h = eval_macro_expr(parts[3], params) * self.unit_scale
                cx = eval_macro_expr(parts[4], params) * self.unit_scale
                cy = eval_macro_expr(parts[5], params) * self.unit_scale
                rect = box(cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)
                if len(parts) > 6:
                    rot = eval_macro_expr(parts[6], params)
                    if abs(rot) > 1e-3:
                        rect = shapely.affinity.rotate(rect, rot, origin=(cx, cy))
                shapes.append(rect)
            elif p_code == 4:
                # Outline polygon: 4, exposure, num_pts, x1, y1, x2, y2, ..., rot
                n_pts = int(eval_macro_expr(parts[2], params))
                poly_pts = []
                idx = 3
                for _ in range(n_pts + 1):
                    if idx + 1 < len(parts):
                        px = eval_macro_expr(parts[idx], params) * self.unit_scale
                        py = eval_macro_expr(parts[idx + 1], params) * self.unit_scale
                        poly_pts.append((px, py))
                        idx += 2
                if len(poly_pts) >= 3:
                    poly = Polygon(poly_pts)
                    if idx < len(parts):
                        rot = eval_macro_expr(parts[idx], params)
                        if abs(rot) > 1e-3:
                            poly = shapely.affinity.rotate(poly, rot, origin=(0, 0))
                    shapes.append(poly)

        return unary_union(shapes) if shapes else None

    def parse(self):
        with open(self.filepath, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read()

        token_pattern = re.compile(r"%([^%]+)%|([^%*]+)\*")
        tokens = token_pattern.findall(content)

        re_fs = re.compile(r"FSLA(?:[A-Z]*)X(\d)(\d)Y(\d)(\d)")
        re_add = re.compile(r"ADD(\d+)([A-Za-z0-9_]+)(?:,(.*))?")
        re_x = re.compile(r"X([\+\-]?[0-9]+)")
        re_y = re.compile(r"Y([\+\-]?[0-9]+)")
        re_i = re.compile(r"I([\+\-]?[0-9]+)")
        re_j = re.compile(r"J([\+\-]?[0-9]+)")
        re_d = re.compile(r"D([0-9]+)")
        re_g = re.compile(r"G([0-9]+)")

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
                    self.unit_scale = 25.4
                elif sub == "MOMM":
                    self.unit_scale = 1.0
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
                        if ap_type == "C":
                            dia = float(ap_params.split("X")[0]) * self.unit_scale
                            self.apertures[d_code] = ("C", dia)
                        elif ap_type == "R":
                            p_parts = ap_params.split("X")
                            w = float(p_parts[0]) * self.unit_scale
                            h = float(p_parts[1]) * self.unit_scale
                            self.apertures[d_code] = ("R", (w, h))
                        elif ap_type == "O":
                            # Obround aperture: width X height
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

        layers = [{"polarity": "D", "shapes": []}]
        cur_x, cur_y = 0.0, 0.0
        cur_aper = None
        g_mode = 1
        in_region = False
        cur_contour = []
        cur_stroke_path = []

        all_raw_x = []
        all_raw_y = []

        for ext, norm in tokens:
            if ext:
                for sub in ext.split("*"):
                    sub = sub.strip()
                    if sub == "LPD":
                        if layers[-1]["polarity"] != "D":
                            layers.append({"polarity": "D", "shapes": []})
                    elif sub == "LPC":
                        if layers[-1]["polarity"] != "C":
                            layers.append({"polarity": "C", "shapes": []})
                continue

            cmd = norm.strip()
            if not cmd:
                continue

            g_matches = re_g.findall(cmd)
            for gm in g_matches:
                g_val = int(gm)
                if g_val in (1, 2, 3):
                    g_mode = g_val
                elif g_val == 36:
                    in_region = True
                    cur_contour = [(cur_x, cur_y)]
                elif g_val == 37:
                    in_region = False
                    if len(cur_contour) >= 3:
                        poly = Polygon(cur_contour)
                        poly = clean_geometry(poly)
                        if poly:
                            layers[-1]["shapes"].append(poly)
                    cur_contour = []

            m_aper = re.search(r"D([1-9][0-9]+)\b", cmd)
            if m_aper:
                cur_aper = int(m_aper.group(1))

            mx = re_x.search(cmd)
            my = re_y.search(cmd)
            mi = re_i.search(cmd)
            mj = re_j.search(cmd)
            md = re_d.search(cmd)

            new_x = cur_x
            new_y = cur_y
            if mx:
                new_x = (float(mx.group(1)) / self.scale_x) * self.unit_scale
                all_raw_x.append(new_x)
            if my:
                new_y = (float(my.group(1)) / self.scale_y) * self.unit_scale
                all_raw_y.append(new_y)

            i_val = (float(mi.group(1)) / self.scale_x) * self.unit_scale if mi else 0.0
            j_val = (float(mj.group(1)) / self.scale_y) * self.unit_scale if mj else 0.0

            d_val = int(md.group(1)) if md else (1 if (mx or my) and not m_aper else None)

            if d_val is not None:
                if d_val == 2:
                    if cur_stroke_path and len(cur_stroke_path) >= 2:
                        self.raw_paths.append(cur_stroke_path)
                    cur_stroke_path = [(new_x, new_y)]
                    cur_x, cur_y = new_x, new_y
                    if in_region:
                        if len(cur_contour) >= 3:
                            poly = Polygon(cur_contour)
                            poly = clean_geometry(poly)
                            if poly:
                                layers[-1]["shapes"].append(poly)
                        cur_contour = [(cur_x, cur_y)]
                elif d_val == 1:
                    if not cur_stroke_path:
                        cur_stroke_path = [(cur_x, cur_y)]
                    if in_region:
                        if g_mode in (2, 3):
                            arc_pts = arc_interpolate(cur_x, cur_y, new_x, new_y, i_val, j_val, is_cw=(g_mode == 2))
                            cur_contour.extend(arc_pts[1:])
                            cur_stroke_path.extend(arc_pts[1:])
                        else:
                            cur_contour.append((new_x, new_y))
                            cur_stroke_path.append((new_x, new_y))
                    else:
                        ap = self.apertures.get(cur_aper)
                        if ap:
                            ap_type, ap_val = ap
                            if g_mode in (2, 3):
                                arc_pts = arc_interpolate(cur_x, cur_y, new_x, new_y, i_val, j_val, is_cw=(g_mode == 2))
                                cur_stroke_path.extend(arc_pts[1:])
                                if ap_type == "C":
                                    stroke = LineString(arc_pts).buffer(ap_val / 2.0)
                                    layers[-1]["shapes"].append(stroke)
                            else:
                                cur_stroke_path.append((new_x, new_y))
                                if ap_type == "C":
                                    stroke = LineString([(cur_x, cur_y), (new_x, new_y)]).buffer(ap_val / 2.0)
                                    layers[-1]["shapes"].append(stroke)
                    cur_x, cur_y = new_x, new_y
                elif d_val == 3:
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

        if cur_stroke_path and len(cur_stroke_path) >= 2:
            self.raw_paths.append(cur_stroke_path)

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

        if all_raw_x and all_raw_y:
            self.raw_bounds = (min(all_raw_x), min(all_raw_y), max(all_raw_x), max(all_raw_y))
        elif self.copper and not self.copper.is_empty:
            self.raw_bounds = self.copper.bounds

        return self.copper


# -----------------------------------------------------------------------------
# Excellon Drill Parser (EasyEDA & KiCad Compatible)
# -----------------------------------------------------------------------------

class ExcellonParser:
    def __init__(self, filepath):
        self.filepath = filepath
        self.tools = {}        # {tool_id: dia_mm}
        self.holes = []        # [(tool_id, x, y)]
        self.slots = []        # [(tool_id, x1, y1, x2, y2)]
        self.unit_scale = 1.0  # mm default
        self.dec_places = 3    # EasyEDA standard is 3:3 (divide integer coords by 1000)
        self.raw_bounds = None

    def parse(self):
        """Parse Excellon drill file with robust implicit decimal detection for EasyEDA."""
        with open(self.filepath, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()

        in_header = True
        cur_tool = None
        cur_x = 0.0
        cur_y = 0.0
        is_slot = False
        slot_start = None

        all_x = []
        all_y = []

        for line in lines:
            line = line.strip()
            if not line:
                continue
            if line == "%":
                in_header = False
                continue

            if in_header:
                if line.startswith(";"):
                    # EasyEDA specifies: ;FILE_FORMAT=3:3
                    m_ff = re.search(r"FILE_FORMAT=\d+:(\d+)", line)
                    if m_ff:
                        self.dec_places = int(m_ff.group(1))
                    continue

                if "INCH" in line:
                    self.unit_scale = 25.4
                    self.dec_places = 4
                elif "METRIC" in line:
                    self.unit_scale = 1.0

                # Detect decimal format from header (e.g. 000.000 -> 3 decimal places)
                m_fmt = re.search(r"(\d+)\.(\d+)", line)
                if m_fmt:
                    self.dec_places = len(m_fmt.group(2))

                m_t = re.match(r"T(\d+)C([\d\.]+)", line)
                if m_t:
                    t_id = int(m_t.group(1))
                    self.tools[t_id] = float(m_t.group(2)) * self.unit_scale
            else:
                if line.startswith(";"):
                    continue

                m_sel = re.match(r"^T(\d+)$", line)
                if m_sel:
                    cur_tool = int(m_sel.group(1))
                    continue
                if line == "M15":
                    is_slot = True
                    slot_start = (cur_x, cur_y)
                    continue
                if line == "M16":
                    is_slot = False
                    slot_start = None
                    continue
                if line.startswith("G05") or line.startswith("G90") or line.startswith("G91"):
                    continue
                if line.startswith("M30"):
                    break

                def parse_coord(val_str):
                    if "." in val_str:
                        return float(val_str)
                    else:
                        return float(val_str) / (10.0 ** self.dec_places)

                # Check for EasyEDA G85 canned cycle slot: X...Y...G85X...Y...
                m_g85 = re.search(r"X([\+\-]?[0-9\.]+)Y([\+\-]?[0-9\.]+)G85X([\+\-]?[0-9\.]+)Y([\+\-]?[0-9\.]+)", line)
                if m_g85:
                    x1 = parse_coord(m_g85.group(1)) * self.unit_scale
                    y1 = parse_coord(m_g85.group(2)) * self.unit_scale
                    x2 = parse_coord(m_g85.group(3)) * self.unit_scale
                    y2 = parse_coord(m_g85.group(4)) * self.unit_scale
                    all_x.extend([x1, x2])
                    all_y.extend([y1, y2])
                    self.slots.append((cur_tool, x1, y1, x2, y2))
                    cur_x, cur_y = x2, y2
                    continue

                mx = re.search(r"X([\+\-]?[0-9\.]+)", line)
                my = re.search(r"Y([\+\-]?[0-9\.]+)", line)

                if mx:
                    cur_x = parse_coord(mx.group(1)) * self.unit_scale
                if my:
                    cur_y = parse_coord(my.group(1)) * self.unit_scale

                if "X" in line or "Y" in line:
                    all_x.append(cur_x)
                    all_y.append(cur_y)

                    if is_slot and slot_start:
                        self.slots.append((cur_tool, slot_start[0], slot_start[1], cur_x, cur_y))
                        slot_start = (cur_x, cur_y)
                    else:
                        if not line.startswith("G00"):
                            self.holes.append((cur_tool, cur_x, cur_y))

        if all_x and all_y:
            self.raw_bounds = (min(all_x), min(all_y), max(all_x), max(all_y))

        return {
            "tools": self.tools,
            "holes": self.holes,
            "slots": self.slots,
            "bounds": self.raw_bounds
        }


# -----------------------------------------------------------------------------
# Coordinate Transformer
# -----------------------------------------------------------------------------

def transform_coords(x, y, master_bounds, margin=0.0, mirror_x=False):
    """
    Transform raw Gerber coordinates into CNC workpiece space.
    - If unmirrored (Top Copper, Cutout, Drills): X' = (X - minx) + margin, Y' = (Y - miny) + margin
    - If mirrored (Bottom Copper, Mirrored Drills): X' = (maxx - X) + margin, Y' = (Y - miny) + margin
    This ensures that in both top and bottom views, the PCB lower-left corner is at (0,0).
    """
    minx, miny, maxx, maxy = master_bounds
    if mirror_x:
        tx = (maxx - x) + margin
    else:
        tx = (x - minx) + margin
    ty = (y - miny) + margin
    return tx, ty


def transform_pts_list(pts, master_bounds, margin=0.0, mirror_x=False):
    return [transform_coords(x, y, master_bounds, margin, mirror_x) for x, y in pts]


def transform_shapely_geometry(geom, master_bounds, margin=0.0, mirror_x=False):
    if geom is None or geom.is_empty:
        return geom
    minx, miny, maxx, maxy = master_bounds
    if mirror_x:
        return shapely.affinity.affine_transform(
            geom, [-1.0, 0.0, 0.0, 1.0, maxx + margin, -miny + margin]
        )
    else:
        return shapely.affinity.affine_transform(
            geom, [1.0, 0.0, 0.0, 1.0, -minx + margin, -miny + margin]
        )


# -----------------------------------------------------------------------------
# Isolation Toolpath Generator
# -----------------------------------------------------------------------------

class ToolpathGenerator:
    def __init__(self, copper_geom, master_bounds=None, tool_dia=0.2, passes=1,
                 stepover=0.7, mirror_x=False, zero_origin=True, margin=0.0,
                 edge_engrave_paths=None):
        self.copper = copper_geom
        self.master_bounds = master_bounds or copper_geom.bounds
        self.tool_dia = tool_dia
        self.tool_rad = tool_dia / 2.0
        self.passes = max(1, passes)
        self.stepover = stepover
        self.mirror_x = mirror_x
        self.zero_origin = zero_origin
        self.margin = margin
        self.edge_engrave_paths = edge_engrave_paths or []
        self.transformed_copper = None
        self.transformed_edge_paths = []
        self.bounds = None

    def _extract_rings(self, geom):
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
        if self.copper is None or self.copper.is_empty:
            return [], [], 0.0

        minx, miny, maxx, maxy = self.master_bounds
        if self.zero_origin:
            geom = transform_shapely_geometry(self.copper, self.master_bounds, self.margin, self.mirror_x)
            self.bounds = (self.margin, self.margin, self.margin + (maxx - minx), self.margin + (maxy - miny))
        else:
            geom = self.copper
            if self.mirror_x:
                geom = shapely.affinity.affine_transform(
                    geom, [-1.0, 0.0, 0.0, 1.0, maxx + minx, 0.0]
                )
            self.bounds = geom.bounds

        # Persist transformed copper geometry for SVG preview
        self.transformed_copper = geom

        raw_rings = []
        for pass_idx in range(self.passes):
            offset = self.tool_rad + pass_idx * (self.tool_dia * self.stepover)
            iso_geom = geom.buffer(offset)
            iso_geom = clean_geometry(iso_geom)
            if iso_geom:
                raw_rings.extend(self._extract_rings(iso_geom))

        for ep in self.edge_engrave_paths:
            t_ep = transform_pts_list(ep, self.master_bounds, self.margin, self.mirror_x)
            self.transformed_edge_paths.append(t_ep)

        optimized_loops, total_rapid = self._optimize_paths(raw_rings)
        return optimized_loops, self.transformed_edge_paths, total_rapid

    def _optimize_paths(self, rings, start_pos=(0.0, 0.0)):
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
# G-Code Writers
# -----------------------------------------------------------------------------

class GcodeWriter:
    def __init__(self, output_path, loops, total_rapid_dist, bounds,
                 tool_dia=0.2, cut_depth=-0.04, safe_z=5.0, travel_z=1.0,
                 feed_rate=300.0, plunge_rate=80.0, spindle_speed=10000,
                 source_filename="", mirrored=False, zeroed=True,
                 edge_engrave_loops=None):
        self.output_path = output_path
        self.loops = loops
        self.total_rapid_dist = total_rapid_dist
        self.bounds = bounds
        self.tool_dia = tool_dia
        self.cut_depth = cut_depth
        self.safe_z = safe_z
        self.feed_rate = feed_rate
        self.plunge_rate = plunge_rate
        self.spindle_speed = spindle_speed
        self.source_filename = source_filename
        self.mirrored = mirrored
        self.zeroed = zeroed
        self.edge_engrave_loops = edge_engrave_loops or []

    def write(self):
        total_cut_dist = sum(
            sum(math.hypot(p2[0] - p1[0], p2[1] - p1[1]) for p1, p2 in zip(loop[:-1], loop[1:]))
            for loop in self.loops
        )
        edge_cut_dist = sum(
            sum(math.hypot(p2[0] - p1[0], p2[1] - p1[1]) for p1, p2 in zip(loop[:-1], loop[1:]))
            for loop in self.edge_engrave_loops
        )
        total_cut_dist += edge_cut_dist

        total_plunges = len(self.loops) + len(self.edge_engrave_loops)
        rapid_rate = 1000.0
        est_time_min = (
            (total_cut_dist / self.feed_rate) +
            (total_plunges * (self.safe_z - self.cut_depth) / self.plunge_rate) +
            (self.total_rapid_dist / rapid_rate) +
            0.1
        )

        minx, miny, maxx, maxy = self.bounds if self.bounds else (0, 0, 0, 0)
        width = maxx - minx
        height = maxy - miny

        lines = [
            "( ========================================================== )",
            "(  Generated by Gerber2GCode - EasyEDA Isolation Router      )",
            f"(  Source File      : {self.source_filename} )",
            f"(  Date             : {time.strftime('%Y-%m-%d %H:%M:%S')} )",
            f"(  Tool Tip Dia     : {self.tool_dia:.3f} mm )",
            f"(  Milling Cut Depth: {self.cut_depth:.4f} mm )",
            f"(  Safe Height (Z)  : {self.safe_z:.2f} mm )",
            f"(  Feed Rate (XY)   : {self.feed_rate:.1f} mm/min )",
            f"(  Plunge Rate (Z)  : {self.plunge_rate:.1f} mm/min )",
            f"(  Spindle Speed    : {self.spindle_speed} RPM )",
            f"(  Mirrored X       : {'YES (Bottom Layer)' if self.mirrored else 'NO (Top Layer)'} )",
            f"(  Zeroed Origin    : {'YES (Workpiece 0,0)' if self.zeroed else 'NO'} )",
            f"(  Edge Engrave     : {'YES (' + str(len(self.edge_engrave_loops)) + ' loops)' if self.edge_engrave_loops else 'NO'} )",
            f"(  Board Dimensions : {width:.2f} mm x {height:.2f} mm )",
            f"(  Est. Machining   : {est_time_min:.1f} min )",
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
            lines.append(f"( --- Isolation Contour {idx}/{len(self.loops)} --- )")
            lines.append(f"G00 X{start_x:.4f} Y{start_y:.4f}")
            lines.append(f"G01 Z{self.cut_depth:.4f} F{self.plunge_rate:.1f}")
            for pt in loop[1:]:
                lines.append(f"G01 X{pt[0]:.4f} Y{pt[1]:.4f} F{self.feed_rate:.1f}")
            lines.append(f"G00 Z{self.safe_z:.4f}")
            lines.append("")

        if self.edge_engrave_loops:
            lines.extend([
                "( ========================================================== )",
                "(  BOARD EDGE OUTLINE ENGRAVE PASS                           )",
                f"(  Engraving Depth: {self.cut_depth:.4f} mm | Feed: {self.feed_rate:.1f} mm/min )",
                "( ========================================================== )",
                ""
            ])
            for e_idx, e_loop in enumerate(self.edge_engrave_loops, 1):
                if not e_loop:
                    continue
                lines.append(f"( --- Edge Outline Loop {e_idx}/{len(self.edge_engrave_loops)} --- )")
                lines.append(f"G00 X{e_loop[0][0]:.4f} Y{e_loop[0][1]:.4f}")
                lines.append(f"G01 Z{self.cut_depth:.4f} F{self.plunge_rate:.1f}")
                for pt in e_loop[1:]:
                    lines.append(f"G01 X{pt[0]:.4f} Y{pt[1]:.4f} F{self.feed_rate:.1f}")
                lines.append(f"G00 Z{self.safe_z:.4f}")
                lines.append("")

        lines.extend([
            "( --- Program Finish --- )",
            f"G00 Z{self.safe_z:.4f}",
            "M05          (Spindle Stop)",
            "G00 X0.0000 Y0.0000 (Return to Workpiece Origin)",
            "M02          (End of Program)"
        ])

        with open(self.output_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

        return {
            "contours": len(self.loops) + len(self.edge_engrave_loops),
            "cut_dist_mm": total_cut_dist,
            "rapid_dist_mm": self.total_rapid_dist,
            "est_time_min": est_time_min,
            "width_mm": width,
            "height_mm": height
        }


class DrillGcodeWriter:
    def __init__(self, output_path, master_bounds, drill_data=None, tools=None, holes=None, slots=None,
                 drill_depth=-1.8, drill_feed=150.0, safe_z=5.0, tool_change_z=20.0,
                 spindle_speed=10000, mirror_x=False, margin=0.0,
                 source_filename=""):
        self.output_path = output_path
        self.master_bounds = master_bounds
        if drill_data is not None:
            self.tools = dict(drill_data.get("tools", {}))
            self.holes = list(drill_data.get("holes", []))
            self.slots = list(drill_data.get("slots", []))
        else:
            self.tools = dict(tools or {})
            self.holes = list(holes or [])
            self.slots = list(slots or [])
        self.drill_depth = drill_depth
        self.drill_feed = drill_feed
        self.safe_z = safe_z
        self.tool_change_z = tool_change_z
        self.spindle_speed = spindle_speed
        self.mirror_x = mirror_x
        self.margin = margin
        self.source_filename = source_filename
        self.transformed_by_tool = {}

    def _sort_holes(self, holes, start_pos=(0.0, 0.0)):
        unvisited = list(holes)
        sorted_holes = []
        cur_x, cur_y = start_pos
        total_rapid = 0.0

        while unvisited:
            best_idx = 0
            best_dist_sq = float("inf")
            for i, (hx, hy) in enumerate(unvisited):
                d_sq = (hx - cur_x) ** 2 + (hy - cur_y) ** 2
                if d_sq < best_dist_sq:
                    best_dist_sq = d_sq
                    best_idx = i
            total_rapid += math.sqrt(best_dist_sq)
            nxt = unvisited.pop(best_idx)
            sorted_holes.append(nxt)
            cur_x, cur_y = nxt

        return sorted_holes, total_rapid

    def write(self):
        for t_id in self.tools:
            self.transformed_by_tool[t_id] = {"holes": [], "slots": []}

        for t_id, raw_x, raw_y in self.holes:
            tx, ty = transform_coords(raw_x, raw_y, self.master_bounds, self.margin, self.mirror_x)
            if t_id not in self.transformed_by_tool:
                self.transformed_by_tool[t_id] = {"holes": [], "slots": []}
            self.transformed_by_tool[t_id]["holes"].append((tx, ty))

        for t_id, x1, y1, x2, y2 in self.slots:
            tx1, ty1 = transform_coords(x1, y1, self.master_bounds, self.margin, self.mirror_x)
            tx2, ty2 = transform_coords(x2, y2, self.master_bounds, self.margin, self.mirror_x)
            if t_id not in self.transformed_by_tool:
                self.transformed_by_tool[t_id] = {"holes": [], "slots": []}
            self.transformed_by_tool[t_id]["slots"].append((tx1, ty1, tx2, ty2))

        total_holes = sum(len(d["holes"]) for d in self.transformed_by_tool.values())
        total_slots = sum(len(d["slots"]) for d in self.transformed_by_tool.values())
        total_rapid_dist = 0.0

        minx, miny, maxx, maxy = self.master_bounds
        width = maxx - minx
        height = maxy - miny

        lines = [
            "( ========================================================== )",
            "(  Generated by Gerber2GCode - EasyEDA PCB Drilling Router  )",
            f"(  Source File      : {self.source_filename} )",
            f"(  Date             : {time.strftime('%Y-%m-%d %H:%M:%S')} )",
            f"(  Drill Depth      : {self.drill_depth:.4f} mm )",
            f"(  Drill Plunge Feed: {self.drill_feed:.1f} mm/min )",
            f"(  Safe Height (Z)  : {self.safe_z:.2f} mm )",
            f"(  Tool Change Z    : {self.tool_change_z:.2f} mm )",
            f"(  Spindle Speed    : {self.spindle_speed} RPM )",
            f"(  Mirrored X       : {'YES (Bottom-Side Drilling)' if self.mirror_x else 'NO (Top-Side Drilling)'} )",
            f"(  Board Dimensions : {width:.2f} mm x {height:.2f} mm )",
            f"(  Total Holes      : {total_holes} )",
            f"(  Total Slots      : {total_slots} )",
            "(  Tool Table:                                               )"
        ]

        for t_id in sorted(self.tools.keys()):
            dia = self.tools[t_id]
            h_cnt = len(self.transformed_by_tool.get(t_id, {}).get("holes", []))
            s_cnt = len(self.transformed_by_tool.get(t_id, {}).get("slots", []))
            lines.append(f"(    T{t_id:02d}: Dia {dia:6.3f} mm -> {h_cnt} holes, {s_cnt} slots )")

        lines.extend([
            "( ========================================================== )",
            "",
            "G21          (Units in Millimeters)",
            "G90          (Absolute Coordinates)",
            "G94          (Feed Rate per Minute)",
            "G17          (XY Work Plane Selection)",
            f"G00 Z{self.safe_z:.4f} (Rapid to Safe Z)",
            ""
        ])

        cur_pos = (0.0, 0.0)
        for t_id in sorted(self.tools.keys()):
            tool_data = self.transformed_by_tool.get(t_id, {})
            t_holes = tool_data.get("holes", [])
            t_slots = tool_data.get("slots", [])
            if not t_holes and not t_slots:
                continue

            dia = self.tools[t_id]
            lines.extend([
                f"( ------------------------------------------------------------- )",
                f"(  TOOL CHANGE: T{t_id} - Diameter: {dia:.3f} mm ({len(t_holes)} holes, {len(t_slots)} slots)  )",
                f"( ------------------------------------------------------------- )",
                "M05          (Spindle Stop for Tool Change)",
                f"G00 Z{self.tool_change_z:.4f} (Retract to Tool Change Height)",
                f"M00 (MSG, Insert Drill Bit T{t_id} Dia {dia:.3f}mm, Zero Z, Resume)",
                f"M03 S{self.spindle_speed} (Spindle Start CW)",
                "G04 P2.0     (Dwell 2.0s for Spindle Ramp-Up)",
                ""
            ])

            sorted_holes, rapid = self._sort_holes(t_holes, start_pos=cur_pos)
            total_rapid_dist += rapid

            for hx, hy in sorted_holes:
                lines.append(f"G00 X{hx:.4f} Y{hy:.4f}")
                lines.append(f"G01 Z{self.drill_depth:.4f} F{self.drill_feed:.1f}")
                lines.append(f"G00 Z{self.safe_z:.4f}")
                cur_pos = (hx, hy)

            for s_idx, (sx1, sy1, sx2, sy2) in enumerate(t_slots, 1):
                lines.append(f"( Slot {s_idx} )")
                lines.append(f"G00 X{sx1:.4f} Y{sy1:.4f}")
                lines.append(f"G01 Z{self.drill_depth:.4f} F{self.drill_feed:.1f}")
                lines.append(f"G01 X{sx2:.4f} Y{sy2:.4f} F{self.drill_feed * 0.8:.1f}")
                lines.append(f"G00 Z{self.safe_z:.4f}")
                cur_pos = (sx2, sy2)

            lines.append("")

        lines.extend([
            "( --- Drilling Completed --- )",
            f"G00 Z{self.tool_change_z:.4f}",
            "M05          (Spindle Stop)",
            "G00 X0.0000 Y0.0000 (Return to Workpiece Origin)",
            "M02          (End of Program)"
        ])

        with open(self.output_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

        est_time_min = (
            (total_holes * 2.0 * (self.safe_z - self.drill_depth) / self.drill_feed) +
            (total_rapid_dist / 1000.0) +
            (len([t for t in self.tools if self.transformed_by_tool.get(t, {}).get("holes")]) * 0.5)
        )

        return {
            "total_holes": total_holes,
            "total_slots": total_slots,
            "rapid_dist_mm": total_rapid_dist,
            "est_time_min": est_time_min
        }


class CutoutGcodeWriter:
    def __init__(self, output_path, edge_paths, master_bounds,
                 cutout_depth=-1.8, passes=3, feed_rate=200.0, plunge_rate=80.0,
                 safe_z=5.0, spindle_speed=10000, tool_dia=1.0,
                 mirror_x=False, margin=0.0, source_filename=""):
        self.output_path = output_path
        self.edge_paths = edge_paths
        self.master_bounds = master_bounds
        self.cutout_depth = cutout_depth
        self.passes = max(1, passes)
        self.feed_rate = feed_rate
        self.plunge_rate = plunge_rate
        self.safe_z = safe_z
        self.spindle_speed = spindle_speed
        self.tool_dia = tool_dia
        self.mirror_x = mirror_x
        self.margin = margin
        self.source_filename = source_filename
        self.transformed_paths = []

    def write(self):
        self.transformed_paths = [
            transform_pts_list(ep, self.master_bounds, self.margin, self.mirror_x)
            for ep in self.edge_paths
        ]

        total_cut_dist = 0.0
        depth_step = self.cutout_depth / self.passes

        lines = [
            "( ========================================================== )",
            "(  Generated by Gerber2GCode - Board Cutout Router          )",
            f"(  Source File      : {self.source_filename} )",
            f"(  Date             : {time.strftime('%Y-%m-%d %H:%M:%S')} )",
            f"(  Cutout Depth     : {self.cutout_depth:.4f} mm )",
            f"(  Number of Passes : {self.passes} (Step: {depth_step:.4f} mm) )",
            f"(  Tool Tip Dia     : {self.tool_dia:.3f} mm )",
            f"(  XY Feed Rate     : {self.feed_rate:.1f} mm/min )",
            f"(  Plunge Feed Rate : {self.plunge_rate:.1f} mm/min )",
            f"(  Safe Height (Z)  : {self.safe_z:.2f} mm )",
            f"(  Mirrored X       : {'YES (Mirrored Cutout)' if self.mirror_x else 'NO (Top-Side Cutout)'} )",
            "( ========================================================== )",
            "",
            "G21          (Units in Millimeters)",
            "G90          (Absolute Coordinates)",
            "G94          (Feed Rate per Minute)",
            "G17          (XY Work Plane Selection)",
            f"G00 Z{self.safe_z:.4f} (Rapid to Safe Z)",
            f"M03 S{self.spindle_speed} (Spindle Start CW)",
            "G04 P2.0     (Dwell 2 seconds for spindle ramp-up)",
            ""
        ]

        for p_idx, path in enumerate(self.transformed_paths, 1):
            if len(path) < 2:
                continue
            start_x, start_y = path[0]
            lines.append(f"G00 X{start_x:.4f} Y{start_y:.4f}")

            for pass_no in range(1, self.passes + 1):
                cur_z = depth_step * pass_no
                lines.append(f"( --- Pass {pass_no}/{self.passes} at Z={cur_z:.4f} --- )")
                lines.append(f"G01 Z{cur_z:.4f} F{self.plunge_rate:.1f}")
                for pt in path[1:]:
                    lines.append(f"G01 X{pt[0]:.4f} Y{pt[1]:.4f} F{self.feed_rate:.1f}")
                total_cut_dist += sum(
                    math.hypot(p2[0] - p1[0], p2[1] - p1[1]) for p1, p2 in zip(path[:-1], path[1:])
                )

            lines.append(f"G00 Z{self.safe_z:.4f}")
            lines.append("")

        lines.extend([
            "( --- Cutout Completed --- )",
            f"G00 Z{self.safe_z:.4f}",
            "M05          (Spindle Stop)",
            "G00 X0.0000 Y0.0000 (Return to Workpiece Origin)",
            "M02          (End of Program)"
        ])

        with open(self.output_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

        est_time_min = (total_cut_dist / self.feed_rate) + (self.passes * 0.1)
        return {
            "cut_dist_mm": total_cut_dist,
            "est_time_min": est_time_min
        }


# -----------------------------------------------------------------------------
# SVG Preview Generator
# -----------------------------------------------------------------------------

def export_svg_preview(svg_path, master_bounds, margin=0.0, title="",
                       copper_geom=None, milling_loops=None, edge_loops=None,
                       holes=None, slots=None, mirrored=False):
    minx, miny, maxx, maxy = master_bounds
    width = maxx - minx
    height = maxy - miny

    vb_pad = max(8.0, margin + 6.0)
    vb_x = -vb_pad
    vb_y = -height - vb_pad
    vb_w = width + 2 * vb_pad
    vb_h = height + 2 * vb_pad

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vb_x:.2f} {vb_y:.2f} {vb_w:.2f} {vb_h:.2f}" width="1100" height="{1100 * vb_h / vb_w:.1f}" style="background:#090d16; font-family: monospace;">',
        '  <defs>',
        '    <pattern id="grid" width="10" height="10" patternUnits="userSpaceOnUse">',
        '      <path d="M 10 0 L 0 0 0 10" fill="none" stroke="#172033" stroke-width="0.25"/>',
        '    </pattern>',
        '  </defs>',
        f'  <rect x="{vb_x:.2f}" y="{vb_y:.2f}" width="{vb_w:.2f}" height="{vb_h:.2f}" fill="url(#grid)" />'
    ]

    bx = margin
    by = margin
    svg.append(f'  <rect x="{bx:.2f}" y="{-by - height:.2f}" width="{width:.2f}" height="{height:.2f}" fill="#102a20" stroke="#059669" stroke-width="0.3" rx="0.5" />')

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
            svg.append(f'  <path d="{d_str}" fill="#d97706" fill-rule="evenodd" opacity="0.88" />')
        elif copper_geom.geom_type == "MultiPolygon":
            for p in copper_geom.geoms:
                d_str = poly_to_svg_path(p)
                svg.append(f'  <path d="{d_str}" fill="#d97706" fill-rule="evenodd" opacity="0.88" />')

    if milling_loops:
        for loop in milling_loops:
            if len(loop) < 2:
                continue
            d = "M " + " L ".join(f"{x:.3f},{-y:.3f}" for x, y in loop)
            svg.append(f'  <path d="{d}" fill="none" stroke="#22d3ee" stroke-width="0.25" opacity="0.9" />')
            svg.append(f'  <circle cx="{loop[0][0]:.3f}" cy="{-loop[0][1]:.3f}" r="0.25" fill="#ef4444" />')

    if edge_loops:
        for el in edge_loops:
            if len(el) < 2:
                continue
            d = "M " + " L ".join(f"{x:.3f},{-y:.3f}" for x, y in el)
            svg.append(f'  <path d="{d}" fill="none" stroke="#facc15" stroke-width="0.4" stroke-dasharray="2,1" opacity="0.95" />')
            svg.append(f'  <circle cx="{el[0][0]:.3f}" cy="{-el[0][1]:.3f}" r="0.3" fill="#facc15" />')

    if slots:
        for sx1, sy1, sx2, sy2, dia in slots:
            svg.append(f'  <line x1="{sx1:.3f}" y1="{-sy1:.3f}" x2="{sx2:.3f}" y2="{-sy2:.3f}" stroke="#38bdf8" stroke-width="{dia:.3f}" stroke-linecap="round" />')

    if holes:
        for hx, hy, dia in holes:
            svg.append(f'  <circle cx="{hx:.3f}" cy="{-hy:.3f}" r="{dia/2.0:.3f}" fill="#38bdf8" stroke="#ffffff" stroke-width="0.1" />')
            svg.append(f'  <circle cx="{hx:.3f}" cy="{-hy:.3f}" r="0.12" fill="#090d16" />')

    # Origin (0,0) indicator
    svg.append('  <circle cx="0" cy="0" r="1.2" fill="none" stroke="#22c55e" stroke-width="0.35" />')
    svg.append('  <line x1="-4" y1="0" x2="4" y2="0" stroke="#22c55e" stroke-width="0.25" />')
    svg.append('  <line x1="0" y1="-4" x2="0" y2="4" stroke="#22c55e" stroke-width="0.25" />')
    svg.append('  <text x="1.5" y="4.5" fill="#22c55e" font-size="2.4" font-weight="bold">(0,0) Origin</text>')
    svg.append(f'  <text x="{margin:.2f}" y="{-height - margin - 2.5:.2f}" fill="#94a3b8" font-size="2.4">{title} | {width:.1f} x {height:.1f} mm {"(Mirrored)" if mirrored else "(Top View)"}</text>')

    svg.append('</svg>')

    with open(svg_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg) + "\n")


# -----------------------------------------------------------------------------
# Layer Classification
# -----------------------------------------------------------------------------

def detect_layer_type(filename):
    """
    Categorize files specifically identifying EasyEDA and KiCad layers:
    - Outline / Cutout: Gerber_BoardOutlineLayer.GKO, .gm1, Edge_Cuts
    - Bottom Copper: Gerber_BottomLayer.GBL, .gbl, B_Cu
    - Top Copper: Gerber_TopLayer.GTL, .gtl, F_Cu
    - Drill: Drill_PTH_Through.DRL, Drill_NPTH_Through.DRL, .drl, .txt
    - Solder Masks (excluded): .GTS, .GBS, soldermask
    - Silkscreens (excluded): .GTO, .GBO, silkscreen
    """
    fn = filename.lower()
    ext = os.path.splitext(fn)[1]

    # Drill files
    if ext in (".drl", ".txt", ".xln", ".exc"):
        return "drill"

    # Board outline / Cutout
    if "outline" in fn or "boardoutline" in fn or "edge_cuts" in fn or "edge.cuts" in fn or "profile" in fn or ext in (".gm1", ".gko"):
        return "edge_cuts"

    # Solder masks (NOT copper - skip for isolation routing)
    if "soldermask" in fn or "solder_mask" in fn or "mask" in fn or ext in (".gts", ".gbs"):
        return "soldermask"

    # Silkscreen (NOT copper - skip for isolation routing)
    if "silkscreen" in fn or "silk" in fn or ext in (".gto", ".gbo"):
        return "silkscreen"

    # Bottom copper layer
    if ext == ".gbl" or "bottomlayer" in fn or "b_cu" in fn or "b.cu" in fn or "bottom" in fn or "bot" in fn:
        return "bottom_copper"

    # Top copper layer
    if ext == ".gtl" or "toplayer" in fn or "f_cu" in fn or "f.cu" in fn or "top" in fn:
        return "top_copper"

    if ext == ".gbr":
        return "gerber_generic"

    return "other"


# -----------------------------------------------------------------------------
# Batch Folder Processor
# -----------------------------------------------------------------------------

def process_folder(input_folder, output_folder, args):
    """Process an entire EasyEDA project folder and generate individual aligned G-code and SVG files."""
    os.makedirs(output_folder, exist_ok=True)
    files = sorted(glob.glob(os.path.join(input_folder, "*")))

    categorized = {
        "bottom_copper": [],
        "top_copper": [],
        "edge_cuts": [],
        "drill": [],
        "soldermask": [],
        "silkscreen": [],
        "gerber_generic": []
    }

    for f in files:
        if os.path.isdir(f):
            continue
        ltype = detect_layer_type(f)
        if ltype in categorized:
            categorized[ltype].append(f)

    print("==========================================================")
    print(" EasyEDA Gerber & Drill to G-Code Project Converter")
    print("==========================================================")
    print(f" Source Project : {input_folder}")
    print(f" Target G-Code  : {output_folder}")
    print(f" Tool Dia (Cut) : {args.tool_dia:.3f} mm")
    print(f" Cut Depth (Cu) : {args.cut_depth:.4f} mm")
    print(f" Drill Depth    : {args.drill_depth:.4f} mm")
    print(f" Cutout Depth   : {args.cutout_depth:.4f} mm")
    print(f" Margin (0,0)   : {args.margin:.2f} mm")
    print(f" Edge Engraving : {not args.no_engrave_edge} (Front & Back copper)")
    print("==========================================================")

    # 1. Determine unified Master Bounding Box from Edge_Cuts / GKO outline
    master_bounds = None
    edge_cuts_file = categorized["edge_cuts"][0] if categorized["edge_cuts"] else None
    edge_cut_raw_paths = []

    if edge_cuts_file:
        print(f"--> Parsing master board outline from '{os.path.basename(edge_cuts_file)}'...")
        p_edge = GerberParser(edge_cuts_file)
        p_edge.parse()
        if p_edge.raw_bounds:
            master_bounds = p_edge.raw_bounds
            edge_cut_raw_paths = p_edge.raw_paths
            print(f"    Master bounds set from Board Outline: X=[{master_bounds[0]:.2f}, {master_bounds[2]:.2f}], Y=[{master_bounds[1]:.2f}, {master_bounds[3]:.2f}]")

    if master_bounds is None:
        print("--> Computing master bounds from all copper and drill layers...")
        all_minx, all_miny, all_maxx, all_maxy = [], [], [], []
        for cf in (categorized["bottom_copper"] + categorized["top_copper"]):
            gp = GerberParser(cf)
            gp.parse()
            if gp.raw_bounds:
                all_minx.append(gp.raw_bounds[0])
                all_miny.append(gp.raw_bounds[1])
                all_maxx.append(gp.raw_bounds[2])
                all_maxy.append(gp.raw_bounds[3])
        for df in categorized["drill"]:
            dp = ExcellonParser(df)
            dp.parse()
            if dp.raw_bounds:
                all_minx.append(dp.raw_bounds[0])
                all_miny.append(dp.raw_bounds[1])
                all_maxx.append(dp.raw_bounds[2])
                all_maxy.append(dp.raw_bounds[3])

        if all_minx:
            master_bounds = (min(all_minx), min(all_miny), max(all_maxx), max(all_maxy))
        else:
            print("Error: Could not determine valid PCB bounding box.", file=sys.stderr)
            sys.exit(1)

    width = master_bounds[2] - master_bounds[0]
    height = master_bounds[3] - master_bounds[1]
    print(f"    Master Dimensions: {width:.2f} mm x {height:.2f} mm")

    if not edge_cut_raw_paths:
        minx, miny, maxx, maxy = master_bounds
        edge_cut_raw_paths = [[(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy), (minx, miny)]]

    top_edge_loops = [transform_pts_list(ep, master_bounds, args.margin, mirror_x=False) for ep in edge_cut_raw_paths]
    bot_edge_loops = [transform_pts_list(ep, master_bounds, args.margin, mirror_x=True) for ep in edge_cut_raw_paths]

    results = []

    # 2. Process Bottom Copper (Back Copper - Mirrored)
    for bf in categorized["bottom_copper"]:
        bname = os.path.splitext(os.path.basename(bf))[0]
        out_gcode = os.path.join(output_folder, f"{bname}.gcode")
        out_svg = os.path.join(output_folder, f"{bname}.svg")
        print(f"\n[Layer] Bottom Copper: {os.path.basename(bf)}")

        gp = GerberParser(bf)
        copper = gp.parse()
        engrave_loops = bot_edge_loops if not args.no_engrave_edge else []

        tg = ToolpathGenerator(
            copper_geom=copper,
            master_bounds=master_bounds,
            tool_dia=args.tool_dia,
            passes=args.passes,
            stepover=args.stepover,
            mirror_x=True,
            zero_origin=True,
            margin=args.margin
        )
        loops, _, rapid = tg.generate()

        gw = GcodeWriter(
            output_path=out_gcode,
            loops=loops,
            total_rapid_dist=rapid,
            bounds=tg.bounds,
            tool_dia=args.tool_dia,
            cut_depth=args.cut_depth,
            safe_z=args.safe_z,
            feed_rate=args.feed_rate,
            plunge_rate=args.plunge_rate,
            spindle_speed=args.spindle_speed,
            source_filename=os.path.basename(bf),
            mirrored=True,
            zeroed=True,
            edge_engrave_loops=engrave_loops
        )
        stats = gw.write()

        export_svg_preview(
            svg_path=out_svg,
            master_bounds=master_bounds,
            margin=args.margin,
            title=f"{bname} (Bottom Copper - Mirrored)",
            copper_geom=tg.transformed_copper,
            milling_loops=loops,
            edge_loops=bot_edge_loops,
            mirrored=True
        )

        op_desc = "Isolation + Edge Engrave (Bottom)" if engrave_loops else "Isolation (Bottom)"
        results.append((f"{bname}.gcode", op_desc, stats["contours"], stats["cut_dist_mm"], stats["est_time_min"]))

    # 3. Process Top Copper (Front Copper - Unmirrored)
    for tf in categorized["top_copper"]:
        bname = os.path.splitext(os.path.basename(tf))[0]
        out_gcode = os.path.join(output_folder, f"{bname}.gcode")
        out_svg = os.path.join(output_folder, f"{bname}.svg")
        print(f"\n[Layer] Top Copper: {os.path.basename(tf)}")

        gp = GerberParser(tf)
        copper = gp.parse()
        engrave_loops = top_edge_loops if not args.no_engrave_edge else []

        tg = ToolpathGenerator(
            copper_geom=copper,
            master_bounds=master_bounds,
            tool_dia=args.tool_dia,
            passes=args.passes,
            stepover=args.stepover,
            mirror_x=False,
            zero_origin=True,
            margin=args.margin
        )
        loops, _, rapid = tg.generate()

        gw = GcodeWriter(
            output_path=out_gcode,
            loops=loops,
            total_rapid_dist=rapid,
            bounds=tg.bounds,
            tool_dia=args.tool_dia,
            cut_depth=args.cut_depth,
            safe_z=args.safe_z,
            feed_rate=args.feed_rate,
            plunge_rate=args.plunge_rate,
            spindle_speed=args.spindle_speed,
            source_filename=os.path.basename(tf),
            mirrored=False,
            zeroed=True,
            edge_engrave_loops=engrave_loops
        )
        stats = gw.write()

        export_svg_preview(
            svg_path=out_svg,
            master_bounds=master_bounds,
            margin=args.margin,
            title=f"{bname} (Top Copper)",
            copper_geom=tg.transformed_copper,
            milling_loops=loops,
            edge_loops=top_edge_loops,
            mirrored=False
        )

        op_desc = "Isolation + Edge Engrave (Top)" if engrave_loops else "Isolation (Top)"
        results.append((f"{bname}.gcode", op_desc, stats["contours"], stats["cut_dist_mm"], stats["est_time_min"]))

    # 4. Process Individual Drill Files (Top & Bottom Mirrored)
    for df in categorized["drill"]:
        bname = os.path.splitext(os.path.basename(df))[0]
        print(f"\n[Layer] Drill File: {os.path.basename(df)}")
        dp = ExcellonParser(df)
        ddata = dp.parse()

        # Top view drilling
        out_gcode_top = os.path.join(output_folder, f"{bname}.gcode")
        out_svg_top = os.path.join(output_folder, f"{bname}.svg")
        dw_top = DrillGcodeWriter(
            output_path=out_gcode_top,
            master_bounds=master_bounds,
            drill_data=ddata,
            drill_depth=args.drill_depth,
            drill_feed=args.drill_feed,
            safe_z=args.safe_z,
            tool_change_z=args.tool_change_z,
            spindle_speed=args.spindle_speed,
            mirror_x=False,
            margin=args.margin,
            source_filename=os.path.basename(df)
        )
        stats_top = dw_top.write()

        top_holes = []
        for tid, h_list in dw_top.transformed_by_tool.items():
            dia = ddata["tools"].get(tid, 0.8)
            for hx, hy in h_list["holes"]:
                top_holes.append((hx, hy, dia))
        top_slots = []
        for tid, s_list in dw_top.transformed_by_tool.items():
            dia = ddata["tools"].get(tid, 0.8)
            for sx1, sy1, sx2, sy2 in s_list["slots"]:
                top_slots.append((sx1, sy1, sx2, sy2, dia))

        export_svg_preview(
            svg_path=out_svg_top,
            master_bounds=master_bounds,
            margin=args.margin,
            title=f"{bname} (Drills - Top View)",
            edge_loops=top_edge_loops,
            holes=top_holes,
            slots=top_slots,
            mirrored=False
        )

        # Mirrored drilling (for bottom-side drilling)
        out_gcode_bot = os.path.join(output_folder, f"{bname}-mirrored.gcode")
        out_svg_bot = os.path.join(output_folder, f"{bname}-mirrored.svg")
        dw_bot = DrillGcodeWriter(
            output_path=out_gcode_bot,
            master_bounds=master_bounds,
            drill_data=ddata,
            drill_depth=args.drill_depth,
            drill_feed=args.drill_feed,
            safe_z=args.safe_z,
            tool_change_z=args.tool_change_z,
            spindle_speed=args.spindle_speed,
            mirror_x=True,
            margin=args.margin,
            source_filename=os.path.basename(df)
        )
        stats_bot = dw_bot.write()

        bot_holes = []
        for tid, h_list in dw_bot.transformed_by_tool.items():
            dia = ddata["tools"].get(tid, 0.8)
            for hx, hy in h_list["holes"]:
                bot_holes.append((hx, hy, dia))
        bot_slots = []
        for tid, s_list in dw_bot.transformed_by_tool.items():
            dia = ddata["tools"].get(tid, 0.8)
            for sx1, sy1, sx2, sy2 in s_list["slots"]:
                bot_slots.append((sx1, sy1, sx2, sy2, dia))

        export_svg_preview(
            svg_path=out_svg_bot,
            master_bounds=master_bounds,
            margin=args.margin,
            title=f"{bname} (Drills - Bottom Mirrored)",
            edge_loops=bot_edge_loops,
            holes=bot_holes,
            slots=bot_slots,
            mirrored=True
        )

        results.append((f"{bname}.gcode", f"Drilling ({stats_top['total_holes']} holes)", stats_top["total_holes"], 0.0, stats_top["est_time_min"]))
        results.append((f"{bname}-mirrored.gcode", "Drilling Mirrored", stats_bot["total_holes"], 0.0, stats_bot["est_time_min"]))

    # 5. If Multiple Drill Files exist, also produce Deduplicated Unified Drill Program
    if len(categorized["drill"]) > 1:
        print(f"\n[Layer] Generating Unified Deduplicated Drill Program from {len(categorized['drill'])} drill files...")
        merged_tools = {}
        merged_holes = []
        merged_slots = []
        tool_id_map = {}

        for df in categorized["drill"]:
            dp = ExcellonParser(df)
            ddata = dp.parse()
            for old_tid, dia in ddata["tools"].items():
                found_tid = None
                for tid, d in merged_tools.items():
                    if abs(d - dia) < 1e-4:
                        found_tid = tid
                        break
                if found_tid is None:
                    new_tid = max(merged_tools.keys()) + 1 if merged_tools else 1
                    merged_tools[new_tid] = dia
                    found_tid = new_tid
                tool_id_map[(df, old_tid)] = found_tid

            for old_tid, hx, hy in ddata["holes"]:
                new_tid = tool_id_map[(df, old_tid)]
                dia = merged_tools[new_tid]
                # Deduplication check: check if hole at this location with same diameter already added
                is_dup = False
                for eh_tid, eh_x, eh_y in merged_holes:
                    if abs(merged_tools[eh_tid] - dia) < 1e-4 and math.hypot(eh_x - hx, eh_y - hy) < 0.02:
                        is_dup = True
                        break
                if not is_dup:
                    merged_holes.append((new_tid, hx, hy))

            for old_tid, x1, y1, x2, y2 in ddata["slots"]:
                new_tid = tool_id_map[(df, old_tid)]
                dia = merged_tools[new_tid]
                is_dup = False
                for es_tid, es_x1, es_y1, es_x2, es_y2 in merged_slots:
                    if abs(merged_tools[es_tid] - dia) < 1e-4 and math.hypot(es_x1 - x1, es_y1 - y1) < 0.02 and math.hypot(es_x2 - x2, es_y2 - y2) < 0.02:
                        is_dup = True
                        break
                if not is_dup:
                    merged_slots.append((new_tid, x1, y1, x2, y2))

        for is_mirrored, suffix, edge_l in [(False, "", top_edge_loops), (True, "-mirrored", bot_edge_loops)]:
            out_gcode = os.path.join(output_folder, f"Drill_Combined{suffix}.gcode")
            out_svg = os.path.join(output_folder, f"Drill_Combined{suffix}.svg")

            dw = DrillGcodeWriter(
                output_path=out_gcode,
                master_bounds=master_bounds,
                tools=merged_tools,
                holes=merged_holes,
                slots=merged_slots,
                drill_depth=args.drill_depth,
                drill_feed=args.drill_feed,
                safe_z=args.safe_z,
                tool_change_z=args.tool_change_z,
                spindle_speed=args.spindle_speed,
                mirror_x=is_mirrored,
                margin=args.margin,
                source_filename="Combined EasyEDA Drills (Deduplicated)"
            )
            stats_drill = dw.write()

            d_holes, d_slots = [], []
            for tid, h_list in dw.transformed_by_tool.items():
                dia = merged_tools.get(tid, 0.8)
                for hx, hy in h_list["holes"]:
                    d_holes.append((hx, hy, dia))
                for sx1, sy1, sx2, sy2 in h_list["slots"]:
                    d_slots.append((sx1, sy1, sx2, sy2, dia))

            export_svg_preview(
                svg_path=out_svg,
                master_bounds=master_bounds,
                margin=args.margin,
                title=f"Drill Combined {'(Mirrored)' if is_mirrored else '(Top View)'}",
                edge_loops=edge_l,
                holes=d_holes,
                slots=d_slots,
                mirrored=is_mirrored
            )
            results.append((f"Drill_Combined{suffix}.gcode", f"Combined Drills ({stats_drill['total_holes']} holes)", stats_drill["total_holes"], 0.0, stats_drill["est_time_min"]))

    # 6. Process Board Cutout (Edge Cuts / Outline) with Matching SVGs
    if edge_cuts_file:
        bname = os.path.splitext(os.path.basename(edge_cuts_file))[0]
        print(f"\n[Layer] Board Cutout: {os.path.basename(edge_cuts_file)}")

        # Top-side cutout
        out_gcode_top = os.path.join(output_folder, f"{bname}.gcode")
        out_svg_top = os.path.join(output_folder, f"{bname}.svg")
        cw_top = CutoutGcodeWriter(
            output_path=out_gcode_top,
            edge_paths=edge_cut_raw_paths,
            master_bounds=master_bounds,
            cutout_depth=args.cutout_depth,
            passes=args.cutout_passes,
            feed_rate=args.cutout_feed,
            plunge_rate=args.plunge_rate,
            safe_z=args.safe_z,
            spindle_speed=args.spindle_speed,
            tool_dia=args.cutout_tool_dia,
            mirror_x=False,
            margin=args.margin,
            source_filename=os.path.basename(edge_cuts_file)
        )
        stats_cut_top = cw_top.write()

        export_svg_preview(
            svg_path=out_svg_top,
            master_bounds=master_bounds,
            margin=args.margin,
            title=f"{bname} (Board Cutout - Top View)",
            edge_loops=top_edge_loops,
            mirrored=False
        )

        # Mirrored cutout
        out_gcode_bot = os.path.join(output_folder, f"{bname}-mirrored.gcode")
        out_svg_bot = os.path.join(output_folder, f"{bname}-mirrored.svg")
        cw_bot = CutoutGcodeWriter(
            output_path=out_gcode_bot,
            edge_paths=edge_cut_raw_paths,
            master_bounds=master_bounds,
            cutout_depth=args.cutout_depth,
            passes=args.cutout_passes,
            feed_rate=args.cutout_feed,
            plunge_rate=args.plunge_rate,
            safe_z=args.safe_z,
            spindle_speed=args.spindle_speed,
            tool_dia=args.cutout_tool_dia,
            mirror_x=True,
            margin=args.margin,
            source_filename=os.path.basename(edge_cuts_file)
        )
        stats_cut_bot = cw_bot.write()

        export_svg_preview(
            svg_path=out_svg_bot,
            master_bounds=master_bounds,
            margin=args.margin,
            title=f"{bname} (Board Cutout - Mirrored)",
            edge_loops=bot_edge_loops,
            mirrored=True
        )

        results.append((f"{bname}.gcode", f"Board Cutout ({args.cutout_passes} passes)", args.cutout_passes, stats_cut_top["cut_dist_mm"], stats_cut_top["est_time_min"]))
        results.append((f"{bname}-mirrored.gcode", "Board Cutout Mirrored", args.cutout_passes, stats_cut_bot["cut_dist_mm"], stats_cut_bot["est_time_min"]))

    # 7. Summary Report Table
    print("\n==========================================================================================")
    print(f" BATCH PROCESSING COMPLETED -> Output Folder: {output_folder}")
    print("==========================================================================================")
    print(f" {'G-Code File':<32} | {'SVG Preview':<32} | {'Length':<10} | {'Est. Time'}")
    print("------------------------------------------------------------------------------------------")
    total_time = 0.0
    for fname, op, feat, length, t_est in results:
        svg_name = fname.replace(".gcode", ".svg")
        len_str = f"{length:.1f} mm" if length > 0 else "-"
        print(f" {fname:<32} | {svg_name:<32} | {len_str:<10} | {t_est:4.1f} min")
        total_time += t_est
    print("------------------------------------------------------------------------------------------")
    print(f" Total Estimated Machining Time: {total_time:.1f} minutes")
    print(" Every G-code file has an accompanying visual SVG preview.")
    print(" Board edge outline is engraved onto both Front & Back copper G-code and SVGs.")
    print(" All files are zero-aligned to the lower-left board corner at (0.0000, 0.0000).")
    print("==========================================================================================\n")


# -----------------------------------------------------------------------------
# Single File Processor
# -----------------------------------------------------------------------------

def process_single_file(input_file, output_file, args):
    """Process a single Gerber or Drill file."""
    ltype = detect_layer_type(input_file)
    print("==========================================================")
    print(" EasyEDA Gerber / Drill to G-Code Converter")
    print("==========================================================")
    print(f" Input File : {input_file} (Detected type: {ltype})")
    print(f" Output File: {output_file}")
    print("==========================================================")

    out_svg = args.preview or (os.path.splitext(output_file)[0] + ".svg")

    if ltype == "drill":
        dp = ExcellonParser(input_file)
        ddata = dp.parse()
        dw = DrillGcodeWriter(
            output_path=output_file,
            master_bounds=ddata["bounds"],
            drill_data=ddata,
            drill_depth=args.drill_depth,
            drill_feed=args.drill_feed,
            safe_z=args.safe_z,
            tool_change_z=args.tool_change_z,
            spindle_speed=args.spindle_speed,
            mirror_x=bool(args.mirror),
            margin=args.margin,
            source_filename=os.path.basename(input_file)
        )
        stats = dw.write()

        holes = []
        for tid, h_list in dw.transformed_by_tool.items():
            dia = ddata["tools"].get(tid, 0.8)
            for hx, hy in h_list["holes"]:
                holes.append((hx, hy, dia))
        slots = []
        for tid, s_list in dw.transformed_by_tool.items():
            dia = ddata["tools"].get(tid, 0.8)
            for sx1, sy1, sx2, sy2 in s_list["slots"]:
                slots.append((sx1, sy1, sx2, sy2, dia))

        export_svg_preview(
            svg_path=out_svg,
            master_bounds=ddata["bounds"],
            margin=args.margin,
            title=os.path.basename(input_file),
            holes=holes,
            slots=slots,
            mirrored=bool(args.mirror)
        )
        print(f"Drill G-code written: {stats['total_holes']} holes, est. time {stats['est_time_min']:.1f} min.")
        print(f"SVG preview written : {out_svg}")

    else:
        is_bottom = (ltype == "bottom_copper")
        mirror = is_bottom if args.mirror is None else args.mirror

        gp = GerberParser(input_file)
        copper = gp.parse()
        if copper is None or copper.is_empty:
            print("Error: No copper geometry found.", file=sys.stderr)
            sys.exit(1)

        tg = ToolpathGenerator(
            copper_geom=copper,
            tool_dia=args.tool_dia,
            passes=args.passes,
            stepover=args.stepover,
            mirror_x=mirror,
            zero_origin=(not args.preserve_origin),
            margin=args.margin
        )
        loops, _, rapid = tg.generate()

        gw = GcodeWriter(
            output_path=output_file,
            loops=loops,
            total_rapid_dist=rapid,
            bounds=tg.bounds,
            tool_dia=args.tool_dia,
            cut_depth=args.cut_depth,
            safe_z=args.safe_z,
            feed_rate=args.feed_rate,
            plunge_rate=args.plunge_rate,
            spindle_speed=args.spindle_speed,
            source_filename=os.path.basename(input_file),
            mirrored=mirror,
            zeroed=(not args.preserve_origin)
        )
        stats = gw.write()

        export_svg_preview(
            svg_path=out_svg,
            master_bounds=tg.master_bounds,
            margin=args.margin,
            title=os.path.basename(input_file),
            copper_geom=tg.transformed_copper,
            milling_loops=loops,
            mirrored=mirror
        )
        print(f"G-code written : {stats['contours']} contours, {stats['cut_dist_mm']:.1f} mm cut length, est. time {stats['est_time_min']:.1f} min.")
        print(f"SVG preview written : {out_svg}")


# -----------------------------------------------------------------------------
# Main CLI Entry Point
# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="EasyEDA Gerber RS-274X and Excellon Drill to CNC Isolation G-Code Converter"
    )
    parser.add_argument("input", help="Input EasyEDA Gerber/Drill file OR project folder (e.g., ./project02)")
    parser.add_argument("output", nargs="?", default=None,
                        help="Output G-code file or folder (defaults to <FolderName>_gcode for folders)")

    # Milling / Isolation Parameters
    parser.add_argument("-t", "--tool-dia", type=float, default=0.2,
                        help="Milling bit tip diameter in mm (default: 0.2)")
    parser.add_argument("-z", "--cut-depth", type=float, default=-0.04,
                        help="Copper isolation cut depth Z in mm (default: -0.04)")
    parser.add_argument("--safe-z", type=float, default=5.0,
                        help="Safe travel height Z in mm (default: 5.0)")
    parser.add_argument("-f", "--feed-rate", type=float, default=300.0,
                        help="XY milling feed rate in mm/min (default: 300.0)")
    parser.add_argument("--plunge-rate", type=float, default=80.0,
                        help="Z plunge feed rate in mm/min (default: 80.0)")
    parser.add_argument("-s", "--spindle-speed", type=int, default=10000,
                        help="Spindle speed in RPM (default: 10000)")
    parser.add_argument("-p", "--passes", type=int, default=1,
                        help="Number of isolation passes (default: 1)")
    parser.add_argument("--stepover", type=float, default=0.7,
                        help="Stepover ratio for multi-pass isolation (default: 0.7)")

    # Board Edge Engraving
    parser.add_argument("--no-engrave-edge", action="store_true",
                        help="Do not engrave board edge outline onto Front and Back copper")

    # Drilling Parameters
    parser.add_argument("--drill-depth", type=float, default=-1.8,
                        help="Through-hole drill depth Z in mm (default: -1.8)")
    parser.add_argument("--drill-feed", type=float, default=150.0,
                        help="Z plunge feed rate for drilling in mm/min (default: 150.0)")
    parser.add_argument("--tool-change-z", type=float, default=20.0,
                        help="Z height for manual bit swap pauses (default: 20.0)")

    # Board Cutout Parameters
    parser.add_argument("--cutout-depth", type=float, default=-1.8,
                        help="Total board cutout depth Z in mm (default: -1.8)")
    parser.add_argument("--cutout-passes", type=int, default=3,
                        help="Number of Z depth passes for board cutout (default: 3)")
    parser.add_argument("--cutout-feed", type=float, default=200.0,
                        help="XY feed rate for board cutout in mm/min (default: 200.0)")
    parser.add_argument("--cutout-tool-dia", type=float, default=1.0,
                        help="End mill diameter for board cutout in mm (default: 1.0)")

    # Coordinate / Alignment Parameters
    parser.add_argument("--mirror", dest="mirror", action="store_true",
                        help="Mirror X coordinates (essential for bottom copper when milled copper-side up)")
    parser.add_argument("--no-mirror", dest="mirror", action="store_false",
                        help="Do not mirror X coordinates")
    parser.set_defaults(mirror=None)
    parser.add_argument("--preserve-origin", action="store_true",
                        help="Preserve raw Gerber coordinates instead of shifting bottom-left to (0,0)")
    parser.add_argument("--margin", type=float, default=0.0,
                        help="Margin in mm from (0,0) when zeroing origin (default: 0.0)")
    parser.add_argument("--preview", type=str, default=None,
                        help="Optional SVG preview output path (single-file mode)")

    args = parser.parse_args()

    input_path = os.path.normpath(args.input)
    if not os.path.exists(input_path):
        print(f"Error: Path '{input_path}' not found.", file=sys.stderr)
        sys.exit(1)

    if os.path.isdir(input_path):
        folder_basename = os.path.basename(input_path.rstrip(os.sep))
        output_folder = args.output if args.output else f"{folder_basename}_gcode"
        process_folder(input_path, output_folder, args)
    else:
        output_file = args.output
        if not output_file:
            base, _ = os.path.splitext(input_path)
            output_file = f"{base}.gcode"
        process_single_file(input_path, output_file, args)


if __name__ == "__main__":
    main()
