#!/usr/bin/env python3
"""
================================================================================
A4 Copper Board CNC PCB Panelizer & Multi-Step Manufacturing Pipeline
================================================================================
Features:
- Reads PCB project folders & project_metadata.json
- Tightly packs PCB designs (0.0mm gap) to allow single-pass perimeter cutting
- Rotates rectangular PCBs (0° or 90°) to maximize boards per sheet
- Attaches a 5mm breakaway tag bar strictly along the WIDTH side of the PCB design
  with single-stroke engraved Board ID, directional arrow pointing into PCB body,
  and mouse-bite drill perforations (0.8mm drill, 1.5mm pitch) for tool-less hand snap-off
- Prioritizes grouping boards by solder mask color onto dedicated sheets
- Engraves a dedicated Sheet Tag with sheet number and metadata on each A4 sheet
- 2-Step Y-axis milling workflow for 275x170mm CNC bed (Front Half and Back Half)
- Fixture clamp collision avoidance loaded from pcb_holders.json
- Standalone Step-by-Step Circuit Gerber Visualizer (SVGs for Drills, Vias, Top Cu,
  Bottom Cu, Solder Mask, Silkscreen, Snap Cutout, and Composite Stackup)
- Full G-code generation:
  * Step 0: Left alignment guides, millimeter scale, shift datum line, 3mm pin holes, sheet tag
  * Step 1: Multi-diameter top drilling with peck cycle (Front & Back halves)
  * Step 2: Chemical through-hole via plating guide
  * Step 3: Top copper isolation engraving (Front & Back halves)
  * Step 4: Bottom copper isolation engraving (X-mirrored for flipped board)
  * Step 5: Solder mask pad clearing (Top & Bottom, with M0 pause for +35µm Z-height touch-off)
  * Step 6: Silkscreen (# TODO: Future flatbed inkjet printer roadmap)
  * Step 7: Dual-sided snapable cutout (-0.85mm top & bottom, bridge tabs, single-pass shared cut)
- Outputs manufacture_metadata.json detailing all boards and sheets
- Generates interactive HTML dashboard in manufacture_guides/index.html with interactive
  setup checklist saving ticks directly into manufacture_guides/checklist_state.json
- FluidNC direct network upload integration (192.168.8.109)
"""

import os
import re
import sys
import glob
import math
import json
import argparse
import http.server
import socketserver
import urllib.parse
import urllib.request
import zipfile
import shutil
import subprocess
import email
import email.policy
import uuid
import time
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape as svg_escape
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional, Any

from shapely.geometry import Polygon, MultiPolygon, Point, box, LineString, MultiLineString
from shapely.affinity import translate, rotate
import shapely.ops as ops


# ---------------------------------------------------------------------------
# Shared Drill Palette & Vector SVG Helpers
# ---------------------------------------------------------------------------

DRILL_COLOR_MAP: Dict[float, str] = {
    0.3: "#22d3ee", 0.4: "#06b6d4", 0.5: "#38bdf8", 0.6: "#60a5fa",
    0.7: "#a78bfa", 0.75: "#c084fc", 0.8: "#d946ef", 0.9: "#fbbf24",
    1.0: "#f59e0b", 1.1: "#fb923c", 1.2: "#f97316", 1.3: "#ef4444",
    1.5: "#f43f5e", 1.8: "#e11d48", 2.0: "#ec4899", 2.5: "#10b981",
    3.0: "#34d399",
}


def get_drill_color(dia: float) -> str:
    """Return consistent high-contrast color hex for a drill diameter."""
    return DRILL_COLOR_MAP.get(round(float(dia), 2), "#38bdf8")


def sort_points_nearest_neighbor(pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Order 2D drill points greedily from (0,0) to minimize CNC rapid air travel."""
    if len(pts) <= 2:
        return list(pts)
    remaining = sorted((round(x, 4), round(y, 4)) for (x, y) in pts)
    ordered: List[Tuple[float, float]] = []
    cx, cy = 0.0, 0.0
    while remaining:
        best_i = 0
        best_d2 = float("inf")
        for i, (px, py) in enumerate(remaining):
            d2 = (px - cx) ** 2 + (py - cy) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        nxt = remaining.pop(best_i)
        ordered.append(nxt)
        cx, cy = nxt
    return ordered


def _shapely_geom_to_svg_path_d(geom, tr_pt_fn) -> str:
    """Convert any Shapely Polygon/MultiPolygon into a single SVG path string."""
    if geom is None or geom.is_empty:
        return ""
    polys = geom.geoms if hasattr(geom, "geoms") else [geom]
    d_parts: List[str] = []
    for p in polys:
        if hasattr(p, "exterior") and p.exterior is not None:
            ext_coords = [tr_pt_fn(pt[0], pt[1]) for pt in p.exterior.coords]
            if ext_coords:
                d_parts.append(
                    f"M {ext_coords[0][0]:.1f},{ext_coords[0][1]:.1f} "
                    + " ".join(f"L {pt[0]:.1f},{pt[1]:.1f}" for pt in ext_coords[1:])
                    + " Z"
                )
            for interior in p.interiors:
                int_coords = [tr_pt_fn(pt[0], pt[1]) for pt in interior.coords]
                if int_coords:
                    d_parts.append(
                        f"M {int_coords[0][0]:.1f},{int_coords[0][1]:.1f} "
                        + " ".join(f"L {pt[0]:.1f},{pt[1]:.1f}" for pt in int_coords[1:])
                        + " Z"
                    )
    return " ".join(d_parts)


def _polylines_to_svg_path_d(polylines: List[List[Tuple[float, float]]], tr_pt_fn) -> str:
    """Convert many open polylines into a single compact SVG path string."""
    parts: List[str] = []
    for poly in polylines:
        if len(poly) < 2:
            continue
        pts = [tr_pt_fn(p[0], p[1]) for p in poly]
        parts.append(
            f"M {pts[0][0]:.1f},{pts[0][1]:.1f} "
            + " ".join(f"L {q[0]:.1f},{q[1]:.1f}" for q in pts[1:])
        )
    return " ".join(parts)


# ---------------------------------------------------------------------------
# Single-Stroke Hershey Font Vectors for CNC Engraving
# ---------------------------------------------------------------------------

STROKE_FONT = {
    '0': [[(0,0),(1,0),(1,2),(0,2),(0,0),(1,2)]],
    '1': [[(0.5,0),(0.5,2),(0,1.5)]],
    '2': [[(0,2),(1,2),(1,1),(0,1),(0,0),(1,0)]],
    '3': [[(0,2),(1,2),(1,1),(0.2,1)],[(1,1),(1,0),(0,0)]],
    '4': [[(1,0),(1,2)],[(0,2),(0,1),(1,1)]],
    '5': [[(1,2),(0,2),(0,1),(1,1),(1,0),(0,0)]],
    '6': [[(1,2),(0,2),(0,0),(1,0),(1,1),(0,1)]],
    '7': [[(0,2),(1,2),(0.3,0)]],
    '8': [[(0,0),(1,0),(1,2),(0,2),(0,0)],[(0,1),(1,1)]],
    '9': [[(0,0),(1,0),(1,2),(0,2),(0,1),(1,1)]],
    'A': [[(0,0),(0.5,2),(1,0)],[(0.2,0.8),(0.8,0.8)]],
    'B': [[(0,0),(0,2),(0.8,2),(0.8,1),(0,1)],[(0.8,1),(0.8,0),(0,0)]],
    'C': [[(1,2),(0,2),(0,0),(1,0)]],
    'D': [[(0,0),(0,2),(0.7,2),(1,1.3),(1,0.7),(0.7,0),(0,0)]],
    'E': [[(1,2),(0,2),(0,0),(1,0)],[(0,1),(0.7,1)]],
    'F': [[(1,2),(0,2),(0,0)],[(0,1),(0.7,1)]],
    'G': [[(1,2),(0,2),(0,0),(1,0),(1,1),(0.5,1)]],
    'H': [[(0,0),(0,2)],[(1,0),(1,2)],[(0,1),(1,1)]],
    'I': [[(0.5,0),(0.5,2)],[(0.1,2),(0.9,2)],[(0.1,0),(0.9,0)]],
    'J': [[(0,0.5),(0.5,0),(1,0),(1,2)]],
    'K': [[(0,0),(0,2)],[(1,2),(0,1),(1,0)]],
    'L': [[(0,2),(0,0),(1,0)]],
    'M': [[(0,0),(0,2),(0.5,1),(1,2),(1,0)]],
    'N': [[(0,0),(0,2),(1,0),(1,2)]],
    'O': [[(0,0),(1,0),(1,2),(0,2),(0,0)]],
    'P': [[(0,0),(0,2),(1,2),(1,1),(0,1)]],
    'Q': [[(0,0),(1,0),(1,2),(0,2),(0,0)],[(0.5,0.5),(1.1,-0.2)]],
    'R': [[(0,0),(0,2),(1,2),(1,1),(0,1)],[(0.4,1),(1,0)]],
    'S': [[(1,2),(0,2),(0,1),(1,1),(1,0),(0,0)]],
    'T': [[(0.5,0),(0.5,2)],[(0,2),(1,2)]],
    'U': [[(0,2),(0,0),(1,0),(1,2)]],
    'V': [[(0,2),(0.5,0),(1,2)]],
    'W': [[(0,2),(0.25,0),(0.5,1),(0.75,0),(1,2)]],
    'X': [[(0,0),(1,2)],[(0,2),(1,0)]],
    'Y': [[(0,2),(0.5,1),(1,2)],[(0.5,1),(0.5,0)]],
    'Z': [[(0,2),(1,2),(0,0),(1,0)]],
    '-': [[(0.1,1),(0.9,1)]],
    '_': [[(0,0),(1,0)]],
    ':': [[(0.5,1.5),(0.5,1.6)],[(0.5,0.4),(0.5,0.5)]],
    '.': [[(0.4,0),(0.6,0)]],
    '/': [[(0,0),(1,2)]],
    ' ': [],
    '+': [[(0.1,1),(0.9,1)],[(0.5,0.2),(0.5,1.8)]],
    '|': [[(0.5,0),(0.5,2)]],
}

def text_to_polylines(text: str, x: float, y: float, char_h: float = 2.2, char_w: float = 1.4, spacing: float = 0.5) -> List[List[Tuple[float, float]]]:
    """Render string into 2D polyline toolpaths using single-stroke font."""
    strokes = []
    curr_x = x
    scale_x = char_w
    scale_y = char_h / 2.0
    for ch in text.upper():
        char_strokes = STROKE_FONT.get(ch, STROKE_FONT.get('?'))
        if char_strokes:
            for s in char_strokes:
                stroke_pts = [(curr_x + pt[0] * scale_x, y + pt[1] * scale_y) for pt in s]
                strokes.append(stroke_pts)
        curr_x += char_w + spacing
    return strokes


# ---------------------------------------------------------------------------
# Project Metadata & File Scanner
# ---------------------------------------------------------------------------

class ProjectInfo:
    """Encapsulates PCB project specifications, visual specs, and layer paths."""
    def __init__(self, folder_path, metadata):
        self.folder_path = os.path.abspath(folder_path)
        self.metadata = metadata
        
        pcb_info = metadata.get("pcb_info", {})
        self.pcb_id = pcb_info.get("pcb_id", os.path.basename(folder_path))
        self.project_name = pcb_info.get("project_name", self.pcb_id)
        self.revision = pcb_info.get("revision", "v1.0")
        
        batch = pcb_info.get("batch", {})
        self.quantity = int(batch.get("quantity", 1))
        self.unit_id_format = batch.get("unit_id_format", "{pcb_id}-{index:03d}")
        self.unit_ids = batch.get("pcb_unit_ids", [
            self.unit_id_format.format(pcb_id=self.pcb_id, index=i+1) for i in range(self.quantity)
        ])
        
        # Physical specs
        phy = metadata.get("physical_specs", {})
        dims = phy.get("dimensions_mm", {})
        self.width = float(dims.get("width", 50.0))
        self.height = float(dims.get("height", 50.0))
        self.thickness = float(dims.get("thickness", 1.6))
        self.shape = phy.get("shape", "rectangular").lower()

        # Visual specs (Solder mask color priority)
        vis = metadata.get("visual_specs", {})
        sm = vis.get("solder_mask", {})
        self.solder_mask_color = sm.get("top_color", "green").lower()
        self.silkscreen_color = vis.get("silkscreen", {}).get("top_color", "white").lower()
        
        # Layer file paths
        pcb_files = metadata.get("pcb_files", {})
        layers = pcb_files.get("layers", {})
        local_dir = os.path.join(self.folder_path, pcb_files.get("local_dir", "./"))
        
        self.outline_file = self._resolve_layer(local_dir, layers.get("outline", {}).get("file"), [
            "*Edge_Cuts*.gbr", "*BoardOutline*.GKO", "*.gm1", "*.gko"
        ])
        self.copper_top = self._resolve_layer(local_dir, layers.get("copper_top", {}).get("file"), [
            "*F_Cu*.gbr", "*TopLayer*.GTL", "*.gtl"
        ])
        self.copper_bottom = self._resolve_layer(local_dir, layers.get("copper_bottom", {}).get("file"), [
            "*B_Cu*.gbr", "*BottomLayer*.GBL", "*.gbl"
        ])
        self.mask_top = self._resolve_layer(local_dir, layers.get("mask_top", {}).get("file"), [
            "*F_Mask*.gbr", "*TopSolderMask*.GTS", "*.gts"
        ])
        self.mask_bottom = self._resolve_layer(local_dir, layers.get("mask_bottom", {}).get("file"), [
            "*B_Mask*.gbr", "*BottomSolderMask*.GBS", "*.gbs"
        ])
        self.silkscreen_top = self._resolve_layer(local_dir, layers.get("silkscreen_top", {}).get("file"), [
            "*F_Silkscreen*.gbr", "*F_Silk*.gbr", "*TopSilkscreen*.GTO", "*.gto"
        ])
        self.silkscreen_bottom = self._resolve_layer(local_dir, layers.get("silkscreen_bottom", {}).get("file"), [
            "*B_Silkscreen*.gbr", "*B_Silk*.gbr", "*BottomSilkscreen*.GBO", "*.gbo"
        ])
        
        # Drills
        self.drill_files = self._find_drill_files(local_dir, layers)
        
    def _resolve_layer(self, base_dir, declared_name, patterns):
        if declared_name:
            p = os.path.join(base_dir, declared_name)
            if os.path.exists(p):
                return p
            p = os.path.join(self.folder_path, declared_name)
            if os.path.exists(p):
                return p
        for pat in patterns:
            matches = glob.glob(os.path.join(self.folder_path, pat))
            if matches:
                return matches[0]
        return None

    def _find_drill_files(self, base_dir, layers):
        drls = []
        drills_info = layers.get("drills", {})
        if isinstance(drills_info, dict):
            for k, v in drills_info.items():
                if isinstance(v, dict) and "file" in v:
                    fp = os.path.join(base_dir, v["file"])
                    if os.path.exists(fp) and fp not in drls:
                        drls.append(fp)
        for ext in ["*.drl", "*.DRL", "*.txt", "*.xln"]:
            for match in glob.glob(os.path.join(self.folder_path, ext)):
                if match not in drls:
                    drls.append(match)
        return drls


def scan_projects(root_dir="."):
    """Discover all folders containing project_metadata.json."""
    projects = []
    search_dirs = [root_dir] + [
        os.path.join(root_dir, d) for d in os.listdir(root_dir)
        if os.path.isdir(os.path.join(root_dir, d)) and not d.startswith(".") and d not in ("pcb_env", "manufacture_gcode", "manufacture_svg", "manufacture_guides")
    ]
    for d in search_dirs:
        meta_path = os.path.join(d, "project_metadata.json")
        if os.path.exists(meta_path):
            try:
                with open(meta_path, "r", encoding="utf-8") as f:
                    meta = json.load(f)
                projects.append(ProjectInfo(d, meta))
            except Exception as e:
                print(f"[WARN] Error reading {meta_path}: {e}")
    return projects


# ---------------------------------------------------------------------------
# Fixture Clamping Keep-Out Manager (pcb_holders.json)
# ---------------------------------------------------------------------------

class HolderManager:
    """
    Manages common single-half CNC bed locking washer clamps and the fixed bed Z-zero limit switch danger zone.
    Since locking points belong to the CNC bed (one bed half), each clamp protects both the Front Half
    (y_bed) and the Y-shifted Back Half (y_bed + y_split) on an A4 sheet without drilling the PCB.
    """
    def __init__(self, json_path="pcb_holders.json", y_split: float = 145.0, bed_travel_y: float = 170.0,
                 config: Optional[Dict] = None):
        self.holders: List[Dict[str, Any]] = []
        self.exclusion_polygons = []
        self.y_split = float(y_split)
        self.bed_travel_y = float(bed_travel_y)
        self.config = config or {}
        self.z_probe_switch: Dict[str, Any] = {
            "enabled": True,
            "x_mm": 272.001,
            "y_mm": 135.002,
            "width_mm": 7.0,
            "length_mm": 20.0,
            "danger_margin_mm": 6.0,
            "label": "Z-ZERO LIMIT SWITCH (DANGER ZONE)",
        }
        self.load(json_path)

    def _build_holder_poly(self, h: Dict[str, Any], y_offset: float = 0.0):
        x = float(h.get("x", 0.0))
        y = float(h.get("y", 0.0)) + y_offset
        gap = float(h.get("holder_gap", 3.0))
        shape = h.get("shape", "circle").lower()
        if shape == "circle":
            dia = float(h.get("diameter", 10.0))
            rad = (dia / 2.0) + gap
            return Point(x, y).buffer(rad)
        w = float(h.get("width", 10.0))
        l = float(h.get("length", 20.0))
        hw = (w / 2.0) + gap
        hl = (l / 2.0) + gap
        return box(x - hw, y - hl, x + hw, y + hl)

    def load(self, json_path):
        file_z_sw = {}
        if os.path.exists(json_path):
            try:
                with open(json_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                self.holders = data.get("holders", [])
                file_z_sw = data.get("z_probe_limit_switch", {}) or {}
                for h in self.holders:
                    # Common bed locking point protects Front Half on bed AND Shifted Back Half (+y_split)
                    self.exclusion_polygons.append(self._build_holder_poly(h, y_offset=0.0))
                    is_common = bool(h.get("common_for_both_halves", True)) or (float(h.get("y", 0.0)) <= self.bed_travel_y)
                    if is_common:
                        self.exclusion_polygons.append(self._build_holder_poly(h, y_offset=self.y_split))
                print(f"[INFO] Loaded {len(self.holders)} common single-half bed washer clamps from {json_path}.")
            except Exception as e:
                print(f"[ERROR] Failed parsing {json_path}: {e}")
        else:
            print(f"[WARN] Holders config {json_path} not found. Operating with zero fixture keepouts.")

        cfg_z_sw = self.config.get("guides_and_fixturing", {}).get("z_probe_limit_switch", {}) or {}
        self.z_probe_switch.update(file_z_sw)
        self.z_probe_switch.update(cfg_z_sw)
        if self.z_probe_switch.get("enabled", True):
            sx = float(self.z_probe_switch.get("x_mm", 272.001))
            sy = float(self.z_probe_switch.get("y_mm", 135.002))
            sw = float(self.z_probe_switch.get("width_mm", 7.0)) / 2.0 + float(self.z_probe_switch.get("danger_margin_mm", 6.0))
            sl = float(self.z_probe_switch.get("length_mm", 20.0)) / 2.0 + float(self.z_probe_switch.get("danger_margin_mm", 6.0))
            self.exclusion_polygons.append(box(sx - sw, sy - sl, sx + sw, sy + sl))
            self.exclusion_polygons.append(box(sx - sw, sy + self.y_split - sl, sx + sw, sy + self.y_split + sl))

    def collides_with(self, geom) -> bool:
        """Check if geometry intersects any physical bed washer clamp or Z-switch danger zone."""
        for ex in self.exclusion_polygons:
            if geom.intersects(ex):
                return True
        return False


# ---------------------------------------------------------------------------
# Gerber & Excellon Layer Geometry Normalizer
# ---------------------------------------------------------------------------

class LayerGeometryCache:
    """Parses and normalizes Gerber and Excellon geometry to (0,0) canonical origin."""
    def __init__(self):
        self.cache = {}
        self.toolpath_cache = {}

    def get_toolpaths(self, project: ProjectInfo, layer_key: str, tool_dia: float = 0.2, passes: int = 1):
        """Cache isolation/clearing polylines per project+layer+tool to avoid redundant Shapely runs."""
        key = (project.pcb_id, layer_key, round(float(tool_dia), 3), int(passes))
        if key in self.toolpath_cache:
            return self.toolpath_cache[key]
        import gerber_to_gcode
        u_data = self.get_project_layers(project)
        geom = u_data.get(layer_key)
        paths = []
        if geom is not None:
            try:
                mb = (0.0, 0.0, u_data["width"], u_data["height"])
                gen = gerber_to_gcode.ToolpathGenerator(geom, master_bounds=mb, tool_dia=tool_dia, passes=passes)
                iso_paths, _, _ = gen.generate()
                paths = iso_paths or []
            except Exception as e:
                print(f"[WARN] Toolpath generation error ({project.pcb_id} {layer_key}): {e}")
        self.toolpath_cache[key] = paths
        return paths

    def get_project_layers(self, project: ProjectInfo):
        if project.pcb_id in self.cache:
            return self.cache[project.pcb_id]

        import gerber_to_gcode

        data = {
            "origin_offset": (0.0, 0.0),
            "raw_origin_offset": (0.0, 0.0),
            "width": project.width,
            "height": project.height,
            "copper_top": None,
            "copper_bottom": None,
            "mask_top": None,
            "mask_bottom": None,
            "silkscreen_top": None,
            "silkscreen_bottom": None,
            "outline": None,
            "drills": {},  # {dia_mm: [(x, y), ...]}
        }

        # 1. Parse Outline to find canonical origin (lower-left zeroing)
        ox, oy = 0.0, 0.0
        if project.outline_file and os.path.exists(project.outline_file):
            try:
                gp = gerber_to_gcode.GerberParser(project.outline_file)
                gp.parse()
                if gp.raw_bounds:
                    ox, oy = gp.raw_bounds[0], gp.raw_bounds[1]
                    data["width"] = gp.raw_bounds[2] - gp.raw_bounds[0]
                    data["height"] = gp.raw_bounds[3] - gp.raw_bounds[1]
                data["outline"] = gp.copper
            except Exception as e:
                print(f"[WARN] Outline parse error {project.outline_file}: {e}")

        # 2. Parse Top Copper
        if project.copper_top and os.path.exists(project.copper_top):
            try:
                gp = gerber_to_gcode.GerberParser(project.copper_top)
                gp.parse()
                data["copper_top"] = gp.copper
            except Exception as e:
                print(f"[WARN] Top Copper parse error {project.copper_top}: {e}")

        # 3. Parse Bottom Copper
        if project.copper_bottom and os.path.exists(project.copper_bottom):
            try:
                gp = gerber_to_gcode.GerberParser(project.copper_bottom)
                gp.parse()
                data["copper_bottom"] = gp.copper
            except Exception as e:
                print(f"[WARN] Bottom Copper parse error {project.copper_bottom}: {e}")

        # 4. Parse Solder Masks
        if project.mask_top and os.path.exists(project.mask_top):
            try:
                gp = gerber_to_gcode.GerberParser(project.mask_top)
                gp.parse()
                data["mask_top"] = gp.copper
            except Exception as e:
                print(f"[WARN] Top Mask parse error: {e}")

        if project.mask_bottom and os.path.exists(project.mask_bottom):
            try:
                gp = gerber_to_gcode.GerberParser(project.mask_bottom)
                gp.parse()
                data["mask_bottom"] = gp.copper
            except Exception as e:
                print(f"[WARN] Bottom Mask parse error: {e}")

        # 5. Parse Silkscreens
        if project.silkscreen_top and os.path.exists(project.silkscreen_top):
            try:
                gp = gerber_to_gcode.GerberParser(project.silkscreen_top)
                gp.parse()
                data["silkscreen_top"] = gp.copper
            except Exception as e:
                print(f"[WARN] Top Silkscreen parse error: {e}")

        if project.silkscreen_bottom and os.path.exists(project.silkscreen_bottom):
            try:
                gp = gerber_to_gcode.GerberParser(project.silkscreen_bottom)
                gp.parse()
                data["silkscreen_bottom"] = gp.copper
            except Exception as e:
                print(f"[WARN] Bottom Silkscreen parse error: {e}")

        # 6. Parse Drill Files with Standardized Tool Binning + Coordinate Deduplication
        seen_drills = set()
        for drl_path in project.drill_files:
            try:
                dp = gerber_to_gcode.ExcellonParser(drl_path)
                dp.parse()
                for (tool_id, hx, hy) in dp.holes:
                    raw_dia = dp.tools.get(tool_id, 0.8)
                    norm_x = hx - ox
                    norm_y = hy - oy
                    
                    dia_key = self.standardize_tool_dia(raw_dia)
                    dedup_key = (dia_key, round(norm_x, 3), round(norm_y, 3))
                    if dedup_key in seen_drills:
                        continue
                    seen_drills.add(dedup_key)
                    if dia_key not in data["drills"]:
                        data["drills"][dia_key] = []
                    data["drills"][dia_key].append((norm_x, norm_y))
            except Exception as e:
                print(f"[WARN] Drill parse error {drl_path}: {e}")

        # 7. Normalize all Shapely geometries to canonical (0, 0) origin
        if ox != 0.0 or oy != 0.0:
            for k in ["outline", "copper_top", "copper_bottom", "mask_top", "mask_bottom", "silkscreen_top", "silkscreen_bottom"]:
                if data.get(k) is not None:
                    data[k] = translate(data[k], xoff=-ox, yoff=-oy)

        data["raw_origin_offset"] = (ox, oy)
        data["origin_offset"] = (0.0, 0.0)

        self.cache[project.pcb_id] = data
        return data

    @staticmethod
    def standardize_tool_dia(dia):
        """Map fractional or unusual drill diameters to standard CNC bit sizes."""
        standards = [0.3, 0.4, 0.5, 0.6, 0.7, 0.75, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.8, 2.0, 2.5, 3.0]
        for s in standards:
            if abs(dia - s) <= 0.05:
                return s
        return round(dia, 2)


# ---------------------------------------------------------------------------
# Breakaway / Snapping Mode Configuration & Shared Seam Geometry Helpers
# ---------------------------------------------------------------------------

BREAKAWAY_MODES = {
    "drilling_only": {"drilling": True, "milling": False, "vbit": False, "label": "Drilling Only (Mouse Bites)"},
    "milling_only": {"drilling": False, "milling": True, "vbit": False, "label": "Milling Only (Routing Tabs)"},
    "v_bit_only": {"drilling": False, "milling": False, "vbit": True, "label": "V-Bit Only"},
    "drilling_and_milling": {"drilling": True, "milling": True, "vbit": False, "label": "Both Drilling & Milling"},
    "v_bit_and_milling": {"drilling": False, "milling": True, "vbit": True, "label": "Both V-Bit & Milling"},
}


def normalize_breakaway_mode(raw_mode: Optional[str]) -> str:
    """Normalize breakaway mode strings from JSON config into one of the 5 canonical modes."""
    if not raw_mode:
        return "v_bit_and_milling"
    key = str(raw_mode).strip().lower().replace("-", "_").replace(" ", "_")
    aliases = {
        "drilling": "drilling_only",
        "drilling_only": "drilling_only",
        "mouse_bites": "drilling_only",
        "milling": "milling_only",
        "milling_only": "milling_only",
        "routing_tabs": "milling_only",
        "vbit": "v_bit_only",
        "v_bit": "v_bit_only",
        "vbit_only": "v_bit_only",
        "v_bit_only": "v_bit_only",
        "drilling_and_milling": "drilling_and_milling",
        "both_drilling_and_milling": "drilling_and_milling",
        "milling_and_drilling": "drilling_and_milling",
        "v_bit_and_milling": "v_bit_and_milling",
        "vbit_and_milling": "v_bit_and_milling",
        "both_v_bit_and_milling": "v_bit_and_milling",
        "milling_and_v_bit": "v_bit_and_milling",
    }
    if key in aliases:
        return aliases[key]
    if key in BREAKAWAY_MODES:
        return key
    print(f"[WARN] Unknown breakaway_mode '{raw_mode}', falling back to 'v_bit_and_milling'.")
    return "v_bit_and_milling"


@dataclass
class BreakawayConfig:
    """Configurable breakaway/snapping settings parsed from manufacture_config.json."""
    mode: str = "v_bit_and_milling"
    retention_tab_count: int = 2
    retention_tab_width_mm: float = 2.0
    smooth_edge_finishing: bool = True
    smooth_edge_recess_mm: float = 0.15
    mouse_bite_drill_dia_mm: float = 0.8
    mouse_bite_pitch_mm: float = 1.5
    milling_tool_dia_mm: float = 1.0
    milling_cut_depth_per_side_mm: float = -0.85
    v_bit_score_depth_per_side_mm: float = -0.65
    v_bit_tab_depth_per_side_mm: float = -0.55

    @property
    def use_drilling(self) -> bool:
        return BREAKAWAY_MODES[self.mode]["drilling"]

    @property
    def use_milling(self) -> bool:
        return BREAKAWAY_MODES[self.mode]["milling"]

    @property
    def use_vbit(self) -> bool:
        return BREAKAWAY_MODES[self.mode]["vbit"]

    @property
    def label(self) -> str:
        return BREAKAWAY_MODES[self.mode]["label"]

    @classmethod
    def from_config(cls, config: Optional[Dict]) -> "BreakawayConfig":
        tag_cfg = (config or {}).get("pcb_tag_strip", {})
        mode = normalize_breakaway_mode(tag_cfg.get("breakaway_mode", "v_bit_and_milling"))
        return cls(
            mode=mode,
            retention_tab_count=2 if (BREAKAWAY_MODES[mode]["milling"] or BREAKAWAY_MODES[mode]["vbit"]) else 0,
            retention_tab_width_mm=float(tag_cfg.get("retention_tab_width_mm", 2.0)),
            smooth_edge_finishing=bool(tag_cfg.get("smooth_edge_finishing", True)),
            smooth_edge_recess_mm=float(tag_cfg.get("smooth_edge_recess_mm", 0.15)),
            mouse_bite_drill_dia_mm=float(tag_cfg.get("mouse_bite_drill_dia_mm", 0.8)),
            mouse_bite_pitch_mm=float(tag_cfg.get("mouse_bite_pitch_mm", 1.5)),
            milling_tool_dia_mm=float(tag_cfg.get("milling_tool_dia_mm", 1.0)),
            milling_cut_depth_per_side_mm=float(tag_cfg.get("milling_cut_depth_per_side_mm", -0.85)),
            v_bit_score_depth_per_side_mm=float(tag_cfg.get("v_bit_score_depth_per_side_mm", -0.65)),
            v_bit_tab_depth_per_side_mm=float(tag_cfg.get("v_bit_tab_depth_per_side_mm", -0.55)),
        )


def compute_seam_breakaway_geometry(seam_length: float, cfg: BreakawayConfig) -> Dict[str, Any]:
    """
    Compute canonical 1D seam intervals [0, seam_length] and outward normal offsets
    (positive normal offset points into the stepping tag / waste side, away from PCB body)
    so that milling/V-bit edges break smoothly with zero rough PCB-edge protrusions.
    """
    L = max(0.0, float(seam_length))
    tool_rad = cfg.milling_tool_dia_mm / 2.0
    drill_rad = cfg.mouse_bite_drill_dia_mm / 2.0

    retention_tabs: List[Tuple[float, float]] = []
    milling_slots: List[Dict[str, Any]] = []
    vbit_scores: List[Dict[str, Any]] = []
    mouse_bites: List[Tuple[float, float]] = []  # (s_pos, outward_offset_into_tag)

    if cfg.use_milling or cfg.use_vbit:
        # Strictly enforce exactly two ~2.0 mm retention tabs distributed evenly along the tag
        tab_w = min(cfg.retention_tab_width_mm, max(1.0, L * 0.2))
        c1 = L / 3.0
        c2 = 2.0 * L / 3.0
        t1 = (c1 - tab_w / 2.0, c1 + tab_w / 2.0)
        t2 = (c2 - tab_w / 2.0, c2 + tab_w / 2.0)
        retention_tabs = [t1, t2]
        open_slots = [(0.0, t1[0]), (t1[1], t2[0]), (t2[1], L)]

        # Smooth-edge tool-radius offset into tag side keeps routed wall flush with PCB boundary
        mill_offset = tool_rad if cfg.smooth_edge_finishing else 0.0

        if cfg.use_milling:
            for (s0, s1) in open_slots:
                if s1 - s0 > 0.2:
                    milling_slots.append({
                        "start": s0, "end": s1, "offset": mill_offset,
                        "depth": cfg.milling_cut_depth_per_side_mm, "kind": "through_slot"
                    })
            # In milling-only mode, also route a flush partial-depth relief across the 2 tabs
            # so hand snapping fractures cleanly along the exact PCB boundary without burrs
            if cfg.mode == "milling_only" and cfg.smooth_edge_finishing:
                for (t0, t1_e) in retention_tabs:
                    milling_slots.append({
                        "start": t0, "end": t1_e, "offset": mill_offset,
                        "depth": -0.50, "kind": "tab_relief"
                    })

        if cfg.use_vbit:
            if cfg.mode == "v_bit_only":
                for (s0, s1) in open_slots:
                    if s1 - s0 > 0.2:
                        vbit_scores.append({
                            "start": s0, "end": s1, "offset": 0.0,
                            "depth": cfg.milling_cut_depth_per_side_mm, "kind": "v_through"
                        })
                for (t0, t1_e) in retention_tabs:
                    vbit_scores.append({
                        "start": t0, "end": t1_e, "offset": 0.0,
                        "depth": cfg.v_bit_tab_depth_per_side_mm, "kind": "v_tab_score"
                    })
            else:
                # v_bit_and_milling: V-score overlaps each 2mm retention tab cleanly at exact edge
                for (t0, t1_e) in retention_tabs:
                    vbit_scores.append({
                        "start": max(0.0, t0 - 0.5), "end": min(L, t1_e + 0.5),
                        "offset": 0.0, "depth": cfg.v_bit_score_depth_per_side_mm, "kind": "v_tab_score"
                    })

        if cfg.use_drilling:
            # drilling_and_milling: place recessed mouse-bite perforations strictly inside the 2 tabs
            recess_off = max(0.0, drill_rad - cfg.smooth_edge_recess_mm) if cfg.smooth_edge_finishing else 0.0
            for (t0, t1_e) in retention_tabs:
                tc = 0.5 * (t0 + t1_e)
                mouse_bites.append((tc - 0.5, recess_off))
                mouse_bites.append((tc + 0.5, recess_off))
    else:
        # drilling_only: full line of mouse bites along the seam
        pitch = max(0.8, cfg.mouse_bite_pitch_mm)
        bite_count = int(L // pitch)
        for i in range(1, bite_count):
            mouse_bites.append((i * pitch, 0.0))

    return {
        "seam_length": L,
        "retention_tabs": retention_tabs,
        "milling_slots": milling_slots,
        "vbit_scores": vbit_scores,
        "mouse_bites": mouse_bites,
        "mode": cfg.mode,
    }


def build_tag_artwork(tag_x: float, tag_y: float, tag_len: float, tag_w: float,
                      attach_side: str, pcb_id: str, engrave_arrow: bool = True) -> Dict[str, Any]:
    """
    Build simplified stepping tag artwork containing ONLY:
    - The unique PCB / Unit ID (upright, never mirrored backwards)
    - A triangle arrow pointing directly toward the PCB body it references.
    attach_side='bottom' -> horizontal tag below PCB, arrow points +Y.
    attach_side='left'   -> vertical tag left of PCB, arrow points +X.
    attach_side='right'  -> vertical tag right of PCB (after X-mirror), arrow points -X.
    """
    char_h = 2.2
    char_w = 1.4
    spacing = 0.5
    text_len = max(0.0, len(pcb_id) * (char_w + spacing) - spacing)
    tri_span = 3.4 if engrave_arrow else 0.0
    gap = 2.2 if engrave_arrow else 0.0
    group_len = tri_span + gap + text_len
    start_offset = max(2.0, (tag_len - group_len) / 2.0)

    triangle_poly: List[Tuple[float, float]] = []
    arrow_strokes: List[List[Tuple[float, float]]] = []
    id_text_strokes: List[List[Tuple[float, float]]] = []

    if attach_side == "bottom":
        if engrave_arrow:
            cx = tag_x + start_offset + tri_span / 2.0
            tip = (cx, tag_y + tag_w - 0.8)
            bl = (cx - 1.6, tag_y + 1.0)
            br = (cx + 1.6, tag_y + 1.0)
            triangle_poly = [tip, bl, br]
            arrow_strokes = [[tip, bl, br, tip]]
        tx = tag_x + start_offset + tri_span + gap
        ty = tag_y + (tag_w - char_h) / 2.0
        id_text_strokes = text_to_polylines(pcb_id, tx, ty, char_h=char_h, char_w=char_w, spacing=spacing)

    elif attach_side == "right":
        if engrave_arrow:
            cy = tag_y + start_offset + tri_span / 2.0
            tip = (tag_x + 0.8, cy)
            bb = (tag_x + tag_w - 1.0, cy - 1.6)
            bt = (tag_x + tag_w - 1.0, cy + 1.6)
            triangle_poly = [tip, bb, bt]
            arrow_strokes = [[tip, bb, bt, tip]]
        # Rotate upright text 90 deg CCW along +Y: (px, py) -> (x_base - py, ty + px)
        x_base = tag_x + (tag_w + char_h) / 2.0
        ty = tag_y + start_offset + tri_span + gap
        raw_text = text_to_polylines(pcb_id, 0.0, 0.0, char_h=char_h, char_w=char_w, spacing=spacing)
        for stroke in raw_text:
            id_text_strokes.append([(x_base - p[1], ty + p[0]) for p in stroke])

    else:  # attach_side == "left"
        if engrave_arrow:
            cy = tag_y + start_offset + tri_span / 2.0
            tip = (tag_x + tag_w - 0.8, cy)
            bb = (tag_x + 1.0, cy - 1.6)
            bt = (tag_x + 1.0, cy + 1.6)
            triangle_poly = [tip, bb, bt]
            arrow_strokes = [[tip, bb, bt, tip]]
        # Rotate upright text 90 deg CCW along +Y: (px, py) -> (x_base - py, ty + px)
        x_base = tag_x + (tag_w + char_h) / 2.0
        ty = tag_y + start_offset + tri_span + gap
        raw_text = text_to_polylines(pcb_id, 0.0, 0.0, char_h=char_h, char_w=char_w, spacing=spacing)
        for stroke in raw_text:
            id_text_strokes.append([(x_base - p[1], ty + p[0]) for p in stroke])

    return {
        "triangle_poly": triangle_poly,
        "arrow_strokes": arrow_strokes,
        "id_text_strokes": id_text_strokes,
    }


# ---------------------------------------------------------------------------
# Placed PCB Unit on A4 Panel (Supports 0° and 90° Rotations, Shortest-Edge Tag)
# ---------------------------------------------------------------------------

class PlacedUnit:
    """
    Represents a single PCB unit nested on an A4 sheet.
    The 5mm stepping tag is attached strictly along the SHORTEST edge of the PCB design.
    """
    def __init__(self, project: ProjectInfo, unit_id: str, x: float, y: float,
                 w: float, h: float, rotation_deg=0, tab_w=5.0, sheet_idx=0,
                 engrave_both_sides=False, engrave_arrow=True,
                 breakaway_cfg: Optional[BreakawayConfig] = None):
        self.project = project
        self.unit_id = unit_id
        self.sheet_idx = sheet_idx
        self.x = x  # lower-left X on sheet
        self.y = y  # lower-left Y on sheet
        self.w = w  # bounding box width on sheet
        self.h = h  # bounding box height on sheet
        self.rotation_deg = rotation_deg
        self.tab_w = tab_w
        self.engrave_both_sides = engrave_both_sides
        self.engrave_arrow = engrave_arrow
        self.breakaway_cfg = breakaway_cfg or BreakawayConfig()

        self.pcb_w = project.width
        self.pcb_h = project.height
        self.shortest_edge_is_width = (self.pcb_w <= self.pcb_h)
        self.shortest_edge_len = min(self.pcb_w, self.pcb_h)

        # Bounding box of full unit (PCB + 5mm tag)
        self.full_box = box(x, y, x + w, y + h)

        # Determine whether the stepping tag is on the 'bottom' (horizontal seam) or 'left' (vertical seam)
        if rotation_deg == 0:
            self.attach_side = "bottom" if self.shortest_edge_is_width else "left"
        else:  # rotation_deg == 90
            # Rotating 90 deg swaps horizontal & vertical axes
            self.attach_side = "left" if self.shortest_edge_is_width else "bottom"

        if self.attach_side == "bottom":
            self.pcb_sheet_w = self.pcb_w if rotation_deg == 0 else self.pcb_h
            self.pcb_sheet_h = self.pcb_h if rotation_deg == 0 else self.pcb_w
            self.tab_box = box(x, y, x + self.pcb_sheet_w, y + tab_w)
            self.pcb_box = box(x, y + tab_w, x + self.pcb_sheet_w, y + tab_w + self.pcb_sheet_h)
            self.seam_line = [(x, y + tab_w), (x + self.pcb_sheet_w, y + tab_w)]
            seam_len = self.pcb_sheet_w
        else:
            self.pcb_sheet_w = self.pcb_w if rotation_deg == 0 else self.pcb_h
            self.pcb_sheet_h = self.pcb_h if rotation_deg == 0 else self.pcb_w
            self.tab_box = box(x, y, x + tab_w, y + self.pcb_sheet_h)
            self.pcb_box = box(x + tab_w, y, x + tab_w + self.pcb_sheet_w, y + self.pcb_sheet_h)
            self.seam_line = [(x + tab_w, y), (x + tab_w, y + self.pcb_sheet_h)]
            seam_len = self.pcb_sheet_h

        # Compute breakaway seam geometry (retention tabs, milling slots, V-bit scores, mouse bites)
        self.seam_geom = compute_seam_breakaway_geometry(seam_len, self.breakaway_cfg)
        self.retention_tabs_sheet: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
        self.milling_slot_segs: List[Dict[str, Any]] = []
        self.vbit_score_segs: List[Dict[str, Any]] = []
        self.mouse_bites: List[Tuple[float, float]] = []

        for (t0, t1) in self.seam_geom["retention_tabs"]:
            self.retention_tabs_sheet.append((self._seam_pt(t0, 0.0), self._seam_pt(t1, 0.0)))
        for sl in self.seam_geom["milling_slots"]:
            self.milling_slot_segs.append({
                "pts": [self._seam_pt(sl["start"], sl["offset"]), self._seam_pt(sl["end"], sl["offset"])],
                "depth": sl["depth"],
                "kind": sl["kind"],
            })
        for vs in self.seam_geom["vbit_scores"]:
            self.vbit_score_segs.append({
                "pts": [self._seam_pt(vs["start"], vs["offset"]), self._seam_pt(vs["end"], vs["offset"])],
                "depth": vs["depth"],
                "kind": vs["kind"],
            })
        for (s_pos, n_off) in self.seam_geom["mouse_bites"]:
            self.mouse_bites.append(self._seam_pt(s_pos, n_off))

        # Unique instance PCB ID + Triangle arrow pointing toward board
        self.seam_len = seam_len
        art = build_tag_artwork(
            tag_x=x, tag_y=y, tag_len=seam_len, tag_w=tab_w,
            attach_side=self.attach_side, pcb_id=self.unit_id,
            engrave_arrow=self.engrave_arrow,
        )
        self.triangle_poly = art["triangle_poly"]
        self.arrow_lines = art["arrow_strokes"]
        self.id_text_lines = art["id_text_strokes"]

    def get_mirrored_tag_artwork(self, sheet_w: float) -> Dict[str, Any]:
        """
        Return stepping tag artwork positioned cleanly inside the X-mirrored tag box
        (flipped horizontally along Y-axis) with UPRIGHT readable unique unit_id text.
        """
        tb = self.tab_box.bounds
        mtx0 = sheet_w - tb[2]
        mty0 = tb[1]
        mir_side = "right" if self.attach_side == "left" else "bottom"
        return build_tag_artwork(
            tag_x=mtx0, tag_y=mty0, tag_len=self.seam_len, tag_w=self.tab_w,
            attach_side=mir_side, pcb_id=self.unit_id, engrave_arrow=self.engrave_arrow,
        )

    def _seam_pt(self, s_pos: float, into_tag_offset: float) -> Tuple[float, float]:
        """Map 1D position along the seam and normal offset (into stepping tag) to sheet coords."""
        if self.attach_side == "bottom":
            return (self.x + s_pos, self.y + self.tab_w - into_tag_offset)
        return (self.x + self.tab_w - into_tag_offset, self.y + s_pos)

    def to_sheet_coords(self, px: float, py: float) -> Tuple[float, float]:
        """Maps local normalized PCB coordinates [0, pcb_w] x [0, pcb_h] to A4 sheet coordinates."""
        if self.rotation_deg == 0:
            ox = self.x + (self.tab_w if self.attach_side == "left" else 0.0)
            oy = self.y + (self.tab_w if self.attach_side == "bottom" else 0.0)
            return ox + px, oy + py
        elif self.rotation_deg == 90:
            rot_x = self.pcb_h - py
            rot_y = px
            ox = self.x + (self.tab_w if self.attach_side == "left" else 0.0)
            oy = self.y + (self.tab_w if self.attach_side == "bottom" else 0.0)
            return ox + rot_x, oy + rot_y
        return self.x + px, self.y + py


# ---------------------------------------------------------------------------
# A4 Copper Board Panelizer (Zero-Gap Tight Packing & Solder Mask Color Grouping)
# ---------------------------------------------------------------------------

class A4Panelizer:
    """
    Panelizes multiple PCB projects onto copper sheets (supporting CNC beds smaller than A4):
    - Prioritizes grouping boards by solder mask color.
    - Attaches 5mm stepping tag strictly to the shortest edge of each PCB.
    - Rotates rectangular PCBs (0° or 90°) to maximize boards per sheet.
    - Packs tightly with zero gap between adjacent designs for single-pass perimeter cutting.
    - Enforces clamp keep-outs from pcb_holders.json.
    - Partitions strictly into Front Half before Back Half for Y-axis-only sheet shifting.
    """
    def __init__(self, sheet_w=210.0, sheet_h=297.0, tab_w=5.0, y_split=145.0,
                 holder_mgr: Optional[HolderManager] = None, config: Optional[Dict] = None):
        self.sheet_w = sheet_w
        self.sheet_h = sheet_h
        self.tab_w = tab_w
        self.holder_mgr = holder_mgr
        self.cfg = config or {}
        self.breakaway_cfg = BreakawayConfig.from_config(self.cfg)

        cnc = self.cfg.get("cnc_limits", {})
        self.bed_travel_x = float(cnc.get("bed_travel_x_mm", 275.0))
        self.bed_travel_y = float(cnc.get("bed_travel_y_mm", 170.0))
        self.left_margin = float(cnc.get("left_margin_guide_width_mm", 18.0))
        self.right_margin = float(cnc.get("right_margin_mm", 5.0))

        # Clamp y_split safely to machine bed travel for Y-only shifting
        raw_split = float(cnc.get("y_split_threshold_mm", y_split))
        if self.sheet_h > self.bed_travel_y:
            max_safe_split = self.bed_travel_y - 5.0
            min_safe_split = max(0.0, self.sheet_h - self.bed_travel_y)
            self.y_split = max(min_safe_split, min(raw_split, max_safe_split))
        else:
            self.y_split = raw_split

        self.front_min_y = float(cnc.get("front_half_min_y_mm", 8.5))
        self.front_max_y = min(float(cnc.get("front_half_max_y_mm", 141.5)), self.y_split - 3.0)
        self.back_min_y = max(float(cnc.get("back_half_min_y_mm", 150.0)), self.y_split + 4.0)
        self.back_max_y = min(
            float(cnc.get("back_half_max_y_mm", self.sheet_h - 5.0)),
            self.y_split + self.bed_travel_y - 5.0,
            self.sheet_h - 5.0,
        )

        panel_cfg = self.cfg.get("panelization", {})
        self.zero_gap = bool(panel_cfg.get("zero_gap_tight_packing", True))
        self.board_gap = 0.0 if self.zero_gap else float(panel_cfg.get("board_gap_mm", 0.0))
        self.allow_rot = bool(panel_cfg.get("allow_rotation_to_maximize_space", True))
        self.group_by_color = bool(panel_cfg.get("group_by_solder_mask_color", True))
        self.strict_color = bool(panel_cfg.get("strict_solder_mask_color", False))

        tag_cfg = self.cfg.get("pcb_tag_strip", {})
        self.engrave_both = bool(tag_cfg.get("engrave_text_both_sides", False))
        self.engrave_arrow = bool(tag_cfg.get("engrave_arrow", True))

    def _get_dims_for_rot(self, p: ProjectInfo, rot: int) -> Tuple[float, float]:
        shortest_is_w = (p.width <= p.height)
        if rot == 0:
            return (p.width, p.height + self.tab_w) if shortest_is_w else (p.width + self.tab_w, p.height)
        # rot == 90
        return (p.height + self.tab_w, p.width) if shortest_is_w else (p.height, p.width + self.tab_w)

    def _candidate_rotations(self, p: ProjectInfo) -> List[int]:
        if not (self.allow_rot and p.shape == "rectangular"):
            return [0]
        # Prefer orientation that maximizes horizontal fit & fits comfortably within half-sheet Y span
        return [90, 0]

    def _fits_at(self, x: float, y: float, u_w: float, u_h: float, max_x: float, max_y: float) -> bool:
        if x + u_w > max_x + 1e-6 or y + u_h > max_y + 1e-6:
            return False
        if self.holder_mgr and self.holder_mgr.collides_with(box(x, y, x + u_w, y + u_h)):
            return False
        return True

    def _pack_half(self, units_to_pack: List[Dict], start_idx: int, active_color: str,
                   y_min: float, y_max: float, sheet_idx: int) -> Tuple[List[PlacedUnit], int]:
        placed: List[PlacedUnit] = []
        idx = start_idx
        max_x = min(self.sheet_w - self.right_margin, self.bed_travel_x)
        curr_x = self.left_margin
        curr_y = y_min
        row_h = 0.0

        while idx < len(units_to_pack):
            item = units_to_pack[idx]
            p = item["project"]
            if self.strict_color and p.solder_mask_color != active_color:
                break

            chosen = None
            for rot in self._candidate_rotations(p):
                u_w, u_h = self._get_dims_for_rot(p, rot)
                if self._fits_at(curr_x, curr_y, u_w, u_h, max_x, y_max):
                    chosen = (rot, u_w, u_h)
                    break

            if chosen is None and curr_x > self.left_margin + 0.1 and row_h > 0.0:
                # Try wrapping to next shelf row inside the same half
                next_y = curr_y + row_h + self.board_gap
                for rot in self._candidate_rotations(p):
                    u_w, u_h = self._get_dims_for_rot(p, rot)
                    if self._fits_at(self.left_margin, next_y, u_w, u_h, max_x, y_max):
                        curr_x = self.left_margin
                        curr_y = next_y
                        row_h = 0.0
                        chosen = (rot, u_w, u_h)
                        break

            if chosen is None:
                break

            rot, u_w, u_h = chosen
            u = PlacedUnit(
                project=p, unit_id=item["unit_id"],
                x=curr_x, y=curr_y, w=u_w, h=u_h,
                rotation_deg=rot, tab_w=self.tab_w, sheet_idx=sheet_idx,
                engrave_both_sides=self.engrave_both, engrave_arrow=self.engrave_arrow,
                breakaway_cfg=self.breakaway_cfg,
            )
            placed.append(u)
            curr_x += u_w + self.board_gap
            row_h = max(row_h, u_h)
            idx += 1

        return placed, idx

    def pack_projects(self, projects: List[ProjectInfo], layer_cache: LayerGeometryCache) -> List[List[PlacedUnit]]:
        if self.group_by_color:
            projects_sorted = sorted(projects, key=lambda p: (p.solder_mask_color, -p.quantity))
        else:
            projects_sorted = sorted(projects, key=lambda p: -p.quantity)

        units_to_pack = []
        for p in projects_sorted:
            for u_id in p.unit_ids:
                units_to_pack.append({"project": p, "unit_id": u_id})

        sheets: List[List[PlacedUnit]] = []
        idx = 0

        while idx < len(units_to_pack):
            sheet_idx = len(sheets)
            active_color = units_to_pack[idx]["project"].solder_mask_color

            # 1. Always pack & route Front Half first
            front_units, idx = self._pack_half(
                units_to_pack, idx, active_color, self.front_min_y, self.front_max_y, sheet_idx
            )
            # 2. Then pack Back Half (reached by Y-axis-only sheet shift)
            back_units, idx = self._pack_half(
                units_to_pack, idx, active_color, self.back_min_y, self.back_max_y, sheet_idx
            )

            sheet_units = front_units + back_units
            if sheet_units:
                sheets.append(sheet_units)
            else:
                print(f"[ERROR] Could not fit unit {units_to_pack[idx]['unit_id']} on sheet within CNC bed limits.")
                idx += 1

        print(f"[INFO] Panelized {len(units_to_pack)} units across {len(sheets)} sheet(s) "
              f"[Mode: {self.breakaway_cfg.mode}, Shortest-Edge Tag, Y-Split: {self.y_split:.1f}mm].")
        return sheets


# ---------------------------------------------------------------------------
# Left Alignment Guides & Front-Corner Sheet Tag Generator
# ---------------------------------------------------------------------------

def generate_left_guides(sheet_w=210.0, sheet_h=297.0, margin_w=18.0, y_shift=145.0,
                         enable_dowel_pins: bool = False, mirrored_x: bool = False):
    """
    Step 0: Margin Alignment Guides, Millimeter Scale & Shift Datum Line.
    - When enable_dowel_pins=False (default per config), zero 3mm dowel pin holes are emitted.
    - When mirrored_x=True, geometry sits on the opposite X margin while keeping all labels UPRIGHT.
    """
    front_lines = []
    front_texts = []
    front_pins = []

    back_lines = []
    back_texts = []
    back_pins = []

    x_guide = (sheet_w - 12.0) if mirrored_x else 12.0
    tick_dir = 1.0 if mirrored_x else -1.0

    front_lines.append([(x_guide, 0.0), (x_guide, y_shift)])
    back_lines.append([(x_guide, y_shift), (x_guide, sheet_h)])

    for y in range(0, int(sheet_h) + 1, 5):
        tick_len = 5.0 if (y % 50 == 0) else (3.5 if (y % 10 == 0) else 2.0)
        seg = [(x_guide, float(y)), (x_guide + tick_dir * tick_len, float(y))]
        txt_x = (x_guide + 4.2) if mirrored_x else 2.0
        if y <= y_shift:
            front_lines.append(seg)
            if y % 50 == 0 and y > 0:
                front_texts.extend(text_to_polylines(f"{y}", txt_x, float(y) - 1.0, char_h=2.0, char_w=1.2))
        else:
            back_lines.append(seg)
            if y % 50 == 0:
                back_texts.extend(text_to_polylines(f"{y}", txt_x, float(y) - 1.0, char_h=2.0, char_w=1.2))

    # Y-Shift Datum Line (at y_shift, engraved in front setup)
    if mirrored_x:
        front_lines.append([(sheet_w - margin_w, y_shift), (sheet_w, y_shift)])
        front_texts.extend(text_to_polylines(f"Y{int(round(y_shift))} SHIFT", sheet_w - margin_w + 1.0, y_shift - 4.0, char_h=2.2, char_w=1.2))
    else:
        front_lines.append([(0.0, y_shift), (margin_w, y_shift)])
        front_texts.extend(text_to_polylines(f"Y{int(round(y_shift))} SHIFT", 1.5, y_shift - 4.0, char_h=2.2, char_w=1.2))

    # Optional Reference Dowel Pin Holes (only generated when enable_dowel_pin_lock=true in config)
    if enable_dowel_pins:
        px_pin = (sheet_w - 6.0) if mirrored_x else 6.0
        front_pins.append((px_pin, 25.0))
        back_pins.append((px_pin, y_shift + 25.0))

    return {
        "lines": front_lines + back_lines,
        "texts": front_texts + back_texts,
        "pin_holes": front_pins + back_pins,
        "front_lines": front_lines,
        "front_texts": front_texts,
        "front_pin_holes": front_pins,
        "back_lines": back_lines,
        "back_texts": back_texts,
        "back_pin_holes": back_pins,
        "pin_dia": 3.0 if enable_dowel_pins else 0.0,
        "enable_dowel_pins": enable_dowel_pins,
    }


def generate_sheet_tag(sheet_num: int, total_sheets: int, units_count: int,
                       solder_mask_color: str = "green",
                       sheet_w: float = 210.0, left_margin: float = 18.0,
                       tag_position: str = "front_right",
                       holders_mgr: Optional[HolderManager] = None,
                       mirrored_x: bool = False,
                       side_label: str = "FRONT"):
    """
    Engraves the dedicated Sheet Tag at the FRONT-SIDE CORNER of the sheet.
    When mirrored_x=True (Backside View), mirrors box location across Y-axis while keeping text UPRIGHT.
    """
    w = 96.0
    h = 6.0
    y = 1.2
    pos = (tag_position or "front_right").lower()

    if "left" in pos:
        x = left_margin + 0.5
    else:
        x = max(left_margin + 0.5, sheet_w - 18.0 - w)

    if holders_mgr:
        step = -5.0 if "right" in pos else 5.0
        for _ in range(15):
            if not holders_mgr.collides_with(box(x, y, x + w, y + h)):
                break
            x += step

    if mirrored_x:
        x = sheet_w - (x + w)

    border = [[(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)]]
    label = f"S{sheet_num:02d}/{total_sheets:02d} | U:{units_count} | {solder_mask_color.upper()} | {side_label}"
    text_strokes = text_to_polylines(label, x + 2.5, y + 1.8, char_h=2.4, char_w=1.35, spacing=0.45)

    arrow_x = x + w - 12.0
    arrow_y_base = y + 1.2
    arrow_y_tip = y + h - 1.2
    arrow_strokes = [
        [(arrow_x, arrow_y_tip), (arrow_x - 1.6, arrow_y_base), (arrow_x + 1.6, arrow_y_base), (arrow_x, arrow_y_tip)]
    ]
    arrow_label = text_to_polylines("Y+", arrow_x + 2.8, y + 1.8, char_h=2.2, char_w=1.2, spacing=0.4)

    return {
        "x": x, "y": y, "w": w, "h": h, "position": pos,
        "border": border,
        "text": text_strokes + arrow_label,
        "arrow": arrow_strokes,
    }


# ---------------------------------------------------------------------------
# Individual Circuit Gerber SVG Visualizer (Step-by-Step Inspector)
# ---------------------------------------------------------------------------

def _render_seam_cutouts_svg(tr_fn, seam_pt_fn, seam_geom: Dict[str, Any], scale: float,
                             highlight_cutout: bool = False) -> List[str]:
    """
    Shared SVG renderer for step cutouts between the stepping tag and the PCB body:
    - Milled routing slot channels (tag-side offset for smooth PCB edge)
    - Exactly two 2.0 mm retention tabs evenly distributed along the seam
    - V-bit score lines
    - Mouse-bite drill holes
    Used across every step SVG, per-unit SVGs, and full-sheet blueprints.
    """
    elems: List[str] = []
    slot_stroke_w = max(1.6, 1.0 * scale) if highlight_cutout else max(1.2, 0.85 * scale)

    # 1. Milling cutout slots (rounded endmill channels flush on PCB boundary)
    for sl in seam_geom.get("milling_slots", []):
        p0 = tr_fn(*seam_pt_fn(sl["start"], sl["offset"]))
        p1 = tr_fn(*seam_pt_fn(sl["end"], sl["offset"]))
        is_relief = (sl.get("kind") == "tab_relief")
        col = "#f97316" if is_relief else "#ef4444"
        dash = ' stroke-dasharray="3,2"' if is_relief else ""
        op = "0.65" if is_relief else ("0.95" if highlight_cutout else "0.78")
        elems.append(
            f'<line x1="{p0[0]:.1f}" y1="{p0[1]:.1f}" x2="{p1[0]:.1f}" y2="{p1[1]:.1f}" '
            f'stroke="{col}" stroke-width="{slot_stroke_w:.1f}" stroke-linecap="round" opacity="{op}"{dash}/>'
        )
        # Inner dark slot core to visually emphasize material removal
        if not is_relief:
            elems.append(
                f'<line x1="{p0[0]:.1f}" y1="{p0[1]:.1f}" x2="{p1[0]:.1f}" y2="{p1[1]:.1f}" '
                f'stroke="#090d16" stroke-width="{max(0.8, slot_stroke_w * 0.48):.1f}" stroke-linecap="butt"/>'
            )

    # 2. Retention Tabs (~2.0 mm each, evenly distributed)
    for idx_t, (t0, t1) in enumerate(seam_geom.get("retention_tabs", []), 1):
        p0 = tr_fn(*seam_pt_fn(t0, 0.0))
        p1 = tr_fn(*seam_pt_fn(t1, 0.0))
        elems.append(
            f'<line x1="{p0[0]:.1f}" y1="{p0[1]:.1f}" x2="{p1[0]:.1f}" y2="{p1[1]:.1f}" '
            f'stroke="#22c55e" stroke-width="{max(2.2, 0.55 * scale):.1f}" stroke-linecap="butt" opacity="0.95"/>'
        )

    # 3. V-bit Score Lines
    for vs in seam_geom.get("vbit_scores", []):
        p0 = tr_fn(*seam_pt_fn(vs["start"], vs["offset"]))
        p1 = tr_fn(*seam_pt_fn(vs["end"], vs["offset"]))
        is_tab = (vs.get("kind") == "v_tab_score")
        col = "#38bdf8" if is_tab else "#f43f5e"
        elems.append(
            f'<line x1="{p0[0]:.1f}" y1="{p0[1]:.1f}" x2="{p1[0]:.1f}" y2="{p1[1]:.1f}" '
            f'stroke="{col}" stroke-width="{max(1.5, 0.35 * scale):.1f}" stroke-dasharray="5,2"/>'
        )

    # 4. Mouse-bite drill perforations
    for (s_pos, n_off) in seam_geom.get("mouse_bites", []):
        sx, sy = tr_fn(*seam_pt_fn(s_pos, n_off))
        elems.append(
            f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{max(1.0, 0.4 * scale):.1f}" '
            f'fill="#0f172a" stroke="#f59e0b" stroke-width="0.7"/>'
        )

    return elems


def _svg_rect_bounds(x0: float, y0: float, x1: float, y1: float, tr_fn) -> Tuple[float, float, float, float]:
    """Return (svg_x, svg_y, width, height) for an axis-aligned rectangle under any affine/mirror transform."""
    p0 = tr_fn(x0, y0)
    p1 = tr_fn(x1, y1)
    return min(p0[0], p1[0]), min(p0[1], p1[1]), abs(p1[0] - p0[0]), abs(p1[1] - p0[1])


class CircuitSvgBuilder:
    """Generates high-resolution vector SVGs for each manufacturing step of a circuit."""
    def __init__(self, project: ProjectInfo, p_data: Dict, scale: float = 4.5, margin: float = 21.0,
                 tab_w: float = 5.0, breakaway_cfg: Optional[BreakawayConfig] = None):
        self.project = project
        self.p_data = p_data
        self.w = float(project.width)
        self.h = float(project.height)
        self.tab_w = float(tab_w)
        self.margin = float(margin)
        self.scale = float(scale)
        self.breakaway_cfg = breakaway_cfg or BreakawayConfig()

        # Attach stepping tag strictly along the SHORTEST edge of the PCB
        self.attach_side = "bottom" if self.w <= self.h else "left"
        self.pcb_x_off = self.tab_w if self.attach_side == "left" else 0.0
        self.pcb_y_off = self.tab_w if self.attach_side == "bottom" else 0.0
        self.seam_len = self.w if self.attach_side == "bottom" else self.h
        self.seam_geom = compute_seam_breakaway_geometry(self.seam_len, self.breakaway_cfg)

        self.board_span_x = self.w + self.pcb_x_off
        self.total_w = self.board_span_x + 2 * self.margin
        self.total_h = self.h + self.pcb_y_off + 2 * self.margin

        self.px_w = self.total_w * self.scale
        self.px_h = self.total_h * self.scale

        self.norm_layers = {
            k: p_data.get(k) for k in [
                "outline", "copper_top", "copper_bottom", "mask_top", "mask_bottom", "silkscreen_top", "silkscreen_bottom"
            ]
        }
        self.drills = p_data.get("drills", {})

    def _tr(self, x: float, y: float, mirror_x: bool = False) -> Tuple[float, float]:
        bx = (self.board_span_x - x) if mirror_x else x
        sx = (self.margin + bx) * self.scale
        sy = ((self.total_h - self.margin) - y) * self.scale
        return sx, sy

    def _seam_pt(self, s_pos: float, into_tag_offset: float) -> Tuple[float, float]:
        if self.attach_side == "bottom":
            return (s_pos, self.tab_w - into_tag_offset)
        return (self.tab_w - into_tag_offset, s_pos)

    def _geom_to_path_d(self, geom, mirror_x: bool = False):
        return _shapely_geom_to_svg_path_d(
            geom, lambda px, py: self._tr(px + self.pcb_x_off, py + self.pcb_y_off, mirror_x=mirror_x)
        )

    def draw_background(self):
        return f'<rect width="{self.px_w:.1f}" height="{self.px_h:.1f}" fill="#0a0e17"/>'

    def draw_grid(self):
        lines = []
        for x in range(0, int(self.total_w) + 1, 5):
            sx = x * self.scale
            col = "#1f293d" if (x % 10 == 0) else "#141c2b"
            lines.append(f'<line x1="{sx:.1f}" y1="0" x2="{sx:.1f}" y2="{self.px_h:.1f}" stroke="{col}" stroke-width="0.7"/>')
        for y in range(0, int(self.total_h) + 1, 5):
            sy = y * self.scale
            col = "#1f293d" if (y % 10 == 0) else "#141c2b"
            lines.append(f'<line x1="0" y1="{sy:.1f}" x2="{self.px_w:.1f}" y2="{sy:.1f}" stroke="{col}" stroke-width="0.7"/>')
        return "".join(lines)

    def draw_dimensions(self, mirror_x: bool = False):
        elems = []
        tr = lambda x, y: self._tr(x, y, mirror_x=mirror_x)
        # PCB Width dimension above PCB
        sx0, sy_top, _, _ = _svg_rect_bounds(
            self.pcb_x_off, self.pcb_y_off + self.h + 4.0,
            self.pcb_x_off + self.w, self.pcb_y_off + self.h + 4.0, tr
        )
        p_w0 = tr(self.pcb_x_off, self.pcb_y_off + self.h + 4.0)
        p_w1 = tr(self.pcb_x_off + self.w, self.pcb_y_off + self.h + 4.0)
        x0, x1 = min(p_w0[0], p_w1[0]), max(p_w0[0], p_w1[0])
        elems.append(f'<line x1="{x0:.1f}" y1="{p_w0[1]:.1f}" x2="{x1:.1f}" y2="{p_w0[1]:.1f}" stroke="#64748b" stroke-width="1.2" marker-start="url(#arr-dim)" marker-end="url(#arr-dim)"/>')
        elems.append(f'<text x="{(x0+x1)/2.0:.1f}" y="{p_w0[1] - 4:.1f}" fill="#94a3b8" font-family="monospace" font-size="11" text-anchor="middle">{self.w:.1f} mm</text>')

        # PCB Height dimension right of SVG frame
        right_edge_x = self.board_span_x + 4.0 if not mirror_x else -4.0
        x_r, y_pcb_b = tr(right_edge_x, self.pcb_y_off)
        _, y_pcb_t = tr(right_edge_x, self.pcb_y_off + self.h)
        elems.append(f'<line x1="{x_r:.1f}" y1="{y_pcb_b:.1f}" x2="{x_r:.1f}" y2="{y_pcb_t:.1f}" stroke="#64748b" stroke-width="1.2"/>')
        elems.append(f'<text x="{x_r + 14:.1f}" y="{(y_pcb_b+y_pcb_t)/2.0:.1f}" fill="#94a3b8" font-family="monospace" font-size="11" text-anchor="middle" transform="rotate(-90 {x_r+14:.1f} {(y_pcb_b+y_pcb_t)/2.0:.1f})">{self.h:.1f} mm</text>')

        # Shortest-edge 5mm tag dimension
        if self.attach_side == "bottom":
            x_tr, y_tb = tr(self.w + 4.0 if not mirror_x else -4.0, 0)
            _, y_tt = tr(self.w + 4.0 if not mirror_x else -4.0, self.tab_w)
            elems.append(f'<line x1="{x_tr:.1f}" y1="{y_tb:.1f}" x2="{x_tr:.1f}" y2="{y_tt:.1f}" stroke="#f59e0b" stroke-width="1.2"/>')
            elems.append(f'<text x="{x_tr + 14:.1f}" y="{(y_tb+y_tt)/2.0 + 3:.1f}" fill="#f59e0b" font-family="monospace" font-size="9" text-anchor="middle" transform="rotate(-90 {x_tr+14:.1f} {(y_tb+y_tt)/2.0 + 3:.1f})">5mm Tab</text>')
        else:
            p_tl = tr(0, -3.5)
            p_tr = tr(self.tab_w, -3.5)
            elems.append(f'<line x1="{p_tl[0]:.1f}" y1="{p_tl[1]:.1f}" x2="{p_tr[0]:.1f}" y2="{p_tr[1]:.1f}" stroke="#f59e0b" stroke-width="1.2"/>')
            elems.append(f'<text x="{(p_tl[0]+p_tr[0])/2.0:.1f}" y="{p_tl[1] + 12:.1f}" fill="#f59e0b" font-family="monospace" font-size="9" text-anchor="middle">5mm</text>')
        return "".join(elems)

    def draw_substrate(self, mirror_x: bool = False):
        tr = lambda x, y: self._tr(x, y, mirror_x=mirror_x)
        x0, y1, w_px, h_px = _svg_rect_bounds(
            self.pcb_x_off, self.pcb_y_off, self.pcb_x_off + self.w, self.pcb_y_off + self.h, tr
        )
        return (f'<rect x="{x0:.1f}" y="{y1:.1f}" width="{w_px:.1f}" height="{h_px:.1f}" '
                f'fill="#0b2416" stroke="#1b4d2e" stroke-width="1.5" rx="2"/>')

    def draw_snap_tab(self, highlight_cutout: bool = False, mirror_x: bool = False):
        """
        Render the simplified stepping tag (PCB ID + triangle arrow ONLY) attached
        strictly to the shortest edge, PLUS the step cutouts between the tag and PCB
        for every step SVG file. Tag text ALWAYS renders upright left-to-right even when mirrored.
        """
        elems = []
        tr = lambda x, y: self._tr(x, y, mirror_x=mirror_x)
        if self.attach_side == "bottom":
            x0, y1, tw_px, th_px = _svg_rect_bounds(0.0, 0.0, self.w, self.tab_w, tr)
        else:
            x0, y1, tw_px, th_px = _svg_rect_bounds(0.0, 0.0, self.tab_w, self.h, tr)

        elems.append(
            f'<rect x="{x0:.1f}" y="{y1:.1f}" width="{tw_px:.1f}" height="{th_px:.1f}" '
            f'fill="rgba(217, 119, 6, 0.16)" stroke="#d97706" stroke-width="1.2" stroke-dasharray="4,2"/>'
        )

        # Simplified Stepping Tag Artwork: ONLY PCB ID + Triangle arrow toward board (upright even when mirrored)
        if mirror_x and self.attach_side == "left":
            art = build_tag_artwork(
                tag_x=self.w, tag_y=0.0, tag_len=self.seam_len, tag_w=self.tab_w,
                attach_side="right", pcb_id=self.project.pcb_id, engrave_arrow=True,
            )
            tr_tag = lambda x, y: self._tr(x, y, mirror_x=False)
        elif mirror_x and self.attach_side == "bottom":
            art = build_tag_artwork(
                tag_x=0.0, tag_y=0.0, tag_len=self.seam_len, tag_w=self.tab_w,
                attach_side="bottom", pcb_id=self.project.pcb_id, engrave_arrow=True,
            )
            tr_tag = lambda x, y: self._tr(x, y, mirror_x=False)
        else:
            art = build_tag_artwork(
                tag_x=0.0, tag_y=0.0, tag_len=self.seam_len, tag_w=self.tab_w,
                attach_side=self.attach_side, pcb_id=self.project.pcb_id, engrave_arrow=True,
            )
            tr_tag = tr

        if art["triangle_poly"]:
            tri_pts = " ".join(f"{tr_tag(px, py)[0]:.1f},{tr_tag(px, py)[1]:.1f}" for (px, py) in art["triangle_poly"])
            elems.append(f'<polygon points="{tri_pts}" fill="#f59e0b"/>')
        for stroke in art["id_text_strokes"]:
            pts = " ".join(f"{tr_tag(px, py)[0]:.1f},{tr_tag(px, py)[1]:.1f}" for (px, py) in stroke)
            elems.append(f'<polyline points="{pts}" fill="none" stroke="#fbbf24" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>')

        # Step Cutouts between stepping tag and circuit board (rendered on EVERY step file)
        elems.extend(_render_seam_cutouts_svg(
            tr, self._seam_pt, self.seam_geom, self.scale, highlight_cutout=highlight_cutout
        ))
        return "".join(elems)

    def draw_drills_layer(self, emphasize: bool = False, mirror_x: bool = False):
        elems = []
        tr = lambda x, y: self._tr(x, y, mirror_x=mirror_x)
        for dia in sorted(self.drills.keys()):
            pts = self.drills[dia]
            col = get_drill_color(dia)
            base_r = (dia / 2.0) * self.scale
            r_px = max(2.4 if emphasize else 1.2, base_r * (1.25 if emphasize else 1.0))
            for (hx, hy) in pts:
                sx, sy = tr(self.pcb_x_off + hx, self.pcb_y_off + hy)
                if emphasize:
                    halo_r = r_px + 2.6
                    ch = r_px + 3.2
                    elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{halo_r:.1f}" fill="{col}" opacity="0.24"/>')
                    elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{r_px:.1f}" fill="{col}" stroke="#ffffff" stroke-width="0.9" opacity="0.98"/>')
                    elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{max(0.8, r_px * 0.42):.1f}" fill="#090d16"/>')
                    elems.append(f'<line x1="{sx - ch:.1f}" y1="{sy:.1f}" x2="{sx + ch:.1f}" y2="{sy:.1f}" stroke="{col}" stroke-width="0.6" opacity="0.7"/>')
                    elems.append(f'<line x1="{sx:.1f}" y1="{sy - ch:.1f}" x2="{sx:.1f}" y2="{sy + ch:.1f}" stroke="{col}" stroke-width="0.6" opacity="0.7"/>')
                else:
                    elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{r_px:.1f}" fill="{col}" stroke="#ffffff" stroke-width="0.5" opacity="0.95"/>')
                    elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{max(0.6, r_px * 0.5):.1f}" fill="#0a0e17"/>')
        return "".join(elems)

    def build_step1_drills_svg(self, mirror_x: bool = False):
        mb_count = len(self.seam_geom.get("mouse_bites", []))
        total_holes = sum(len(pts) for pts in self.drills.values())
        body = [
            self.draw_background(),
            self.draw_grid(),
            self.draw_substrate(mirror_x=mirror_x),
        ]
        cu_ref_key = "copper_bottom" if mirror_x else "copper_top"
        silk_ref_key = "silkscreen_bottom" if mirror_x else "silkscreen_top"
        d_cu_ref = self._geom_to_path_d(self.norm_layers.get(cu_ref_key), mirror_x=mirror_x)
        if d_cu_ref:
            body.append(f'<path d="{d_cu_ref}" fill="#ca8a04" fill-rule="evenodd" opacity="0.18"/>')
        d_silk_ref = self._geom_to_path_d(self.norm_layers.get(silk_ref_key), mirror_x=mirror_x)
        if d_silk_ref:
            body.append(f'<path d="{d_silk_ref}" fill="#94a3b8" fill-rule="evenodd" opacity="0.12"/>')

        body.extend([
            self.draw_snap_tab(mirror_x=mirror_x),
            self.draw_dimensions(mirror_x=mirror_x),
            self.draw_drills_layer(emphasize=True, mirror_x=mirror_x),
        ])

        if self.drills:
            cols_per_row = 6
            col_step = (self.px_w - 2 * self.margin * self.scale) / cols_per_row
            col_step = max(col_step, 84.0)
            row_start_x = self.margin * self.scale
            items_sorted = sorted(self.drills.keys())
            total_rows = (len(items_sorted) + cols_per_row - 1) // cols_per_row
            base_ly = self.px_h - 14.0 - (total_rows - 1) * 14.0
            for idx_d, dia in enumerate(items_sorted):
                r_idx = idx_d // cols_per_row
                c_idx = idx_d % cols_per_row
                lx = row_start_x + c_idx * col_step
                ly = base_ly + r_idx * 14.0
                cnt = len(self.drills[dia])
                col = get_drill_color(dia)
                lbl = f"{dia:.2f}mm ({cnt})"
                body.append(f'<circle cx="{lx + 4:.1f}" cy="{ly - 3:.1f}" r="3.5" fill="{col}" stroke="#ffffff" stroke-width="0.6"/>')
                body.append(f'<text x="{lx + 11:.1f}" y="{ly:.1f}" fill="#cbd5e1" font-family="monospace" font-size="9.5">{svg_escape(lbl)}</text>')

        m_tag = " (MIRRORED VIEW)" if mirror_x else ""
        hdr = svg_escape(f"STEP 1: MULTI-DIAMETER TOP CNC DRILLING{m_tag}")
        sub = svg_escape(
            f"Total Holes: {total_holes} ({len(self.drills)} bit sizes) | "
            f"Tag Mouse-Bites: {mb_count} | Mode: {self.breakaway_cfg.label}"
        )
        body.append(f'<text x="{self.margin*self.scale:.1f}" y="20" fill="#38bdf8" font-family="monospace" font-weight="bold" font-size="14">{hdr}</text>')
        body.append(f'<text x="{self.margin*self.scale:.1f}" y="36" fill="#94a3b8" font-family="monospace" font-size="11">{sub}</text>')
        return self._wrap_svg(body)

    def build_step2_vias_svg(self, mirror_x: bool = False):
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(mirror_x=mirror_x),
            self.draw_snap_tab(mirror_x=mirror_x), self.draw_dimensions(mirror_x=mirror_x)
        ]
        via_count = len(self.drills.get(0.3, []))
        for (hx, hy) in self.drills.get(0.3, []):
            sx, sy = self._tr(self.pcb_x_off + hx, self.pcb_y_off + hy, mirror_x=mirror_x)
            elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{1.4 * self.scale:.1f}" fill="rgba(34, 211, 238, 0.25)" stroke="#22d3ee" stroke-width="1.3"/>')
            elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{0.55 * self.scale:.1f}" fill="#22d3ee"/>')
            elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{0.2 * self.scale:.1f}" fill="#090d16"/>')

        hdr = svg_escape("STEP 2: CHEMICAL THROUGH-HOLE VIA METALLIZATION")
        sub = svg_escape(f"Via Count: {via_count} holes (0.30mm dia) | Target Barrel Plating: 25µm Cu")
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="22" fill="#22d3ee" font-family="monospace" font-weight="bold" font-size="14">{hdr}</text>')
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">{sub}</text>')
        return self._wrap_svg(elems)

    def build_step3_top_copper_svg(self, mirror_x: bool = False):
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(mirror_x=mirror_x),
            self.draw_snap_tab(mirror_x=mirror_x), self.draw_dimensions(mirror_x=mirror_x)
        ]
        d_cu = self._geom_to_path_d(self.norm_layers.get("copper_top"), mirror_x=mirror_x)
        if d_cu:
            elems.append(f'<path d="{d_cu}" fill="#eab308" fill-rule="evenodd" opacity="0.92" stroke="#ca8a04" stroke-width="0.5"/>')
        elems.append(self.draw_drills_layer(mirror_x=mirror_x))
        m_tag = " (MIRRORED VIEW)" if mirror_x else " (FRONT SIDE)"
        hdr = svg_escape(f"STEP 3: TOP COPPER ISOLATION ENGRAVING{m_tag}")
        sub = svg_escape("Tool: 30° / 0.1mm V-Bit | Cut Depth: -0.04mm | Front Copper Layer")
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="22" fill="#eab308" font-family="monospace" font-weight="bold" font-size="14">{hdr}</text>')
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">{sub}</text>')
        return self._wrap_svg(elems)

    def build_step4_bottom_copper_svg(self, mirror_x: bool = True):
        """Render Bottom Copper (mirrored horizontally along Y-axis by default, or unmirrored if mirror_x=False)."""
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(mirror_x=mirror_x),
            self.draw_snap_tab(mirror_x=mirror_x), self.draw_dimensions(mirror_x=mirror_x)
        ]
        d_cu = self._geom_to_path_d(self.norm_layers.get("copper_bottom"), mirror_x=mirror_x)
        if d_cu:
            elems.append(f'<path d="{d_cu}" fill="#f59e0b" fill-rule="evenodd" opacity="0.92" stroke="#b45309" stroke-width="0.5"/>')
        elems.append(self.draw_drills_layer(mirror_x=mirror_x))
        m_tag = "MIRRORED VIEW (Y-AXIS FLIP)" if mirror_x else "UNMIRRORED TOP-DOWN X-RAY"
        hdr = svg_escape(f"STEP 4: BOTTOM COPPER ISOLATION ENGRAVING · {m_tag}")
        sub = svg_escape("Back Copper Layer | Tool: 30° V-Bit (-0.04mm)")
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="22" fill="#f59e0b" font-family="monospace" font-weight="bold" font-size="14">{hdr}</text>')
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">{sub}</text>')
        return self._wrap_svg(elems)

    def build_step5_solder_mask_svg(self, side: str = "front", mirror_x: bool = False):
        """Render Front Side (`mask_top`) or Back Side (`mask_bottom`) Solder Mask openings with optional X-mirror."""
        is_back = (side == "back")
        layer_key = "mask_bottom" if is_back else "mask_top"
        cu_key = "copper_bottom" if is_back else "copper_top"
        tr = lambda x, y: self._tr(x, y, mirror_x=mirror_x)
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(mirror_x=mirror_x),
            self.draw_snap_tab(mirror_x=mirror_x), self.draw_dimensions(mirror_x=mirror_x)
        ]
        x0, y1, w_px, h_px = _svg_rect_bounds(
            self.pcb_x_off, self.pcb_y_off, self.pcb_x_off + self.w, self.pcb_y_off + self.h, tr
        )
        elems.append(f'<rect x="{x0:.1f}" y="{y1:.1f}" width="{w_px:.1f}" height="{h_px:.1f}" fill="#14532d" opacity="0.78" rx="2"/>')
        d_cu = self._geom_to_path_d(self.norm_layers.get(cu_key), mirror_x=mirror_x)
        if d_cu:
            elems.append(f'<path d="{d_cu}" fill="#ca8a04" fill-rule="evenodd" opacity="0.20"/>')
        d_mask = self._geom_to_path_d(self.norm_layers.get(layer_key), mirror_x=mirror_x)
        if d_mask:
            elems.append(f'<path d="{d_mask}" fill="#fbbf24" fill-rule="evenodd" stroke="#f59e0b" stroke-width="0.8"/>')
        else:
            elems.append(
                f'<text x="{x0 + w_px/2:.1f}" y="{y1 + h_px/2:.1f}" fill="#86efac" '
                f'font-family="monospace" font-size="11" text-anchor="middle">NO SOLDER MASK OPENINGS ON {side.upper()} SIDE</text>'
            )
        elems.append(self.draw_drills_layer(mirror_x=mirror_x))
        side_txt = "BACK SIDE" if is_back else "FRONT SIDE"
        m_txt = " · MIRRORED" if mirror_x else " · UNMIRRORED"
        hdr = svg_escape(f"STEP 5: SOLDER MASK PAD CLEARING ({side_txt}{m_txt})")
        sub = svg_escape(f"Layer: {layer_key} | Z-Height Compensation: +35µm | M0 Pause for surface re-touch")
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="22" fill="#22c55e" font-family="monospace" font-weight="bold" font-size="14">{hdr}</text>')
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">{sub}</text>')
        return self._wrap_svg(elems)

    def build_step6_silkscreen_svg(self, side: str = "front", mirror_x: bool = False):
        """Render Front Side (`silkscreen_top`) or Back Side (`silkscreen_bottom`) Silkscreen with optional X-mirror."""
        is_back = (side == "back")
        layer_key = "silkscreen_bottom" if is_back else "silkscreen_top"
        cu_key = "copper_bottom" if is_back else "copper_top"
        tr = lambda x, y: self._tr(x, y, mirror_x=mirror_x)
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(mirror_x=mirror_x),
            self.draw_snap_tab(mirror_x=mirror_x), self.draw_dimensions(mirror_x=mirror_x)
        ]
        x0, y1, w_px, h_px = _svg_rect_bounds(
            self.pcb_x_off, self.pcb_y_off, self.pcb_x_off + self.w, self.pcb_y_off + self.h, tr
        )
        d_cu = self._geom_to_path_d(self.norm_layers.get(cu_key), mirror_x=mirror_x)
        if d_cu:
            elems.append(f'<path d="{d_cu}" fill="#ca8a04" fill-rule="evenodd" opacity="0.16"/>')
        d_silk = self._geom_to_path_d(self.norm_layers.get(layer_key), mirror_x=mirror_x)
        if d_silk:
            elems.append(f'<path d="{d_silk}" fill="#ffffff" fill-rule="evenodd" opacity="0.95"/>')
        else:
            elems.append(
                f'<text x="{x0 + w_px/2:.1f}" y="{y1 + h_px/2:.1f}" fill="#94a3b8" '
                f'font-family="monospace" font-size="11" text-anchor="middle">NO {side.upper()} SILKSCREEN FEATURES IN GERBER</text>'
            )
        elems.append(self.draw_drills_layer(mirror_x=mirror_x))
        side_txt = "BACK SIDE" if is_back else "FRONT SIDE"
        m_txt = " · MIRRORED" if mirror_x else " · UNMIRRORED"
        hdr = svg_escape(f"STEP 6: SILKSCREEN LEGEND PRINTING ({side_txt}{m_txt})")
        sub = svg_escape(f"Layer: {layer_key} | Flatbed Inkjet DoD Printhead | UV Curable White Ink")
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="22" fill="#f8fafc" font-family="monospace" font-weight="bold" font-size="14">{hdr}</text>')
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">{sub}</text>')
        return self._wrap_svg(elems)

    def build_step7_snap_cutout_svg(self, mirror_x: bool = False):
        tr = lambda x, y: self._tr(x, y, mirror_x=mirror_x)
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(mirror_x=mirror_x),
            self.draw_snap_tab(highlight_cutout=True, mirror_x=mirror_x), self.draw_dimensions(mirror_x=mirror_x)
        ]
        bx0, by0 = self.pcb_x_off, self.pcb_y_off
        bx1, by1 = self.pcb_x_off + self.w, self.pcb_y_off + self.h
        mid_x, mid_y = 0.5 * (bx0 + bx1), 0.5 * (by0 + by1)
        tab_half = 1.5

        def add_cut_seg(ax0, ay0, ax1, ay1):
            p0, p1 = tr(ax0, ay0), tr(ax1, ay1)
            elems.append(f'<line x1="{p0[0]:.1f}" y1="{p0[1]:.1f}" x2="{p1[0]:.1f}" y2="{p1[1]:.1f}" stroke="#ef4444" stroke-width="2.5"/>')

        add_cut_seg(bx0, by1, mid_x - tab_half, by1)
        add_cut_seg(mid_x + tab_half, by1, bx1, by1)
        add_cut_seg(bx1, by0, bx1, mid_y - tab_half)
        add_cut_seg(bx1, mid_y + tab_half, bx1, by1)
        if self.attach_side == "bottom":
            add_cut_seg(bx0, by0, bx0, mid_y - tab_half)
            add_cut_seg(bx0, mid_y + tab_half, bx0, by1)
        else:
            add_cut_seg(bx0, by0, mid_x - tab_half, by0)
            add_cut_seg(mid_x + tab_half, by0, bx1, by0)

        tab_count = len(self.seam_geom.get("retention_tabs", []))
        sub_txt = (
            f"Mode: {self.breakaway_cfg.label} | {tab_count} x {self.breakaway_cfg.retention_tab_width_mm:.1f}mm Retention Tabs | Smooth Edge"
            if tab_count else f"Mode: {self.breakaway_cfg.label} | Mouse-Bite Seam Perforations"
        )
        m_tag = " (MIRRORED VIEW)" if mirror_x else ""
        hdr = svg_escape(f"STEP 7: DUAL-SIDED SNAPABLE CUTOUT & BREAKAWAY{m_tag}")
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="22" fill="#ef4444" font-family="monospace" font-weight="bold" font-size="14">{hdr}</text>')
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">{svg_escape(sub_txt)}</text>')
        return self._wrap_svg(elems)

    def build_composite_stackup_svg(self, mirror_x: bool = False):
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(mirror_x=mirror_x),
            self.draw_snap_tab(mirror_x=mirror_x), self.draw_dimensions(mirror_x=mirror_x)
        ]
        d_cu_b = self._geom_to_path_d(self.norm_layers.get("copper_bottom"), mirror_x=mirror_x)
        if d_cu_b:
            elems.append(f'<path d="{d_cu_b}" fill="#b45309" fill-rule="evenodd" opacity="0.45"/>')
        d_cu = self._geom_to_path_d(self.norm_layers.get("copper_top"), mirror_x=mirror_x)
        if d_cu:
            elems.append(f'<path d="{d_cu}" fill="#eab308" fill-rule="evenodd" opacity="0.85" stroke="#ca8a04" stroke-width="0.3"/>')
        d_mask = self._geom_to_path_d(self.norm_layers.get("mask_top"), mirror_x=mirror_x)
        if d_mask:
            elems.append(f'<path d="{d_mask}" fill="none" stroke="#22c55e" stroke-width="0.8" opacity="0.7"/>')
        d_silk = self._geom_to_path_d(self.norm_layers.get("silkscreen_top"), mirror_x=mirror_x)
        if d_silk:
            elems.append(f'<path d="{d_silk}" fill="#ffffff" fill-rule="evenodd" opacity="0.95"/>')
        elems.append(self.draw_drills_layer(mirror_x=mirror_x))
        m_tag = " (MIRRORED VIEW)" if mirror_x else ""
        hdr = svg_escape(f"COMPOSITE CIRCUIT STACKUP OVERVIEW{m_tag}")
        sub = svg_escape(f"{self.project.project_name} ({self.project.pcb_id}) | {self.w:.1f} x {self.h:.1f} mm | Breakaway: {self.breakaway_cfg.label}")
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="22" fill="#38bdf8" font-family="monospace" font-weight="bold" font-size="14">{hdr}</text>')
        elems.append(f'<text x="{self.margin*self.scale:.1f}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">{sub}</text>')
        return self._wrap_svg(elems)

    def _wrap_svg(self, elements):
        header = (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.px_w:.1f} {self.px_h:.1f}" '
            f'width="{self.px_w:.1f}" height="{self.px_h:.1f}">\n'
            f'<defs>\n'
            f'  <marker id="arr-dim" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">\n'
            f'    <path d="M 0 1.5 L 8 5 L 0 8.5 z" fill="#64748b"/>\n'
            f'  </marker>\n'
            f'</defs>\n'
        )
        footer = '</svg>\n'
        return header + "\n".join(elements) + "\n" + footer


# ---------------------------------------------------------------------------
# Individual Manufactured Unit SVG Builder
# ---------------------------------------------------------------------------

class UnitSvgBuilder:
    """Renders an individual manufactured PCB unit with explicit Front Side / Back Side and unique unit_id."""
    def __init__(self, unit: PlacedUnit, p_data: Dict, scale: float = 3.6, margin: float = 10.0,
                 side: str = "front", mirror_x: bool = False, y_split: float = 145.0):
        self.unit = unit
        self.p_data = p_data
        self.scale = scale
        self.margin = max(float(margin), 18.0 if unit.w < 95.0 else 12.0)
        self.side = side
        self.mirror_x = mirror_x
        self.y_split = y_split
        self.total_w = unit.w + 2 * self.margin
        self.total_h = unit.h + 2 * self.margin + 12.0  # room for explicit Front/Back Side header
        self.px_w = self.total_w * scale
        self.px_h = self.total_h * scale

    def _tr(self, sheet_x: float, sheet_y: float, mirror_x: Optional[bool] = None) -> Tuple[float, float]:
        use_mirror = self.mirror_x if mirror_x is None else mirror_x
        lx = sheet_x - self.unit.x
        if use_mirror:
            lx = self.unit.w - lx
        ly = sheet_y - self.unit.y
        sx = (self.margin + lx) * self.scale
        sy = ((self.total_h - self.margin) - ly) * self.scale
        return sx, sy

    def build_svg(self) -> str:
        u = self.unit
        is_back = (self.side == "back")
        elems = [
            f'<rect width="{self.px_w:.1f}" height="{self.px_h:.1f}" fill="#0a0e17"/>',
        ]

        pb = u.pcb_box.bounds
        px, py, pw, ph = _svg_rect_bounds(pb[0], pb[1], pb[2], pb[3], self._tr)
        border_col = "#7c3aed" if is_back else "#1b4d2e"
        elems.append(
            f'<rect x="{px:.1f}" y="{py:.1f}" width="{pw:.1f}" height="{ph:.1f}" '
            f'fill="#0b2416" stroke="{border_col}" stroke-width="1.4" rx="1.5"/>'
        )

        # Render primary side copper vividly + opposite side faintly for layer awareness
        primary_key = "copper_bottom" if is_back else "copper_top"
        secondary_key = "copper_top" if is_back else "copper_bottom"
        cu_sec = self.p_data.get(secondary_key)
        if cu_sec:
            d_s = _shapely_geom_to_svg_path_d(cu_sec, lambda lx, ly: self._tr(*u.to_sheet_coords(lx, ly)))
            if d_s:
                elems.append(f'<path d="{d_s}" fill="#b45309" fill-rule="evenodd" opacity="0.22"/>')

        cu_pri = self.p_data.get(primary_key)
        if cu_pri:
            pri_col = "#f59e0b" if is_back else "#eab308"
            d_p = _shapely_geom_to_svg_path_d(cu_pri, lambda lx, ly: self._tr(*u.to_sheet_coords(lx, ly)))
            if d_p:
                elems.append(f'<path d="{d_p}" fill="{pri_col}" fill-rule="evenodd" opacity="0.92"/>')

        # Drills
        for dia, pts in self.p_data.get("drills", {}).items():
            col = get_drill_color(dia)
            rad = max(1.0, (dia / 2.0) * self.scale)
            for (hx, hy) in pts:
                sx, sy = self._tr(*u.to_sheet_coords(hx, hy))
                elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{rad:.1f}" fill="{col}" stroke="#ffffff" stroke-width="0.4"/>')
                elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{max(0.45, rad * 0.42):.1f}" fill="#0a0e17"/>')

        # Stepping tag on shortest edge (unique unit_id ALWAYS upright left-to-right even when mirrored)
        tb = u.tab_box.bounds
        tx, ty, tw, th = _svg_rect_bounds(tb[0], tb[1], tb[2], tb[3], self._tr)
        elems.append(
            f'<rect x="{tx:.1f}" y="{ty:.1f}" width="{tw:.1f}" height="{th:.1f}" '
            f'fill="rgba(217, 119, 6, 0.22)" stroke="#d97706" stroke-width="1.2"/>'
        )
        if self.mirror_x:
            if u.attach_side == "left":
                u_art = build_tag_artwork(
                    tag_x=u.x + u.pcb_sheet_w, tag_y=u.y, tag_len=u.seam_len, tag_w=u.tab_w,
                    attach_side="right", pcb_id=u.unit_id, engrave_arrow=u.engrave_arrow,
                )
            else:
                u_art = build_tag_artwork(
                    tag_x=u.x, tag_y=u.y, tag_len=u.seam_len, tag_w=u.tab_w,
                    attach_side="bottom", pcb_id=u.unit_id, engrave_arrow=u.engrave_arrow,
                )
            tri_poly = u_art["triangle_poly"]
            id_strokes = u_art["id_text_strokes"]
            tr_tag = lambda sx0, sy0: self._tr(sx0, sy0, mirror_x=False)
        else:
            tri_poly = u.triangle_poly
            id_strokes = u.id_text_lines
            tr_tag = self._tr

        if tri_poly:
            tri_pts = " ".join(f"{tr_tag(px0, py0)[0]:.1f},{tr_tag(px0, py0)[1]:.1f}" for (px0, py0) in tri_poly)
            elems.append(f'<polygon points="{tri_pts}" fill="#f59e0b"/>')
        for stroke in id_strokes:
            pts = " ".join(f"{tr_tag(px0, py0)[0]:.1f},{tr_tag(px0, py0)[1]:.1f}" for (px0, py0) in stroke)
            elems.append(f'<polyline points="{pts}" fill="none" stroke="#fbbf24" stroke-width="1.3" stroke-linecap="round"/>')

        # Step cutouts between stepping tag & board
        elems.extend(_render_seam_cutouts_svg(self._tr, u._seam_pt, u.seam_geom, self.scale, highlight_cutout=True))

        # Top Unit Label: Explicit Front Side / Back Side + Unique Instance ID
        side_txt = f"BACK SIDE ({'MIRRORED' if self.mirror_x else 'UNMIRRORED'})" if is_back else "FRONT SIDE"
        half_txt = "Front Bed Half" if (u.y + u.h <= self.y_split + 1e-6) else "Back Bed Half"
        hdr = svg_escape(f"{u.unit_id} · {side_txt}")
        sub = svg_escape(f"S{u.sheet_idx + 1:02d} ({half_txt}) · Rot {u.rotation_deg}° · {u.pcb_w:.1f}x{u.pcb_h:.1f}mm")
        hdr_col = "#c084fc" if is_back else "#fbbf24"
        elems.append(
            f'<text x="{self.margin * self.scale:.1f}" y="16" fill="{hdr_col}" font-family="monospace" font-weight="bold" font-size="10.5">{hdr}</text>'
        )
        elems.append(
            f'<text x="{self.margin * self.scale:.1f}" y="28" fill="#94a3b8" font-family="monospace" font-size="8.4">{sub}</text>'
        )

        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.px_w:.1f} {self.px_h:.1f}" '
            f'width="{self.px_w:.1f}" height="{self.px_h:.1f}">\n' + "\n".join(elems) + "\n</svg>\n"
        )


# ---------------------------------------------------------------------------
# SVG Sheet Blueprint Renderer & Constrained CNC Bed Visualizer
# ---------------------------------------------------------------------------

def _render_unit_on_canvas(unit: PlacedUnit, tr_sheet_fn, scale: float,
                           layer_cache: Optional[LayerGeometryCache] = None,
                           layer_key: str = "copper_top",
                           opacity: float = 1.0,
                           show_badge: bool = False,
                           mirrored_label: bool = False,
                           tr_upright_fn=None,
                           sheet_w: float = 210.0) -> List[str]:
    """Shared helper to draw a placed PCB unit onto an SVG canvas (Top Side or Y-Flipped Back Side with upright text)."""
    elems: List[str] = []
    if opacity < 0.99:
        elems.append(f'<g opacity="{opacity:.2f}">')

    # 1. PCB main body substrate
    pcb_b = unit.pcb_box.bounds
    px, py, pw, ph = _svg_rect_bounds(pcb_b[0], pcb_b[1], pcb_b[2], pcb_b[3], tr_sheet_fn)
    elems.append(
        f'<rect x="{px:.1f}" y="{py:.1f}" width="{pw:.1f}" height="{ph:.1f}" '
        f'fill="#082012" stroke="#15803d" stroke-width="1.2"/>'
    )

    # 2. Render circuit traces and drill holes
    if layer_cache:
        p_data = layer_cache.get_project_layers(unit.project)
        geom = p_data.get(layer_key)
        if geom is not None:
            fill_col = "#f59e0b" if "bottom" in layer_key else "#eab308"
            d_path = _shapely_geom_to_svg_path_d(geom, lambda lx, ly: tr_sheet_fn(*unit.to_sheet_coords(lx, ly)))
            if d_path:
                elems.append(f'<path d="{d_path}" fill="{fill_col}" fill-rule="evenodd" opacity="0.86"/>')

        for dia, pts in p_data.get("drills", {}).items():
            rad = max(0.5, (dia / 2.0) * scale)
            col = get_drill_color(dia)
            for (hx, hy) in pts:
                sx, sy = tr_sheet_fn(*unit.to_sheet_coords(hx, hy))
                elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{rad:.1f}" fill="{col}" stroke="#090d16" stroke-width="0.35"/>')

    # 3. 5mm Stepping Tag along shortest edge
    tab_b = unit.tab_box.bounds
    tx, ty, tw, th = _svg_rect_bounds(tab_b[0], tab_b[1], tab_b[2], tab_b[3], tr_sheet_fn)
    elems.append(
        f'<rect x="{tx:.1f}" y="{ty:.1f}" width="{tw:.1f}" height="{th:.1f}" '
        f'fill="rgba(217, 119, 6, 0.28)" stroke="#d97706" stroke-width="1.1"/>'
    )

    # 4. Mode-aware step cutouts (milling slots, 2mm retention tabs, V-score, mouse bites)
    elems.extend(_render_seam_cutouts_svg(
        tr_sheet_fn, unit._seam_pt, unit.seam_geom, scale, highlight_cutout=False
    ))

    # 5 & 6. Triangle Arrow & Unique unit_id strokes (ALWAYS upright via tr_upright_fn when X-mirrored)
    if tr_upright_fn is not None:
        mir_art = unit.get_mirrored_tag_artwork(sheet_w)
        tri_poly = mir_art["triangle_poly"]
        id_lines = mir_art["id_text_strokes"]
        tr_tag = tr_upright_fn
    else:
        tri_poly = unit.triangle_poly
        id_lines = unit.id_text_lines
        tr_tag = tr_sheet_fn

    if tri_poly:
        tri_pts = " ".join(f"{tr_tag(px0, py0)[0]:.1f},{tr_tag(px0, py0)[1]:.1f}" for (px0, py0) in tri_poly)
        elems.append(f'<polygon points="{tri_pts}" fill="#38bdf8"/>')

    for stroke in id_lines:
        pts = " ".join(f"{tr_tag(px0, py0)[0]:.1f},{tr_tag(px0, py0)[1]:.1f}" for (px0, py0) in stroke)
        elems.append(f'<polyline points="{pts}" fill="none" stroke="#fbbf24" stroke-width="1.2"/>')

    # 7. Crisp upright identification overlay pill inside unit corner
    if show_badge and pw > 45 and ph > 24:
        badge_txt = f"{unit.unit_id} (BACK SIDE)" if mirrored_label else unit.unit_id
        bw = min(pw - 8, len(badge_txt) * 5.8 + 10)
        bx_pill = px + 4
        by_pill = py + 4
        col_border = "#a78bfa" if mirrored_label else "#38bdf8"
        col_text = "#e9d5ff" if mirrored_label else "#e0f2fe"
        elems.append(
            f'<rect x="{bx_pill:.1f}" y="{by_pill:.1f}" width="{bw:.1f}" height="15" rx="3" '
            f'fill="rgba(9, 13, 22, 0.84)" stroke="{col_border}" stroke-width="0.8"/>'
        )
        elems.append(
            f'<text x="{bx_pill + 5:.1f}" y="{by_pill + 10.5:.1f}" fill="{col_text}" '
            f'font-family="monospace" font-size="8.5" font-weight="bold">{svg_escape(badge_txt)}</text>'
        )

    if opacity < 0.99:
        elems.append('</g>')
    return elems


class SvgVisualizer:
    """Renders detailed SVG previews for full A4 panel layouts (Front Side & Back Side with upright text)."""
    def __init__(self, width=210.0, height=297.0, scale=3.5, mirrored_x: bool = False):
        self.width = width
        self.height = height
        self.scale = scale
        self.mirrored_x = mirrored_x
        self.elements = []

    def _tr(self, x, y):
        mx = (self.width - x) if self.mirrored_x else x
        sx = mx * self.scale
        sy = (self.height - y) * self.scale
        return sx, sy

    def _tr_direct(self, x, y):
        """Map pre-mirrored coordinates cleanly onto SVG pixel canvas without double-mirroring text."""
        return x * self.scale, (self.height - y) * self.scale

    def draw_board(self):
        w = self.width * self.scale
        h = self.height * self.scale
        stroke_col = "#a78bfa" if self.mirrored_x else "#d29922"
        self.elements.append(
            f'<rect x="0" y="0" width="{w:.1f}" height="{h:.1f}" fill="#0d1117" stroke="{stroke_col}" stroke-width="2"/>'
        )
        for x in range(0, int(self.width), 10):
            sx, _ = self._tr(x, 0)
            col = "#1f293d" if (x % 50 != 0) else "#2d3b55"
            self.elements.append(f'<line x1="{sx:.1f}" y1="0" x2="{sx:.1f}" y2="{h:.1f}" stroke="{col}" stroke-width="0.8"/>')
        for y in range(0, int(self.height), 10):
            _, sy = self._tr(0, y)
            col = "#1f293d" if (y % 50 != 0) else "#2d3b55"
            self.elements.append(f'<line x1="0" y1="{sy:.1f}" x2="{w:.1f}" y2="{sy:.1f}" stroke="{col}" stroke-width="0.8"/>')

    def draw_holders(self, holders, y_split: float = 145.0):
        """
        Render common CNC bed locking washer clamps (saved once for a single bed half, NO holes drilled
        through PCB) along with Y-shifted Back Half reuse positions on the full sheet.
        """
        for h in holders:
            hx = float(h.get("x", 0))
            hy = float(h.get("y", 0))
            gap = float(h.get("holder_gap", 3.0))
            shape = h.get("shape", "circle").lower()
            is_common = bool(h.get("common_for_both_halves", True))

            y_positions = [(hy, False)]
            if is_common and hy <= y_split + 1.0 and (hy + y_split) <= self.height + 2.0:
                y_positions.append((hy + y_split, True))

            for cy_pos, is_shifted in y_positions:
                if shape == "circle":
                    dia = float(h.get("diameter", 10.0))
                    rad = dia / 2.0
                    tot_r = rad + gap
                    sx, sy = self._tr(hx, cy_pos)
                    dash_attr = '4,3' if is_shifted else '3,2'
                    op = '0.55' if is_shifted else '0.92'
                    self.elements.append(
                        f'<g opacity="{op}">'
                        f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{tot_r * self.scale:.1f}" fill="rgba(239, 68, 68, 0.14)" stroke="#ef4444" stroke-width="1.2" stroke-dasharray="{dash_attr}"/>'
                        f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{rad * self.scale:.1f}" fill="rgba(220, 38, 38, 0.42)" stroke="#f87171" stroke-width="1.8"/>'
                        f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{max(2.2, rad * self.scale * 0.38):.1f}" fill="#0d1117" stroke="#fca5a5" stroke-width="1.0"/>'
                        f'</g>'
                    )
                else:
                    w = float(h.get("width", 10.0))
                    l = float(h.get("length", 20.0))
                    rx0, ry0, rsw, rsh = _svg_rect_bounds(hx - w/2.0 - gap, cy_pos - l/2.0 - gap, hx + w/2.0 + gap, cy_pos + l/2.0 + gap, self._tr)
                    self.elements.append(
                        f'<rect x="{rx0:.1f}" y="{ry0:.1f}" width="{rsw:.1f}" height="{rsh:.1f}" fill="rgba(239, 68, 68, 0.16)" stroke="#ef4444" stroke-width="1.2" stroke-dasharray="3,3"/>'
                    )
                    rx1, ry1, rsw1, rsh1 = _svg_rect_bounds(hx - w/2.0, cy_pos - l/2.0, hx + w/2.0, cy_pos + l/2.0, self._tr)
                    self.elements.append(
                        f'<rect x="{rx1:.1f}" y="{ry1:.1f}" width="{rsw1:.1f}" height="{rsh1:.1f}" fill="rgba(220, 38, 38, 0.45)" stroke="#f87171" stroke-width="1.5"/>'
                    )

    def draw_unit(self, unit: PlacedUnit, layer_cache: Optional[LayerGeometryCache] = None):
        layer_key = "copper_bottom" if self.mirrored_x else "copper_top"
        self.elements.extend(
            _render_unit_on_canvas(
                unit, self._tr, self.scale, layer_cache=layer_cache,
                layer_key=layer_key, opacity=1.0, show_badge=True, mirrored_label=self.mirrored_x,
                tr_upright_fn=(self._tr_direct if self.mirrored_x else None),
                sheet_w=self.width,
            )
        )

    def draw_y_split(self, y_split):
        _, sy = self._tr(0, y_split)
        w = self.width * self.scale
        self.elements.append(
            f'<line x1="0" y1="{sy:.1f}" x2="{w:.1f}" y2="{sy:.1f}" stroke="#fb7185" stroke-width="2" stroke-dasharray="6,4"/>'
        )
        lbl = svg_escape(f"Y-SHIFT BOUNDARY ({y_split:.1f}mm)")
        lbl_x = 18.0 if self.mirrored_x else (w - 220.0)
        self.elements.append(
            f'<text x="{lbl_x:.1f}" y="{sy - 8:.1f}" fill="#fb7185" font-family="monospace" font-size="11" font-weight="bold">{lbl}</text>'
        )

    def draw_polylines(self, polylines, color="#38bdf8", width=1.0, upright_mirrored: bool = False):
        tr_fn = self._tr_direct if (upright_mirrored and self.mirrored_x) else self._tr
        d = _polylines_to_svg_path_d(polylines, tr_fn)
        if d:
            self.elements.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linecap="round"/>')

    def draw_circles(self, points, color="#fb7185", radius=1.5, upright_mirrored: bool = False):
        tr_fn = self._tr_direct if (upright_mirrored and self.mirrored_x) else self._tr
        r_px = radius * self.scale
        for pt in points:
            sx, sy = tr_fn(pt[0], pt[1])
            self.elements.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{r_px:.1f}" fill="none" stroke="{color}" stroke-width="1.2"/>')

    def draw_orientation_banner(self, sheet_num: int):
        w = self.width * self.scale
        lbl = (f"SHEET {sheet_num:02d} · BACK SIDE MIRRORED (Y-AXIS FLIP | UPRIGHT TEXT READABILITY | COMMON BED WASHER CLAMPS)"
               if self.mirrored_x else
               f"SHEET {sheet_num:02d} · FRONT SIDE VIEW (TOP COPPER | COMMON BED WASHER CLAMPS · NO PCB ANCHOR DRILLS)")
        bg = "rgba(139, 92, 246, 0.2)" if self.mirrored_x else "rgba(56, 189, 248, 0.14)"
        col = "#ddd6fe" if self.mirrored_x else "#bae6fd"
        self.elements.append(f'<rect x="6" y="6" width="{w - 12:.1f}" height="22" rx="4" fill="{bg}" stroke="{col}" stroke-width="0.6" opacity="0.9"/>')
        self.elements.append(f'<text x="14" y="21" fill="{col}" font-family="monospace" font-size="10.0" font-weight="bold">{svg_escape(lbl)}</text>')

    def render(self):
        w = self.width * self.scale
        h = self.height * self.scale
        content = "\n".join(self.elements)
        return f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w:.1f} {h:.1f}" width="{w:.1f}" height="{h:.1f}">\n{content}\n</svg>'


class BedSvgBuilder:
    """
    Renders the physical CNC Bed (275 x 170 mm) setup for a single machining operation:
    - Fixed single-half common M4 washer bed clamps (never drilled into PCB)
    - Z-Zero Limit Switch Danger Zone at X=272.001mm, Y=135.002mm
    - Accurate X-mirroring on Back Side with upright readable sheet tags, guides, and unit IDs.
    """
    def __init__(self, bed_w: float = 275.0, bed_h: float = 170.0,
                 sheet_w: float = 210.0, sheet_h: float = 297.0, y_split: float = 145.0,
                 scale: float = 3.2):
        self.bed_w = bed_w
        self.bed_h = bed_h
        self.sheet_w = sheet_w
        self.sheet_h = sheet_h
        self.y_split = y_split
        self.scale = scale
        self.pad_l = 16.0
        self.pad_r = 14.0
        self.pad_b = 14.0
        self.pad_t = 24.0
        self.px_w = (self.bed_w + self.pad_l + self.pad_r) * self.scale
        self.px_h = (self.bed_h + self.pad_t + self.pad_b) * self.scale

    def _tr_bed(self, bx: float, by: float) -> Tuple[float, float]:
        sx = (self.pad_l + bx) * self.scale
        sy = (self.pad_t + self.bed_h - by) * self.scale
        return sx, sy

    def _draw_z_limit_switch_danger_zone(self, z_probe_switch: Optional[Dict]) -> List[str]:
        """Render the fixed Z-Zero Limit Switch at X=272.001mm, Y=135.002mm as an explicit red/amber DANGER ZONE."""
        if not z_probe_switch or not z_probe_switch.get("enabled", True):
            return []
        zx = float(z_probe_switch.get("x_mm", 272.001))
        zy = float(z_probe_switch.get("y_mm", 135.002))
        zw = float(z_probe_switch.get("width_mm", 7.0))
        zl = float(z_probe_switch.get("length_mm", 20.0))
        zm = float(z_probe_switch.get("danger_margin_mm", 6.0))

        elems: List[str] = []
        # Danger keep-out zone polygon around the bed switch
        dx0, dy0, dw, dh = _svg_rect_bounds(zx - zw/2.0 - zm, zy - zl/2.0 - zm, zx + zw/2.0 + zm, zy + zl/2.0 + zm, self._tr_bed)
        elems.append(
            f'<rect x="{dx0:.1f}" y="{dy0:.1f}" width="{dw:.1f}" height="{dh:.1f}" rx="3" '
            f'fill="rgba(239, 68, 68, 0.18)" stroke="#ef4444" stroke-width="1.6" stroke-dasharray="4,2"/>'
        )
        # Switch physical housing
        hx0, hy0, hw, hh = _svg_rect_bounds(zx - zw/2.0, zy - zl/2.0, zx + zw/2.0, zy + zl/2.0, self._tr_bed)
        elems.append(
            f'<rect x="{hx0:.1f}" y="{hy0:.1f}" width="{hw:.1f}" height="{hh:.1f}" rx="1.5" '
            f'fill="#b91c1c" stroke="#fca5a5" stroke-width="1.3"/>'
        )
        # Precision probe target crosshair
        px_z, py_z = self._tr_bed(zx, zy)
        elems.append(f'<circle cx="{px_z:.1f}" cy="{py_z:.1f}" r="5.0" fill="rgba(251, 191, 36, 0.25)" stroke="#fbbf24" stroke-width="1.2"/>')
        elems.append(f'<circle cx="{px_z:.1f}" cy="{py_z:.1f}" r="2.0" fill="#fef08a"/>')
        elems.append(f'<line x1="{px_z - 7:.1f}" y1="{py_z:.1f}" x2="{px_z + 7:.1f}" y2="{py_z:.1f}" stroke="#fbbf24" stroke-width="0.9"/>')
        elems.append(f'<line x1="{px_z:.1f}" y1="{py_z - 7:.1f}" x2="{px_z:.1f}" y2="{py_z + 7:.1f}" stroke="#fbbf24" stroke-width="0.9"/>')

        # Callout badge positioned safely inside bed bounds to the left of the switch
        lx_badge = dx0 - 156.0
        ly_badge = py_z - 15.0
        elems.append(f'<line x1="{dx0:.1f}" y1="{py_z:.1f}" x2="{lx_badge + 150:.1f}" y2="{py_z:.1f}" stroke="#ef4444" stroke-width="1.0" stroke-dasharray="2,2"/>')
        elems.append(
            f'<rect x="{lx_badge:.1f}" y="{ly_badge:.1f}" width="150" height="30" rx="4" '
            f'fill="rgba(127, 29, 29, 0.92)" stroke="#ef4444" stroke-width="1.2"/>'
        )
        elems.append(
            f'<text x="{lx_badge + 7:.1f}" y="{ly_badge + 12:.1f}" fill="#fecaca" font-family="monospace" font-size="8.2" font-weight="bold">DANGER ZONE: Z-ZERO SWITCH</text>'
        )
        elems.append(
            f'<text x="{lx_badge + 7:.1f}" y="{ly_badge + 23:.1f}" fill="#fde047" font-family="monospace" font-size="8.0">X={zx:.3f}  Y={zy:.3f} mm</text>'
        )
        return elems

    def build_operation_svg(
        self,
        sheet_num: int,
        op_seq: int,
        title: str,
        subtitle: str,
        board_side: str,          # "top" | "bottom"
        bed_half: str,            # "front" | "back"
        units: List[PlacedUnit],
        holders: List[Dict],
        guides: Dict,
        sheet_tag: Dict,
        layer_cache: LayerGeometryCache,
        active_polylines_bed: Optional[List[List[Tuple[float, float]]]] = None,
        polyline_color: str = "#fbbf24",
        polyline_width: float = 1.1,
        polyline_dash: str = "",
        active_drill_dia: Optional[float] = None,
        active_drill_pts_bed: Optional[List[Tuple[float, float]]] = None,
        inactive_drill_pts_bed: Optional[List[Tuple[float, float]]] = None,
        pin_holes_bed: Optional[List[Tuple[float, float]]] = None,
        z_probe_switch: Optional[Dict] = None,
    ) -> str:
        mirrored_x = (board_side == "bottom")
        y_off = self.y_split if bed_half == "back" else 0.0

        def sheet_to_bed(sx: float, sy: float) -> Tuple[float, float]:
            bx = (self.sheet_w - sx) if mirrored_x else sx
            by = sy - y_off
            return bx, by

        def tr_sheet(sx: float, sy: float) -> Tuple[float, float]:
            return self._tr_bed(*sheet_to_bed(sx, sy))

        def tr_sheet_upright(sx: float, sy: float) -> Tuple[float, float]:
            """Transform pre-mirrored coordinates onto bed without double-mirroring text."""
            return self._tr_bed(sx, sy - y_off)

        elems: List[str] = [
            f'<rect width="{self.px_w:.1f}" height="{self.px_h:.1f}" fill="#090d16"/>',
        ]

        cx0, cy0, cw, ch = _svg_rect_bounds(-4.0, -6.0, self.bed_w + 8.0, self.bed_h + 6.0, self._tr_bed)
        elems.append(f'<defs><clipPath id="bed-clip"><rect x="{cx0:.1f}" y="{cy0:.1f}" width="{cw:.1f}" height="{ch:.1f}" rx="4"/></clipPath></defs>')

        # 1. CNC Bed grid across [0..bed_w] x [0..bed_h]
        bx0, by0, bw, bh = _svg_rect_bounds(0.0, 0.0, self.bed_w, self.bed_h, self._tr_bed)
        elems.append(f'<rect x="{bx0:.1f}" y="{by0:.1f}" width="{bw:.1f}" height="{bh:.1f}" fill="#0d1320" stroke="#334155" stroke-width="1.5"/>')
        for x in range(0, int(self.bed_w) + 1, 10):
            px_x, _ = self._tr_bed(x, 0)
            col = "#243047" if (x % 50 == 0) else "#151d2e"
            elems.append(f'<line x1="{px_x:.1f}" y1="{by0:.1f}" x2="{px_x:.1f}" y2="{by0 + bh:.1f}" stroke="{col}" stroke-width="0.7"/>')
            if x % 50 == 0:
                elems.append(f'<text x="{px_x:.1f}" y="{by0 + bh + 13:.1f}" fill="#64748b" font-family="monospace" font-size="9" text-anchor="middle">{x}</text>')
        for y in range(0, int(self.bed_h) + 1, 10):
            _, py_y = self._tr_bed(0, y)
            col = "#243047" if (y % 50 == 0) else "#151d2e"
            elems.append(f'<line x1="{bx0:.1f}" y1="{py_y:.1f}" x2="{bx0 + bw:.1f}" y2="{py_y:.1f}" stroke="{col}" stroke-width="0.7"/>')
            if y % 50 == 0:
                elems.append(f'<text x="{bx0 - 6:.1f}" y="{py_y + 3:.1f}" fill="#64748b" font-family="monospace" font-size="9" text-anchor="end">{y}</text>')

        elems.append('<g clip-path="url(#bed-clip)">')

        # 2. Full A4 Copper Sheet Envelope at current Y shift (Front Half & Back Half zones)
        f_rx, f_ry, f_rw, f_rh = _svg_rect_bounds(0.0, 0.0, self.sheet_w, self.y_split, tr_sheet)
        b_rx, b_ry, b_rw, b_rh = _svg_rect_bounds(0.0, self.y_split, self.sheet_w, self.sheet_h, tr_sheet)
        elems.append(
            f'<rect x="{f_rx:.1f}" y="{f_ry:.1f}" width="{f_rw:.1f}" height="{f_rh:.1f}" '
            f'fill="rgba(16, 185, 129, 0.05)" stroke="#10b981" stroke-width="1.2" opacity="{"0.95" if bed_half=="front" else "0.28"}"/>'
        )
        elems.append(
            f'<rect x="{b_rx:.1f}" y="{b_ry:.1f}" width="{b_rw:.1f}" height="{b_rh:.1f}" '
            f'fill="rgba(139, 92, 246, 0.05)" stroke="#8b5cf6" stroke-width="1.2" opacity="{"0.95" if bed_half=="back" else "0.28"}"/>'
        )

        # 3. Fixed Single-Half Common Bed Washer Clamps (identical bed coords for both Front Half & Back Half; no PCB drill)
        for h in holders:
            hx, hy = float(h.get("x", 0)), float(h.get("y", 0))
            if hy < -15.0 or hy > self.bed_h + 15.0:
                continue
            gap = float(h.get("holder_gap", 3.0))
            if h.get("shape", "circle").lower() == "circle":
                rad = float(h.get("diameter", 10.0)) / 2.0
                cx, cy = self._tr_bed(hx, hy)
                elems.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{(rad + gap) * self.scale:.1f}" fill="rgba(239,68,68,0.14)" stroke="#ef4444" stroke-width="0.9" stroke-dasharray="3,2"/>')
                elems.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{rad * self.scale:.1f}" fill="rgba(220,38,38,0.42)" stroke="#f87171" stroke-width="1.6"/>')
                elems.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{max(2.0, rad * self.scale * 0.36):.1f}" fill="#090d16" stroke="#fca5a5" stroke-width="0.9"/>')
            else:
                hw = float(h.get("width", 10.0)) / 2.0
                hl = float(h.get("length", 20.0)) / 2.0
                crx, cry, crw, crh = _svg_rect_bounds(hx - hw, hy - hl, hx + hw, hy + hl, self._tr_bed)
                elems.append(f'<rect x="{crx:.1f}" y="{cry:.1f}" width="{crw:.1f}" height="{crh:.1f}" fill="rgba(220,38,38,0.45)" stroke="#f87171" stroke-width="1.4"/>')

        # 4. Left Margin Guides, Sheet Tag, and Units (upright left-to-right text even when mirrored_x=True)
        layer_key = "copper_bottom" if mirrored_x else "copper_top"
        tr_g = tr_sheet_upright if mirrored_x else tr_sheet
        g_d = _polylines_to_svg_path_d(guides.get("lines", []) + guides.get("texts", []), tr_g)
        if g_d:
            elems.append(f'<path d="{g_d}" fill="none" stroke="#38bdf8" stroke-width="0.8" opacity="0.50"/>')

        stag_d = _polylines_to_svg_path_d(sheet_tag.get("border", []) + sheet_tag.get("text", []) + sheet_tag.get("arrow", []), tr_g)
        if stag_d:
            elems.append(f'<path d="{stag_d}" fill="none" stroke="#f59e0b" stroke-width="1.0" opacity="0.85"/>')

        for u in units:
            u_is_active = ((u.y + u.h <= self.y_split + 1e-6) and bed_half == "front") or ((u.y >= self.y_split - 1e-6) and bed_half == "back")
            elems.extend(
                _render_unit_on_canvas(
                    u, tr_sheet, self.scale, layer_cache=layer_cache, layer_key=layer_key,
                    opacity=0.85 if u_is_active else 0.18, show_badge=u_is_active, mirrored_label=mirrored_x,
                    tr_upright_fn=(tr_sheet_upright if mirrored_x else None),
                    sheet_w=self.sheet_w,
                )
            )

        # 5. Active CNC Toolpaths for THIS operation (already in Bed coordinates)
        if active_polylines_bed:
            tp_d = _polylines_to_svg_path_d(active_polylines_bed, self._tr_bed)
            if tp_d:
                dash_attr = f' stroke-dasharray="{polyline_dash}"' if polyline_dash else ""
                elems.append(
                    f'<path d="{tp_d}" fill="none" stroke="{polyline_color}" '
                    f'stroke-width="{polyline_width:.2f}" stroke-linecap="round"{dash_attr}/>'
                )

        if inactive_drill_pts_bed:
            for (dx, dy) in inactive_drill_pts_bed:
                px_d, py_d = self._tr_bed(dx, dy)
                elems.append(f'<circle cx="{px_d:.1f}" cy="{py_d:.1f}" r="1.4" fill="#334155" opacity="0.5"/>')

        if active_drill_pts_bed:
            if len(active_drill_pts_bed) >= 2:
                trav_d = _polylines_to_svg_path_d([active_drill_pts_bed], self._tr_bed)
                elems.append(f'<path d="{trav_d}" fill="none" stroke="#38bdf8" stroke-width="0.7" stroke-dasharray="3,3" opacity="0.45"/>')
            d_col = get_drill_color(active_drill_dia or 0.8)
            d_rad = max(2.8, ((active_drill_dia or 0.8) / 2.0) * self.scale * 1.4)
            for idx_p, (dx, dy) in enumerate(active_drill_pts_bed):
                px_d, py_d = self._tr_bed(dx, dy)
                ch = d_rad + 2.8
                elems.append(f'<circle cx="{px_d:.1f}" cy="{py_d:.1f}" r="{d_rad + 2.2:.1f}" fill="{d_col}" opacity="0.25"/>')
                elems.append(f'<circle cx="{px_d:.1f}" cy="{py_d:.1f}" r="{d_rad:.1f}" fill="{d_col}" stroke="#ffffff" stroke-width="0.9"/>')
                elems.append(f'<line x1="{px_d - ch:.1f}" y1="{py_d:.1f}" x2="{px_d + ch:.1f}" y2="{py_d:.1f}" stroke="#ffffff" stroke-width="0.6" opacity="0.8"/>')
                elems.append(f'<line x1="{px_d:.1f}" y1="{py_d - ch:.1f}" x2="{px_d:.1f}" y2="{py_d + ch:.1f}" stroke="#ffffff" stroke-width="0.6" opacity="0.8"/>')
                if idx_p == 0:
                    elems.append(f'<circle cx="{px_d:.1f}" cy="{py_d:.1f}" r="{d_rad + 5:.1f}" fill="none" stroke="#22c55e" stroke-width="1.3"/>')

        if pin_holes_bed:
            for (px_h, py_h) in pin_holes_bed:
                ux, uy = self._tr_bed(px_h, py_h)
                r_pin = 1.5 * self.scale
                elems.append(f'<circle cx="{ux:.1f}" cy="{uy:.1f}" r="{r_pin + 2:.1f}" fill="none" stroke="#fb7185" stroke-width="1.4"/>')
                elems.append(f'<circle cx="{ux:.1f}" cy="{uy:.1f}" r="{r_pin:.1f}" fill="#fb7185" opacity="0.35"/>')

        elems.append('</g>')

        # 6. Bed Travel Envelope Box, Origin Crosshair, and Z-Zero Limit Switch Danger Zone
        elems.append(f'<rect x="{bx0:.1f}" y="{by0:.1f}" width="{bw:.1f}" height="{bh:.1f}" fill="none" stroke="#38bdf8" stroke-width="1.6" stroke-dasharray="6,3"/>')
        ox_p, oy_p = self._tr_bed(0.0, 0.0)
        elems.append(f'<circle cx="{ox_p:.1f}" cy="{oy_p:.1f}" r="4.5" fill="none" stroke="#22c55e" stroke-width="1.5"/>')
        elems.append(f'<text x="{ox_p + 6:.1f}" y="{oy_p + 13:.1f}" fill="#22c55e" font-family="monospace" font-size="8.5" font-weight="bold">G54 (0,0)</text>')
        elems.extend(self._draw_z_limit_switch_danger_zone(z_probe_switch))

        # Header & Orientation Badges (separated rows prevent horizontal collision)
        hdr_col = "#c084fc" if mirrored_x else "#38bdf8"
        bed_badge = f"BED {self.bed_w:.0f}x{self.bed_h:.0f}mm · {'FRONT HALF' if bed_half=='front' else f'SHIFT Y-{self.y_split:.0f}'}"
        elems.append(f'<text x="{self.pad_l * self.scale:.1f}" y="20" fill="{hdr_col}" font-family="monospace" font-weight="bold" font-size="10.8">{svg_escape(title)}</text>')
        elems.append(f'<text x="{self.px_w - self.pad_r * self.scale:.1f}" y="20" fill="#94a3b8" font-family="monospace" font-size="9.5" text-anchor="end">{svg_escape(bed_badge)}</text>')
        elems.append(f'<text x="{self.pad_l * self.scale:.1f}" y="38" fill="#cbd5e1" font-family="monospace" font-size="9.6">{svg_escape(subtitle)}</text>')

        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.px_w:.1f} {self.px_h:.1f}" '
            f'width="{self.px_w:.1f}" height="{self.px_h:.1f}">\n' + "\n".join(elems) + "\n</svg>\n"
        )


# ---------------------------------------------------------------------------
# G-code Formatter & Dialect Generator
# ---------------------------------------------------------------------------

class GcodeBuilder:
    """Standardizes CNC G-code formatting with FluidNC compatibility."""
    def __init__(self, title, feed=300.0, plunge=100.0, cut_z=-0.04, safe_z=5.0, spindle_rpm=10000):
        self.title = title
        self.feed = feed
        self.plunge = plunge
        self.cut_z = cut_z
        self.safe_z = safe_z
        self.spindle_rpm = spindle_rpm
        self.lines = []

    def header(self, comments=None):
        self.lines.append(f"; === {self.title} ===")
        self.lines.append("; Generated by A4 Copper Board CNC Panelizer")
        if comments:
            for c in comments:
                self.lines.append(f"; {c}")
        self.lines.append("G21 ; Millimeter units")
        self.lines.append("G90 ; Absolute coordinates")
        self.lines.append(f"G0 Z{self.safe_z:.3f} ; Move to safe Z height")

    def spindle_start(self):
        self.lines.append(f"M3 S{self.spindle_rpm} ; Start spindle clockwise")
        self.lines.append("G4 P1.5 ; Dwell 1.5s for spindle to reach speed")

    def engrave_polyline(self, polyline: List[Tuple[float, float]], z_depth: Optional[float] = None):
        if not polyline:
            return
        z = z_depth if z_depth is not None else self.cut_z
        start_x, start_y = polyline[0]
        self.lines.append(f"G0 X{start_x:.3f} Y{start_y:.3f}")
        self.lines.append(f"G1 Z{z:.3f} F{self.plunge:.1f}")
        for pt in polyline[1:]:
            self.lines.append(f"G1 X{pt[0]:.3f} Y{pt[1]:.3f} F{self.feed:.1f}")
        self.lines.append(f"G0 Z{self.safe_z:.3f}")

    def peck_drill(self, x: float, y: float, depth: float = -1.8, step: float = 0.4):
        self.lines.append(f"G0 X{x:.3f} Y{y:.3f}")
        curr_z = 0.0
        while curr_z > depth:
            curr_z = max(depth, curr_z - step)
            self.lines.append(f"G1 Z{curr_z:.3f} F{self.plunge:.1f}")
            self.lines.append(f"G0 Z0.500")
        self.lines.append(f"G0 Z{self.safe_z:.3f}")

    def footer(self):
        self.lines.append(f"G0 Z{self.safe_z:.3f}")
        self.lines.append("M5 ; Stop spindle")
        self.lines.append("G0 X0.000 Y0.000 ; Return to datum")
        self.lines.append("M2 ; End program")

    def get_text(self):
        return "\n".join(self.lines) + "\n"


# ---------------------------------------------------------------------------
# Multi-Step Manufacturing Pipeline Coordinator
# ---------------------------------------------------------------------------

def _split_polyline_by_y(poly: List[Tuple[float, float]], y_split: float) -> Tuple[List[List[Tuple[float, float]]], List[List[Tuple[float, float]]]]:
    """
    Split a polyline cleanly across y = y_split so front-half paths stay strictly <= y_split
    and back-half paths (shifted by -y_split) stay strictly >= 0 for Y-axis-only bed shifts.
    """
    if not poly or len(poly) < 2:
        return [], []
    front_segs: List[List[Tuple[float, float]]] = []
    back_segs: List[List[Tuple[float, float]]] = []
    cur_front: List[Tuple[float, float]] = []
    cur_back: List[Tuple[float, float]] = []

    def flush(cur, target):
        if len(cur) >= 2:
            target.append(cur)

    for i in range(len(poly) - 1):
        x0, y0 = poly[i]
        x1, y1 = poly[i + 1]
        in_f0 = (y0 <= y_split + 1e-6)
        in_f1 = (y1 <= y_split + 1e-6)

        if in_f0 and in_f1:
            if not cur_front:
                cur_front.append((x0, y0))
            cur_front.append((x1, y1))
        elif (not in_f0) and (not in_f1):
            if not cur_back:
                cur_back.append((x0, y0 - y_split))
            cur_back.append((x1, y1 - y_split))
        else:
            # Segment crosses y_split: interpolate exact crossing point
            dy = y1 - y0
            t = (y_split - y0) / dy if abs(dy) > 1e-9 else 0.0
            xc = x0 + t * (x1 - x0)
            if in_f0:
                if not cur_front:
                    cur_front.append((x0, y0))
                cur_front.append((xc, y_split))
                flush(cur_front, front_segs)
                cur_front = []
                cur_back = [(xc, 0.0), (x1, y1 - y_split)]
            else:
                if not cur_back:
                    cur_back.append((x0, y0 - y_split))
                cur_back.append((xc, 0.0))
                flush(cur_back, back_segs)
                cur_back = []
                cur_front = [(xc, y_split), (x1, y1)]

    flush(cur_front, front_segs)
    flush(cur_back, back_segs)
    return front_segs, back_segs


def _deduplicate_and_tab_perimeter_edges(half_units: List[PlacedUnit], y_off: float, bridge_tab_len: float = 3.0) -> List[List[Tuple[float, float]]]:
    """
    Collect all non-seam perimeter edges of unit PCB boxes (plus outer tag outer edges),
    deduplicate shared boundaries between zero-gap adjacent boards for single-pass cutting,
    and insert retention bridge tabs on long outer/shared edges so boards stay anchored.
    """
    if not half_units:
        return []

    horiz_edges: Dict[float, List[Tuple[float, float, bool]]] = {}
    vert_edges: Dict[float, List[Tuple[float, float, bool]]] = {}

    def add_h(y_val: float, x0: float, x1: float):
        yk = round(y_val, 3)
        a, b = (min(x0, x1), max(x0, x1))
        if b - a > 0.1:
            horiz_edges.setdefault(yk, []).append((a, b, False))

    def add_v(x_val: float, y0: float, y1: float):
        xk = round(x_val, 3)
        a, b = (min(y0, y1), max(y0, y1))
        if b - a > 0.1:
            vert_edges.setdefault(xk, []).append((a, b, False))

    for u in half_units:
        px0, py0, px1, py1 = u.pcb_box.bounds
        tx0, ty0, tx1, ty1 = u.tab_box.bounds
        if u.attach_side == "bottom":
            # Bottom of pcb_box is the stepping-tag seam; add top, left, right of pcb_box
            add_h(py1, px0, px1)
            add_v(px0, py0, py1)
            add_v(px1, py0, py1)
            # Outer edges of stepping tag (bottom, left, right)
            add_h(ty0, tx0, tx1)
            add_v(tx0, ty0, ty1)
            add_v(tx1, ty0, ty1)
        else:
            # Left of pcb_box is the stepping-tag seam; add top, bottom, right of pcb_box
            add_h(py0, px0, px1)
            add_h(py1, px0, px1)
            add_v(px1, py0, py1)
            # Outer edges of stepping tag (left, top, bottom)
            add_v(tx0, ty0, ty1)
            add_h(ty0, tx0, tx1)
            add_h(ty1, tx0, tx1)

    def merge_intervals(intervals: List[Tuple[float, float, bool]]) -> List[Tuple[float, float]]:
        sorted_iv = sorted((a, b) for (a, b, _) in intervals)
        merged: List[Tuple[float, float]] = []
        for a, b in sorted_iv:
            if not merged or a > merged[-1][1] + 0.05:
                merged.append((a, b))
            else:
                merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        return merged

    cut_segs: List[List[Tuple[float, float]]] = []

    for yk, ivs in sorted(horiz_edges.items()):
        ym = yk - y_off
        for (a, b) in merge_intervals(ivs):
            if (b - a) > bridge_tab_len * 4.0:
                mid = 0.5 * (a + b)
                cut_segs.append([(a, ym), (mid - bridge_tab_len / 2.0, ym)])
                cut_segs.append([(mid + bridge_tab_len / 2.0, ym), (b, ym)])
            else:
                cut_segs.append([(a, ym), (b, ym)])

    for xk, ivs in sorted(vert_edges.items()):
        for (a, b) in merge_intervals(ivs):
            am, bm = a - y_off, b - y_off
            if (bm - am) > bridge_tab_len * 4.0:
                mid = 0.5 * (am + bm)
                cut_segs.append([(xk, am), (xk, mid - bridge_tab_len / 2.0)])
                cut_segs.append([(xk, mid + bridge_tab_len / 2.0), (xk, bm)])
            else:
                cut_segs.append([(xk, am), (xk, bm)])

    return cut_segs


class ManufacturingPipeline:
    """Executes multi-step G-code, SVG (circuit, unit, sheet & CNC bed views), and metadata generation."""
    def __init__(self, sheets: List[List[PlacedUnit]], layer_cache: LayerGeometryCache,
                 holders_mgr: HolderManager, config: Dict,
                 out_gcode_dir="manufacture_gcode", out_svg_dir="manufacture_svg",
                 out_guides_dir="manufacture_guides"):
        self.sheets = sheets
        self.layer_cache = layer_cache
        self.holders_mgr = holders_mgr
        self.cfg = config
        self.out_gcode_dir = out_gcode_dir
        self.out_svg_dir = out_svg_dir
        self.out_guides_dir = out_guides_dir
        self.breakaway_cfg = BreakawayConfig.from_config(config)

        self.sheet_w = float(config.get("copper_sheet", {}).get("sheet_width_mm", 210.0))
        self.sheet_h = float(config.get("copper_sheet", {}).get("sheet_height_mm", 297.0))
        cnc = config.get("cnc_limits", {})
        self.bed_travel_x = float(cnc.get("bed_travel_x_mm", 275.0))
        self.bed_travel_y = float(cnc.get("bed_travel_y_mm", 170.0))
        self.left_margin = float(cnc.get("left_margin_guide_width_mm", 18.0))
        raw_split = float(cnc.get("y_split_threshold_mm", 145.0))
        if self.sheet_h > self.bed_travel_y:
            self.y_split = max(max(0.0, self.sheet_h - self.bed_travel_y), min(raw_split, self.bed_travel_y - 5.0))
        else:
            self.y_split = raw_split

        self.sheet_tag_pos = config.get("sheet_tag_strip", {}).get("tag_position", "front_right")
        self.enable_dowel_pin_lock = bool(config.get("guides_and_fixturing", {}).get("enable_dowel_pin_lock", False))
        self.bed_svg_builder = BedSvgBuilder(
            bed_w=self.bed_travel_x, bed_h=self.bed_travel_y,
            sheet_w=self.sheet_w, sheet_h=self.sheet_h, y_split=self.y_split,
        )

        os.makedirs(self.out_gcode_dir, exist_ok=True)
        os.makedirs(self.out_svg_dir, exist_ok=True)
        os.makedirs(self.out_guides_dir, exist_ok=True)

        self.generated_files: List[str] = []
        self.bed_operations: Dict[int, List[Dict[str, Any]]] = {}

    def _clean_old_files(self):
        """Clean old generated G-code and SVG files to prevent obsolete files from lingering."""
        for d, exts in [(self.out_gcode_dir, [".gcode", ".txt"]), (self.out_svg_dir, [".svg"])]:
            if os.path.exists(d):
                for fname in os.listdir(d):
                    if any(fname.endswith(ext) for ext in exts):
                        try:
                            os.remove(os.path.join(d, fname))
                        except OSError:
                            pass

    def _validate_all_svgs(self):
        """Ensure every generated SVG parses cleanly as valid XML so browsers never blank out."""
        invalid = []
        for fname in sorted(os.listdir(self.out_svg_dir)):
            if not fname.endswith(".svg"):
                continue
            fpath = os.path.join(self.out_svg_dir, fname)
            try:
                ET.parse(fpath)
            except Exception as exc:
                invalid.append((fname, str(exc)))
        if invalid:
            raise ValueError(f"Generated invalid XML SVG(s): {invalid[:5]}")

    def _record_bed_op(
        self,
        sheet_num: int,
        units: List[PlacedUnit],
        step_code: str,
        step_title: str,
        work_type: str,
        tool_name: str,
        tool_dia_mm: float,
        board_side: str,          # "top" | "bottom"
        bed_half: str,            # "front" | "back"
        gcode_fname: str,
        depth_mm: float,
        feed_mm_min: float,
        spindle_rpm: int,
        shift_action: str,
        stats_label: str,
        started_back_same_tool: bool = False,
        active_polylines_bed: Optional[List[List[Tuple[float, float]]]] = None,
        polyline_color: str = "#fbbf24",
        polyline_width: float = 1.1,
        polyline_dash: str = "",
        active_drill_dia: Optional[float] = None,
        active_drill_pts_bed: Optional[List[Tuple[float, float]]] = None,
        inactive_drill_pts_bed: Optional[List[Tuple[float, float]]] = None,
        pin_holes_bed: Optional[List[Tuple[float, float]]] = None,
    ) -> Dict[str, Any]:
        """Generate a CNC Bed SVG for one physical setup and append its metadata entry."""
        ops_list = self.bed_operations.setdefault(sheet_num, [])
        seq = len(ops_list) + 1
        sm_color = units[0].project.solder_mask_color if units else "green"
        is_mirrored_op = (board_side == "bottom")
        guides = generate_left_guides(
            self.sheet_w, self.sheet_h, margin_w=self.left_margin, y_shift=self.y_split,
            enable_dowel_pins=self.enable_dowel_pin_lock, mirrored_x=is_mirrored_op,
        )
        s_tag = generate_sheet_tag(
            sheet_num, len(self.sheets), len(units), solder_mask_color=sm_color,
            sheet_w=self.sheet_w, left_margin=self.left_margin,
            tag_position=self.sheet_tag_pos, holders_mgr=self.holders_mgr,
            mirrored_x=is_mirrored_op, side_label=("BACK" if is_mirrored_op else "FRONT"),
        )

        # Compute actual coordinate bounds within CNC bed travel
        xs: List[float] = []
        ys: List[float] = []
        for poly in (active_polylines_bed or []):
            for px, py in poly:
                xs.append(px)
                ys.append(py)
        for px, py in (active_drill_pts_bed or []) + (pin_holes_bed or []):
            xs.append(px)
            ys.append(py)
        bounds_x = [round(min(xs), 2), round(max(xs), 2)] if xs else [0.0, 0.0]
        bounds_y = [round(min(ys), 2), round(max(ys), 2)] if ys else [0.0, 0.0]

        side_tag = "BACK SIDE (MIRRORED FLIP Y)" if board_side == "bottom" else "FRONT SIDE (TOP UP)"
        half_tag = "FRONT BED HALF" if bed_half == "front" else f"BACK BED HALF (Y-{self.y_split:.0f}mm)"
        svg_title = f"SHEET {sheet_num:02d} · OP #{seq:02d} · {step_title.upper()} · {side_tag} · {half_tag}"
        svg_sub = f"Tool: {tool_name} | Depth: {depth_mm:.2f}mm | {stats_label} | File: {gcode_fname}"

        raw_slug = f"{step_code}_{board_side}_{bed_half}_{tool_name}".replace("°", "deg")
        safe_slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", raw_slug).strip("_")
        svg_fname = f"Bed_Sheet{sheet_num:02d}_Op{seq:02d}_{safe_slug}.svg"
        svg_content = self.bed_svg_builder.build_operation_svg(
            sheet_num=sheet_num,
            op_seq=seq,
            title=svg_title,
            subtitle=svg_sub,
            board_side=board_side,
            bed_half=bed_half,
            units=units,
            holders=self.holders_mgr.holders,
            guides=guides,
            sheet_tag=s_tag,
            layer_cache=self.layer_cache,
            active_polylines_bed=active_polylines_bed,
            polyline_color=polyline_color,
            polyline_width=polyline_width,
            polyline_dash=polyline_dash,
            active_drill_dia=active_drill_dia,
            active_drill_pts_bed=active_drill_pts_bed,
            inactive_drill_pts_bed=inactive_drill_pts_bed,
            pin_holes_bed=pin_holes_bed,
            z_probe_switch=self.holders_mgr.z_probe_switch,
        )
        with open(os.path.join(self.out_svg_dir, svg_fname), "w", encoding="utf-8") as f:
            f.write(svg_content)

        op_entry = {
            "op_id": f"S{sheet_num:02d}-OP{seq:02d}",
            "sheet_num": sheet_num,
            "seq": seq,
            "step_code": step_code,
            "step_title": step_title,
            "work_type": work_type,
            "tool_name": tool_name,
            "tool_dia_mm": round(float(tool_dia_mm), 3),
            "board_side": board_side,
            "mirrored": (board_side == "bottom"),
            "bed_half": bed_half,
            "y_shift_mm": 0.0 if bed_half == "front" else round(self.y_split, 2),
            "started_back_same_tool": started_back_same_tool,
            "shift_action": shift_action,
            "gcode_file": gcode_fname,
            "svg_file": svg_fname,
            "depth_mm": round(float(depth_mm), 3),
            "feed_mm_min": round(float(feed_mm_min), 1),
            "spindle_rpm": int(spindle_rpm),
            "stats_label": stats_label,
            "bounds_x": bounds_x,
            "bounds_y": bounds_y,
        }
        ops_list.append(op_entry)
        return op_entry

    def run(self):
        self._clean_old_files()
        self.bed_operations = {}

        # 1. Generate Individual Circuit Gerber SVGs (Step-by-Step Inspector)
        self._generate_circuit_gerber_svgs()

        # 2. Generate Individual Manufactured Unit SVGs (Every Unit in the Batch)
        self._generate_unit_svgs()

        # 3. Process each Sheet (Dual-Sided + Constrained Bed Y-axis-only + Serpentine Drill Optimization)
        for s_idx, units in enumerate(self.sheets):
            sheet_num = s_idx + 1
            print(f"\n>>> Generating Manufacturing Package for Sheet {sheet_num}/{len(self.sheets)} ({len(units)} units)...")

            self._generate_composite_svg(sheet_num, units)
            self._generate_step0_alignment_guides(sheet_num, units)
            self._generate_step1_drilling(sheet_num, units)
            self._generate_step2_vias_guide(sheet_num)
            self._generate_step3_top_isolation(sheet_num, units)
            self._generate_step4_bottom_isolation(sheet_num, units)
            self._generate_step5_solder_mask(sheet_num, units)
            self._generate_step6_silkscreen(sheet_num)
            self._generate_step7_snapable_cutout(sheet_num, units)

        # 4. Validate XML integrity of every generated SVG
        self._validate_all_svgs()

        # 5. Synchronize dynamic step-by-step fabrication checklist (with Limit Switch Z-Zero & tool changes)
        self._sync_fabrication_checklist()

        # 6. Output manufacture_metadata.json
        self._generate_manufacture_metadata_json()

        print(f"\n[SUCCESS] All files generated under '{self.out_gcode_dir}', '{self.out_svg_dir}', and '{self.out_guides_dir}'.")
        return self.generated_files

    def _generate_circuit_gerber_svgs(self):
        """Generates normal & mirrored SVGs across all manufacturing layers (including Front & Back Mask/Silkscreen)."""
        seen = set()
        for sheet in self.sheets:
            for u in sheet:
                p = u.project
                if p.pcb_id in seen:
                    continue
                seen.add(p.pcb_id)
                p_data = self.layer_cache.get_project_layers(p)
                builder = CircuitSvgBuilder(
                    p, p_data, tab_w=u.tab_w, breakaway_cfg=self.breakaway_cfg
                )

                step_svgs = [
                    (f"Circuit_{p.pcb_id}_Composite_Stackup.svg", builder.build_composite_stackup_svg(mirror_x=False)),
                    (f"Circuit_{p.pcb_id}_Composite_Stackup_Mirrored.svg", builder.build_composite_stackup_svg(mirror_x=True)),
                    (f"Circuit_{p.pcb_id}_Step1_Drills.svg", builder.build_step1_drills_svg(mirror_x=False)),
                    (f"Circuit_{p.pcb_id}_Step1_Drills_Mirrored.svg", builder.build_step1_drills_svg(mirror_x=True)),
                    (f"Circuit_{p.pcb_id}_Step3_Top_Copper.svg", builder.build_step3_top_copper_svg(mirror_x=False)),
                    (f"Circuit_{p.pcb_id}_Step3_Top_Copper_Mirrored.svg", builder.build_step3_top_copper_svg(mirror_x=True)),
                    (f"Circuit_{p.pcb_id}_Step4_Bottom_Copper.svg", builder.build_step4_bottom_copper_svg(mirror_x=False)),
                    (f"Circuit_{p.pcb_id}_Step4_Bottom_Copper_Mirrored.svg", builder.build_step4_bottom_copper_svg(mirror_x=True)),
                    (f"Circuit_{p.pcb_id}_Step5_Solder_Mask_Front.svg", builder.build_step5_solder_mask_svg(side="front", mirror_x=False)),
                    (f"Circuit_{p.pcb_id}_Step5_Solder_Mask_Front_Mirrored.svg", builder.build_step5_solder_mask_svg(side="front", mirror_x=True)),
                    (f"Circuit_{p.pcb_id}_Step5_Solder_Mask_Back.svg", builder.build_step5_solder_mask_svg(side="back", mirror_x=False)),
                    (f"Circuit_{p.pcb_id}_Step5_Solder_Mask_Back_Mirrored.svg", builder.build_step5_solder_mask_svg(side="back", mirror_x=True)),
                    (f"Circuit_{p.pcb_id}_Step6_Silkscreen_Front.svg", builder.build_step6_silkscreen_svg(side="front", mirror_x=False)),
                    (f"Circuit_{p.pcb_id}_Step6_Silkscreen_Front_Mirrored.svg", builder.build_step6_silkscreen_svg(side="front", mirror_x=True)),
                    (f"Circuit_{p.pcb_id}_Step6_Silkscreen_Back.svg", builder.build_step6_silkscreen_svg(side="back", mirror_x=False)),
                    (f"Circuit_{p.pcb_id}_Step6_Silkscreen_Back_Mirrored.svg", builder.build_step6_silkscreen_svg(side="back", mirror_x=True)),
                    (f"Circuit_{p.pcb_id}_Step7_Cutout_SnapTab.svg", builder.build_step7_snap_cutout_svg(mirror_x=False)),
                    (f"Circuit_{p.pcb_id}_Step7_Cutout_SnapTab_Mirrored.svg", builder.build_step7_snap_cutout_svg(mirror_x=True)),
                ]
                for fname, svg_content in step_svgs:
                    fpath = os.path.join(self.out_svg_dir, fname)
                    with open(fpath, "w", encoding="utf-8") as f:
                        f.write(svg_content)
                print(f"  [+] Generated {len(step_svgs)} circuit inspection SVGs (Front/Back + Mirror variants) for {p.pcb_id} ({p.project_name})")

    def _generate_unit_svgs(self):
        """Generates Front Side & Back Side (Normal/Mirrored) SVG previews for EVERY unit manufactured in the run."""
        count = 0
        for sheet in self.sheets:
            for u in sheet:
                p_data = self.layer_cache.get_project_layers(u.project)
                variants = [
                    (f"Unit_{u.unit_id}_FrontSide.svg", "front", False),
                    (f"Unit_{u.unit_id}_FrontSide_Mirrored.svg", "front", True),
                    (f"Unit_{u.unit_id}_BackSide.svg", "back", False),
                    (f"Unit_{u.unit_id}_BackSide_Mirrored.svg", "back", True),
                ]
                for fname, side_mode, mir in variants:
                    ub = UnitSvgBuilder(u, p_data, side=side_mode, mirror_x=mir, y_split=self.y_split)
                    fpath = os.path.join(self.out_svg_dir, fname)
                    with open(fpath, "w", encoding="utf-8") as f:
                        f.write(ub.build_svg())
                    count += 1
        print(f"  [+] Generated {count} individual unit SVGs (Front Side / Back Side & Mirrored) for per-unit inspection.")

    def _generate_composite_svg(self, sheet_num, units):
        sm_color = units[0].project.solder_mask_color if units else "green"

        # Render BOTH Front Side View AND Y-Axis-Flipped Mirrored Backside View (with upright left-to-right text!)
        variants = [
            (False, "FRONT", f"Sheet{sheet_num:02d}_Overview_Panel_Blueprint.svg"),
            (True, "BACK", f"Sheet{sheet_num:02d}_BackSide_Mirrored_Blueprint.svg"),
        ]
        for is_mirrored, side_lbl, fname in variants:
            guides = generate_left_guides(
                self.sheet_w, self.sheet_h, margin_w=self.left_margin, y_shift=self.y_split,
                enable_dowel_pins=self.enable_dowel_pin_lock, mirrored_x=is_mirrored,
            )
            s_tag = generate_sheet_tag(
                sheet_num, len(self.sheets), len(units), solder_mask_color=sm_color,
                sheet_w=self.sheet_w, left_margin=self.left_margin,
                tag_position=self.sheet_tag_pos, holders_mgr=self.holders_mgr,
                mirrored_x=is_mirrored, side_label=side_lbl,
            )
            svg = SvgVisualizer(self.sheet_w, self.sheet_h, mirrored_x=is_mirrored)
            svg.draw_board()
            svg.draw_holders(self.holders_mgr.holders, y_split=self.y_split)
            svg.draw_y_split(self.y_split)
            svg.draw_polylines(guides["lines"], color="#38bdf8", width=1.0, upright_mirrored=is_mirrored)
            svg.draw_polylines(guides["texts"], color="#7dd3fc", width=0.8, upright_mirrored=is_mirrored)
            svg.draw_circles(guides["pin_holes"], color="#fb7185", radius=1.5, upright_mirrored=is_mirrored)
            svg.draw_polylines(s_tag["border"], color="#d97706", width=1.2, upright_mirrored=is_mirrored)
            svg.draw_polylines(s_tag["text"], color="#fbbf24", width=1.0, upright_mirrored=is_mirrored)
            svg.draw_polylines(s_tag["arrow"], color="#38bdf8", width=1.5, upright_mirrored=is_mirrored)
            for u in units:
                svg.draw_unit(u, self.layer_cache)
            svg.draw_orientation_banner(sheet_num)
            fpath = os.path.join(self.out_svg_dir, fname)
            with open(fpath, "w", encoding="utf-8") as f:
                f.write(svg.render())
            print(f"  [+] Saved sheet blueprint SVG: {fname}")

    def _generate_step0_alignment_guides(self, sheet_num, units):
        guides = generate_left_guides(
            self.sheet_w, self.sheet_h, margin_w=self.left_margin, y_shift=self.y_split,
            enable_dowel_pins=self.enable_dowel_pin_lock, mirrored_x=False,
        )
        sm_color = units[0].project.solder_mask_color if units else "green"
        s_tag = generate_sheet_tag(
            sheet_num, len(self.sheets), len(units), solder_mask_color=sm_color,
            sheet_w=self.sheet_w, left_margin=self.left_margin,
            tag_position=self.sheet_tag_pos, holders_mgr=self.holders_mgr,
            mirrored_x=False, side_label="FRONT",
        )
        pin_tool_suffix = " + 3.0mm Dowel Pin" if self.enable_dowel_pin_lock else ""
        pin_tool_note = (
            "Tool: 30 deg / 0.1mm V-Bit for Sheet Data/Ref Engraving + 3.0mm drill for optional dowel pin lock"
            if self.enable_dowel_pin_lock
            else "Tool: 30 deg / 0.1mm V-Bit ONLY (Dowel Pin Lock DISABLED in manufacture_config.json)"
        )

        if self.sheet_h > self.bed_travel_y:
            front_strokes = guides["front_lines"] + guides["front_texts"] + s_tag["border"] + s_tag["text"] + s_tag["arrow"]
            b0a = GcodeBuilder(f"Sheet {sheet_num} - Step 0A Front Sheet Data, Reference Spots & Tag", feed=250.0, cut_z=-0.04)
            b0a.header([
                "Step 0A: FRONT HALF Sheet Data, Margin Guides, Reference Spots & Front-Corner Sheet Tag",
                f"Bed Travel Y: {self.bed_travel_y:.1f}mm (< Sheet {self.sheet_h:.1f}mm) -> All coordinates <= {self.y_split:.1f}mm",
                pin_tool_note,
            ])
            b0a.spindle_start()
            for stroke in front_strokes:
                b0a.engrave_polyline(stroke)
            if self.enable_dowel_pin_lock:
                for (px, py) in guides["front_pin_holes"]:
                    b0a.peck_drill(px, py, depth=-1.8, step=0.4)
            b0a.footer()

            fname_0a = f"Sheet{sheet_num:02d}_Step0A_Front_Alignment_Guides.gcode"
            fpath_0a = os.path.join(self.out_gcode_dir, fname_0a)
            with open(fpath_0a, "w", encoding="utf-8") as f:
                f.write(b0a.get_text())
            self.generated_files.append(fpath_0a)

            self._record_bed_op(
                sheet_num, units, "Step0A", "Step 0A: Sheet Data & Ref Marks",
                "Alignment Guides", f"30° V-Bit{pin_tool_suffix}", 0.1,
                "top", "front", fname_0a, -0.04, 250.0, 10000,
                "Mount A4 sheet Top-Side Up at G54 (0,0) FRONT HALF",
                f"{len(front_strokes)} strokes, {len(guides['front_pin_holes'])} pins",
                active_polylines_bed=front_strokes, polyline_color="#38bdf8", polyline_width=1.2,
                pin_holes_bed=guides["front_pin_holes"] if self.enable_dowel_pin_lock else [],
            )

            back_strokes = [
                [(p[0], p[1] - self.y_split) for p in stroke]
                for stroke in (guides["back_lines"] + guides["back_texts"])
            ]
            back_pins_bed = [(px, py - self.y_split) for (px, py) in guides["back_pin_holes"]] if self.enable_dowel_pin_lock else []

            b0b = GcodeBuilder(f"Sheet {sheet_num} - Step 0B Back Alignment Guides & Reference Marks", feed=250.0, cut_z=-0.04)
            b0b.header([
                "Step 0B: BACK HALF Margin Guides & Reference Marks",
                f"*** ACTION: Slide sheet forward by {self.y_split:.1f} mm along Y-axis ONLY! ***",
                pin_tool_note,
            ])
            b0b.spindle_start()
            for stroke in back_strokes:
                b0b.engrave_polyline(stroke)
            for (px, py) in back_pins_bed:
                b0b.peck_drill(px, py, depth=-1.8, step=0.4)
            b0b.footer()

            fname_0b = f"Sheet{sheet_num:02d}_Step0B_Back_Alignment_Guides.gcode"
            fpath_0b = os.path.join(self.out_gcode_dir, fname_0b)
            with open(fpath_0b, "w", encoding="utf-8") as f:
                f.write(b0b.get_text())
            self.generated_files.append(fpath_0b)

            self._record_bed_op(
                sheet_num, units, "Step0B", "Step 0B: Back Alignment Guides",
                "Alignment Guides", f"30° V-Bit{pin_tool_suffix}", 0.1,
                "top", "back", fname_0b, -0.04, 250.0, 10000,
                f"Slide sheet -{self.y_split:.1f}mm along Y-axis ONLY -> BACK HALF",
                f"{len(back_strokes)} strokes, {len(back_pins_bed)} pins",
                active_polylines_bed=back_strokes, polyline_color="#38bdf8", polyline_width=1.2,
                pin_holes_bed=back_pins_bed,
            )
            print(f"  [+] Step 0A & 0B: {fname_0a}, {fname_0b} (Dowel Pin Lock: {'ENABLED' if self.enable_dowel_pin_lock else 'DISABLED'})")
        else:
            all_strokes = guides["lines"] + guides["texts"] + s_tag["border"] + s_tag["text"] + s_tag["arrow"]
            builder = GcodeBuilder(f"Sheet {sheet_num} - Step 0 Sheet Data, Guides & Tag", feed=250.0, cut_z=-0.04)
            builder.header(["Step 0: Margin Guides, Reference Spots & Front-Corner Sheet Tag", pin_tool_note])
            builder.spindle_start()
            for stroke in all_strokes:
                builder.engrave_polyline(stroke)
            if self.enable_dowel_pin_lock:
                for (px, py) in guides["pin_holes"]:
                    builder.peck_drill(px, py, depth=-1.8, step=0.4)
            builder.footer()
            fname = f"Sheet{sheet_num:02d}_Step0_Left_Alignment_Guides.gcode"
            fpath = os.path.join(self.out_gcode_dir, fname)
            with open(fpath, "w", encoding="utf-8") as f:
                f.write(builder.get_text())
            self.generated_files.append(fpath)

            self._record_bed_op(
                sheet_num, units, "Step0", "Step 0: Sheet Data & Guides",
                "Alignment Guides", f"30° V-Bit{pin_tool_suffix}", 0.1,
                "top", "front", fname, -0.04, 250.0, 10000,
                "Full sheet fits bed travel without Y-shift",
                f"{len(all_strokes)} strokes, {len(guides['pin_holes'])} pins",
                active_polylines_bed=all_strokes, polyline_color="#38bdf8", polyline_width=1.2,
                pin_holes_bed=guides["pin_holes"] if self.enable_dowel_pin_lock else [],
            )
            print(f"  [+] Step 0: {fname} (Dowel Pin Lock: {'ENABLED' if self.enable_dowel_pin_lock else 'DISABLED'})")

    def _generate_step1_drilling(self, sheet_num, units):
        """
        Generate Step 1 multi-diameter drilling G-code and CNC Bed views.
        Scheduling Rule:
        - Start Drilling at FRONT HALF before BACK HALF for the first drill size.
        - For subsequent different drill sizes (same drilling work with same tool type),
          if the sheet is already resting at BACK HALF from the previous drill size,
          START FROM THE BACK HALF (avoiding redundant unclamp/Y-shift back to Front!).
        """
        tool_drills_front: Dict[float, List[Tuple[float, float]]] = {}
        tool_drills_back: Dict[float, List[Tuple[float, float]]] = {}
        mb_dia = self.breakaway_cfg.mouse_bite_drill_dia_mm

        for u in units:
            u_data = self.layer_cache.get_project_layers(u.project)
            for dia, pts in u_data.get("drills", {}).items():
                for (hx, hy) in pts:
                    sx, sy = u.to_sheet_coords(hx, hy)
                    if sy <= self.y_split:
                        tool_drills_front.setdefault(dia, []).append((sx, sy))
                    else:
                        tool_drills_back.setdefault(dia, []).append((sx, sy - self.y_split))

            for (bx, by) in u.mouse_bites:
                if by <= self.y_split:
                    tool_drills_front.setdefault(mb_dia, []).append((bx, by))
                else:
                    tool_drills_back.setdefault(mb_dia, []).append((bx, by - self.y_split))

        all_tools = sorted(list(set(list(tool_drills_front.keys()) + list(tool_drills_back.keys()))))
        all_front_flat = [pt for d_pts in tool_drills_front.values() for pt in d_pts]
        all_back_flat = [pt for d_pts in tool_drills_back.values() for pt in d_pts]

        current_half = "front"  # First drill diameter always starts Front -> Back
        drill_seq = 0
        back_starts_count = 0

        for idx_tool, dia in enumerate(all_tools):
            f_pts = sort_points_nearest_neighbor(tool_drills_front.get(dia, []))
            b_pts = sort_points_nearest_neighbor(tool_drills_back.get(dia, []))

            # If already clamped on Back Half from previous drill tool & new drill size has Back holes:
            # start from Back Half (same drilling work, different size bit!)
            if idx_tool > 0 and current_half == "back" and b_pts:
                halves_order = ["back", "front"]
                started_from_back_for_tool = True
                back_starts_count += 1
            else:
                halves_order = ["front", "back"]
                started_from_back_for_tool = False

            for h_pos_idx, half in enumerate(halves_order):
                pts = f_pts if half == "front" else b_pts
                if not pts:
                    continue

                drill_seq += 1
                is_serpentine_back_first = (h_pos_idx == 0 and started_from_back_for_tool and half == "back")
                half_cap = "Front" if half == "front" else "Back"

                if idx_tool == 0 and h_pos_idx == 0:
                    shift_action = "Set sheet Top-Side Up at FRONT HALF (Y=0 datum)"
                elif half == current_half:
                    shift_action = f"Stay at {half.upper()} HALF & change drill to {dia:.2f}mm (Serpentine: same work, no Y-shift needed!)"
                elif half == "back":
                    shift_action = f"Slide sheet -{self.y_split:.1f}mm along Y-axis ONLY -> BACK HALF"
                else:
                    shift_action = f"Slide sheet +{self.y_split:.1f}mm back along Y-axis ONLY -> FRONT HALF"

                b_drl = GcodeBuilder(
                    f"Sheet {sheet_num} - Step 1.{drill_seq:02d} {half_cap} Top Drills {dia:.2f}mm",
                    feed=150.0, plunge=80.0, spindle_rpm=10000
                )
                b_drl.header([
                    f"Step 1 (Seq #{drill_seq:02d}): {half.upper()} HALF Top Drilling - Tool: {dia:.2f}mm Carbide Drill",
                    f"Breakaway Mode: {self.breakaway_cfg.label}",
                    f"Y-Shift Action: {shift_action}",
                    f"Hole Count: {len(pts)} | Cycle: Peck drilling (0.4mm depth increments)"
                ])
                b_drl.spindle_start()
                for (x, y) in pts:
                    b_drl.peck_drill(x, y, depth=-1.8, step=0.4)
                b_drl.footer()

                fname = f"Sheet{sheet_num:02d}_Step1_{drill_seq:02d}_{half_cap}_Top_Drills_{dia:.2f}mm.gcode"
                fpath = os.path.join(self.out_gcode_dir, fname)
                with open(fpath, "w", encoding="utf-8") as f:
                    f.write(b_drl.get_text())
                self.generated_files.append(fpath)

                bg_pts = all_front_flat if half == "front" else all_back_flat
                self._record_bed_op(
                    sheet_num, units, f"Step1_{drill_seq:02d}",
                    f"Step 1 Drilling ({dia:.2f}mm)",
                    "CNC Drilling", f"{dia:.2f}mm Carbide Drill", dia,
                    "top", half, fname, -1.80, 150.0, 10000,
                    shift_action, f"{len(pts)} holes @ {dia:.2f}mm",
                    started_back_same_tool=is_serpentine_back_first,
                    active_drill_dia=dia,
                    active_drill_pts_bed=pts,
                    inactive_drill_pts_bed=bg_pts,
                )
                current_half = half

        print(f"  [+] Step 1: Generated {drill_seq} sequential drill files across {len(all_tools)} diameters "
              f"({back_starts_count} serpentine Back-first tool changes).")

    def _generate_step2_vias_guide(self, sheet_num):
        content = (
            f"; === Sheet {sheet_num:02d} - Step 2: Chemical Through-Hole Via Metallization ===\n"
            "; This step is performed off-CNC via wet chemical process after Step 1 drilling.\n"
            "; Process sequence:\n"
            ";   1. Ultrasonic clean & acid desmear\n"
            ";   2. Conductive carbon-black / palladium catalyst activation\n"
            ";   3. Micro-etch bare copper surface\n"
            ";   4. Acid copper electroplating bath (CuSO4 + H2SO4 @ 1.5A/dm² for 45 min = 25µm Cu barrel)\n"
            ";   5. DI water rinse and oven dry at 80°C\n"
            "G21 ; Metric units\n"
            "G90 ; Absolute positioning\n"
            "(MSG, Off-CNC Chemical Vias complete. Ready for Step 3 Top Isolation)\n"
            "M0 (Pause: Verify via plating before proceeding to copper isolation)\n"
            "M2 ; End of program\n"
        )
        fname = f"Sheet{sheet_num:02d}_Step2_Chemical_Vias_Guide.gcode"
        fpath = os.path.join(self.out_gcode_dir, fname)
        with open(fpath, "w", encoding="utf-8") as f:
            f.write(content)
        self.generated_files.append(fpath)
        print("  [+] Step 2: Via Chemical Process Guide written as G-code.")

    def _route_polylines_by_half(self, polylines: List[List[Tuple[float, float]]],
                                 front_list: List[List[Tuple[float, float]]],
                                 back_list: List[List[Tuple[float, float]]]):
        for poly in polylines:
            f_segs, b_segs = _split_polyline_by_y(poly, self.y_split)
            front_list.extend(f_segs)
            back_list.extend(b_segs)

    def _emit_polyline_job(
        self,
        sheet_num: int,
        units: List[PlacedUnit],
        polylines: List[List[Tuple[float, float]]],
        step_code: str,
        step_title: str,
        work_type: str,
        tool_name: str,
        tool_dia_mm: float,
        board_side: str,
        bed_half: str,
        fname: str,
        gcode_title: str,
        header_lines: List[str],
        shift_action: str,
        cut_z: float = -0.04,
        feed: float = 300.0,
        spindle_rpm: int = 10000,
        polyline_color: str = "#fbbf24",
        polyline_width: float = 1.0,
        polyline_dash: str = "",
        m0_pause: bool = False,
    ):
        """Write one G-code polyline job and record its corresponding CNC Bed SVG preview."""
        if not polylines:
            return
        gb = GcodeBuilder(gcode_title, feed=feed, cut_z=cut_z, spindle_rpm=spindle_rpm)
        gb.header(header_lines)
        if m0_pause:
            gb.lines.append("M0 ; Operator pause for Z-probe verification")
        gb.spindle_start()
        for poly in polylines:
            gb.engrave_polyline(poly)
        gb.footer()
        fpath = os.path.join(self.out_gcode_dir, fname)
        with open(fpath, "w", encoding="utf-8") as f:
            f.write(gb.get_text())
        self.generated_files.append(fpath)

        self._record_bed_op(
            sheet_num, units, step_code, step_title, work_type, tool_name, tool_dia_mm,
            board_side, bed_half, fname, cut_z, feed, spindle_rpm, shift_action,
            f"{len(polylines)} toolpaths",
            active_polylines_bed=polylines, polyline_color=polyline_color,
            polyline_width=polyline_width, polyline_dash=polyline_dash,
        )

    def _generate_step3_top_isolation(self, sheet_num, units):
        front_polylines: List[List[Tuple[float, float]]] = []
        back_polylines: List[List[Tuple[float, float]]] = []

        for u in units:
            self._route_polylines_by_half(u.arrow_lines + u.id_text_lines, front_polylines, back_polylines)
            iso_paths = self.layer_cache.get_toolpaths(u.project, "copper_top", tool_dia=0.2, passes=1)
            if iso_paths:
                shifted = [[u.to_sheet_coords(p[0], p[1]) for p in poly] for poly in iso_paths]
                self._route_polylines_by_half(shifted, front_polylines, back_polylines)

        self._emit_polyline_job(
            sheet_num, units, front_polylines, "Step3A", "Step 3A: Top Copper Isolation",
            "Top Isolation", "30° / 0.2mm V-Bit", 0.2, "top", "front",
            f"Sheet{sheet_num:02d}_Step3A_Front_Top_Copper_Isolation.gcode",
            f"Sheet {sheet_num} - Step 3A Front Top Copper Isolation",
            ["Step 3A: FRONT HALF Top Copper Layer Isolation Routing", "Tool: 30 deg / 0.1mm V-Bit"],
            "Mount sheet Top-Side Up at G54 (0,0) FRONT HALF",
            cut_z=-0.04, feed=300.0, polyline_color="#fbbf24", polyline_width=0.9,
        )
        self._emit_polyline_job(
            sheet_num, units, back_polylines, "Step3B", "Step 3B: Top Copper Isolation",
            "Top Isolation", "30° / 0.2mm V-Bit", 0.2, "top", "back",
            f"Sheet{sheet_num:02d}_Step3B_Back_Top_Copper_Isolation.gcode",
            f"Sheet {sheet_num} - Step 3B Back Top Copper Isolation",
            ["Step 3B: BACK HALF Top Copper Layer Isolation Routing",
             f"*** ACTION: Slide sheet forward by {self.y_split:.1f} mm along Y-axis ONLY! ***"],
            f"Slide sheet -{self.y_split:.1f}mm along Y-axis ONLY -> BACK HALF",
            cut_z=-0.04, feed=300.0, polyline_color="#fbbf24", polyline_width=0.9,
        )
        print("  [+] Step 3: Top isolation routing generated.")

    def _generate_step4_bottom_isolation(self, sheet_num, units):
        front_polylines: List[List[Tuple[float, float]]] = []
        back_polylines: List[List[Tuple[float, float]]] = []

        for u in units:
            if u.engrave_both_sides:
                flipped_strokes = [[(self.sheet_w - p[0], p[1]) for p in s] for s in (u.arrow_lines + u.id_text_lines)]
                self._route_polylines_by_half(flipped_strokes, front_polylines, back_polylines)

            iso_paths = self.layer_cache.get_toolpaths(u.project, "copper_bottom", tool_dia=0.2, passes=1)
            if iso_paths:
                shifted = [
                    [(self.sheet_w - pt[0], pt[1]) for pt in [u.to_sheet_coords(p[0], p[1]) for p in poly]]
                    for poly in iso_paths
                ]
                self._route_polylines_by_half(shifted, front_polylines, back_polylines)

        self._emit_polyline_job(
            sheet_num, units, front_polylines, "Step4A", "Step 4A: Bottom Copper Isolation",
            "Bottom Isolation", "30° / 0.2mm V-Bit", 0.2, "bottom", "front",
            f"Sheet{sheet_num:02d}_Step4A_Front_Bottom_Copper_Isolation.gcode",
            f"Sheet {sheet_num} - Step 4A Front Bottom Copper Isolation",
            ["Step 4A: FRONT HALF Bottom Copper Isolation", "Board Orientation: FLIPPED horizontally along Y-axis"],
            "FLIP board horizontally along Y-axis (Left <-> Right) -> FRONT HALF",
            cut_z=-0.04, feed=300.0, polyline_color="#fb923c", polyline_width=0.9,
        )
        self._emit_polyline_job(
            sheet_num, units, back_polylines, "Step4B", "Step 4B: Bottom Copper Isolation",
            "Bottom Isolation", "30° / 0.2mm V-Bit", 0.2, "bottom", "back",
            f"Sheet{sheet_num:02d}_Step4B_Back_Bottom_Copper_Isolation.gcode",
            f"Sheet {sheet_num} - Step 4B Back Bottom Copper Isolation",
            ["Step 4B: BACK HALF Bottom Copper Isolation",
             f"*** ACTION: Slide flipped sheet forward by {self.y_split:.1f} mm along Y-axis ONLY! ***"],
            f"Slide flipped sheet -{self.y_split:.1f}mm along Y-axis ONLY -> BACK HALF",
            cut_z=-0.04, feed=300.0, polyline_color="#fb923c", polyline_width=0.9,
        )
        print("  [+] Step 4: Bottom isolation routing generated.")

    def _generate_step5_solder_mask(self, sheet_num, units):
        front_top_pads: List[List[Tuple[float, float]]] = []
        back_top_pads: List[List[Tuple[float, float]]] = []
        front_bot_pads: List[List[Tuple[float, float]]] = []
        back_bot_pads: List[List[Tuple[float, float]]] = []

        for u in units:
            t_paths = self.layer_cache.get_toolpaths(u.project, "mask_top", tool_dia=0.2, passes=1)
            if t_paths:
                shifted = [[u.to_sheet_coords(p[0], p[1]) for p in poly] for poly in t_paths]
                self._route_polylines_by_half(shifted, front_top_pads, back_top_pads)

            b_paths = self.layer_cache.get_toolpaths(u.project, "mask_bottom", tool_dia=0.2, passes=1)
            if b_paths:
                shifted = [
                    [(self.sheet_w - pt[0], pt[1]) for pt in [u.to_sheet_coords(p[0], p[1]) for p in poly]]
                    for poly in b_paths
                ]
                self._route_polylines_by_half(shifted, front_bot_pads, back_bot_pads)

        mask_jobs = [
            (front_top_pads, "Step5A", "Step 5A: Top Mask Clear", "top", "front",
             f"Sheet{sheet_num:02d}_Step5A_Front_Top_Mask_Clear.gcode",
             "Mount sheet Top-Side Up at FRONT HALF & re-zero Z on cured mask (M0)",
             ["Step 5A: FRONT HALF Top Solder Mask Pad Clearing", "M0 (PAUSE: Verify Z0 touch-off on cured solder mask film)"]),
            (back_top_pads, "Step5B", "Step 5B: Top Mask Clear", "top", "back",
             f"Sheet{sheet_num:02d}_Step5B_Back_Top_Mask_Clear.gcode",
             f"Slide sheet -{self.y_split:.1f}mm along Y-axis ONLY -> BACK HALF (Top Mask)",
             ["Step 5B: BACK HALF Top Solder Mask Pad Clearing",
              f"*** ACTION: Slide sheet forward by {self.y_split:.1f} mm along Y-axis ONLY! ***",
              "M0 (PAUSE: Verify Z0 touch-off on cured solder mask film)"]),
            (front_bot_pads, "Step5C", "Step 5C: Bottom Mask Clear", "bottom", "front",
             f"Sheet{sheet_num:02d}_Step5C_Front_Bottom_Mask_Clear.gcode",
             "FLIP board horizontally along Y-axis -> FRONT HALF & re-zero Z on bottom mask (M0)",
             ["Step 5C: FRONT HALF Bottom Solder Mask Pad Clearing",
              "Board Orientation: FLIPPED horizontally along Y-axis.",
              "M0 (PAUSE: Verify Z0 touch-off on cured bottom solder mask)"]),
            (back_bot_pads, "Step5D", "Step 5D: Bottom Mask Clear", "bottom", "back",
             f"Sheet{sheet_num:02d}_Step5D_Back_Bottom_Mask_Clear.gcode",
             f"Slide flipped sheet -{self.y_split:.1f}mm along Y-axis ONLY -> BACK HALF (Bottom Mask)",
             ["Step 5D: BACK HALF Bottom Solder Mask Pad Clearing",
              f"*** ACTION: Slide flipped sheet forward by {self.y_split:.1f} mm along Y-axis ONLY! ***",
              "M0 (PAUSE: Verify Z0 touch-off on cured bottom solder mask)"]),
        ]
        for pads, scode, stitle, bside, bhalf, fname, sact, hdr in mask_jobs:
            self._emit_polyline_job(
                sheet_num, units, pads, scode, stitle, "Solder Mask Clear",
                "0.4mm Flat Endmill / V-Bit", 0.4, bside, bhalf, fname,
                f"Sheet {sheet_num} - {stitle}", hdr, sact,
                cut_z=-0.06, feed=300.0, polyline_color="#34d399", polyline_width=1.0, m0_pause=True,
            )
        print("  [+] Step 5: Solder mask pad clearing generated (Top & Bottom, Front & Back).")

    def _generate_step6_silkscreen(self, sheet_num):
        content = (
            f"; === Sheet {sheet_num:02d} - Step 6: Silkscreen Printing Roadmap ===\n"
            "; # TODO: Future flatbed drop-on-demand (DoD) inkjet printer integration.\n"
            "; Silkscreen component legends (R1, C1, U1, pin-1 markers) do not require CNC engraving.\n"
            "; Will be deposited directly using UV-curable white pigment ink with LED-395nm pinning.\n"
            "G21 ; Metric units\n"
            "G90 ; Absolute positioning\n"
            "(MSG, Off-CNC Silkscreen process completed. Ready for Step 7 Perimeter Cutouts)\n"
            "M0 (Pause: Silkscreen legend inspection)\n"
            "M2 ; End of program\n"
        )
        fname = f"Sheet{sheet_num:02d}_Step6_Silkscreen_TODO.gcode"
        fpath = os.path.join(self.out_gcode_dir, fname)
        with open(fpath, "w", encoding="utf-8") as f:
            f.write(content)
        self.generated_files.append(fpath)
        print("  [+] Step 6: Silkscreen TODO written as G-code.")

    def _generate_step7_snapable_cutout(self, sheet_num, units):
        """
        Step 7: Dual-Sided Snapable Perimeter & Breakaway Cutouts.
        Executes ENTIRE Top Side (Endmill Front -> Back, then V-Bit Score Front -> Back)
        before flipping once horizontally along Y-axis for Bottom Side (Endmill & V-Bit Score).
        """
        depth_per_side = self.breakaway_cfg.milling_cut_depth_per_side_mm
        front_units = [u for u in units if u.y + u.h <= self.y_split + 1e-6]
        back_units = [u for u in units if u.y >= self.y_split - 1e-6]

        def collect_half_ops(half_units: List[PlacedUnit], is_back: bool) -> Tuple[List[Tuple[List[Tuple[float, float]], float]], List[Tuple[List[Tuple[float, float]], float]]]:
            y_off = self.y_split if is_back else 0.0
            mill_ops: List[Tuple[List[Tuple[float, float]], float]] = []
            vbit_ops: List[Tuple[List[Tuple[float, float]], float]] = []
            if not half_units:
                return mill_ops, vbit_ops

            perim_segs = _deduplicate_and_tab_perimeter_edges(half_units, y_off, bridge_tab_len=3.0)
            if self.breakaway_cfg.mode == "v_bit_only":
                for seg in perim_segs:
                    vbit_ops.append((seg, depth_per_side))
            else:
                for seg in perim_segs:
                    mill_ops.append((seg, depth_per_side))

            for u in half_units:
                for sl in u.milling_slot_segs:
                    pts = [(p[0], p[1] - y_off) for p in sl["pts"]]
                    mill_ops.append((pts, sl["depth"]))
                for vs in u.vbit_score_segs:
                    pts = [(p[0], p[1] - y_off) for p in vs["pts"]]
                    vbit_ops.append((pts, vs["depth"]))
            return mill_ops, vbit_ops

        front_mill, front_vbit = collect_half_ops(front_units, is_back=False)
        back_mill, back_vbit = collect_half_ops(back_units, is_back=True)

        def mirror_ops(ops_list):
            return [([(self.sheet_w - p[0], p[1]) for p in seg], z_d) for seg, z_d in ops_list]

        def emit_cutout_job(ops_list, scode, stitle, bside, bhalf, tool_name, tool_dia,
                            fname, header_lines, shift_action, feed, pcol, pdash=""):
            if not ops_list:
                return
            gb = GcodeBuilder(f"Sheet {sheet_num} - {stitle}", feed=feed, cut_z=depth_per_side)
            gb.header(header_lines)
            gb.spindle_start()
            for seg, z_d in ops_list:
                gb.engrave_polyline(seg, z_depth=z_d)
            gb.footer()
            fpath = os.path.join(self.out_gcode_dir, fname)
            with open(fpath, "w", encoding="utf-8") as f:
                f.write(gb.get_text())
            self.generated_files.append(fpath)

            segs_only = [seg for seg, _ in ops_list]
            self._record_bed_op(
                sheet_num, units, scode, stitle, "Cutout & Breakaway",
                tool_name, tool_dia, bside, bhalf, fname, depth_per_side, feed, 10000,
                shift_action, f"{len(segs_only)} cut segments",
                active_polylines_bed=segs_only, polyline_color=pcol, polyline_width=2.0, polyline_dash=pdash,
            )

        # TOP SIDE (Endmill Front->Back, then V-Bit Score Front->Back)
        emit_cutout_job(
            front_mill, "Step7A", "Step 7A: Top Endmill Cutout", "top", "front",
            "1.0mm Corn Endmill", 1.0, f"Sheet{sheet_num:02d}_Step7A_Front_Top_Snap_Cutout.gcode",
            [f"Step 7A: FRONT HALF Top Milling Cutout ({self.breakaway_cfg.label})",
             "Smooth-Edge Offset Active | Single-Pass Shared Boundaries | 2x 2.0mm Retention Tabs"],
            "Mount Top-Side Up at FRONT HALF (Y=0 datum) with 1.0mm Endmill", 200.0, "#ef4444"
        )
        emit_cutout_job(
            back_mill, "Step7B", "Step 7B: Top Endmill Cutout", "top", "back",
            "1.0mm Corn Endmill", 1.0, f"Sheet{sheet_num:02d}_Step7B_Back_Top_Snap_Cutout.gcode",
            [f"Step 7B: BACK HALF Top Milling Cutout ({self.breakaway_cfg.label})",
             f"*** ACTION: Shift sheet forward by {self.y_split:.1f} mm along Y-axis ONLY! ***"],
            f"Slide sheet -{self.y_split:.1f}mm along Y-axis ONLY -> BACK HALF", 200.0, "#ef4444"
        )
        emit_cutout_job(
            front_vbit, "Step7C", "Step 7C: Top V-Bit Breakaway Score", "top", "front",
            "30° V-Bit (Score)", 0.2, f"Sheet{sheet_num:02d}_Step7C_Front_Top_VBit_Score.gcode",
            [f"Step 7C: FRONT HALF Top V-Bit Scoring ({self.breakaway_cfg.label})",
             "Tool: 30 deg V-Bit | Smooth chamfered breakaway along exact PCB-tag seam"],
            "Mount 30° V-Bit & return to FRONT HALF (Y=0 datum)", 220.0, "#38bdf8", pdash="5,2"
        )
        emit_cutout_job(
            back_vbit, "Step7D", "Step 7D: Top V-Bit Breakaway Score", "top", "back",
            "30° V-Bit (Score)", 0.2, f"Sheet{sheet_num:02d}_Step7D_Back_Top_VBit_Score.gcode",
            [f"Step 7D: BACK HALF Top V-Bit Scoring ({self.breakaway_cfg.label})",
             f"*** ACTION: Shift sheet forward by {self.y_split:.1f} mm along Y-axis ONLY! ***"],
            f"Slide sheet -{self.y_split:.1f}mm along Y-axis ONLY -> BACK HALF", 220.0, "#38bdf8", pdash="5,2"
        )

        # FLIP BOARD HORIZONTALLY ALONG Y-AXIS -> BOTTOM SIDE (MIRRORED X)
        emit_cutout_job(
            mirror_ops(front_mill), "Step7E", "Step 7E: Bottom Endmill Cutout (Mirrored)", "bottom", "front",
            "1.0mm Corn Endmill", 1.0, f"Sheet{sheet_num:02d}_Step7E_Front_Bottom_Snap_Cutout.gcode",
            [f"Step 7E: FRONT HALF Bottom Milling Cutout ({self.breakaway_cfg.label})",
             "Board Orientation: FLIPPED horizontally along Y-axis."],
            "FLIP board horizontally along Y-axis -> FRONT HALF (Mirrored X) with 1.0mm Endmill", 200.0, "#ef4444"
        )
        emit_cutout_job(
            mirror_ops(back_mill), "Step7F", "Step 7F: Bottom Endmill Cutout (Mirrored)", "bottom", "back",
            "1.0mm Corn Endmill", 1.0, f"Sheet{sheet_num:02d}_Step7F_Back_Bottom_Snap_Cutout.gcode",
            [f"Step 7F: BACK HALF Bottom Milling Cutout ({self.breakaway_cfg.label})",
             f"*** ACTION: Shift flipped sheet forward by {self.y_split:.1f} mm along Y-axis ONLY! ***"],
            f"Slide flipped sheet -{self.y_split:.1f}mm along Y-axis ONLY -> BACK HALF (Mirrored X)", 200.0, "#ef4444"
        )
        emit_cutout_job(
            mirror_ops(front_vbit), "Step7G", "Step 7G: Bottom V-Bit Score (Mirrored)", "bottom", "front",
            "30° V-Bit (Score)", 0.2, f"Sheet{sheet_num:02d}_Step7G_Front_Bottom_VBit_Score.gcode",
            [f"Step 7G: FRONT HALF Bottom V-Bit Scoring ({self.breakaway_cfg.label})",
             "Board Orientation: FLIPPED horizontally along Y-axis."],
            "Mount 30° V-Bit at FRONT HALF (Bottom Side Mirrored X)", 220.0, "#38bdf8", pdash="5,2"
        )
        emit_cutout_job(
            mirror_ops(back_vbit), "Step7H", "Step 7H: Bottom V-Bit Score (Mirrored)", "bottom", "back",
            "30° V-Bit (Score)", 0.2, f"Sheet{sheet_num:02d}_Step7H_Back_Bottom_VBit_Score.gcode",
            [f"Step 7H: BACK HALF Bottom V-Bit Scoring ({self.breakaway_cfg.label})",
             f"*** ACTION: Shift flipped sheet forward by {self.y_split:.1f} mm along Y-axis ONLY! ***"],
            f"Slide flipped sheet -{self.y_split:.1f}mm along Y-axis ONLY -> BACK HALF (Mirrored X)", 220.0, "#38bdf8", pdash="5,2"
        )
        print(f"  [+] Step 7: Dual-sided cutout files generated ({self.breakaway_cfg.label}).")

    def _generate_manufacture_metadata_json(self):
        """Generates comprehensive manufacture_metadata.json detailing all boards, sheets, and CNC bed operations."""
        sheets_data = []
        total_units = sum(len(s) for s in self.sheets)

        for s_idx, u_list in enumerate(self.sheets):
            s_num = s_idx + 1
            sm_color = u_list[0].project.solder_mask_color if u_list else "green"
            s_tag = generate_sheet_tag(
                s_num, len(self.sheets), len(u_list), solder_mask_color=sm_color,
                sheet_w=self.sheet_w, left_margin=self.left_margin,
                tag_position=self.sheet_tag_pos, holders_mgr=self.holders_mgr
            )
            sheet_tag_lbl = f"S{s_num:02d}/{len(self.sheets):02d} | U:{len(u_list)} | {sm_color.upper()} | FRONT"

            u_data = []
            for u in u_list:
                p_layers = self.layer_cache.get_project_layers(u.project)
                drl_count = sum(len(pts) for pts in p_layers.get("drills", {}).values())
                half = "front" if (u.y + u.h <= self.y_split + 1e-6) else "back"
                u_data.append({
                    "unit_id": u.unit_id,
                    "project_name": u.project.project_name,
                    "pcb_id": u.project.pcb_id,
                    "orientation_deg": u.rotation_deg,
                    "bounding_box_on_sheet": {
                        "x_min": round(u.x, 2), "y_min": round(u.y, 2),
                        "x_max": round(u.x + u.w, 2), "y_max": round(u.y + u.h, 2)
                    },
                    "pcb_dimensions_mm": {"width": u.pcb_w, "height": u.pcb_h},
                    "tag_strip": {
                        "width_mm": u.tab_w,
                        "attached_edge": "shortest",
                        "attached_side_on_sheet": u.attach_side,
                        "shortest_edge_mm": round(u.shortest_edge_len, 2),
                        "tag_content": "pcb_id_and_triangle_arrow_only",
                        "has_triangle_arrow": u.engrave_arrow,
                        "breakaway_mode": self.breakaway_cfg.mode,
                        "retention_tabs": [
                            {"start_mm": round(t0, 2), "end_mm": round(t1, 2), "width_mm": round(t1 - t0, 2)}
                            for (t0, t1) in u.seam_geom.get("retention_tabs", [])
                        ],
                        "mouse_bites_count": len(u.mouse_bites),
                    },
                    "half": half,
                    "solder_mask_color": u.project.solder_mask_color,
                    "drill_holes_count": drl_count
                })

            sheets_data.append({
                "sheet_number": s_num,
                "sheet_tag": sheet_tag_lbl,
                "sheet_tag_corner": {
                    "position": self.sheet_tag_pos,
                    "x_mm": round(s_tag["x"], 2),
                    "y_mm": round(s_tag["y"], 2),
                    "width_mm": round(s_tag["w"], 2),
                    "height_mm": round(s_tag["h"], 2),
                },
                "solder_mask_color": sm_color,
                "unit_count": len(u_list),
                "units": u_data,
                "cnc_bed_operations": self.bed_operations.get(s_num, []),
            })

        metadata = {
            "$schema_version": "1.3.0",
            "production_summary": {
                "batch_name": "A4_PCB_PANEL_BATCH",
                "total_sheets": len(self.sheets),
                "total_units": total_units,
                "sheet_dimensions_mm": {"width": self.sheet_w, "height": self.sheet_h},
                "cnc_bed_travel_mm": {"x": self.bed_travel_x, "y": self.bed_travel_y},
                "y_split_threshold_mm": self.y_split,
                "y_axis_only_shift": self.sheet_h > self.bed_travel_y,
                "serpentine_same_tool_back_first": True,
                "breakaway_mode": self.breakaway_cfg.mode,
                "breakaway_mode_label": self.breakaway_cfg.label,
                "smooth_edge_finishing": self.breakaway_cfg.smooth_edge_finishing,
                "enable_dowel_pin_lock": self.enable_dowel_pin_lock,
                "bed_common_locking_points_single_half": len(self.holders_mgr.holders),
                "locking_drill_through_pcb": False,
                "z_probe_limit_switch": self.holders_mgr.z_probe_switch,
                "zero_gap_tight_packing": True,
                "single_pass_perimeter_cut": True
            },
            "sheets": sheets_data,
            "generated_files_count": len(self.generated_files)
        }

        meta_path = self.cfg.get("output_paths", {}).get("metadata_json", "manufacture_metadata.json")
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2)
        print(f"[INFO] Saved manufacturing metadata to: {meta_path}")

    def _sync_fabrication_checklist(self):
        """
        Synchronize manufacture_guides/checklist_state.json with exact step-by-step operations:
        - Bed cleanliness & Z-Zero Limit Switch Danger Zone (X=272.001, Y=135.002 mm)
        - V-bit sheet data & reference spot engraving
        - Tool changes + Limit Switch Z-zeroing for EVERY drill diameter step-by-step
        - Steps 2..7 with Y-axis flip and tool-change Z-zero prompts.
        Preserves prior checked states for matching item keys.
        """
        chk_path = self.cfg.get("output_paths", {}).get("checklist_json", os.path.join(self.out_guides_dir, "checklist_state.json"))
        old_items: Dict[str, Any] = {}
        if os.path.exists(chk_path):
            try:
                with open(chk_path, "r", encoding="utf-8") as f:
                    old_items = json.load(f).get("items", {})
            except Exception:
                old_items = {}

        drill_counts: Dict[float, int] = {}
        for s in self.sheets:
            for u in s:
                for d, pts in self.layer_cache.get_project_layers(u.project).get("drills", {}).items():
                    drill_counts[float(d)] = drill_counts.get(float(d), 0) + len(pts)
                if u.mouse_bites:
                    mb_d = float(self.breakaway_cfg.mouse_bite_drill_dia_mm)
                    drill_counts[mb_d] = drill_counts.get(mb_d, 0) + len(u.mouse_bites)

        zsw = self.holders_mgr.z_probe_switch
        zx = float(zsw.get("x_mm", 272.001))
        zy = float(zsw.get("y_mm", 135.002))

        items = _default_checklist_items(
            y_split=self.y_split,
            drill_counts=drill_counts,
            z_switch_x=zx,
            z_switch_y=zy,
            enable_dowel_pin_lock=self.enable_dowel_pin_lock,
        )
        for k, entry in items.items():
            if k in old_items and isinstance(old_items[k], dict):
                entry["checked"] = bool(old_items[k].get("checked", False))

        os.makedirs(os.path.dirname(chk_path) or ".", exist_ok=True)
        payload = {
            "batch": "A4_PCB_PANEL_BATCH",
            "z_probe_limit_switch": {"x_mm": zx, "y_mm": zy, "danger_zone": True},
            "enable_dowel_pin_lock": self.enable_dowel_pin_lock,
            "items": items,
        }
        with open(chk_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        print(f"[INFO] Synchronized fabrication checklist ({len(items)} steps) -> {chk_path}")


# ---------------------------------------------------------------------------
# Interactive HTML Dashboard & Setup Checklist Generator
# ---------------------------------------------------------------------------

def _default_checklist_items(
    y_split: float = 145.0,
    drill_counts: Optional[Dict[float, int]] = None,
    z_switch_x: float = 272.001,
    z_switch_y: float = 135.002,
    enable_dowel_pin_lock: bool = False,
) -> Dict[str, Dict[str, Any]]:
    """Build step-by-step fabrication checklist items including tool changes & Z-Zero Limit Switch checks."""
    pin_msg = "Optional 3.0mm dowel pin lock ENABLED." if enable_dowel_pin_lock else "Dowel pin lock DISABLED (configure enable_dowel_pin_lock in manufacture_config.json)."
    items: Dict[str, Dict[str, Any]] = {
        "check_bed_clean": {
            "label": "1. Spoilboard Cleanliness & Flatness Check",
            "category": "Prep & Safety", "checked": False,
            "description": "Remove debris and verify A4 spoilboard fixture surface is flat before clamping."
        },
        "check_z_switch_danger_zone": {
            "label": f"2. Verify Bed Z-Zero Limit Switch Danger Zone (X={z_switch_x:.3f}, Y={z_switch_y:.3f} mm)",
            "category": "Danger Zone", "checked": False,
            "description": (
                f"Confirm Z-Zero probe limit switch at X={z_switch_x:.3f}mm, Y={z_switch_y:.3f}mm is wired, "
                f"unobstructed, and NEVER overlapped by copper stock, M4 washer clamps, or rapid moves."
            ),
        },
        "check_stock_washers": {
            "label": "3. Clamp A4 Sheet Using Common Single-Half Bed M4 Washers (No Through-PCB Drill)",
            "category": "Prep & Safety", "checked": False,
            "description": "Clamp copper edges at Front Half using the 4 shared bed M4 washer clamps (common bed coordinates reused for Y-shifted Back Half)."
        },
        "check_vbit_tool_z0": {
            "label": f"4. Install 30° V-Bit & Zero Z-Axis via Limit Switch (X={z_switch_x:.3f}, Y={z_switch_y:.3f} mm)",
            "category": "Tool & Z-Zero", "checked": False,
            "description": f"Insert 30° / 0.1mm V-Bit, probe tool length on bed Z-Zero Limit Switch (X={z_switch_x:.3f}, Y={z_switch_y:.3f}mm Danger Zone), and run FR4 height map."
        },
        "check_step0_sheet_data": {
            "label": "5. Engrave Sheet Data, Reference Spots & Front-Corner Tag (Step 0A Front -> 0B Back)",
            "category": "Step 0 - Guides", "checked": False,
            "description": f"Engrave Sheet ID tag, Y{y_split:.0f} SHIFT mark & reference guides on Front Half, slide -{y_split:.1f}mm along Y-axis only, then run 0B Back. {pin_msg}"
        },
    }

    step_num = 6
    if drill_counts:
        sorted_dias = sorted(drill_counts.keys())
        cur_half = "front"
        for idx_d, dia in enumerate(sorted_dias):
            cnt = drill_counts[dia]
            d_slug = f"{dia:.2f}".replace(".", "_")
            if idx_d == 0:
                order_txt = f"Front Half -> Y-{y_split:.1f}mm Shift -> Back Half"
                next_half = "back"
            elif cur_half == "back":
                order_txt = f"Start directly at Back Half (Serpentine) -> Y+{y_split:.1f}mm -> Front Half"
                next_half = "front"
            else:
                order_txt = f"Start at Front Half -> Y-{y_split:.1f}mm -> Back Half"
                next_half = "back"
            cur_half = next_half

            items[f"check_drill_{d_slug}_tool_z0"] = {
                "label": f"{step_num}. Change Bit -> {dia:.2f} mm Carbide Drill & Zero Z on Limit Switch (X={z_switch_x:.3f}, Y={z_switch_y:.3f} mm)",
                "category": "Tool & Z-Zero", "checked": False,
                "description": f"Swap collet to {dia:.2f} mm drill bit, jog safely into Limit Switch Danger Zone (X={z_switch_x:.3f}, Y={z_switch_y:.3f}mm) and set Z-Zero before starting."
            }
            step_num += 1
            items[f"check_drill_{d_slug}_execute"] = {
                "label": f"{step_num}. Step 1: Drill {cnt} Hole(s) Across Sheet Using {dia:.2f} mm Bit",
                "category": "Step 1 - Drilling", "checked": False,
                "description": f"Execute peck drilling for all {cnt} holes at {dia:.2f} mm ({order_txt})."
            }
            step_num += 1
    else:
        items["check_drill_generic_tool_z0"] = {
            "label": f"{step_num}. Change Carbide Drill Bits & Zero Z via Limit Switch per Diameter",
            "category": "Tool & Z-Zero", "checked": False,
            "description": f"For each drill diameter, swap tool bit and Z-zero at bed Limit Switch (X={z_switch_x:.3f}, Y={z_switch_y:.3f}mm) before running Step 1 drilling."
        }
        step_num += 1

    items["check_step2_vias"] = {
        "label": f"{step_num}. Step 2: Chemical Through-Hole Via Metallization Bath",
        "category": "Step 2 - Vias", "checked": False,
        "description": "Perform desmear, palladium catalyst activation, micro-etch & 25µm acid Cu barrel electroplating on drilled vias."
    }
    step_num += 1
    items["check_step3_tool_z0"] = {
        "label": f"{step_num}. Change to 30° V-Bit, Z-Zero at Limit Switch & Mill Top Copper (Step 3A/3B)",
        "category": "Step 3 - Top Cu", "checked": False,
        "description": f"Install 30° V-Bit, Z-zero via bed Limit Switch (X={z_switch_x:.3f}, Y={z_switch_y:.3f}mm), re-autolevel, then isolate Front Half (3A) & Back Half (3B)."
    }
    step_num += 1
    items["check_step4_flip_y_z0"] = {
        "label": f"{step_num}. Flip Sheet Along Y-Axis, Re-Zero Z via Limit Switch & Mill Mirrored Bottom Copper (4A/4B)",
        "category": "Step 4 - Bottom Cu", "checked": False,
        "description": f"Flip A4 sheet Left<->Right along Y-axis, re-clamp with the same 4 bed washers, probe Z at Limit Switch (X={z_switch_x:.3f}, Y={z_switch_y:.3f}mm), then run Step 4A/4B."
    }
    step_num += 1
    items["check_step5_mask_z0"] = {
        "label": f"{step_num}. Step 5: Cure Solder Mask -> Change to 0.4mm Bit & Z-Zero at Limit Switch -> Clear Pads",
        "category": "Step 5 - Mask", "checked": False,
        "description": f"Install 0.4mm pad-clear bit, probe Z on Limit Switch (X={z_switch_x:.3f}, Y={z_switch_y:.3f}mm) + M0 touch-off on cured mask, and clear Front & Back pads."
    }
    step_num += 1
    items["check_step6_silk"] = {
        "label": f"{step_num}. Step 6: UV Silkscreen Legend Printing (Front & Back Sides)",
        "category": "Step 6 - Silkscreen", "checked": False,
        "description": "Deposit UV-curable white component legends for Front and Back sides."
    }
    step_num += 1
    items["check_step7_endmill_z0"] = {
        "label": f"{step_num}. Change to 1.0mm Corn Endmill, Z-Zero at Limit Switch & Run Breakaway Cutout (Step 7A..7H)",
        "category": "Step 7 - Cutout", "checked": False,
        "description": f"Install 1.0mm endmill, Z-zero at Limit Switch (X={z_switch_x:.3f}, Y={z_switch_y:.3f}mm), mill Top Side Front->Back, flip along Y-axis, then mill Mirrored Bottom Side."
    }
    return items


def build_html_dashboard_content(sheets, holders_mgr: HolderManager, generated_files,
                                 config: Dict, fluidnc_ip="192.168.8.109",
                                 img_prefix="../manufacture_svg/",
                                 bed_operations: Optional[Dict[int, List[Dict[str, Any]]]] = None,
                                 layer_cache: Optional[LayerGeometryCache] = None):
    """Builds the HTML markup for the modern interactive CNC PCB fabrication dashboard."""
    total_units = sum(len(s) for s in sheets)
    breakaway_cfg = BreakawayConfig.from_config(config)
    enable_dowel_pin_lock = bool(config.get("guides_and_fixturing", {}).get("enable_dowel_pin_lock", False))
    zsw = holders_mgr.z_probe_switch
    z_switch_x = float(zsw.get("x_mm", 272.001))
    z_switch_y = float(zsw.get("y_mm", 135.002))
    sheet_w = float(config.get("copper_sheet", {}).get("sheet_width_mm", 210.0))
    sheet_h = float(config.get("copper_sheet", {}).get("sheet_height_mm", 297.0))
    cnc = config.get("cnc_limits", {})
    bed_x = float(cnc.get("bed_travel_x_mm", 275.0))
    bed_y = float(cnc.get("bed_travel_y_mm", 170.0))
    raw_split = float(cnc.get("y_split_threshold_mm", 145.0))
    y_split = max(max(0.0, sheet_h - bed_y), min(raw_split, bed_y - 5.0)) if sheet_h > bed_y else raw_split

    # Unique projects & per-project drill summaries
    projects_dict: Dict[str, ProjectInfo] = {}
    all_units: List[PlacedUnit] = []
    for s in sheets:
        for u in s:
            all_units.append(u)
            if u.project.pcb_id not in projects_dict:
                projects_dict[u.project.pcb_id] = u.project
    unique_projects = list(projects_dict.values())

    project_drill_meta: Dict[str, Dict[str, Any]] = {}
    for p in unique_projects:
        drills_dict = layer_cache.get_project_layers(p).get("drills", {}) if layer_cache else {}
        tools_list = [
            {"dia": float(d), "count": len(pts), "color": get_drill_color(d)}
            for d, pts in sorted(drills_dict.items())
        ]
        project_drill_meta[p.pcb_id] = {
            "total_holes": sum(t["count"] for t in tools_list),
            "via_count": len(drills_dict.get(0.3, [])),
            "tools": tools_list,
        }

    # Build JSON manifest for Section 2 (One-by-One Manufactured Unit View + Sub-Menu)
    unit_records: List[Dict[str, Any]] = []
    for idx_u, u in enumerate(all_units, 1):
        is_front_half = (u.y + u.h <= y_split + 1e-6)
        half_lbl = "Front Bed Half" if is_front_half else "Back Bed Half"
        tabs_count = len(u.seam_geom.get("retention_tabs", []))
        tab_summary = f"{tabs_count} x {u.breakaway_cfg.retention_tab_width_mm:.1f}mm Tabs" if tabs_count else f"{len(u.mouse_bites)} Mouse Bites"
        drl_cnt = project_drill_meta.get(u.project.pcb_id, {}).get("total_holes", 0)
        unit_records.append({
            "index": idx_u,
            "pcb_id": u.project.pcb_id,
            "project_name": u.project.project_name,
            "unit_id": u.unit_id,
            "sheet_num": u.sheet_idx + 1,
            "half": "front" if is_front_half else "back",
            "half_label": half_lbl,
            "rotation_deg": u.rotation_deg,
            "pcb_w": round(u.pcb_w, 1),
            "pcb_h": round(u.pcb_h, 1),
            "x": round(u.x, 1),
            "y": round(u.y, 1),
            "attach_side": u.attach_side.upper(),
            "shortest_edge": round(u.shortest_edge_len, 1),
            "tab_summary": tab_summary,
            "drill_holes": drl_cnt,
            "svg_front": f"Unit_{u.unit_id}_FrontSide.svg",
            "svg_front_mirrored": f"Unit_{u.unit_id}_FrontSide_Mirrored.svg",
            "svg_back": f"Unit_{u.unit_id}_BackSide.svg",
            "svg_back_mirrored": f"Unit_{u.unit_id}_BackSide_Mirrored.svg",
        })

    # Build Sheet-Wise Inspection View tabs & panels (Section 3)
    sheet_tabs_html = []
    sheet_panels_html = []
    for s_idx, u_list in enumerate(sheets):
        s_num = s_idx + 1
        is_first = (s_idx == 0)
        display_style = "block" if is_first else "none"
        sm_color = u_list[0].project.solder_mask_color.upper() if u_list else "GREEN"
        front_cnt = sum(1 for u in u_list if u.y + u.h <= y_split + 1e-6)
        back_cnt = len(u_list) - front_cnt

        sheet_tabs_html.append(
            f'<button type="button" onclick="selectSheet({s_num})" id="tab-btn-{s_num}" aria-selected="{"true" if is_first else "false"}" '
            f'class="sheet-tab-btn px-4 py-2.5 text-xs sm:text-sm font-semibold rounded-xl border transition-all cursor-pointer '
            f'{"bg-blue-600/20 border-blue-500/60 text-blue-300 shadow-sm" if is_first else "bg-[#0d121d] border-slate-800 text-slate-400 hover:text-slate-100 hover:border-slate-700"}">'
            f'Sheet {s_num:02d} &bull; {len(u_list)} Units ({sm_color})</button>'
        )

        units_rows = "".join([
            f'<tr class="border-b border-slate-800/80 hover:bg-slate-800/40 font-mono text-xs transition-colors">'
            f'<td class="py-2.5 px-3 text-amber-400 font-semibold">{svg_escape(u.project.pcb_id)}</td>'
            f'<td class="py-2.5 px-3 text-slate-100 font-semibold">{svg_escape(u.unit_id)}</td>'
            f'<td class="py-2.5 px-3 text-slate-300 font-sans">{svg_escape(u.project.project_name)}</td>'
            f'<td class="py-2.5 px-3 text-slate-400">{u.pcb_w:.1f} &times; {u.pcb_h:.1f} mm</td>'
            f'<td class="py-2.5 px-3 text-sky-400">{u.rotation_deg}&deg; ({u.attach_side.upper()}: {u.shortest_edge_len:.1f}mm)</td>'
            f'<td class="py-2.5 px-3"><span class="px-2 py-0.5 rounded text-[11px] font-semibold {"bg-emerald-500/15 text-emerald-300 border border-emerald-500/30" if u.y + u.h <= y_split + 1e-6 else "bg-violet-500/15 text-violet-300 border border-violet-500/30"}">{"Front Bed Half" if u.y + u.h <= y_split + 1e-6 else "Back Bed Half"}</span></td>'
            f'<td class="py-2.5 px-3 text-slate-400">X {u.x:.1f}, Y {u.y:.1f}</td>'
            f'<td class="py-2.5 px-3"><button type="button" onclick="jumpToUnit(\'{u.unit_id}\')" class="px-2.5 py-1 rounded-lg bg-slate-800 hover:bg-blue-600 text-slate-200 hover:text-white text-[11px] font-sans font-medium transition cursor-pointer">Inspect Unit &rarr;</button></td>'
            f'</tr>' for u in u_list
        ])

        sheet_panels_html.append(f'''
        <div id="sheet-panel-{s_num}" style="display: {display_style};" class="sheet-content">
          <div class="flex flex-wrap items-center justify-between gap-3 mb-4">
            <div class="flex flex-wrap items-center gap-2">
              <h3 class="font-semibold text-base sm:text-lg text-slate-100">Sheet {s_num:02d} Full Manufacturing Blueprint</h3>
              <span class="text-xs font-mono bg-slate-800/90 text-slate-300 px-2.5 py-0.5 rounded-md border border-slate-700/70">{sheet_w:.0f} &times; {sheet_h:.0f} mm</span>
              <span class="text-xs font-mono bg-emerald-500/10 text-emerald-300 border border-emerald-500/30 px-2.5 py-0.5 rounded-md">{sm_color} MASK</span>
              <span class="text-xs font-mono bg-slate-800/90 text-slate-300 border border-slate-700/70 px-2.5 py-0.5 rounded-md">Front Half: {front_cnt} | Back Half: {back_cnt}</span>
            </div>
            <div class="inline-flex rounded-lg bg-[#0a0e17] p-1 border border-slate-800" role="group" aria-label="Sheet Side View">
              <button type="button" onclick="setSheetSide({s_num}, 'top')" id="sheet-side-{s_num}-top" aria-pressed="true"
                      class="sheet-side-btn-{s_num} px-3 py-1 text-xs font-semibold rounded-md bg-blue-600 text-white transition cursor-pointer">
                Front Side (Top Copper)
              </button>
              <button type="button" onclick="setSheetSide({s_num}, 'back')" id="sheet-side-{s_num}-back" aria-pressed="false"
                      class="sheet-side-btn-{s_num} px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white transition cursor-pointer">
                Back Side (Mirrored Y-Flip &bull; Upright Text)
              </button>
            </div>
          </div>

          <div class="blueprint-stage p-4 rounded-xl border border-slate-800 flex justify-center items-center overflow-auto min-h-[420px]">
            <img id="sheet-img-{s_num}" src="{img_prefix}Sheet{s_num:02d}_Overview_Panel_Blueprint.svg" alt="Sheet {s_num} Manufacturing Blueprint" class="sheet-preview-img max-h-[760px] w-auto object-contain rounded transition-all">
          </div>

          <div class="mt-5 overflow-x-auto rounded-xl border border-slate-800/90 bg-[#0b0f19]">
            <div class="px-4 py-2.5 border-b border-slate-800 flex flex-wrap items-center justify-between gap-2">
              <h4 class="text-xs font-semibold text-slate-300 uppercase tracking-wider">Nested Units on Sheet {s_num:02d}</h4>
              <span class="text-[11px] font-mono text-amber-300/90">Y-Shift Line: {y_split:.1f} mm &bull; Common Single-Half Bed Washers (No Through-PCB Drills)</span>
            </div>
            <table class="w-full text-left border-collapse">
              <thead class="text-slate-400 border-b border-slate-800 bg-[#090d16] text-[11px] uppercase tracking-wider">
                <tr>
                  <th class="py-2.5 px-3">PCB ID</th>
                  <th class="py-2.5 px-3">Unique Unit ID</th>
                  <th class="py-2.5 px-3">Project</th>
                  <th class="py-2.5 px-3">Dimensions</th>
                  <th class="py-2.5 px-3">Rotation &amp; Shortest Tag</th>
                  <th class="py-2.5 px-3">Bed Half</th>
                  <th class="py-2.5 px-3">Origin (X, Y)</th>
                  <th class="py-2.5 px-3">Action</th>
                </tr>
              </thead>
              <tbody>{units_rows}</tbody>
            </table>
          </div>
        </div>
        ''')

    # Load initial checklist state
    chk_path = config.get("output_paths", {}).get("checklist_json", "manufacture_guides/checklist_state.json")
    checklist_data = {}
    if os.path.exists(chk_path):
        try:
            with open(chk_path, "r", encoding="utf-8") as f:
                checklist_data = json.load(f)
        except Exception:
            pass
    chk_items = checklist_data.get("items") or _default_checklist_items(
        y_split=y_split, z_switch_x=z_switch_x, z_switch_y=z_switch_y, enable_dowel_pin_lock=enable_dowel_pin_lock
    )

    chk_categories = ["ALL"]
    for it in chk_items.values():
        cat = it.get("category", "Setup")
        if cat not in chk_categories:
            chk_categories.append(cat)

    chk_rows_html = []
    for item_id, item in chk_items.items():
        checked_attr = "checked" if item.get("checked", False) else ""
        checked_class = "line-through text-slate-500" if item.get("checked", False) else "text-slate-100"
        cat = item.get("category", "Setup")
        chk_rows_html.append(f'''
        <label for="{item_id}" data-chk-cat="{svg_escape(cat)}"
               class="chk-row flex items-start gap-3 p-3.5 bg-[#0b101b] rounded-xl border border-slate-800 hover:border-blue-500/40 transition-colors cursor-pointer">
          <input type="checkbox" id="{item_id}" {checked_attr} onchange="toggleChecklistItem('{item_id}', this.checked)"
                 class="mt-0.5 w-4 h-4 rounded text-blue-500 bg-slate-900 border-slate-600 focus:ring-2 focus:ring-blue-500 focus:ring-offset-0 cursor-pointer">
          <div class="flex-1 min-w-0">
            <div class="flex flex-wrap items-center justify-between gap-2">
              <span id="label-{item_id}" class="text-sm font-medium {checked_class}">{svg_escape(item.get("label", item_id))}</span>
              <span class="text-[10px] font-mono uppercase tracking-wide px-2 py-0.5 rounded bg-slate-800/90 text-slate-300 border border-slate-700/70">{svg_escape(cat)}</span>
            </div>
            <p class="text-xs text-slate-400 mt-1 leading-relaxed">{svg_escape(item.get("description", ""))}</p>
          </div>
        </label>
        ''')

    first_proj_id = unique_projects[0].pcb_id if unique_projects else "PCB-AC-01"
    bed_ops_json = json.dumps(bed_operations or {})
    unit_records_json = json.dumps(unit_records)
    project_drill_json = json.dumps(project_drill_meta)

    html = f"""<!DOCTYPE html>
<html lang="en" class="dark scroll-smooth">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>PCB CAM &amp; Dual-Sided CNC Inspection Studio</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;600;700&display=swap" rel="stylesheet">
  <script src="https://cdn.tailwindcss.com"></script>
  <script>
    tailwind.config = {{
      theme: {{
        extend: {{
          fontFamily: {{
            sans: ['IBM Plex Sans', 'Inter', 'system-ui', '-apple-system', 'sans-serif'],
            mono: ['JetBrains Mono', 'Fira Code', 'ui-monospace', 'monospace'],
          }}
        }}
      }}
    }};
  </script>
  <style>
    :root {{
      --bg-canvas: #090b10;
      --bg-surface: #111622;
      --bg-elevated: #171f30;
      --border-subtle: rgba(148, 163, 184, 0.16);
      --text-primary: #f8fafc;
      --text-secondary: #94a3b8;
    }}
    body {{
      background-color: var(--bg-canvas);
      background-image: radial-gradient(circle at 15% 0%, rgba(59, 130, 246, 0.08), transparent 45%),
                        radial-gradient(circle at 85% 10%, rgba(16, 185, 129, 0.06), transparent 40%);
      color: var(--text-primary);
      font-family: 'IBM Plex Sans', system-ui, -apple-system, sans-serif;
      -webkit-font-smoothing: antialiased;
    }}
    .font-mono, code {{
      font-family: 'JetBrains Mono', ui-monospace, SFMono-Regular, monospace;
    }}
    .glass-panel {{
      background: rgba(17, 22, 34, 0.92);
      border: 1px solid var(--border-subtle);
      box-shadow: 0 12px 32px -12px rgba(0, 0, 0, 0.65);
    }}
    .blueprint-stage {{
      background-color: #070a11;
      background-image: radial-gradient(rgba(148, 163, 184, 0.12) 1px, transparent 1px);
      background-size: 18px 18px;
    }}
    .svg-container svg, .svg-container img {{
      width: 100%;
      height: auto;
      max-height: 540px;
      border-radius: 0.5rem;
    }}
    .step-btn.active {{
      border-color: rgba(59, 130, 246, 0.85);
      background: linear-gradient(135deg, rgba(59, 130, 246, 0.18), rgba(37, 99, 235, 0.06));
      color: #93c5fd;
      box-shadow: 0 0 0 1px rgba(59, 130, 246, 0.25);
    }}
    .submenu-item.active {{
      border-color: rgba(59, 130, 246, 0.85);
      background: rgba(59, 130, 246, 0.14);
      color: #f8fafc;
    }}
    ::-webkit-scrollbar {{ width: 8px; height: 8px; }}
    ::-webkit-scrollbar-track {{ background: #0a0e17; }}
    ::-webkit-scrollbar-thumb {{ background: #293548; border-radius: 999px; }}
    ::-webkit-scrollbar-thumb:hover {{ background: #3b4b66; }}
    @media (prefers-reduced-motion: reduce) {{
      *, *::before, *::after {{ transition: none !important; animation: none !important; scroll-behavior: auto !important; }}
    }}
  </style>
  <script>
    const imgPrefix = '{img_prefix}';
    let currentProject = '{first_proj_id}';
    let currentStep = 'Composite_Stackup';
    let circuitSide = 'front';          // 'front' | 'back' for Mask & Silkscreen
    let circuitMirrorEnabled = false;   // Optional Mirror View Enable / Disable toggle

    const projectDrills = {project_drill_json};
    const allUnitRecords = {unit_records_json};
    const bedOperationsBySheet = {bed_ops_json};

    const stepInfo = {{
      'Composite_Stackup': {{
        title: 'Composite Circuit Stackup Overview',
        tool: 'All layers combined ({breakaway_cfg.label})',
        depth: 'N/A', feed: 'N/A', rpm: 'N/A',
        instructions: 'Full inspection of FR4 substrate, top/bottom copper, mask openings, silkscreen legends, multi-size drills, and shortest-edge 5mm stepping tag.'
      }},
      'Step1_Drills': {{
        title: 'Step 1: Multi-Diameter CNC Drilling',
        tool: 'Carbide Drills (Per-diameter toolpath)',
        depth: '-1.80 mm', feed: '150 mm/min', rpm: '10,000 RPM',
        instructions: 'Peck drilling (0.4mm increments). Starts on Front Bed Half; subsequent different drill sizes start directly on Back Bed Half when sheet rests at Back to save Y-shifts.'
      }},
      'Step2_Vias': {{
        title: 'Step 2: Chemical Through-Hole Via Metallization',
        tool: 'Electroless / Electroplating Bath (No CNC Image)',
        depth: '25 µm Cu', feed: 'Chemical', rpm: '45 min',
        instructions: 'Chemical immersion process — no Gerber image needed. Desmear -> Palladium catalyst activation -> Micro-etch -> Acid copper barrel deposition across all 0.30mm via holes.'
      }},
      'Step3_Top_Copper': {{
        title: 'Step 3: Top Copper Isolation Engraving (Front Side)',
        tool: '30° / 0.1–0.2mm V-Bit',
        depth: '-0.04 mm', feed: '300 mm/min', rpm: '10,000 RPM',
        instructions: 'Front-side isolation routing (Front Half before Back Half). Engraves ONLY PCB ID and triangle arrow pointing toward PCB body.'
      }},
      'Step4_Bottom_Copper': {{
        title: 'Step 4: Bottom Copper Isolation Engraving (Back Side)',
        tool: '30° / 0.1–0.2mm V-Bit',
        depth: '-0.04 mm', feed: '300 mm/min', rpm: '10,000 RPM',
        instructions: 'Toggle Mirror View Enable/Disable above. Machined FLIPPED horizontally along the Y-axis with upright readable stepping tags.'
      }},
      'Step5_Solder_Mask': {{
        title: 'Step 5: Solder Mask Pad Clearing (Front & Back Sides)',
        tool: '0.4mm Endmill / V-Bit',
        depth: '-0.06 mm', feed: '300 mm/min', rpm: '10,000 RPM',
        instructions: 'Switch between Front Side (mask_top) and Back Side (mask_bottom) and optional Mirror View. Re-touch Z0 at M0 pause (~35µm cured film).'
      }},
      'Step6_Silkscreen': {{
        title: 'Step 6: Silkscreen Component Legend Printing (Front & Back Sides)',
        tool: 'Flatbed Inkjet DoD Printhead (# TODO)',
        depth: 'Surface', feed: 'Inkjet', rpm: 'UV 395nm',
        instructions: 'Switch between Front Side (silkscreen_top) and Back Side (silkscreen_bottom) with optional Mirror View. Empty Back side indicates no back silkscreen features in Gerber.'
      }},
      'Step7_Cutout_SnapTab': {{
        title: 'Step 7: Dual-Sided Smooth Breakaway & Step Cutouts',
        tool: '1.0mm Corn Endmill &/or 30° V-Bit',
        depth: '-0.85 mm / side', feed: '200 mm/min', rpm: '10,000 RPM',
        instructions: 'Active mode: {breakaway_cfg.label}. Two evenly distributed ~2.0mm retention tabs along shortest edge with smooth tool-radius compensation.'
      }}
    }};

    function selectCircuitProject(pcbId) {{
      currentProject = pcbId;
      document.querySelectorAll('.proj-btn').forEach(btn => {{
        const active = (btn.id === 'proj-btn-' + pcbId);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
        btn.className = active
          ? 'proj-btn px-3.5 py-1.5 text-xs font-semibold rounded-lg bg-blue-600 text-white shadow-sm cursor-pointer transition'
          : 'proj-btn px-3.5 py-1.5 text-xs font-semibold rounded-lg bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70 cursor-pointer transition';
      }});
      updateCircuitView();
    }}

    function selectCircuitStep(stepName) {{
      currentStep = stepName;
      circuitMirrorEnabled = (stepName === 'Step4_Bottom_Copper');
      document.querySelectorAll('.step-btn').forEach(btn => {{
        const active = (btn.id === 'step-btn-' + stepName);
        btn.classList.toggle('active', active);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
      }});
      updateCircuitView();
    }}

    function setCircuitSide(side) {{
      circuitSide = side;
      updateCircuitView();
    }}

    function toggleCircuitMirror() {{
      circuitMirrorEnabled = !circuitMirrorEnabled;
      updateCircuitView();
    }}

    function updateCircuitView() {{
      const isDualLayer = (currentStep === 'Step5_Solder_Mask' || currentStep === 'Step6_Silkscreen');
      const isVias = (currentStep === 'Step2_Vias');

      const sideWrap = document.getElementById('circuit-side-controls');
      if (sideWrap) sideWrap.style.display = isDualLayer ? 'inline-flex' : 'none';
      document.querySelectorAll('.circuit-side-btn').forEach(btn => {{
        const active = (btn.id === 'circuit-side-' + circuitSide);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
        btn.className = active
          ? `circuit-side-btn px-3 py-1 text-xs font-semibold rounded-md ${{circuitSide === 'back' ? 'bg-violet-600' : 'bg-blue-600'}} text-white cursor-pointer transition`
          : 'circuit-side-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition';
      }});

      const mirrorWrap = document.getElementById('circuit-mirror-wrap');
      const mirrorBtn = document.getElementById('circuit-mirror-btn');
      if (mirrorWrap) mirrorWrap.style.display = isVias ? 'none' : 'inline-flex';
      if (mirrorBtn) {{
        mirrorBtn.setAttribute('aria-pressed', circuitMirrorEnabled ? 'true' : 'false');
        mirrorBtn.innerText = circuitMirrorEnabled ? 'Mirror View: Enabled (Y-Axis Flip)' : 'Mirror View: Disabled';
        mirrorBtn.className = circuitMirrorEnabled
          ? 'px-3.5 py-1.5 text-xs font-semibold rounded-lg bg-violet-600 hover:bg-violet-500 text-white border border-violet-400/60 shadow-sm cursor-pointer transition'
          : 'px-3.5 py-1.5 text-xs font-semibold rounded-lg bg-slate-800/90 hover:bg-slate-700 text-slate-300 border border-slate-700 cursor-pointer transition';
      }}

      const img = document.getElementById('circuit-svg-img');
      const viasCard = document.getElementById('circuit-vias-spec-card');
      const dMeta = projectDrills[currentProject] || {{ total_holes: 0, via_count: 0, tools: [] }};

      if (isVias) {{
        if (img) img.style.display = 'none';
        if (viasCard) {{
          viasCard.style.display = 'flex';
          const vc = document.getElementById('via-spec-count');
          if (vc) vc.innerText = `${{dMeta.via_count}} Via Holes (0.30 mm)`;
          const vp = document.getElementById('via-spec-project');
          if (vp) vp.innerText = currentProject;
        }}
      }} else {{
        if (viasCard) viasCard.style.display = 'none';
        let base = currentStep;
        if (currentStep === 'Step5_Solder_Mask') {{
          base = (circuitSide === 'back') ? 'Step5_Solder_Mask_Back' : 'Step5_Solder_Mask_Front';
        }} else if (currentStep === 'Step6_Silkscreen') {{
          base = (circuitSide === 'back') ? 'Step6_Silkscreen_Back' : 'Step6_Silkscreen_Front';
        }}
        const suffix = circuitMirrorEnabled ? '_Mirrored' : '';
        const svgPath = `${{imgPrefix}}Circuit_${{currentProject}}_${{base}}${{suffix}}.svg`;
        if (img) {{
          img.style.display = 'block';
          img.src = svgPath;
        }}
      }}

      const info = stepInfo[currentStep] || {{}};
      document.getElementById('card-step-title').innerText = info.title || currentStep;
      document.getElementById('card-tool').innerText = info.tool || '-';
      document.getElementById('card-depth').innerText = info.depth || '-';
      document.getElementById('card-feed').innerText = info.feed || '-';
      document.getElementById('card-rpm').innerText = info.rpm || '-';
      document.getElementById('card-instructions').innerText = info.instructions || '-';

      const drillPanel = document.getElementById('card-drill-breakdown');
      const drillChips = document.getElementById('card-drill-chips');
      if (drillPanel && drillChips && dMeta) {{
        const showDrills = (currentStep === 'Step1_Drills' || currentStep === 'Composite_Stackup');
        drillPanel.style.display = showDrills ? 'block' : 'none';
        if (showDrills) {{
          document.getElementById('card-drill-total').innerText = `${{dMeta.total_holes}} holes / ${{dMeta.tools.length}} sizes`;
          drillChips.innerHTML = dMeta.tools.map(t => `
            <span class="inline-flex items-center gap-1.5 px-2 py-1 rounded-md bg-slate-900 border border-slate-800 text-[11px] font-mono text-slate-200">
              <span class="w-2.5 h-2.5 rounded-full border border-white/60" style="background:${{t.color}}"></span>
              <span>${{t.dia.toFixed(2)}}mm</span>
              <span class="text-slate-400">(&times;${{t.count}})</span>
            </span>
          `).join('');
        }}
      }}
    }}

    // -------------------------------------------------------------------
    // Section 2: Individual Manufactured Unit View (Front Side / Back Side)
    // -------------------------------------------------------------------
    let unitFilterProject = 'ALL';
    let filteredUnits = [...allUnitRecords];
    let selectedUnitId = allUnitRecords.length ? allUnitRecords[0].unit_id : '';
    let unitViewSide = 'front';       // 'front' | 'back'
    let unitMirrorEnabled = false;

    function filterUnitSubmenu(pcbId) {{
      unitFilterProject = pcbId;
      document.querySelectorAll('.unit-filter-btn').forEach(btn => {{
        const active = (btn.id === 'unit-filter-' + pcbId);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
        btn.className = active
          ? 'unit-filter-btn px-3.5 py-1.5 text-xs font-semibold rounded-lg bg-blue-600 text-white shadow-sm cursor-pointer transition'
          : 'unit-filter-btn px-3.5 py-1.5 text-xs font-semibold rounded-lg bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70 cursor-pointer transition';
      }});
      filteredUnits = (pcbId === 'ALL')
        ? [...allUnitRecords]
        : allUnitRecords.filter(u => u.pcb_id === pcbId);

      if (!filteredUnits.some(u => u.unit_id === selectedUnitId) && filteredUnits.length) {{
        selectedUnitId = filteredUnits[0].unit_id;
      }}
      renderUnitSubmenu();
      renderSelectedUnit();
    }}

    function setUnitSide(side) {{
      unitViewSide = side;
      unitMirrorEnabled = (side === 'back');
      renderSelectedUnit();
    }}

    function toggleUnitMirror() {{
      unitMirrorEnabled = !unitMirrorEnabled;
      renderSelectedUnit();
    }}

    function renderUnitSubmenu() {{
      const listEl = document.getElementById('unit-submenu-list');
      const countEl = document.getElementById('unit-submenu-count');
      if (countEl) countEl.innerText = `${{filteredUnits.length}} unit(s)`;
      if (!listEl) return;

      listEl.innerHTML = filteredUnits.map((u, idx) => {{
        const active = (u.unit_id === selectedUnitId);
        const halfCls = u.half === 'front'
          ? 'bg-emerald-500/15 text-emerald-300 border-emerald-500/30'
          : 'bg-violet-500/15 text-violet-300 border-violet-500/30';
        return `
          <button type="button" onclick="selectUnit('${{u.unit_id}}')" aria-current="${{active ? 'true' : 'false'}}"
                  class="submenu-item ${{active ? 'active' : ''}} w-full text-left p-2.5 rounded-xl border border-slate-800/90 bg-[#0b101b] hover:border-slate-600 transition cursor-pointer flex items-center justify-between gap-2">
            <div class="min-w-0">
              <div class="flex items-center gap-1.5">
                <span class="text-[11px] font-mono text-slate-500">#${{idx + 1}}</span>
                <span class="text-xs font-mono font-semibold text-amber-300 truncate">${{u.unit_id}}</span>
              </div>
              <div class="text-[11px] text-slate-400 truncate mt-0.5">${{u.project_name}} &bull; S0${{u.sheet_num}} &bull; ${{u.rotation_deg}}&deg;</div>
            </div>
            <span class="shrink-0 px-1.5 py-0.5 text-[10px] font-mono rounded border ${{halfCls}}">${{u.half_label}}</span>
          </button>
        `;
      }}).join('');
    }}

    function selectUnit(unitId) {{
      selectedUnitId = unitId;
      renderUnitSubmenu();
      renderSelectedUnit();
    }}

    function stepUnit(delta) {{
      if (!filteredUnits.length) return;
      const curIdx = Math.max(0, filteredUnits.findIndex(u => u.unit_id === selectedUnitId));
      const nextIdx = (curIdx + delta + filteredUnits.length) % filteredUnits.length;
      selectedUnitId = filteredUnits[nextIdx].unit_id;
      renderUnitSubmenu();
      renderSelectedUnit();
    }}

    function renderSelectedUnit() {{
      const idx = filteredUnits.findIndex(u => u.unit_id === selectedUnitId);
      const u = idx >= 0 ? filteredUnits[idx] : filteredUnits[0];
      if (!u) return;

      document.querySelectorAll('.unit-side-btn').forEach(btn => {{
        const active = (btn.id === 'unit-side-' + unitViewSide);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
        btn.className = active
          ? `unit-side-btn px-3 py-1 text-xs font-semibold rounded-md ${{unitViewSide === 'back' ? 'bg-violet-600' : 'bg-blue-600'}} text-white cursor-pointer transition`
          : 'unit-side-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition';
      }});

      const uMirBtn = document.getElementById('unit-mirror-btn');
      if (uMirBtn) {{
        uMirBtn.setAttribute('aria-pressed', unitMirrorEnabled ? 'true' : 'false');
        uMirBtn.innerText = unitMirrorEnabled ? 'Mirror: Enabled' : 'Mirror: Disabled';
        uMirBtn.className = unitMirrorEnabled
          ? 'px-3 py-1.5 text-xs font-semibold rounded-lg bg-violet-600 text-white border border-violet-400/60 cursor-pointer transition'
          : 'px-3 py-1.5 text-xs font-semibold rounded-lg bg-slate-800 text-slate-300 border border-slate-700 hover:text-white cursor-pointer transition';
      }}

      let svgFile = u.svg_front;
      if (unitViewSide === 'front') {{
        svgFile = unitMirrorEnabled ? u.svg_front_mirrored : u.svg_front;
      }} else {{
        svgFile = unitMirrorEnabled ? u.svg_back_mirrored : u.svg_back;
      }}

      document.getElementById('unit-stage-title').innerText = u.unit_id;
      document.getElementById('unit-stage-pcb').innerText = u.pcb_id;
      document.getElementById('unit-stage-project').innerText = u.project_name;
      document.getElementById('unit-stage-pos').innerText = `Unit ${{idx + 1}} of ${{filteredUnits.length}}`;
      document.getElementById('unit-stage-img').src = `${{imgPrefix}}${{svgFile}}`;
      document.getElementById('unit-stage-img').alt = `${{u.unit_id}} (${{unitViewSide}} side) preview`;

      const sidePill = document.getElementById('unit-stage-side');
      if (sidePill) {{
        const sideDesc = unitViewSide === 'back'
          ? `BACK SIDE (${{unitMirrorEnabled ? 'MIRRORED' : 'UNMIRRORED'}})`
          : `FRONT SIDE${{unitMirrorEnabled ? ' (MIRRORED)' : ''}}`;
        sidePill.innerText = sideDesc;
        sidePill.className = unitViewSide === 'back'
          ? 'px-2.5 py-0.5 text-[11px] font-mono font-semibold rounded-md border bg-violet-500/20 text-violet-200 border-violet-500/40'
          : 'px-2.5 py-0.5 text-[11px] font-mono font-semibold rounded-md border bg-sky-500/15 text-sky-200 border-sky-500/40';
      }}

      const halfBadge = document.getElementById('unit-stage-half');
      halfBadge.innerText = u.half_label;
      halfBadge.className = u.half === 'front'
        ? 'px-2.5 py-0.5 text-[11px] font-mono font-semibold rounded-md border bg-emerald-500/15 text-emerald-300 border-emerald-500/30'
        : 'px-2.5 py-0.5 text-[11px] font-mono font-semibold rounded-md border bg-amber-500/15 text-amber-300 border-amber-500/30';

      document.getElementById('unit-spec-size').innerText = `${{u.pcb_w}} × ${{u.pcb_h}} mm`;
      document.getElementById('unit-spec-sheet').innerText = `Sheet 0${{u.sheet_num}} (${{u.half_label}})`;
      document.getElementById('unit-spec-rot').innerText = `${{u.rotation_deg}}°`;
      document.getElementById('unit-spec-origin').innerText = `X ${{u.x}}, Y ${{u.y}} mm`;
      document.getElementById('unit-spec-tag').innerText = `${{u.attach_side}} (${{u.shortest_edge}} mm)`;
      document.getElementById('unit-spec-seam').innerText = u.tab_summary;
      document.getElementById('unit-spec-drills').innerText = `${{u.drill_holes}} holes`;
    }}

    function jumpToUnit(unitId) {{
      if (unitFilterProject !== 'ALL') filterUnitSubmenu('ALL');
      selectUnit(unitId);
      const sec = document.getElementById('section-units');
      if (sec) sec.scrollIntoView({{ behavior: 'smooth', block: 'start' }});
    }}

    // -------------------------------------------------------------------
    // Section 3: Sheet-Wise Inspection View (Front / Mirrored Back Toggle)
    // -------------------------------------------------------------------
    function selectSheet(num) {{
      document.querySelectorAll('.sheet-content').forEach(el => el.style.display = 'none');
      document.querySelectorAll('.sheet-tab-btn').forEach(btn => {{
        const active = (btn.id === 'tab-btn-' + num);
        btn.setAttribute('aria-selected', active ? 'true' : 'false');
        btn.className = active
          ? 'sheet-tab-btn px-4 py-2.5 text-xs sm:text-sm font-semibold rounded-xl border transition-all cursor-pointer bg-blue-600/20 border-blue-500/60 text-blue-300 shadow-sm'
          : 'sheet-tab-btn px-4 py-2.5 text-xs sm:text-sm font-semibold rounded-xl border transition-all cursor-pointer bg-[#0d121d] border-slate-800 text-slate-400 hover:text-slate-100 hover:border-slate-700';
      }});
      const panel = document.getElementById('sheet-panel-' + num);
      if (panel) panel.style.display = 'block';
    }}

    function setSheetSide(num, side) {{
      const sPad = String(num).padStart(2, '0');
      const img = document.getElementById('sheet-img-' + num);
      if (img) {{
        img.src = side === 'back'
          ? `${{imgPrefix}}Sheet${{sPad}}_BackSide_Mirrored_Blueprint.svg`
          : `${{imgPrefix}}Sheet${{sPad}}_Overview_Panel_Blueprint.svg`;
      }}
      document.querySelectorAll(`.sheet-side-btn-${{num}}`).forEach(btn => {{
        const active = (btn.id === `sheet-side-${{num}}-${{side}}`);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
        btn.className = active
          ? `sheet-side-btn-${{num}} px-3 py-1 text-xs font-semibold rounded-md ${{side === 'back' ? 'bg-violet-600' : 'bg-blue-600'}} text-white transition cursor-pointer`
          : `sheet-side-btn-${{num}} px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white transition cursor-pointer`;
      }});
    }}

    function setSheetZoom(mode) {{
      document.querySelectorAll('.sheet-zoom-btn').forEach(btn => {{
        const active = (btn.id === 'sheet-zoom-' + mode);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
        btn.className = active
          ? 'sheet-zoom-btn px-3 py-1 text-xs font-semibold rounded-md bg-blue-600 text-white cursor-pointer transition'
          : 'sheet-zoom-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition';
      }});
      document.querySelectorAll('.sheet-preview-img').forEach(img => {{
        if (mode === 'fit') {{
          img.style.maxHeight = '760px';
          img.style.width = 'auto';
        }} else if (mode === 'full') {{
          img.style.maxHeight = 'none';
          img.style.width = '100%';
        }} else if (mode === 'zoom') {{
          img.style.maxHeight = 'none';
          img.style.width = '145%';
        }}
      }});
    }}

    // -------------------------------------------------------------------
    // Section 4: CNC Bed & Dual-Sided Milling View (Tool-by-Tool)
    // -------------------------------------------------------------------
    let bedSheet = 1;
    let bedSideFilter = 'ALL';    // ALL | top | bottom
    let bedHalfFilter = 'ALL';    // ALL | front | back
    let filteredBedOps = [];
    let selectedBedOpId = '';

    function selectBedSheet(sNum) {{
      bedSheet = sNum;
      document.querySelectorAll('.bed-sheet-btn').forEach(btn => {{
        const active = (btn.id === 'bed-sheet-' + sNum);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
        btn.className = active
          ? 'bed-sheet-btn px-3 py-1.5 text-xs font-semibold rounded-lg bg-blue-600 text-white cursor-pointer transition'
          : 'bed-sheet-btn px-3 py-1.5 text-xs font-semibold rounded-lg bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70 cursor-pointer transition';
      }});
      refreshBedOps();
    }}

    function setBedSideFilter(side) {{
      bedSideFilter = side;
      document.querySelectorAll('.bed-side-btn').forEach(btn => {{
        const active = (btn.id === 'bed-side-' + side);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
        const activeBg = side === 'bottom' ? 'bg-violet-600' : 'bg-blue-600';
        btn.className = active
          ? `bed-side-btn px-3 py-1 text-xs font-semibold rounded-md ${{activeBg}} text-white cursor-pointer transition`
          : 'bed-side-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition';
      }});
      refreshBedOps();
    }}

    function setBedHalfFilter(half) {{
      bedHalfFilter = half;
      document.querySelectorAll('.bed-half-btn').forEach(btn => {{
        const active = (btn.id === 'bed-half-' + half);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
        btn.className = active
          ? 'bed-half-btn px-3 py-1 text-xs font-semibold rounded-md bg-emerald-600 text-white cursor-pointer transition'
          : 'bed-half-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition';
      }});
      refreshBedOps();
    }}

    function refreshBedOps() {{
      const rawOps = bedOperationsBySheet[String(bedSheet)] || [];
      filteredBedOps = rawOps.filter(op => {{
        if (bedSideFilter !== 'ALL' && op.board_side !== bedSideFilter) return false;
        if (bedHalfFilter !== 'ALL' && op.bed_half !== bedHalfFilter) return false;
        return true;
      }});
      if (!filteredBedOps.some(op => op.op_id === selectedBedOpId) && filteredBedOps.length) {{
        selectedBedOpId = filteredBedOps[0].op_id;
      }}
      renderBedSubmenu();
      renderSelectedBedOp();
    }}

    function renderBedSubmenu() {{
      const el = document.getElementById('bed-ops-submenu');
      const cntEl = document.getElementById('bed-ops-count');
      if (cntEl) cntEl.innerText = `${{filteredBedOps.length}} tool ops`;
      if (!el) return;
      if (!filteredBedOps.length) {{
        el.innerHTML = '<div class="p-4 text-xs text-slate-400 text-center">No CNC operations match this filter combination.</div>';
        return;
      }}
      el.innerHTML = filteredBedOps.map(op => {{
        const active = (op.op_id === selectedBedOpId);
        const sideBadge = op.mirrored
          ? '<span class="px-1.5 py-0.5 text-[10px] font-mono rounded bg-violet-500/20 text-violet-300 border border-violet-500/40">BACK SIDE (MIRROR)</span>'
          : '<span class="px-1.5 py-0.5 text-[10px] font-mono rounded bg-sky-500/15 text-sky-300 border border-sky-500/30">FRONT SIDE</span>';
        const halfBadge = op.bed_half === 'front'
          ? '<span class="px-1.5 py-0.5 text-[10px] font-mono rounded bg-emerald-500/15 text-emerald-300 border border-emerald-500/30">Front Bed Half</span>'
          : '<span class="px-1.5 py-0.5 text-[10px] font-mono rounded bg-amber-500/15 text-amber-300 border border-amber-500/30">Back Bed Half</span>';
        const serpBadge = op.started_back_same_tool
          ? '<span class="px-1.5 py-0.5 text-[10px] font-mono rounded bg-cyan-500/20 text-cyan-300 border border-cyan-500/40">BACK FIRST (SERPENTINE)</span>'
          : '';
        return `
          <button type="button" onclick="selectBedOp('${{op.op_id}}')" aria-current="${{active ? 'true' : 'false'}}"
                  class="submenu-item ${{active ? 'active' : ''}} w-full text-left p-2.5 rounded-xl border border-slate-800/90 bg-[#0b101b] hover:border-slate-600 transition cursor-pointer">
            <div class="flex items-center justify-between gap-2">
              <span class="text-xs font-mono font-semibold text-slate-100">Op #${{String(op.seq).padStart(2, '0')}} &bull; ${{op.tool_name}}</span>
            </div>
            <div class="text-[11px] text-slate-400 mt-0.5 truncate">${{op.step_title}}</div>
            <div class="flex flex-wrap items-center gap-1 mt-1.5">
              ${{sideBadge}} ${{halfBadge}} ${{serpBadge}}
            </div>
          </button>
        `;
      }}).join('');
    }}

    function selectBedOp(opId) {{
      selectedBedOpId = opId;
      renderBedSubmenu();
      renderSelectedBedOp();
    }}

    function stepBedOp(delta) {{
      if (!filteredBedOps.length) return;
      const curIdx = Math.max(0, filteredBedOps.findIndex(op => op.op_id === selectedBedOpId));
      const nextIdx = (curIdx + delta + filteredBedOps.length) % filteredBedOps.length;
      selectedBedOpId = filteredBedOps[nextIdx].op_id;
      renderBedSubmenu();
      renderSelectedBedOp();
    }}

    function renderSelectedBedOp() {{
      const idx = filteredBedOps.findIndex(op => op.op_id === selectedBedOpId);
      const op = idx >= 0 ? filteredBedOps[idx] : filteredBedOps[0];
      if (!op) return;

      document.getElementById('bed-op-counter').innerText = `Op ${{idx + 1}} of ${{filteredBedOps.length}} (Seq #${{op.seq}})`;
      document.getElementById('bed-op-title').innerText = `${{op.step_title}} — ${{op.tool_name}}`;
      document.getElementById('bed-op-action').innerText = op.shift_action;
      document.getElementById('bed-op-img').src = `${{imgPrefix}}${{op.svg_file}}`;
      document.getElementById('bed-op-img').alt = `${{op.step_title}} CNC Bed Setup`;

      const mirrorPill = document.getElementById('bed-mirror-pill');
      if (op.mirrored) {{
        mirrorPill.innerText = 'BACK SIDE: MIRRORED HORIZONTALLY ALONG Y-AXIS';
        mirrorPill.className = 'px-2.5 py-1 text-[11px] font-mono font-semibold rounded-md bg-violet-500/20 text-violet-200 border border-violet-500/40';
      }} else {{
        mirrorPill.innerText = 'TOP SIDE: DIRECT ORIENTATION (UNMIRRORED)';
        mirrorPill.className = 'px-2.5 py-1 text-[11px] font-mono font-semibold rounded-md bg-sky-500/15 text-sky-200 border border-sky-500/40';
      }}

      document.getElementById('bed-spec-tool').innerText = op.tool_name;
      document.getElementById('bed-spec-depth').innerText = `${{op.depth_mm.toFixed(2)}} mm (Feed ${{op.feed_mm_min}} / ${{op.spindle_rpm}} RPM)`;
      document.getElementById('bed-spec-yshift').innerText = op.bed_half === 'front' ? 'FRONT HALF (0.0 mm)' : `BACK HALF (-${{op.y_shift_mm.toFixed(1)}} mm Y-Shift)`;
      document.getElementById('bed-spec-bounds').innerText = `X[${{op.bounds_x[0]}}..${{op.bounds_x[1]}}] Y[${{op.bounds_y[0]}}..${{op.bounds_y[1]}}] mm`;
      document.getElementById('bed-spec-gcode').innerText = op.gcode_file;
      document.getElementById('bed-spec-stats').innerText = op.stats_label;
    }}

    // -------------------------------------------------------------------
    // Section 5: Checklist interactive persistence
    // -------------------------------------------------------------------
    let checklistState = {json.dumps(chk_items)};

    function filterChecklistCat(cat) {{
      document.querySelectorAll('.chk-cat-btn').forEach(btn => {{
        const active = (btn.dataset.cat === cat);
        btn.setAttribute('aria-pressed', active ? 'true' : 'false');
        btn.className = active
          ? 'chk-cat-btn px-3 py-1 text-xs font-semibold rounded-lg bg-blue-600 text-white cursor-pointer transition'
          : 'chk-cat-btn px-3 py-1 text-xs font-semibold rounded-lg bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70 cursor-pointer transition';
      }});
      document.querySelectorAll('.chk-row').forEach(row => {{
        row.style.display = (cat === 'ALL' || row.dataset.chkCat === cat) ? 'flex' : 'none';
      }});
    }}

    function toggleChecklistItem(itemId, isChecked) {{
      if (checklistState[itemId]) {{
        checklistState[itemId].checked = isChecked;
      }}
      const lbl = document.getElementById('label-' + itemId);
      if (lbl) {{
        lbl.classList.toggle('line-through', isChecked);
        lbl.classList.toggle('text-slate-500', isChecked);
        lbl.classList.toggle('text-slate-100', !isChecked);
      }}
      updateChecklistProgress();
      saveChecklistToDisk();
    }}

    function resetChecklist() {{
      Object.keys(checklistState).forEach(k => {{
        checklistState[k].checked = false;
        const cb = document.getElementById(k);
        if (cb) cb.checked = false;
        const lbl = document.getElementById('label-' + k);
        if (lbl) {{
          lbl.classList.remove('line-through', 'text-slate-500');
          lbl.classList.add('text-slate-100');
        }}
      }});
      updateChecklistProgress();
      saveChecklistToDisk();
    }}

    function updateChecklistProgress() {{
      const keys = Object.keys(checklistState);
      const total = keys.length || 1;
      const checkedCount = Object.values(checklistState).filter(item => item.checked).length;
      const pct = Math.round((checkedCount / total) * 100);
      document.getElementById('chk-progress-bar').style.width = pct + '%';
      document.getElementById('chk-progress-text').innerText = `${{checkedCount}} / ${{keys.length}} Completed (${{pct}}%)`;
    }}

    async function saveChecklistToDisk() {{
      const payload = {{
        updated_at: new Date().toISOString(),
        checklist_version: "1.0",
        items: checklistState
      }};
      try {{
        localStorage.setItem('pcb_checklist_state', JSON.stringify(payload));
        fetch('/api/save_checklist', {{
          method: 'POST',
          headers: {{ 'Content-Type': 'application/json' }},
          body: JSON.stringify(payload)
        }}).catch(() => {{}});
      }} catch (e) {{}}
    }}

    // -------------------------------------------------------------------
    // Section: FluidNC G-Code Dispatcher & Wireless Transfer Station
    // -------------------------------------------------------------------
    let fluidncTargetIp = '{svg_escape(fluidnc_ip)}';
    let fluidncDispatcherSheet = '1';
    let fluidncDispatcherCat = 'ALL';
    let fluidncDispatcherSearch = '';
    const fluidncSentFiles = new Set();

    function getApiBase() {{
      return (window.location.protocol === 'file:') ? 'http://localhost:8088' : '';
    }}

    function updateFluidNCIp(newIp) {{
      fluidncTargetIp = newIp.trim() || '192.168.8.109';
      try {{ localStorage.setItem('fluidnc_ip', fluidncTargetIp); }} catch (e) {{}}
      const link = document.getElementById('fluidnc-open-link');
      if (link) link.href = `http://${{fluidncTargetIp}}`;
      const frameLink = document.getElementById('fluidnc-frame-external-link');
      if (frameLink) frameLink.href = `http://${{fluidncTargetIp}}`;
      const ipLabel = document.getElementById('fluidnc-embedded-ip-label');
      if (ipLabel) ipLabel.innerText = fluidncTargetIp;
      testFluidNCConnection();
    }}

    let fluidncMachineState = 'Unknown';
    let fluidncMPos = '';
    let fluidncSdMounted = false;
    let activeStreamWs = null;
    let streamAborted = false;

    async function testFluidNCConnection() {{
      const badge = document.getElementById('fluidnc-status-badge');
      const dot = document.getElementById('fluidnc-status-dot');
      const text = document.getElementById('fluidnc-status-text');
      const sdIndicator = document.getElementById('fluidnc-sd-indicator');
      if (!badge || !dot || !text) return;

      dot.className = 'w-2 h-2 rounded-full bg-amber-400 animate-ping';
      text.innerText = `Pinging ${{fluidncTargetIp}}...`;

      // 1. Check HTTP reachability directly
      let httpReachable = false;
      try {{
        const ctrl = new AbortController();
        const tid = setTimeout(() => ctrl.abort(), 2000);
        await fetch(`http://${{fluidncTargetIp}}/`, {{ signal: ctrl.signal, mode: 'no-cors' }});
        clearTimeout(tid);
        httpReachable = true;
      }} catch (e) {{}}

      // 2. Query SD card status via /upload?path=/
      try {{
        const sdRes = await fetch(`http://${{fluidncTargetIp}}/upload?path=/`, {{ cache: 'no-store' }});
        if (sdRes.ok) {{
          const sdData = await sdRes.json();
          fluidncSdMounted = (sdData.status !== 'No SD card');
          if (sdIndicator) {{
            sdIndicator.innerHTML = fluidncSdMounted
              ? `<span class="text-emerald-400 font-semibold">SD Card: Mounted (${{sdData.total || 'OK'}})</span>`
              : `<span class="text-amber-400 font-semibold">SD Card: Missing (TF Slot Empty) - Streaming Active</span>`;
          }}
        }}
      }} catch (e) {{
        if (sdIndicator) {{
          sdIndicator.innerHTML = `<span class="text-slate-400">SD Card: Missing / Direct Stream Active</span>`;
        }}
      }}

      // 3. Connect WebSocket with subprotocol 'arduino' to get live machine state (Idle, Alarm, MPos)
      try {{
        const wsUrl = `ws://${{fluidncTargetIp}}/?independent_session=1`;
        const ws = new WebSocket(wsUrl, 'arduino');
        let gotStatus = false;

        const wsTimeout = setTimeout(() => {{
          if (!gotStatus) {{
            try {{ ws.close(); }} catch (e) {{}}
            if (httpReachable) {{
              dot.className = 'w-2 h-2 rounded-full bg-emerald-400';
              text.innerText = `FluidNC Online (HTTP Ready) - ${{fluidncTargetIp}}`;
              badge.className = 'inline-flex items-center gap-2 px-3 py-1.5 rounded-lg bg-emerald-950/40 border border-emerald-500/50 text-xs font-mono text-emerald-300';
            }} else {{
              setOfflineBadge();
            }}
          }}
        }}, 2000);

        ws.onopen = () => {{
          const q = new Uint8Array([63]); // '?' status query
          ws.send(q);
        }};

        ws.onmessage = (evt) => {{
          const msg = (typeof evt.data === 'string') ? evt.data : new TextDecoder().decode(evt.data);
          for (const line of msg.split('\\n')) {{
            const trimmed = line.trim();
            if (trimmed.startsWith('<') && trimmed.endsWith('>')) {{
              gotStatus = true;
              clearTimeout(wsTimeout);
              const body = trimmed.slice(1, -1);
              const parts = body.split('|');
              fluidncMachineState = parts[0] || 'Idle';
              for (const p of parts) {{
                if (p.startsWith('MPos:')) fluidncMPos = p.replace('MPos:', '');
              }}
              dot.className = 'w-2 h-2 rounded-full bg-emerald-400';
              text.innerText = `FluidNC Online (${{fluidncMachineState}}${{fluidncMPos ? ' | MPos: ' + fluidncMPos : ''}})`;
              badge.className = 'inline-flex items-center gap-2 px-3 py-1.5 rounded-lg bg-emerald-950/40 border border-emerald-500/50 text-xs font-mono text-emerald-300';
              try {{ ws.close(); }} catch (e) {{}}
              break;
            }}
          }}
        }};

        ws.onerror = () => {{
          if (!gotStatus) {{
            clearTimeout(wsTimeout);
            if (httpReachable) {{
              dot.className = 'w-2 h-2 rounded-full bg-emerald-400';
              text.innerText = `FluidNC Reachable (${{fluidncTargetIp}})`;
              badge.className = 'inline-flex items-center gap-2 px-3 py-1.5 rounded-lg bg-emerald-950/40 border border-emerald-500/50 text-xs font-mono text-emerald-300';
            }} else {{
              setOfflineBadge();
            }}
          }}
        }};
        return;
      }} catch (err) {{
        if (httpReachable) {{
          dot.className = 'w-2 h-2 rounded-full bg-emerald-400';
          text.innerText = `FluidNC Reachable (${{fluidncTargetIp}})`;
          badge.className = 'inline-flex items-center gap-2 px-3 py-1.5 rounded-lg bg-emerald-950/40 border border-emerald-500/50 text-xs font-mono text-emerald-300';
          return;
        }}
        setOfflineBadge();
      }}

      function setOfflineBadge() {{
        dot.className = 'w-2 h-2 rounded-full bg-red-400';
        text.innerText = `FluidNC Offline (${{fluidncTargetIp}})`;
        badge.className = 'inline-flex items-center gap-2 px-3 py-1.5 rounded-lg bg-red-950/40 border border-red-500/50 text-xs font-mono text-red-300';
      }}
    }}

    async function sendFluidNCRealtime(cmd) {{
      try {{
        const ws = new WebSocket(`ws://${{fluidncTargetIp}}/?independent_session=1`, 'arduino');
        ws.onopen = () => {{
          ws.send(cmd + '\\n');
          setTimeout(() => {{
            try {{ ws.close(); }} catch (e) {{}}
            testFluidNCConnection();
          }}, 300);
        }};
      }} catch (e) {{
        alert('Failed to send command to FluidNC: ' + e);
      }}
    }}

    function toggleFluidNCFrame() {{
      const panel = document.getElementById('fluidnc-embedded-panel');
      const iframe = document.getElementById('fluidnc-iframe');
      if (!panel || !iframe) return;

      const isHidden = panel.classList.contains('hidden');
      if (isHidden) {{
        panel.classList.remove('hidden');
        if (iframe.src === 'about:blank' || !iframe.src) {{
          iframe.src = `http://${{fluidncTargetIp}}`;
        }}
      }} else {{
        panel.classList.add('hidden');
      }}
    }}

    function reloadFluidNCFrame() {{
      const iframe = document.getElementById('fluidnc-iframe');
      if (iframe) {{
        iframe.src = `http://${{fluidncTargetIp}}?t=${{Date.now()}}`;
      }}
    }}

    const gcodeDirAbs = {json.dumps(os.path.abspath("manufacture_gcode"))};

    function getFullGCodePath(filename) {{
      const sep = gcodeDirAbs.includes('\\\\') ? '\\\\' : '/';
      return `${{gcodeDirAbs}}${{sep}}${{filename}}`;
    }}

    function copyGCodePath(filename, buttonEl) {{
      const fullPath = getFullGCodePath(filename);
      const showCopied = () => {{
        if (!buttonEl) return;
        const orig = buttonEl.innerHTML;
        buttonEl.innerHTML = '✓ Copied!';
        buttonEl.classList.add('bg-emerald-600', 'text-white', 'border-emerald-500');
        setTimeout(() => {{
          buttonEl.innerHTML = orig;
          buttonEl.classList.remove('bg-emerald-600', 'text-white', 'border-emerald-500');
        }}, 1600);
      }};

      if (navigator.clipboard && navigator.clipboard.writeText) {{
        navigator.clipboard.writeText(fullPath).then(showCopied).catch(() => fallbackCopy(fullPath, showCopied));
      }} else {{
        fallbackCopy(fullPath, showCopied);
      }}
    }}

    function fallbackCopy(text, cb) {{
      const ta = document.createElement('textarea');
      ta.value = text;
      ta.style.position = 'fixed';
      ta.style.left = '-9999px';
      document.body.appendChild(ta);
      ta.select();
      try {{
        document.execCommand('copy');
        if (cb) cb();
      }} catch (e) {{
        prompt('Copy file path:', text);
      }}
      document.body.removeChild(ta);
    }}

    function onGCodeDragStart(e, filename) {{
      const host = window.location.host || 'localhost:8088';
      const proto = window.location.protocol.startsWith('http') ? window.location.protocol : 'http:';
      const fullUrl = `${{proto}}//${{host}}/manufacture_gcode/${{filename}}`;

      e.dataTransfer.setData('DownloadURL', `application/octet-stream:${{filename}}:${{fullUrl}}`);
      e.dataTransfer.setData('text/plain', filename);
      e.dataTransfer.setData('text/uri-list', fullUrl);
      e.dataTransfer.effectAllowed = 'copyMove';
    }}

    async function fetchGCodeText(filename) {{
      const candidates = [
        `${{getApiBase()}}/manufacture_gcode/${{filename}}`,
        `../manufacture_gcode/${{filename}}`,
        `./manufacture_gcode/${{filename}}`,
      ];
      for (const url of candidates) {{
        if (!url || url.startsWith('/manufacture_gcode') && window.location.protocol === 'file:') continue;
        try {{
          const res = await fetch(url, {{ cache: 'no-store' }});
          if (res.ok) return await res.text();
        }} catch (e) {{}}
      }}
      throw new Error('Could not fetch G-code via HTTP. Start local server with: python3 gcode_paralel_manufacture.py --serve');
    }}

    async function sendGCodeToFluidNC(filename, buttonEl, runAfterUpload = false) {{
      const ip = fluidncTargetIp;
      const targetSelect = document.getElementById('fluidnc-target-select');
      const target = targetSelect ? targetSelect.value : 'sd';

      if (target === 'sd' && !fluidncSdMounted) {{
        alert(
          `FluidNC at ${{ip}} reports 'No SD card' mounted in the TF/SD slot.\\n\\n` +
          `To upload and run files on FluidNC:\\n` +
          `1. Insert a FAT32 micro-SD card into the MKS-DLC32 board, OR\\n` +
          `2. Click '📋 Copy Path' and load the file into FluidNC FigUI, OR\\n` +
          `3. Click '👁️ View G-Code' to inspect the toolpath directly in this dashboard.`
        );
        return false;
      }}

      let origText = '';
      if (buttonEl) {{
        origText = buttonEl.innerHTML;
        buttonEl.disabled = true;
        buttonEl.innerHTML = `<svg class="animate-spin w-3 h-3 text-white inline" viewBox="0 0 24 24" fill="none" stroke="currentColor"><circle cx="12" cy="12" r="10" stroke-opacity="0.25"/><path d="M12 2a10 10 0 0 1 10 10" stroke-linecap="round"/></svg> ${{runAfterUpload ? 'Uploading & Starting...' : 'Uploading...'}}`;
      }}

      try {{
        const res = await fetch(`${{getApiBase()}}/api/send_existing_gcode`, {{
          method: 'POST',
          headers: {{ 'Content-Type': 'application/json' }},
          body: JSON.stringify({{ ip: ip, filename: filename, target: target, run: runAfterUpload }})
        }});
        const data = await res.json();
        if (data.success) {{
          fluidncSentFiles.add(filename);
          if (buttonEl) {{
            buttonEl.innerHTML = runAfterUpload ? '▶ Running on CNC' : `✓ Uploaded (/${{target}}/)`;
            buttonEl.className = 'px-2.5 py-1 rounded-lg bg-emerald-600 text-white font-medium text-xs flex items-center gap-1 shadow-sm';
            buttonEl.disabled = false;
          }}
          return true;
        }} else {{
          if (buttonEl) {{
            buttonEl.disabled = false;
            buttonEl.innerHTML = `⚠️ Retry`;
            buttonEl.className = 'px-2.5 py-1 rounded-lg bg-amber-600 hover:bg-amber-500 text-white font-medium text-xs flex items-center gap-1 shadow-sm';
          }}
          alert(`FluidNC upload failed for ${{filename}}: ${{data.message || data.error}}`);
          return false;
        }}
      }} catch (err) {{
        try {{
          const text = await fetchGCodeText(filename);
          const blob = new Blob([text], {{ type: 'application/octet-stream' }});
          const remoteName = `/${{filename}}`;
          const formData = new FormData();
          formData.append('path', '/');
          formData.append(`${{remoteName}}S`, String(blob.size));
          formData.append('myfile[]', blob, remoteName);
          const endpoint = (target === 'sd') ? `http://${{ip}}/upload` : `http://${{ip}}/files`;
          const direct = await fetch(endpoint, {{ method: 'POST', body: formData }});
          if (direct.ok) {{
            const respTxt = await direct.text();
            if (respTxt.includes('No SD card')) {{
              throw new Error('No SD card mounted on FluidNC');
            }}
            if (runAfterUpload) {{
              const runCmd = (target === 'sd') ? `$SD/Run=${{remoteName}}` : `$LocalFS/Run=${{remoteName}}`;
              await sendFluidNCRealtime(runCmd);
            }}
            fluidncSentFiles.add(filename);
            if (buttonEl) {{
              buttonEl.innerHTML = runAfterUpload ? '▶ Running on CNC' : `✓ Uploaded (/${{target}}/)`;
              buttonEl.className = 'px-2.5 py-1 rounded-lg bg-emerald-600 text-white font-medium text-xs flex items-center gap-1 shadow-sm';
              buttonEl.disabled = false;
            }}
            return true;
          }}
        }} catch (e2) {{}}

        if (buttonEl) {{
          buttonEl.disabled = false;
          buttonEl.innerHTML = origText || '⚠️ Error';
          buttonEl.className = 'px-2.5 py-1 rounded-lg bg-red-600 hover:bg-red-500 text-white font-medium text-xs flex items-center gap-1 shadow-sm';
        }}
        alert(`Cannot upload to FluidNC at ${{ip}}. Ensure the controller is online, a FAT32 micro-SD card is inserted, or run 'python3 gcode_paralel_manufacture.py --serve'.`);
        return false;
      }}
    }}

    // -------------------------------------------------------------------
    // Interactive Built-In G-Code Toolpath Viewer
    // -------------------------------------------------------------------
    const BED_TOTAL_W = 275.0;
    const BED_TOTAL_H = 170.0;
    const BED_MILL_W = 260.0;
    const BED_MILL_H = 170.0;

    let gcodeViewerState = {{
      filename: '',
      rawLines: [],
      segments: [],
      drills: [],
      bounds: {{ minX: 0, maxX: BED_TOTAL_W, minY: 0, maxY: BED_TOTAL_H, minZ: 0, maxZ: 5 }},
      stats: {{ cutDist: 0, rapidDist: 0, drillCount: 0 }},
      viewMode: '2d', // '2d' or '3d'
      view: {{ ox: 40, oy: 40, scale: 2.0 }},
      cam3d: {{ yaw: -0.65, pitch: 0.72, zoom: 2.4, panX: 0, panY: 0, zExag: 4.0 }},
      showRapids: true,
      showDrills: true,
      showBed: true,
      progressIdx: 0,
      simT: 1.0, // fractional interpolation along active segment [0..1]
      speedPct: 100,
      playing: false,
      rafId: null,
      lastFrameTs: 0,
      dragging: false,
      dragMode: 'pan', // 'pan' or 'orbit'
      lastMouse: {{ x: 0, y: 0 }}
    }};

    async function openGCodeViewer(filename) {{
      const modal = document.getElementById('gcode-viewer-modal');
      const titleEl = document.getElementById('gv-filename');
      const pathEl = document.getElementById('gv-filepath');
      const fallbackBanner = document.getElementById('gv-file-fallback');
      if (!modal) return;

      gcodeViewerState.filename = filename;
      if (titleEl) titleEl.innerText = filename;
      if (pathEl) pathEl.innerText = getFullGCodePath(filename);
      stopGCodePlayback();
      modal.classList.remove('hidden');
      document.body.style.overflow = 'hidden';

      try {{
        const text = await fetchGCodeText(filename);
        if (fallbackBanner) fallbackBanner.classList.add('hidden');
        loadGCodeIntoViewer(text, filename);
      }} catch (err) {{
        if (fallbackBanner) fallbackBanner.classList.remove('hidden');
        loadGCodeIntoViewer('; Could not auto-fetch via file:// CORS.\\n; Click "Load Local File" above or run: python3 gcode_paralel_manufacture.py --serve\\n', filename);
      }}
    }}

    function closeGCodeViewer() {{
      stopGCodePlayback();
      const modal = document.getElementById('gcode-viewer-modal');
      if (modal) modal.classList.add('hidden');
      document.body.style.overflow = '';
    }}

    function handleLocalGCodeFileSelect(inputEl) {{
      const file = inputEl.files && inputEl.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = (e) => {{
        const fallbackBanner = document.getElementById('gv-file-fallback');
        if (fallbackBanner) fallbackBanner.classList.add('hidden');
        gcodeViewerState.filename = file.name;
        const titleEl = document.getElementById('gv-filename');
        if (titleEl) titleEl.innerText = file.name;
        loadGCodeIntoViewer(e.target.result, file.name);
      }};
      reader.readAsText(file);
    }}

    function parseGCodeToolpath(text) {{
      const lines = text.split(/\\r?\\n/);
      const segments = [];
      const drills = [];
      let x = 0, y = 0, z = 5;
      let motion = 0; // 0=G0, 1=G1, 2=G2, 3=G3
      let feed = 300;
      let absMode = true;
      let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
      let cutDist = 0, rapidDist = 0;

      for (let i = 0; i < lines.length; i++) {{
        let raw = lines[i].trim();
        const stripped = raw.replace(/\\([^)]*\\)/g, '').split(';')[0].trim().toUpperCase();
        if (!stripped) continue;

        if (/\\bG90\\b/.test(stripped)) absMode = true;
        if (/\\bG91\\b/.test(stripped)) absMode = false;
        const gMatch = stripped.match(/\\bG0*([0-3])\\b/);
        if (gMatch) motion = parseInt(gMatch[1], 10);

        const fm = stripped.match(/F(\\d+(?:\\.\\d+)?)/);
        if (fm) feed = Math.max(10, parseFloat(fm[1]));

        const xm = stripped.match(/X(-?\\d+(?:\\.\\d+)?)/);
        const ym = stripped.match(/Y(-?\\d+(?:\\.\\d+)?)/);
        const zm = stripped.match(/Z(-?\\d+(?:\\.\\d+)?)/);

        const nx = xm ? (absMode ? parseFloat(xm[1]) : x + parseFloat(xm[1])) : x;
        const ny = ym ? (absMode ? parseFloat(ym[1]) : y + parseFloat(ym[1])) : y;
        const nz = zm ? (absMode ? parseFloat(zm[1]) : z + parseFloat(zm[1])) : z;

        const movedXY = (nx !== x || ny !== y);
        const movedZ = (nz !== z);

        if (movedXY || movedZ) {{
          const dist3d = Math.hypot(nx - x, ny - y, nz - z);
          const dist2d = Math.hypot(nx - x, ny - y);
          const isRapid = (motion === 0);
          if (isRapid) rapidDist += dist2d;
          else cutDist += dist2d;

          if (!movedXY && nz < z && nz < 0) {{
            drills.push({{ x: nx, y: ny, z: nz, zTop: z, lineIdx: i }});
          }}

          segments.push({{
            x1: x, y1: y, z1: z,
            x2: nx, y2: ny, z2: nz,
            rapid: isRapid || (!movedXY && nz > z),
            motion: motion,
            feed: isRapid ? 1500 : feed,
            len: Math.max(0.05, dist3d),
            lineIdx: i
          }});

          minX = Math.min(minX, x, nx);
          maxX = Math.max(maxX, x, nx);
          minY = Math.min(minY, y, ny);
          maxY = Math.max(maxY, y, ny);
          minZ = Math.min(minZ, z, nz);
          maxZ = Math.max(maxZ, z, nz);
          x = nx; y = ny; z = nz;
        }}
      }}

      if (!isFinite(minX)) {{ minX = 0; maxX = BED_TOTAL_W; minY = 0; maxY = BED_TOTAL_H; }}
      if (!isFinite(minZ)) {{ minZ = 0; maxZ = 5; }}
      return {{
        lines,
        segments,
        drills,
        bounds: {{ minX, maxX, minY, maxY, minZ, maxZ }},
        stats: {{ cutDist, rapidDist, drillCount: drills.length }}
      }};
    }}

    function loadGCodeIntoViewer(text, filename) {{
      const parsed = parseGCodeToolpath(text);
      gcodeViewerState.rawLines = parsed.lines;
      gcodeViewerState.segments = parsed.segments;
      gcodeViewerState.drills = parsed.drills;
      gcodeViewerState.bounds = parsed.bounds;
      gcodeViewerState.stats = parsed.stats;
      gcodeViewerState.progressIdx = parsed.segments.length;
      gcodeViewerState.simT = 1.0;

      const slider = document.getElementById('gv-progress-slider');
      if (slider) {{
        slider.max = Math.max(1, parsed.segments.length);
        slider.value = parsed.segments.length;
      }}

      const statsEl = document.getElementById('gv-stats');
      if (statsEl) {{
        const b = parsed.bounds;
        statsEl.innerHTML =
          `<span class="px-2 py-0.5 rounded bg-slate-800 text-slate-200">Lines: <b>${{parsed.lines.length}}</b></span>` +
          `<span class="px-2 py-0.5 rounded bg-emerald-950/60 text-emerald-300 border border-emerald-500/30">Cut: <b>${{parsed.stats.cutDist.toFixed(1)}} mm</b></span>` +
          `<span class="px-2 py-0.5 rounded bg-sky-950/60 text-sky-300 border border-sky-500/30">Rapid: <b>${{parsed.stats.rapidDist.toFixed(1)}} mm</b></span>` +
          `<span class="px-2 py-0.5 rounded bg-amber-950/60 text-amber-300 border border-amber-500/30">Plunges: <b>${{parsed.stats.drillCount}}</b></span>` +
          `<span class="px-2 py-0.5 rounded bg-indigo-950/60 text-indigo-300 border border-indigo-500/30">Mill Bed: <b>260&times;170</b> / Total: <b>275&times;170</b></span>` +
          `<span class="px-2 py-0.5 rounded bg-slate-800 text-slate-300">X:[${{b.minX.toFixed(1)}}..${{b.maxX.toFixed(1)}}] Y:[${{b.minY.toFixed(1)}}..${{b.maxY.toFixed(1)}}] Z:[${{b.minZ.toFixed(2)}}..${{b.maxZ.toFixed(2)}}]</span>`;
      }}

      const codeBox = document.getElementById('gv-source-code');
      if (codeBox) {{
        const previewLines = parsed.lines.slice(0, 600).map((l, idx) =>
          `<div id="gv-line-${{idx}}" class="flex gap-2 px-2 py-0.5 hover:bg-slate-800/70"><span class="text-slate-600 select-none w-10 text-right">${{idx + 1}}</span><span class="text-slate-200 whitespace-pre">${{l.replace(/</g, '&lt;')}}</span></div>`
        ).join('');
        const extra = parsed.lines.length > 600 ? `<div class="p-2 text-amber-400 italic">... showing first 600 of ${{parsed.lines.length}} lines</div>` : '';
        codeBox.innerHTML = previewLines + extra;
      }}

      setGCodeSpeedPct(gcodeViewerState.speedPct);
      fitGCodeView();
      updateSimTelemetry();
    }}

    function setGCodeViewMode(mode) {{
      gcodeViewerState.viewMode = mode === '3d' ? '3d' : '2d';
      const btn2d = document.getElementById('gv-mode-2d');
      const btn3d = document.getElementById('gv-mode-3d');
      const presetWrap = document.getElementById('gv-3d-presets');
      if (btn2d && btn3d) {{
        btn2d.className = gcodeViewerState.viewMode === '2d'
          ? 'px-2.5 py-1 rounded-l-lg bg-cyan-600 text-white font-semibold text-xs cursor-pointer'
          : 'px-2.5 py-1 rounded-l-lg bg-slate-800 text-slate-300 hover:text-white font-semibold text-xs cursor-pointer';
        btn3d.className = gcodeViewerState.viewMode === '3d'
          ? 'px-2.5 py-1 rounded-r-lg bg-cyan-600 text-white font-semibold text-xs cursor-pointer'
          : 'px-2.5 py-1 rounded-r-lg bg-slate-800 text-slate-300 hover:text-white font-semibold text-xs cursor-pointer';
      }}
      if (presetWrap) {{
        presetWrap.classList.toggle('hidden', gcodeViewerState.viewMode !== '3d');
      }}
      fitGCodeView();
    }}

    function setGCode3DPreset(preset) {{
      const c = gcodeViewerState.cam3d;
      if (preset === 'iso') {{ c.yaw = -0.65; c.pitch = 0.72; }}
      else if (preset === 'front') {{ c.yaw = 0.0; c.pitch = 0.25; }}
      else if (preset === 'side') {{ c.yaw = -Math.PI / 2; c.pitch = 0.3; }}
      c.panX = 0; c.panY = 0;
      renderGCodeCanvas();
    }}

    function setGCodeSpeedPct(pct) {{
      gcodeViewerState.speedPct = pct;
      document.querySelectorAll('.gv-speed-btn').forEach(btn => {{
        const val = parseInt(btn.dataset.pct, 10);
        btn.className = (val === pct)
          ? 'gv-speed-btn px-2 py-0.5 rounded bg-amber-500 text-slate-950 font-bold text-[11px] cursor-pointer shadow-sm'
          : 'gv-speed-btn px-2 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700 text-[11px] cursor-pointer';
      }});
    }}

    function fitGCodeView() {{
      const canvas = document.getElementById('gv-canvas');
      if (!canvas) return;
      const rect = canvas.getBoundingClientRect();
      canvas.width = Math.max(400, rect.width || 720);
      canvas.height = Math.max(320, rect.height || 460);

      const b = gcodeViewerState.bounds;
      const minX = Math.min(0, b.minX);
      const maxX = Math.max(BED_TOTAL_W, b.maxX);
      const minY = Math.min(0, b.minY);
      const maxY = Math.max(BED_TOTAL_H, b.maxY);

      if (gcodeViewerState.viewMode === '2d') {{
        const w = Math.max(10, maxX - minX);
        const h = Math.max(10, maxY - minY);
        const pad = 34;
        const scale = Math.min((canvas.width - 2 * pad) / w, (canvas.height - 2 * pad) / h);
        gcodeViewerState.view.scale = scale;
        gcodeViewerState.view.ox = (canvas.width - w * scale) / 2 - minX * scale;
        gcodeViewerState.view.oy = (canvas.height + h * scale) / 2 + minY * scale;
      }} else {{
        gcodeViewerState.cam3d.zoom = Math.min(canvas.width / 360, canvas.height / 260) * 1.15;
        gcodeViewerState.cam3d.panX = 0;
        gcodeViewerState.cam3d.panY = 0;
      }}
      renderGCodeCanvas();
    }}

    function project3D(x, y, z, w, h) {{
      const c = gcodeViewerState.cam3d;
      // Center around bed center (137.5, 85, 0)
      const cx = x - BED_TOTAL_W / 2;
      const cy = y - BED_TOTAL_H / 2;
      const cz = z * c.zExag;

      const cosY = Math.cos(c.yaw), sinY = Math.sin(c.yaw);
      const cosP = Math.cos(c.pitch), sinP = Math.sin(c.pitch);

      // Rotate around Z (yaw) then X (pitch)
      const rx = cx * cosY - cy * sinY;
      const ry = cx * sinY + cy * cosY;
      const rz = cz;

      const py = ry * cosP - rz * sinP;
      const pz = ry * sinP + rz * cosP;

      const fov = 900;
      const dist = fov / Math.max(120, fov - py);
      const sx = w / 2 + c.panX + rx * c.zoom * dist;
      const sy = h / 2 + c.panY - pz * c.zoom * dist;
      return {{ x: sx, y: sy, depth: py }};
    }}

    function getActiveToolPos() {{
      const segs = gcodeViewerState.segments;
      const idx = gcodeViewerState.progressIdx;
      if (!segs.length || idx <= 0) return {{ x: 0, y: 0, z: 5, feed: 0, motion: 0, lineIdx: 0 }};
      const clampedIdx = Math.min(segs.length, idx);
      const seg = segs[clampedIdx - 1];
      const t = Math.max(0, Math.min(1, gcodeViewerState.simT));
      return {{
        x: seg.x1 + (seg.x2 - seg.x1) * t,
        y: seg.y1 + (seg.y2 - seg.y1) * t,
        z: seg.z1 + (seg.z2 - seg.z1) * t,
        feed: seg.feed,
        motion: seg.motion,
        lineIdx: seg.lineIdx
      }};
    }}

    function updateSimTelemetry() {{
      const hud = document.getElementById('gv-sim-hud');
      if (!hud) return;
      const pos = getActiveToolPos();
      const segs = gcodeViewerState.segments;
      const pct = segs.length ? Math.round((gcodeViewerState.progressIdx / segs.length) * 100) : 0;
      hud.innerHTML =
        `<span>X: <b class="text-red-300">${{pos.x.toFixed(2)}}</b></span>` +
        `<span>Y: <b class="text-emerald-300">${{pos.y.toFixed(2)}}</b></span>` +
        `<span>Z: <b class="text-sky-300">${{pos.z.toFixed(2)}}</b></span>` +
        `<span>G${{pos.motion}} F${{pos.feed}}</span>` +
        `<span>Line: <b>${{pos.lineIdx + 1}}</b></span>` +
        `<span class="text-cyan-300 font-bold">${{pct}}%</span>`;
    }}

    function renderGCodeCanvas() {{
      const canvas = document.getElementById('gv-canvas');
      if (!canvas) return;
      if (gcodeViewerState.viewMode === '3d') {{
        renderGCodeCanvas3D(canvas);
      }} else {{
        renderGCodeCanvas2D(canvas);
      }}
      updateSimTelemetry();
    }}

    function renderGCodeCanvas2D(canvas) {{
      const ctx = canvas.getContext('2d');
      const {{ ox, oy, scale }} = gcodeViewerState.view;
      const w = canvas.width, h = canvas.height;

      ctx.fillStyle = '#070a10';
      ctx.fillRect(0, 0, w, h);

      const toX = x => ox + x * scale;
      const toY = y => oy - y * scale;

      // 1. Total Bed (0..275 x 0..170) & Milling Bed (0..260 x 0..170)
      if (gcodeViewerState.showBed) {{
        // Total bed background
        ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
        ctx.fillRect(toX(0), toY(BED_TOTAL_H), BED_TOTAL_W * scale, BED_TOTAL_H * scale);

        // Non-milling probe/reserve strip (X: 260..275, Y: 0..170)
        ctx.fillStyle = 'rgba(245, 158, 11, 0.10)';
        ctx.fillRect(toX(BED_MILL_W), toY(BED_TOTAL_H), (BED_TOTAL_W - BED_MILL_W) * scale, BED_TOTAL_H * scale);

        // Bed Grid (10mm minor, 50mm major) strictly on 0..275 x 0..170
        ctx.lineWidth = 1;
        for (let gx = 0; gx <= BED_TOTAL_W; gx += 10) {{
          ctx.strokeStyle = (gx % 50 === 0) ? 'rgba(56,189,248,0.22)' : 'rgba(148,163,184,0.08)';
          ctx.beginPath(); ctx.moveTo(toX(gx), toY(0)); ctx.lineTo(toX(gx), toY(BED_TOTAL_H)); ctx.stroke();
        }}
        for (let gy = 0; gy <= BED_TOTAL_H; gy += 10) {{
          ctx.strokeStyle = (gy % 50 === 0) ? 'rgba(56,189,248,0.22)' : 'rgba(148,163,184,0.08)';
          ctx.beginPath(); ctx.moveTo(toX(0), toY(gy)); ctx.lineTo(toX(BED_TOTAL_W), toY(gy)); ctx.stroke();
        }}

        // Total Bed Border (0..275 x 0..170)
        ctx.strokeStyle = '#94a3b8';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        ctx.strokeRect(toX(0), toY(BED_TOTAL_H), BED_TOTAL_W * scale, BED_TOTAL_H * scale);
        ctx.setLineDash([]);

        // Milling Bed Border (0..260 x 0..170)
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2;
        ctx.strokeRect(toX(0), toY(BED_MILL_H), BED_MILL_W * scale, BED_MILL_H * scale);

        // Bed Ruler Labels
        ctx.fillStyle = '#94a3b8';
        ctx.font = '10px monospace';
        [0, 50, 100, 150, 200, 250, 260, 275].forEach(gx => {{
          ctx.fillText(String(gx), toX(gx) - 6, toY(0) + 13);
        }});
        [0, 50, 100, 150, 170].forEach(gy => {{
          ctx.fillText(String(gy), toX(0) - 24, toY(gy) + 3);
        }});

        ctx.fillStyle = '#38bdf8';
        ctx.font = 'bold 11px monospace';
        ctx.fillText('Milling Bed: X 0..260mm, Y 0..170mm', toX(4), toY(BED_TOTAL_H) - 8);
        ctx.fillStyle = '#fbbf24';
        ctx.fillText('Total: 275x170', toX(BED_MILL_W - 20), toY(BED_TOTAL_H) - 8);
      }}

      // Origin (0,0) axes
      ctx.strokeStyle = 'rgba(239,68,68,0.85)';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(toX(0), toY(0)); ctx.lineTo(toX(18), toY(0)); ctx.stroke();
      ctx.strokeStyle = 'rgba(34,197,94,0.85)';
      ctx.beginPath(); ctx.moveTo(toX(0), toY(0)); ctx.lineTo(toX(0), toY(18)); ctx.stroke();

      const segs = gcodeViewerState.segments;
      const limit = Math.min(segs.length, gcodeViewerState.progressIdx);

      // Draw rapids
      if (gcodeViewerState.showRapids && limit > 0) {{
        ctx.strokeStyle = 'rgba(56,189,248,0.40)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        for (let i = 0; i < limit; i++) {{
          const s = segs[i];
          if (!s.rapid) continue;
          const endT = (i === limit - 1) ? gcodeViewerState.simT : 1.0;
          ctx.moveTo(toX(s.x1), toY(s.y1));
          ctx.lineTo(toX(s.x1 + (s.x2 - s.x1) * endT), toY(s.y1 + (s.y2 - s.y1) * endT));
        }}
        ctx.stroke();
        ctx.setLineDash([]);
      }}

      // Draw cuts
      if (limit > 0) {{
        ctx.strokeStyle = '#10b981';
        ctx.lineWidth = Math.max(1.3, Math.min(3.5, scale * 0.25));
        ctx.beginPath();
        for (let i = 0; i < limit; i++) {{
          const s = segs[i];
          if (s.rapid) continue;
          const endT = (i === limit - 1) ? gcodeViewerState.simT : 1.0;
          ctx.moveTo(toX(s.x1), toY(s.y1));
          ctx.lineTo(toX(s.x1 + (s.x2 - s.x1) * endT), toY(s.y1 + (s.y2 - s.y1) * endT));
        }}
        ctx.stroke();
      }}

      // Draw drill plunges
      if (gcodeViewerState.showDrills) {{
        ctx.fillStyle = '#f59e0b';
        const maxLine = limit > 0 ? segs[limit - 1].lineIdx : -1;
        for (const d of gcodeViewerState.drills) {{
          if (d.lineIdx > maxLine) continue;
          ctx.beginPath();
          ctx.arc(toX(d.x), toY(d.y), Math.max(2.2, Math.min(5, scale * 0.4)), 0, Math.PI * 2);
          ctx.fill();
        }}
      }}

      // Active simulated toolhead marker
      if (limit > 0 && limit <= segs.length) {{
        const pos = getActiveToolPos();
        ctx.fillStyle = pos.motion === 0 ? '#38bdf8' : '#22d3ee';
        ctx.beginPath();
        ctx.arc(toX(pos.x), toY(pos.y), 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.stroke();
      }}
    }}

    function renderGCodeCanvas3D(canvas) {{
      const ctx = canvas.getContext('2d');
      const w = canvas.width, h = canvas.height;
      ctx.fillStyle = '#070a10';
      ctx.fillRect(0, 0, w, h);

      const p = (x, y, z) => project3D(x, y, z, w, h);

      if (gcodeViewerState.showBed) {{
        // Total Bed Floor Polygon (0..275 x 0..170 at Z=0)
        const b0 = p(0, 0, 0), b1 = p(BED_TOTAL_W, 0, 0), b2 = p(BED_TOTAL_W, BED_TOTAL_H, 0), b3 = p(0, BED_TOTAL_H, 0);
        ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
        ctx.beginPath();
        ctx.moveTo(b0.x, b0.y); ctx.lineTo(b1.x, b1.y); ctx.lineTo(b2.x, b2.y); ctx.lineTo(b3.x, b3.y);
        ctx.closePath();
        ctx.fill();

        // Reserve Strip (260..275)
        const r0 = p(BED_MILL_W, 0, 0), r1 = p(BED_TOTAL_W, 0, 0), r2 = p(BED_TOTAL_W, BED_TOTAL_H, 0), r3 = p(BED_MILL_W, BED_TOTAL_H, 0);
        ctx.fillStyle = 'rgba(245, 158, 11, 0.12)';
        ctx.beginPath();
        ctx.moveTo(r0.x, r0.y); ctx.lineTo(r1.x, r1.y); ctx.lineTo(r2.x, r2.y); ctx.lineTo(r3.x, r3.y);
        ctx.closePath();
        ctx.fill();

        // 3D Bed Grid (10mm minor, 50mm major)
        ctx.lineWidth = 1;
        for (let gx = 0; gx <= BED_TOTAL_W; gx += 10) {{
          ctx.strokeStyle = (gx % 50 === 0) ? 'rgba(56,189,248,0.22)' : 'rgba(148,163,184,0.08)';
          const a = p(gx, 0, 0), b = p(gx, BED_TOTAL_H, 0);
          ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        }}
        for (let gy = 0; gy <= BED_TOTAL_H; gy += 10) {{
          ctx.strokeStyle = (gy % 50 === 0) ? 'rgba(56,189,248,0.22)' : 'rgba(148,163,184,0.08)';
          const a = p(0, gy, 0), b = p(BED_TOTAL_W, gy, 0);
          ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        }}

        // Total Bed Outline (dashed)
        ctx.strokeStyle = '#94a3b8';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        ctx.moveTo(b0.x, b0.y); ctx.lineTo(b1.x, b1.y); ctx.lineTo(b2.x, b2.y); ctx.lineTo(b3.x, b3.y);
        ctx.closePath(); ctx.stroke();
        ctx.setLineDash([]);

        // Milling Bed Outline (solid cyan)
        const m0 = p(0, 0, 0), m1 = p(BED_MILL_W, 0, 0), m2 = p(BED_MILL_W, BED_MILL_H, 0), m3 = p(0, BED_MILL_H, 0);
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(m0.x, m0.y); ctx.lineTo(m1.x, m1.y); ctx.lineTo(m2.x, m2.y); ctx.lineTo(m3.x, m3.y);
        ctx.closePath(); ctx.stroke();
      }}

      // 3D Origin Axes (X=red, Y=green, Z=blue)
      const o = p(0, 0, 0), ax = p(25, 0, 0), ay = p(0, 25, 0), az = p(0, 0, 15);
      ctx.lineWidth = 2.2;
      ctx.strokeStyle = '#ef4444'; ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(ax.x, ax.y); ctx.stroke();
      ctx.strokeStyle = '#22c55e'; ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(ay.x, ay.y); ctx.stroke();
      ctx.strokeStyle = '#3b82f6'; ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(az.x, az.y); ctx.stroke();

      const segs = gcodeViewerState.segments;
      const limit = Math.min(segs.length, gcodeViewerState.progressIdx);

      // Draw 3D rapids
      if (gcodeViewerState.showRapids && limit > 0) {{
        ctx.strokeStyle = 'rgba(56,189,248,0.40)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        for (let i = 0; i < limit; i++) {{
          const s = segs[i];
          if (!s.rapid) continue;
          const endT = (i === limit - 1) ? gcodeViewerState.simT : 1.0;
          const a = p(s.x1, s.y1, s.z1);
          const b = p(s.x1 + (s.x2 - s.x1) * endT, s.y1 + (s.y2 - s.y1) * endT, s.z1 + (s.z2 - s.z1) * endT);
          ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
        }}
        ctx.stroke();
        ctx.setLineDash([]);
      }}

      // Draw 3D cuts
      if (limit > 0) {{
        ctx.strokeStyle = '#10b981';
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        for (let i = 0; i < limit; i++) {{
          const s = segs[i];
          if (s.rapid) continue;
          const endT = (i === limit - 1) ? gcodeViewerState.simT : 1.0;
          const a = p(s.x1, s.y1, s.z1);
          const b = p(s.x1 + (s.x2 - s.x1) * endT, s.y1 + (s.y2 - s.y1) * endT, s.z1 + (s.z2 - s.z1) * endT);
          ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
        }}
        ctx.stroke();
      }}

      // Draw 3D drill plunge dots
      if (gcodeViewerState.showDrills) {{
        ctx.fillStyle = '#f59e0b';
        const maxLine = limit > 0 ? segs[limit - 1].lineIdx : -1;
        for (const d of gcodeViewerState.drills) {{
          if (d.lineIdx > maxLine) continue;
          const pt = p(d.x, d.y, d.z);
          ctx.beginPath();
          ctx.arc(pt.x, pt.y, 3, 0, Math.PI * 2);
          ctx.fill();
        }}
      }}

      // Draw 3D Spindle Cone at active simulated position
      if (limit > 0 && limit <= segs.length) {{
        const pos = getActiveToolPos();
        const tip = p(pos.x, pos.y, pos.z);
        const top = p(pos.x, pos.y, pos.z + 8);
        ctx.strokeStyle = '#22d3ee';
        ctx.lineWidth = 3;
        ctx.beginPath(); ctx.moveTo(tip.x, tip.y); ctx.lineTo(top.x, top.y); ctx.stroke();
        ctx.fillStyle = '#ffffff';
        ctx.beginPath(); ctx.arc(tip.x, tip.y, 4, 0, Math.PI * 2); ctx.fill();
      }}
    }}

    function toggleGCodeRapids(el) {{
      gcodeViewerState.showRapids = !!el.checked;
      renderGCodeCanvas();
    }}

    function toggleGCodeDrills(el) {{
      gcodeViewerState.showDrills = !!el.checked;
      renderGCodeCanvas();
    }}

    function toggleGCodeBed(el) {{
      gcodeViewerState.showBed = !!el.checked;
      renderGCodeCanvas();
    }}

    function scrubGCodeProgress(val) {{
      gcodeViewerState.progressIdx = parseInt(val, 10) || 0;
      gcodeViewerState.simT = 1.0;
      renderGCodeCanvas();
    }}

    function resetGCodeSimulation() {{
      stopGCodePlayback();
      gcodeViewerState.progressIdx = 0;
      gcodeViewerState.simT = 0.0;
      const slider = document.getElementById('gv-progress-slider');
      if (slider) slider.value = 0;
      renderGCodeCanvas();
    }}

    function toggleGCodePlayback() {{
      const btn = document.getElementById('gv-play-btn');
      if (gcodeViewerState.playing) {{
        stopGCodePlayback();
        return;
      }}
      const segs = gcodeViewerState.segments;
      if (!segs.length) return;
      if (gcodeViewerState.progressIdx >= segs.length) {{
        gcodeViewerState.progressIdx = 1;
        gcodeViewerState.simT = 0.0;
      }} else if (gcodeViewerState.progressIdx === 0) {{
        gcodeViewerState.progressIdx = 1;
        gcodeViewerState.simT = 0.0;
      }}

      gcodeViewerState.playing = true;
      gcodeViewerState.lastFrameTs = performance.now();
      if (btn) btn.innerText = '⏸ Pause';

      function stepSim(now) {{
        if (!gcodeViewerState.playing) return;
        const dt = Math.min(0.1, Math.max(0.001, (now - gcodeViewerState.lastFrameTs) / 1000.0));
        gcodeViewerState.lastFrameTs = now;

        const speedMult = gcodeViewerState.speedPct / 100.0;
        let idx = gcodeViewerState.progressIdx;
        let t = gcodeViewerState.simT;
        // Advance distance based on segment feedrate (mm/min -> mm/sec) * speedPct
        let budget = dt * speedMult;

        while (budget > 0 && idx <= segs.length) {{
          const seg = segs[idx - 1];
          const mmPerSec = Math.max(5.0, (seg.feed || 300) / 60.0);
          const segTime = seg.len / mmPerSec;
          const remTime = (1.0 - t) * segTime;
          if (budget >= remTime) {{
            budget -= remTime;
            if (idx < segs.length) {{
              idx++;
              t = 0.0;
            }} else {{
              t = 1.0;
              budget = 0;
              gcodeViewerState.progressIdx = segs.length;
              gcodeViewerState.simT = 1.0;
              renderGCodeCanvas();
              stopGCodePlayback();
              return;
            }}
          }} else {{
            t += budget / segTime;
            budget = 0;
          }}
        }}

        gcodeViewerState.progressIdx = idx;
        gcodeViewerState.simT = t;
        const slider = document.getElementById('gv-progress-slider');
        if (slider) slider.value = idx;
        renderGCodeCanvas();
        gcodeViewerState.rafId = requestAnimationFrame(stepSim);
      }}

      gcodeViewerState.rafId = requestAnimationFrame(stepSim);
    }}

    function stopGCodePlayback() {{
      gcodeViewerState.playing = false;
      if (gcodeViewerState.rafId) {{
        cancelAnimationFrame(gcodeViewerState.rafId);
        gcodeViewerState.rafId = null;
      }}
      const btn = document.getElementById('gv-play-btn');
      if (btn) btn.innerText = '▶ Simulate';
    }}

    function initGCodeCanvasEvents() {{
      const canvas = document.getElementById('gv-canvas');
      if (!canvas || canvas.dataset.bound) return;
      canvas.dataset.bound = '1';

      canvas.addEventListener('contextmenu', e => e.preventDefault());

      canvas.addEventListener('wheel', (e) => {{
        e.preventDefault();
        const factor = e.deltaY < 0 ? 1.15 : 0.87;
        if (gcodeViewerState.viewMode === '3d') {{
          gcodeViewerState.cam3d.zoom = Math.max(0.3, Math.min(25, gcodeViewerState.cam3d.zoom * factor));
          renderGCodeCanvas();
          return;
        }}
        const rect = canvas.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const oldScale = gcodeViewerState.view.scale;
        const newScale = Math.max(0.2, Math.min(50, oldScale * factor));
        gcodeViewerState.view.ox = mx - (mx - gcodeViewerState.view.ox) * (newScale / oldScale);
        gcodeViewerState.view.oy = my - (my - gcodeViewerState.view.oy) * (newScale / oldScale);
        gcodeViewerState.view.scale = newScale;
        renderGCodeCanvas();
      }}, {{ passive: false }});

      canvas.addEventListener('mousedown', (e) => {{
        gcodeViewerState.dragging = true;
        gcodeViewerState.dragMode = (gcodeViewerState.viewMode === '3d' && e.button === 0 && !e.shiftKey) ? 'orbit' : 'pan';
        gcodeViewerState.lastMouse = {{ x: e.clientX, y: e.clientY }};
      }});
      window.addEventListener('mousemove', (e) => {{
        if (!gcodeViewerState.dragging) return;
        const dx = e.clientX - gcodeViewerState.lastMouse.x;
        const dy = e.clientY - gcodeViewerState.lastMouse.y;
        gcodeViewerState.lastMouse = {{ x: e.clientX, y: e.clientY }};
        if (gcodeViewerState.viewMode === '3d' && gcodeViewerState.dragMode === 'orbit') {{
          gcodeViewerState.cam3d.yaw += dx * 0.008;
          gcodeViewerState.cam3d.pitch = Math.max(0.05, Math.min(Math.PI / 2 - 0.02, gcodeViewerState.cam3d.pitch + dy * 0.008));
        }} else if (gcodeViewerState.viewMode === '3d') {{
          gcodeViewerState.cam3d.panX += dx;
          gcodeViewerState.cam3d.panY += dy;
        }} else {{
          gcodeViewerState.view.ox += dx;
          gcodeViewerState.view.oy += dy;
        }}
        renderGCodeCanvas();
      }});
      window.addEventListener('mouseup', () => {{ gcodeViewerState.dragging = false; }});
    }}

    async function uploadAllSheetGCodeToFluidNC() {{
      const targetOps = getFilteredDispatcherOps();
      if (!targetOps.length) return;

      const progressWrap = document.getElementById('fluidnc-batch-progress-wrap');
      const progressBar = document.getElementById('fluidnc-batch-progress-bar');
      const progressPct = document.getElementById('fluidnc-batch-pct');
      const statusText = document.getElementById('fluidnc-batch-status-text');
      const batchBtn = document.getElementById('btn-batch-upload');

      progressWrap.classList.remove('hidden');
      batchBtn.disabled = true;
      batchBtn.classList.add('opacity-50');

      let successCount = 0;
      for (let i = 0; i < targetOps.length; i++) {{
        const op = targetOps[i];
        const pct = Math.round(((i + 1) / targetOps.length) * 100);
        progressBar.style.width = pct + '%';
        progressPct.innerText = pct + '%';
        statusText.innerHTML = `<span class="flex items-center gap-2"><svg class="animate-spin w-3.5 h-3.5 text-emerald-400" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><circle cx="12" cy="12" r="10" stroke-opacity="0.25"/><path d="M12 2a10 10 0 0 1 10 10" stroke-linecap="round"/></svg>Uploading file ${{i + 1}} of ${{targetOps.length}}: <strong class="font-mono text-white">${{op.gcode_file}}</strong></span>`;

        const btn = document.getElementById(`btn-send-${{op.sheet_num}}-${{op.seq}}`);
        const ok = await sendGCodeToFluidNC(op.gcode_file, btn);
        if (ok) successCount++;
      }}

      batchBtn.disabled = false;
      batchBtn.classList.remove('opacity-50');
      statusText.innerHTML = `<span class="text-emerald-400 font-bold flex items-center gap-1.5"><svg class="w-4 h-4" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><polyline points="20 6 9 17 4 12"/></svg>Batch Upload Complete: Successfully sent ${{successCount}} of ${{targetOps.length}} files to FluidNC SD Card!</span>`;
      renderFluidNCDispatcher();
    }}

    function getFilteredDispatcherOps() {{
      let ops = [];
      if (fluidncDispatcherSheet === 'ALL') {{
        Object.values(bedOperationsBySheet).forEach(list => {{ ops = ops.concat(list); }});
      }} else {{
        ops = bedOperationsBySheet[fluidncDispatcherSheet] || [];
      }}

      return ops.filter(op => {{
        if (fluidncDispatcherCat !== 'ALL') {{
          if (!op.step_code.startsWith(fluidncDispatcherCat)) return false;
        }}
        if (fluidncDispatcherSearch) {{
          const q = fluidncDispatcherSearch.toLowerCase();
          const matchName = op.gcode_file.toLowerCase().includes(q);
          const matchTool = op.tool_name.toLowerCase().includes(q);
          const matchTitle = op.step_title.toLowerCase().includes(q);
          if (!matchName && !matchTool && !matchTitle) return false;
        }}
        return true;
      }});
    }}

    function setFluidNCDispatcherSheet(sheetNum) {{
      fluidncDispatcherSheet = sheetNum;
      document.querySelectorAll('.fnc-sheet-btn').forEach(btn => {{
        const active = (btn.id === `btn-sheet-${{sheetNum}}`);
        btn.className = active
          ? 'fnc-sheet-btn active px-3 py-1.5 rounded-lg font-semibold transition cursor-pointer bg-blue-600 text-white shadow-sm'
          : 'fnc-sheet-btn px-3 py-1.5 rounded-lg font-semibold transition cursor-pointer bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70';
      }});

      const batchLbl = document.getElementById('fluidnc-batch-sheet-lbl');
      if (batchLbl) {{
        const count = (sheetNum === 'ALL')
          ? Object.values(bedOperationsBySheet).reduce((acc, l) => acc + l.length, 0)
          : (bedOperationsBySheet[sheetNum] || []).length;
        batchLbl.innerText = sheetNum === 'ALL' ? `All Sheets (${{count}} files)` : `Sheet ${{sheetNum}} (${{count}} files)`;
      }}

      renderFluidNCDispatcher();
    }}

    function setFluidNCDispatcherCat(cat) {{
      fluidncDispatcherCat = cat;
      document.querySelectorAll('.fnc-cat-btn').forEach(btn => {{
        const active = (btn.innerText.toLowerCase().includes(cat.toLowerCase()) || (cat === 'ALL' && btn.innerText === 'All'));
        btn.className = active
          ? 'fnc-cat-btn px-2 py-1 rounded bg-slate-800 text-blue-300 border border-blue-500/30'
          : 'fnc-cat-btn px-2 py-1 rounded bg-slate-900 text-slate-400 hover:text-white';
      }});
      renderFluidNCDispatcher();
    }}

    function filterFluidNCSearch(q) {{
      fluidncDispatcherSearch = (q || '').trim();
      renderFluidNCDispatcher();
    }}

    function renderFluidNCDispatcher() {{
      const container = document.getElementById('fluidnc-dispatcher-table');
      if (!container) return;

      const ops = getFilteredDispatcherOps();
      if (!ops.length) {{
        container.innerHTML = `<div class="p-8 text-center text-slate-500 text-xs">No matching G-code operations found for current filters.</div>`;
        return;
      }}

      let html = '';
      ops.forEach(op => {{
        const isSent = fluidncSentFiles.has(op.gcode_file);
        const btnClass = isSent
          ? 'px-2.5 py-1 rounded-lg bg-emerald-600 text-white font-medium text-xs flex items-center gap-1 shadow-sm transition'
          : 'px-2.5 py-1 rounded-lg bg-blue-600 hover:bg-blue-500 text-white font-medium text-xs flex items-center gap-1 shadow-sm transition cursor-pointer';
        const btnText = isSent ? '✓ Sent to /sd/' : '⚡ Send to FluidNC';

        const halfBadge = op.bed_half === 'front'
          ? '<span class="px-1.5 py-0.5 text-[10px] font-mono rounded bg-emerald-500/15 text-emerald-300 border border-emerald-500/30">Front Half</span>'
          : '<span class="px-1.5 py-0.5 text-[10px] font-mono rounded bg-amber-500/15 text-amber-300 border border-amber-500/30">Back Half (-145mm)</span>';

        const mirrorBadge = op.mirrored
          ? '<span class="px-1.5 py-0.5 text-[10px] font-mono rounded bg-violet-500/20 text-violet-300 border border-violet-500/40">Mirrored X</span>'
          : '';

        html += `
          <div class="glass-panel p-2.5 sm:p-3 rounded-xl border border-slate-800/90 hover:border-slate-700/80 transition flex flex-col md:flex-row items-start md:items-center justify-between gap-3 text-xs">
            <div class="flex items-center gap-2.5 min-w-0">
              <!-- Draggable Handle -->
              <div draggable="true"
                   ondragstart="onGCodeDragStart(event, '${{op.gcode_file}}')"
                   class="cursor-grab active:cursor-grabbing px-2 py-1 rounded-lg bg-slate-800/90 hover:bg-blue-600/20 text-slate-300 hover:text-blue-300 border border-slate-700/80 hover:border-blue-500/40 font-mono text-[11px] inline-flex items-center gap-1.5 select-none shrink-0"
                   title="Click and drag directly into FluidNC WebUI dropzone or another browser tab">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><circle cx="9" cy="5" r="1"/><circle cx="9" cy="12" r="1"/><circle cx="9" cy="19" r="1"/><circle cx="15" cy="5" r="1"/><circle cx="15" cy="12" r="1"/><circle cx="15" cy="19" r="1"/></svg>
                <span>Drag to CNC</span>
              </div>

              <div class="min-w-0">
                <div class="flex items-center gap-2 flex-wrap">
                  <a href="${{getApiBase()}}/manufacture_gcode/${{op.gcode_file}}" download="${{op.gcode_file}}"
                     class="font-mono font-semibold text-slate-100 hover:text-blue-300 transition truncate max-w-xs sm:max-w-md block"
                     title="Click to download ${{op.gcode_file}}">
                    ${{op.gcode_file}}
                  </a>
                  <span class="text-[10px] font-mono px-1.5 py-0.2 rounded bg-slate-800 text-slate-400">Sheet ${{op.sheet_num}} &bull; Op #${{String(op.seq).padStart(2, '0')}}</span>
                  ${{halfBadge}} ${{mirrorBadge}}
                </div>
                <div class="text-[11px] text-slate-400 mt-0.5 flex items-center gap-2 flex-wrap">
                  <span class="text-slate-300 font-medium">${{op.step_title}}</span>
                  <span>&bull;</span>
                  <span class="text-amber-300">${{op.tool_name}}</span>
                  <span>&bull;</span>
                  <span class="font-mono text-slate-400">Z=${{op.depth_mm.toFixed(2)}}mm, F${{op.feed_mm_min}}</span>
                </div>
              </div>
            </div>

            <div class="flex items-center gap-1.5 flex-wrap shrink-0 self-end md:self-auto">
              <button type="button"
                      onclick="openGCodeViewer('${{op.gcode_file}}')"
                      class="px-2.5 py-1 rounded-lg bg-cyan-600/90 hover:bg-cyan-500 text-white font-medium text-xs flex items-center gap-1 shadow-sm transition cursor-pointer"
                      title="Open interactive G-code toolpath viewer">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/><circle cx="12" cy="12" r="3"/></svg>
                <span>View G-Code</span>
              </button>
              <button type="button"
                      onclick="copyGCodePath('${{op.gcode_file}}', this)"
                      class="px-2.5 py-1 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 font-medium text-xs flex items-center gap-1 transition cursor-pointer"
                      title="Copy full file path to clipboard">
                <svg width="11" height="11" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="9" y="9" width="13" height="13" rx="2" ry="2"/><path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"/></svg>
                <span>Copy Path</span>
              </button>
              <button type="button" id="btn-send-${{op.sheet_num}}-${{op.seq}}"
                      onclick="sendGCodeToFluidNC('${{op.gcode_file}}', this, false)"
                      class="${{btnClass}}"
                      title="Upload file to FluidNC storage">
                <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/></svg>
                <span>${{btnText}}</span>
              </button>
              <button type="button"
                      onclick="sendGCodeToFluidNC('${{op.gcode_file}}', this, true)"
                      class="px-2.5 py-1 rounded-lg bg-emerald-600 hover:bg-emerald-500 text-white font-medium text-xs flex items-center gap-1 shadow-sm transition cursor-pointer"
                      title="Upload to FluidNC and immediately run CNC job ($SD/Run)">
                <svg width="11" height="11" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><polygon points="5 3 19 12 5 21 5 3"/></svg>
                <span>Upload &amp; Run</span>
              </button>
              <a href="${{getApiBase()}}/manufacture_gcode/${{op.gcode_file}}" download="${{op.gcode_file}}"
                 class="px-2 py-1 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-300 hover:text-white border border-slate-700 font-medium text-xs transition flex items-center gap-1"
                 title="Download ${{op.gcode_file}}">
                <svg width="11" height="11" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="7 10 12 15 17 10"/><line x1="12" y1="15" x2="12" y2="3"/></svg>
                <span>DL</span>
              </a>
            </div>
          </div>
        `;
      }});

      container.innerHTML = html;
    }}

    function initFluidNCDispatcher() {{
      try {{
        const savedIp = localStorage.getItem('fluidnc_ip');
        if (savedIp) {{
          const ipInput = document.getElementById('fluidnc-ip-input');
          if (ipInput) ipInput.value = savedIp;
          fluidncTargetIp = savedIp;
          const link = document.getElementById('fluidnc-open-link');
          if (link) link.href = `http://${{fluidncTargetIp}}`;
          const frameLink = document.getElementById('fluidnc-frame-external-link');
          if (frameLink) frameLink.href = `http://${{fluidncTargetIp}}`;
          const ipLabel = document.getElementById('fluidnc-embedded-ip-label');
          if (ipLabel) ipLabel.innerText = fluidncTargetIp;
        }}
      }} catch (e) {{}}

      testFluidNCConnection();
      renderFluidNCDispatcher();
    }}

    window.addEventListener('DOMContentLoaded', function() {{
      updateCircuitView();
      filterUnitSubmenu('ALL');
      selectBedSheet(1);
      updateChecklistProgress();
      initFluidNCDispatcher();
      initGCodeCanvasEvents();
    }});
  </script>
</head>
<body class="min-h-screen antialiased selection:bg-blue-500/30 selection:text-white">
  <!-- Sticky Navigation Header -->
  <header class="sticky top-0 z-30 border-b border-slate-800/80 bg-[#090b10]/90 backdrop-blur-md">
    <div class="max-w-7xl mx-auto px-4 sm:px-6 py-3.5 flex flex-wrap items-center justify-between gap-4">
      <div class="flex items-center gap-3">
        <div class="w-9 h-9 rounded-xl bg-gradient-to-br from-blue-500 to-cyan-500 flex items-center justify-center shadow-lg shadow-blue-500/20" aria-hidden="true">
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="white" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <rect x="4" y="4" width="16" height="16" rx="2"/><circle cx="9" cy="9" r="1.5"/><circle cx="15" cy="15" r="1.5"/>
            <path d="M9 10.5V15h4.5"/><path d="M2 9h2m-2 6h2m16-6h2m-2 6h2M9 2v2m6-2v2M9 20v2m6-2v2"/>
          </svg>
        </div>
        <div>
          <h1 class="text-base sm:text-lg font-bold tracking-tight text-slate-100">PCB CAM &amp; Dual-Sided CNC Inspector</h1>
          <p class="text-xs text-slate-400">Bed {bed_x:.0f}&times;{bed_y:.0f} mm &bull; Y-Shift {y_split:.0f} mm &bull; Mode: <span class="text-amber-300 font-medium">{svg_escape(breakaway_cfg.label)}</span></p>
        </div>
      </div>

      <nav class="flex flex-wrap items-center gap-1.5 text-xs" aria-label="Dashboard Sections">
        <a href="#section-fluidnc-upload" class="px-2.5 py-1.5 rounded-lg text-emerald-300 hover:text-white hover:bg-emerald-950/60 border border-emerald-500/30 transition font-medium flex items-center gap-1.5">
          <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/>
          </svg>
          <span>⚡ Send to FluidNC</span>
        </a>
        <a href="#section-gerber" class="px-2.5 py-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800/80 transition font-medium">1. Gerber Steps</a>
        <a href="#section-units" class="px-2.5 py-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800/80 transition font-medium">2. Unit View</a>
        <a href="#section-sheets" class="px-2.5 py-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800/80 transition font-medium">3. Sheets</a>
        <a href="#section-cncbed" class="px-2.5 py-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800/80 transition font-medium">4. CNC Bed</a>
        <a href="#section-checklist" class="px-2.5 py-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800/80 transition font-medium">5. Checklist</a>
        <a href="http://{svg_escape(fluidnc_ip)}" target="_blank" rel="noopener noreferrer"
           class="ml-1 inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg bg-blue-600 hover:bg-blue-500 text-white font-semibold transition">
          <span class="w-1.5 h-1.5 rounded-full bg-emerald-300 animate-pulse" aria-hidden="true"></span>
          <span>FluidNC ({svg_escape(fluidnc_ip)})</span>
        </a>
      </nav>
    </div>
        <a href="#section-gerber" class="px-2.5 py-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800/80 transition font-medium">1. Gerber Steps</a>
        <a href="#section-units" class="px-2.5 py-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800/80 transition font-medium">2. Unit View</a>
        <a href="#section-sheets" class="px-2.5 py-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800/80 transition font-medium">3. Sheets</a>
        <a href="#section-cncbed" class="px-2.5 py-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800/80 transition font-medium">4. CNC Bed</a>
        <a href="#section-checklist" class="px-2.5 py-1.5 rounded-lg text-slate-300 hover:text-white hover:bg-slate-800/80 transition font-medium">5. Checklist</a>
        <a href="http://{svg_escape(fluidnc_ip)}" target="_blank" rel="noopener noreferrer"
           class="ml-1 inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg bg-blue-600 hover:bg-blue-500 text-white font-semibold transition">
          <span class="w-1.5 h-1.5 rounded-full bg-emerald-300 animate-pulse" aria-hidden="true"></span>
          <span>FluidNC ({svg_escape(fluidnc_ip)})</span>
        </a>
      </nav>
    </div>
  </header>

  <main class="max-w-7xl mx-auto px-4 sm:px-6 py-8 space-y-10">
    <!-- Top Telemetry Summary Strip -->
    <div class="grid grid-cols-2 sm:grid-cols-4 gap-3">
      <div class="glass-panel rounded-xl px-4 py-3">
        <div class="text-[11px] uppercase tracking-wider text-slate-400">Manufacturing Sheets</div>
        <div class="text-lg font-bold font-mono text-slate-100 mt-0.5">{len(sheets)} A4 Sheets</div>
      </div>
      <div class="glass-panel rounded-xl px-4 py-3">
        <div class="text-[11px] uppercase tracking-wider text-slate-400">Batch Output</div>
        <div class="text-lg font-bold font-mono text-emerald-400 mt-0.5">{total_units} PCB Units</div>
      </div>
      <div class="glass-panel rounded-xl px-4 py-3">
        <div class="text-[11px] uppercase tracking-wider text-slate-400">Constrained Bed</div>
        <div class="text-lg font-bold font-mono text-sky-400 mt-0.5">{bed_x:.0f} &times; {bed_y:.0f} mm</div>
      </div>
      <div class="glass-panel rounded-xl px-4 py-3">
        <div class="text-[11px] uppercase tracking-wider text-slate-400">Common Bed Washer Clamps</div>
        <div class="text-sm font-bold font-mono text-amber-300 mt-1">{len(holders_mgr.holders)} Single-Half Washers</div>
        <div class="text-[11px] text-slate-400 mt-0.5">No Through-PCB Drills &bull; Dowel Lock: <span class="text-slate-200">{'ENABLED' if enable_dowel_pin_lock else 'DISABLED'}</span></div>
      </div>
    </div>

    <!-- Bed Limit Switch Danger Zone Alert Bar -->
    <div class="rounded-xl border border-red-500/40 bg-red-950/30 px-4 py-3 flex flex-wrap items-center justify-between gap-3">
      <div class="flex items-center gap-3">
        <span class="inline-flex items-center justify-center w-8 h-8 rounded-lg bg-red-500/20 border border-red-500/50 text-red-300 shrink-0" aria-hidden="true">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M10.29 3.86 1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/>
          </svg>
        </span>
        <div>
          <div class="text-xs sm:text-sm font-bold text-red-200">BED Z-ZERO LIMIT SWITCH DANGER ZONE &mdash; X = {z_switch_x:.3f} mm, Y = {z_switch_y:.3f} mm</div>
          <p class="text-[11px] sm:text-xs text-red-200/80">Keep A4 copper stock, M4 washer clamps, and non-probing rapid tool moves strictly clear of this bed switch zone. Only jog here intentionally during tool-change Z-Zero calibration.</p>
        </div>
      </div>
      <span class="text-[11px] font-mono px-2.5 py-1 rounded-md bg-red-500/20 text-red-200 border border-red-500/40 shrink-0">DANGER KEEP-OUT</span>
    </div>

    <!-- FLUIDNC G-CODE DISPATCHER & WIRELESS TRANSFER STATION -->
    <section id="section-fluidnc-upload" class="glass-panel p-5 sm:p-6 rounded-2xl scroll-mt-20 border border-blue-500/40 relative overflow-hidden shadow-2xl shadow-blue-950/20">
      <!-- Glow ambient background -->
      <div class="absolute -top-24 -right-24 w-80 h-80 bg-blue-600/10 rounded-full blur-3xl pointer-events-none" aria-hidden="true"></div>
      <div class="absolute -bottom-24 -left-24 w-80 h-80 bg-emerald-600/10 rounded-full blur-3xl pointer-events-none" aria-hidden="true"></div>

      <div class="relative z-10 flex flex-col md:flex-row items-start md:items-center justify-between pb-4 mb-5 border-b border-slate-800/80 gap-4">
        <div>
          <div class="flex items-center gap-2 mb-1">
            <span class="inline-flex items-center justify-center w-7 h-7 rounded-lg bg-blue-500/20 border border-blue-500/40 text-blue-400 font-mono text-xs font-bold">CNC</span>
            <h2 class="text-lg sm:text-xl font-bold text-slate-100 flex items-center gap-2">
              <span>FluidNC G-Code Dispatcher &amp; Wireless Transfer Station</span>
            </h2>
            <span class="text-[10px] font-mono bg-emerald-500/15 text-emerald-300 border border-emerald-500/30 px-2 py-0.5 rounded-md font-semibold">WEB &rarr; FLUIDNC</span>
          </div>
          <p class="text-xs text-slate-400">
            Send generated PCB G-codes from this dashboard directly into your FluidNC Web UI. Use <strong>1-Click Wireless Push</strong>, <strong>Batch Sheet Upload</strong>, or <strong>Drag-and-Drop</strong> file chips directly into FluidNC.
          </p>
        </div>

        <!-- Controller Status & Action Pills -->
        <div class="flex flex-wrap items-center gap-2 shrink-0">
          <div id="fluidnc-status-badge" class="inline-flex items-center gap-2 px-3 py-1.5 rounded-lg bg-slate-800/90 border border-slate-700/80 text-xs font-mono text-slate-300">
            <span id="fluidnc-status-dot" class="w-2 h-2 rounded-full bg-slate-500"></span>
            <span id="fluidnc-status-text">Checking controller...</span>
          </div>
          <button type="button" onclick="testFluidNCConnection()" class="px-2.5 py-1.5 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-200 text-xs font-semibold border border-slate-700 transition cursor-pointer flex items-center gap-1">
            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M21.5 2v6h-6M2.5 22v-6h6M2 11.5a10 10 0 0 1 18.8-4.3M22 12.5a10 10 0 0 1-18.8 4.2"/></svg>
            <span>Ping</span>
          </button>
          <button type="button" onclick="sendFluidNCRealtime('$X')" class="px-2.5 py-1.5 rounded-lg bg-amber-600/80 hover:bg-amber-500 text-white text-xs font-semibold transition cursor-pointer flex items-center gap-1" title="Unlock Alarm ($X)">
            <span>🔓 $X</span>
          </button>
          <button type="button" onclick="sendFluidNCRealtime('$H')" class="px-2.5 py-1.5 rounded-lg bg-sky-600/80 hover:bg-sky-500 text-white text-xs font-semibold transition cursor-pointer flex items-center gap-1" title="Run Homing Cycle ($H)">
            <span>🏠 $H</span>
          </button>
          <button type="button" onclick="sendFluidNCRealtime('G10 L20 P1 X0 Y0 Z0')" class="px-2.5 py-1.5 rounded-lg bg-purple-600/80 hover:bg-purple-500 text-white text-xs font-semibold transition cursor-pointer flex items-center gap-1" title="Zero Work Coordinates (G10 L20 P1 X0 Y0 Z0)">
            <span>🎯 Zero Work</span>
          </button>
          <button type="button" onclick="toggleFluidNCFrame()" id="btn-toggle-fluidnc-frame" class="px-3 py-1.5 rounded-lg bg-indigo-600/90 hover:bg-indigo-500 text-white text-xs font-semibold transition cursor-pointer flex items-center gap-1.5 shadow-sm">
            <svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="2" y="3" width="20" height="14" rx="2" ry="2"/><line x1="8" y1="21" x2="16" y2="21"/><line x1="12" y1="17" x2="12" y2="21"/></svg>
            <span>Embedded View</span>
          </button>
          <a id="fluidnc-open-link" href="http://{svg_escape(fluidnc_ip)}" target="_blank" rel="noopener noreferrer" class="px-3 py-1.5 rounded-lg bg-blue-600/90 hover:bg-blue-500 text-white text-xs font-semibold transition cursor-pointer flex items-center gap-1.5 shadow-sm shadow-blue-500/20">
            <span>Open FluidNC Tab ↗</span>
          </a>
        </div>
      </div>

      <!-- Diagnostic & Storage Alert Banner -->
      <div class="mb-4 p-3 bg-slate-900/90 border border-slate-800 rounded-xl text-xs space-y-1.5">
        <div class="flex items-center justify-between">
          <span class="font-semibold text-slate-200 flex items-center gap-1.5">
            <svg class="w-4 h-4 text-amber-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z"/></svg>
            FluidNC Controller Diagnostics &amp; SD Storage
          </span>
          <span id="fluidnc-sd-indicator" class="font-mono text-[11px] text-slate-300">SD Card: Checking...</span>
        </div>
        <div class="grid grid-cols-1 md:grid-cols-3 gap-2 text-[11px] text-slate-400">
          <div class="bg-slate-950/70 p-2.5 rounded-lg border border-slate-800/80">
            <strong class="text-slate-200">1. Upload &amp; Run Shortcut:</strong>
            <p class="mt-0.5">Use <strong class="text-blue-300">Send to FluidNC</strong> or <strong class="text-emerald-400">Upload &amp; Run</strong> to push to SD (<code class="text-slate-300">/sd/</code>) and start the job. Requires a FAT32 micro-SD card in the MKS-DLC32 TF slot.</p>
          </div>
          <div class="bg-slate-950/70 p-2.5 rounded-lg border border-slate-800/80">
            <strong class="text-slate-200">2. Why FluidNC Shows "Offline":</strong>
            <p class="mt-0.5">FluidNC allows <strong>1 active WebSocket client</strong>. If you open multiple browser tabs or devices to <code class="text-blue-300">192.168.8.109</code>, the controller disconnects extra tabs. Keep only 1 tab open.</p>
          </div>
          <div class="bg-slate-950/70 p-2.5 rounded-lg border border-slate-800/80">
            <strong class="text-slate-200">3. Built-in G-Code Viewer &amp; Copy Path:</strong>
            <p class="mt-0.5">Click <strong class="text-cyan-300">View G-Code</strong> to inspect toolpaths, rapids, and drill plunges right here, or click <strong class="text-slate-200">Copy Path</strong> to paste the exact file path into FluidNC or your file dialog.</p>
          </div>
        </div>
      </div>

      <!-- Settings & Controls Toolbar -->
      <div class="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-3 mb-4 text-xs">
        <div class="bg-slate-900/80 border border-slate-800 rounded-xl p-2.5">
          <label class="block text-[11px] font-medium text-slate-400 mb-1" for="fluidnc-ip-input">Controller IP Address</label>
          <input type="text" id="fluidnc-ip-input" value="{svg_escape(fluidnc_ip)}" onchange="updateFluidNCIp(this.value)"
                 class="w-full bg-slate-950 border border-slate-700/80 rounded-lg px-2.5 py-1 text-slate-100 font-mono text-xs focus:outline-none focus:border-blue-500">
        </div>

        <div class="bg-slate-900/80 border border-slate-800 rounded-xl p-2.5">
          <label class="block text-[11px] font-medium text-slate-400 mb-1" for="fluidnc-target-select">Destination Storage</label>
          <select id="fluidnc-target-select" class="w-full bg-slate-950 border border-slate-700/80 rounded-lg px-2.5 py-1 text-slate-100 font-mono text-xs focus:outline-none focus:border-blue-500">
            <option value="sd" selected>SD Card (/upload) [Recommended]</option>
            <option value="flash">Internal Flash (/files)</option>
          </select>
        </div>

        <div class="bg-slate-900/80 border border-slate-800 rounded-xl p-2.5 flex items-center justify-between">
          <div>
            <div class="font-medium text-slate-200">Active Sheet Dispatch</div>
            <div id="fluidnc-batch-sheet-lbl" class="text-[10px] text-slate-400">Sheet 1 (36 files)</div>
          </div>
          <button type="button" onclick="uploadAllSheetGCodeToFluidNC()" id="btn-batch-upload"
                  class="px-3 py-1.5 rounded-lg bg-emerald-600 hover:bg-emerald-500 text-white font-semibold text-xs flex items-center gap-1.5 shadow-sm transition cursor-pointer">
            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><polyline points="20 6 9 17 4 12"/></svg>
            <span>Batch Upload Sheet</span>
          </button>
        </div>

        <div class="bg-slate-900/80 border border-slate-800 rounded-xl p-2.5 flex items-center justify-between">
          <div>
            <div class="font-medium text-slate-200">Search G-Code</div>
            <div class="text-[10px] text-slate-400">Filter operations list</div>
          </div>
          <input type="text" id="fluidnc-search-input" placeholder="e.g. drill, 0.8mm, top..." oninput="filterFluidNCSearch(this.value)"
                 class="w-36 bg-slate-950 border border-slate-700/80 rounded-lg px-2 py-1 text-slate-100 font-mono text-xs focus:outline-none focus:border-blue-500">
        </div>
      </div>

      <!-- Batch Transfer Progress Strip (Hidden until triggered) -->
      <div id="fluidnc-batch-progress-wrap" class="hidden mb-4 p-3 bg-slate-900/90 border border-slate-800 rounded-xl space-y-2">
        <div class="flex items-center justify-between text-xs">
          <span id="fluidnc-batch-status-text" class="font-medium text-slate-200 flex items-center gap-2">
            <svg class="animate-spin w-3.5 h-3.5 text-emerald-400" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><circle cx="12" cy="12" r="10" stroke-opacity="0.25"/><path d="M12 2a10 10 0 0 1 10 10" stroke-linecap="round"/></svg>
            <span>Uploading batch to FluidNC...</span>
          </span>
          <span id="fluidnc-batch-pct" class="font-mono text-emerald-400 font-semibold">0%</span>
        </div>
        <div class="w-full h-2 rounded-full bg-slate-950 border border-slate-800 overflow-hidden">
          <div id="fluidnc-batch-progress-bar" class="h-full bg-gradient-to-r from-emerald-500 to-cyan-400 transition-all duration-200 rounded-full" style="width: 0%;"></div>
        </div>
      </div>

      <!-- Embedded FluidNC Split-View / Live Console (Toggleable) -->
      <div id="fluidnc-embedded-panel" class="hidden mb-5 rounded-2xl border border-indigo-500/40 overflow-hidden bg-slate-950 shadow-2xl">
        <div class="px-4 py-3 bg-slate-900/90 border-b border-slate-800 flex flex-wrap items-center justify-between gap-3">
          <div class="flex items-center gap-2">
            <span class="w-2.5 h-2.5 rounded-full bg-emerald-400 animate-pulse"></span>
            <span class="text-xs font-mono font-bold text-slate-100">Live FluidNC WebUI &mdash; http://<span id="fluidnc-embedded-ip-label">{svg_escape(fluidnc_ip)}</span></span>
            <span class="text-[10px] font-mono px-2 py-0.5 rounded bg-blue-500/20 text-blue-300 border border-blue-500/40">DRAG FILES TO RIGHT TO UPLOAD</span>
          </div>
          <div class="flex items-center gap-2">
            <button type="button" onclick="reloadFluidNCFrame()" class="text-slate-300 hover:text-white text-xs px-2.5 py-1 rounded-lg bg-slate-800 hover:bg-slate-700 border border-slate-700 transition">↻ Reload Frame</button>
            <a id="fluidnc-frame-external-link" href="http://{svg_escape(fluidnc_ip)}" target="_blank" rel="noopener noreferrer" class="text-blue-300 hover:text-white text-xs px-2.5 py-1 rounded-lg bg-blue-600/40 hover:bg-blue-600 border border-blue-500/40 transition">Open in Tab ↗</a>
            <button type="button" onclick="toggleFluidNCFrame()" class="text-slate-400 hover:text-white text-xs px-2 py-1 rounded-lg bg-slate-800 hover:bg-slate-700 border border-slate-700 transition">✕ Close Frame</button>
          </div>
        </div>
        <iframe id="fluidnc-iframe" src="about:blank" title="FluidNC WebUI" class="w-full h-[580px] border-0 bg-[#121620]"></iframe>
      </div>

      <!-- Sheet Tabs & Category Filters -->
      <div class="flex flex-col sm:flex-row items-start sm:items-center justify-between pb-3 mb-3 border-b border-slate-800/80 gap-3 text-xs">
        <div class="flex flex-wrap items-center gap-1.5" role="tablist" aria-label="Sheet Filter">
          <button type="button" onclick="setFluidNCDispatcherSheet('1')" id="btn-sheet-1" class="fnc-sheet-btn active px-3 py-1.5 rounded-lg font-semibold transition cursor-pointer bg-blue-600 text-white shadow-sm">Sheet 1</button>
          <button type="button" onclick="setFluidNCDispatcherSheet('2')" id="btn-sheet-2" class="fnc-sheet-btn px-3 py-1.5 rounded-lg font-semibold transition cursor-pointer bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70">Sheet 2</button>
          <button type="button" onclick="setFluidNCDispatcherSheet('3')" id="btn-sheet-3" class="fnc-sheet-btn px-3 py-1.5 rounded-lg font-semibold transition cursor-pointer bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70">Sheet 3</button>
          <button type="button" onclick="setFluidNCDispatcherSheet('ALL')" id="btn-sheet-ALL" class="fnc-sheet-btn px-3 py-1.5 rounded-lg font-semibold transition cursor-pointer bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70">All Sheets (102)</button>
        </div>

        <div class="flex flex-wrap items-center gap-1 text-[11px]">
          <span class="text-slate-500 mr-1">Step:</span>
          <button type="button" onclick="setFluidNCDispatcherCat('ALL')" class="fnc-cat-btn px-2 py-1 rounded bg-slate-800 text-blue-300 border border-blue-500/30">All</button>
          <button type="button" onclick="setFluidNCDispatcherCat('Step0')" class="fnc-cat-btn px-2 py-1 rounded bg-slate-900 text-slate-400 hover:text-white">Guides</button>
          <button type="button" onclick="setFluidNCDispatcherCat('Step1')" class="fnc-cat-btn px-2 py-1 rounded bg-slate-900 text-slate-400 hover:text-white">Drills</button>
          <button type="button" onclick="setFluidNCDispatcherCat('Step3')" class="fnc-cat-btn px-2 py-1 rounded bg-slate-900 text-slate-400 hover:text-white">Top Cu</button>
          <button type="button" onclick="setFluidNCDispatcherCat('Step4')" class="fnc-cat-btn px-2 py-1 rounded bg-slate-900 text-slate-400 hover:text-white">Bottom Cu</button>
          <button type="button" onclick="setFluidNCDispatcherCat('Step5')" class="fnc-cat-btn px-2 py-1 rounded bg-slate-900 text-slate-400 hover:text-white">Mask</button>
          <button type="button" onclick="setFluidNCDispatcherCat('Step7')" class="fnc-cat-btn px-2 py-1 rounded bg-slate-900 text-slate-400 hover:text-white">Cutout</button>
        </div>
      </div>

      <!-- Live G-Code Dispatch Cards / Table -->
      <div id="fluidnc-dispatcher-table" class="space-y-2 max-h-[520px] overflow-y-auto pr-1">
        <!-- Rendered dynamically by renderFluidNCDispatcher() -->
      </div>
    </section>

    <!-- SECTION 1: Step-by-Step Gerber Inspection View -->
    <section id="section-gerber" class="glass-panel p-5 sm:p-6 rounded-2xl scroll-mt-20">
      <div class="flex flex-col md:flex-row items-start md:items-center justify-between pb-4 mb-5 border-b border-slate-800/80 gap-4">
        <div>
          <h2 class="text-lg sm:text-xl font-bold text-slate-100 flex flex-wrap items-center gap-2">
            <span>1. Step-by-Step Gerber Inspection View</span>
            <span class="text-[11px] font-mono bg-sky-500/15 text-sky-300 border border-sky-500/30 px-2 py-0.5 rounded-md">Front &amp; Back Layers &bull; Optional Mirror</span>
          </h2>
          <p class="text-xs text-slate-400 mt-1">Inspect all CNC steps including multi-size drills, Front/Back Solder Mask, Front/Back Silkscreen, and toggleable Mirror View. Step 2 Via Plating displays process parameters (no image needed).</p>
        </div>
        <div class="flex flex-wrap items-center gap-2" role="group" aria-label="Select PCB Project">
          {"".join([f'''
          <button type="button" onclick="selectCircuitProject('{p.pcb_id}')" id="proj-btn-{p.pcb_id}" aria-pressed="{'true' if idx==0 else 'false'}"
                  class="proj-btn px-3.5 py-1.5 text-xs font-semibold rounded-lg transition cursor-pointer {'bg-blue-600 text-white shadow-sm' if idx==0 else 'bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70'}">
            {svg_escape(p.project_name)} ({svg_escape(p.pcb_id)})
          </button>
          ''' for idx, p in enumerate(unique_projects)])}
        </div>
      </div>

      <div class="grid grid-cols-2 sm:grid-cols-4 lg:grid-cols-8 gap-2 mb-4" role="tablist" aria-label="Gerber Fabrication Steps">
        <button type="button" onclick="selectCircuitStep('Composite_Stackup')" id="step-btn-Composite_Stackup" aria-pressed="true" class="step-btn active p-2.5 rounded-xl border border-slate-800 bg-[#0b101b] text-left text-xs transition cursor-pointer">
          <span class="block font-semibold">Overview</span>
          <span class="text-[10px] text-slate-400">Full Stackup</span>
        </button>
        <button type="button" onclick="selectCircuitStep('Step1_Drills')" id="step-btn-Step1_Drills" aria-pressed="false" class="step-btn p-2.5 rounded-xl border border-slate-800 bg-[#0b101b] text-left text-xs transition cursor-pointer">
          <span class="block font-semibold">Step 1</span>
          <span class="text-[10px] text-slate-400">CNC Drills</span>
        </button>
        <button type="button" onclick="selectCircuitStep('Step2_Vias')" id="step-btn-Step2_Vias" aria-pressed="false" class="step-btn p-2.5 rounded-xl border border-slate-800 bg-[#0b101b] text-left text-xs transition cursor-pointer">
          <span class="block font-semibold">Step 2</span>
          <span class="text-[10px] text-slate-400">Via Coating</span>
        </button>
        <button type="button" onclick="selectCircuitStep('Step3_Top_Copper')" id="step-btn-Step3_Top_Copper" aria-pressed="false" class="step-btn p-2.5 rounded-xl border border-slate-800 bg-[#0b101b] text-left text-xs transition cursor-pointer">
          <span class="block font-semibold">Step 3</span>
          <span class="text-[10px] text-slate-400">Top Copper</span>
        </button>
        <button type="button" onclick="selectCircuitStep('Step4_Bottom_Copper')" id="step-btn-Step4_Bottom_Copper" aria-pressed="false" class="step-btn p-2.5 rounded-xl border border-slate-800 bg-[#0b101b] text-left text-xs transition cursor-pointer">
          <span class="block font-semibold">Step 4</span>
          <span class="text-[10px] text-slate-400">Bottom Copper</span>
        </button>
        <button type="button" onclick="selectCircuitStep('Step5_Solder_Mask')" id="step-btn-Step5_Solder_Mask" aria-pressed="false" class="step-btn p-2.5 rounded-xl border border-slate-800 bg-[#0b101b] text-left text-xs transition cursor-pointer">
          <span class="block font-semibold">Step 5</span>
          <span class="text-[10px] text-slate-400">Mask (F / B)</span>
        </button>
        <button type="button" onclick="selectCircuitStep('Step6_Silkscreen')" id="step-btn-Step6_Silkscreen" aria-pressed="false" class="step-btn p-2.5 rounded-xl border border-slate-800 bg-[#0b101b] text-left text-xs transition cursor-pointer">
          <span class="block font-semibold">Step 6</span>
          <span class="text-[10px] text-slate-400">Silk (F / B)</span>
        </button>
        <button type="button" onclick="selectCircuitStep('Step7_Cutout_SnapTab')" id="step-btn-Step7_Cutout_SnapTab" aria-pressed="false" class="step-btn p-2.5 rounded-xl border border-slate-800 bg-[#0b101b] text-left text-xs transition cursor-pointer">
          <span class="block font-semibold">Step 7</span>
          <span class="text-[10px] text-slate-400">Breakaway</span>
        </button>
      </div>

      <!-- Layer Side & Optional Mirror View Controls -->
      <div class="flex flex-wrap items-center justify-between gap-3 mb-5">
        <div id="circuit-side-controls" style="display: none;" class="inline-flex rounded-lg bg-[#0a0e17] p-1 border border-slate-800" role="group" aria-label="Layer Side Selector">
          <button type="button" onclick="setCircuitSide('front')" id="circuit-side-front" aria-pressed="true" class="circuit-side-btn px-3 py-1 text-xs font-semibold rounded-md bg-blue-600 text-white cursor-pointer transition">Front Side</button>
          <button type="button" onclick="setCircuitSide('back')" id="circuit-side-back" aria-pressed="false" class="circuit-side-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition">Back Side</button>
        </div>
        <div id="circuit-mirror-wrap" class="inline-flex items-center gap-2 ml-auto">
          <button type="button" onclick="toggleCircuitMirror()" id="circuit-mirror-btn" aria-pressed="false" class="px-3.5 py-1.5 text-xs font-semibold rounded-lg bg-slate-800/90 hover:bg-slate-700 text-slate-300 border border-slate-700 cursor-pointer transition">
            Mirror View: Disabled
          </button>
        </div>
      </div>

      <div class="grid grid-cols-1 lg:grid-cols-12 gap-6">
        <div class="lg:col-span-8 blueprint-stage p-4 rounded-xl border border-slate-800/90 flex justify-center items-center svg-container min-h-[380px]">
          <img id="circuit-svg-img" src="{img_prefix}Circuit_{first_proj_id}_Composite_Stackup.svg" alt="Circuit Gerber Step Preview" class="max-h-[520px] object-contain">
          <div id="circuit-vias-spec-card" style="display: none;" class="w-full max-w-lg bg-[#0b101b] border border-cyan-500/40 rounded-2xl p-6 flex-col items-start gap-4 text-left shadow-lg">
            <div class="flex items-center gap-3">
              <div class="w-10 h-10 rounded-xl bg-cyan-500/15 border border-cyan-400/40 flex items-center justify-center text-cyan-300 shrink-0" aria-hidden="true">
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                  <path d="M9 3h6"/><path d="M10 3v5l-4.5 9A2 2 0 0 0 7.3 20h9.4a2 2 0 0 0 1.8-3L14 8V3"/>
                </svg>
              </div>
              <div>
                <span class="text-[11px] font-mono uppercase tracking-wider text-cyan-400">Chemical Via Coating &bull; No Gerber Image Required</span>
                <h3 class="text-base font-bold text-slate-100">Step 2: Electroless &amp; Electrolytic Via Barrel Plating</h3>
              </div>
            </div>
            <div class="grid grid-cols-2 gap-3 w-full text-xs font-mono">
              <div class="bg-[#0f1625] p-3 rounded-lg border border-slate-800"><span class="text-slate-400 block text-[10px]">BOARD PROJECT</span><strong id="via-spec-project" class="text-amber-300">-</strong></div>
              <div class="bg-[#0f1625] p-3 rounded-lg border border-slate-800"><span class="text-slate-400 block text-[10px]">TARGET VIAS</span><strong id="via-spec-count" class="text-cyan-300">-</strong></div>
              <div class="bg-[#0f1625] p-3 rounded-lg border border-slate-800"><span class="text-slate-400 block text-[10px]">CU BARREL THICKNESS</span><strong class="text-emerald-300">25 &mu;m Nominal</strong></div>
              <div class="bg-[#0f1625] p-3 rounded-lg border border-slate-800"><span class="text-slate-400 block text-[10px]">PLATING BATH</span><strong class="text-slate-200">1.5 A/dm&sup2; &times; 45 min</strong></div>
            </div>
            <ol class="text-xs text-slate-300 space-y-1.5 list-decimal list-inside leading-relaxed">
              <li>Ultrasonic clean and chemical desmear after Step 1 drilling.</li>
              <li>Palladium / carbon catalyst seeding inside drilled through-holes.</li>
              <li>Acid copper electroplating bath (`CuSO4 + H2SO4`) followed by DI rinse &amp; bake dry.</li>
            </ol>
          </div>
        </div>

        <div class="lg:col-span-4 bg-[#0b101b] p-5 rounded-xl border border-slate-800 flex flex-col justify-between gap-4">
          <div>
            <h3 id="card-step-title" class="font-semibold text-base text-slate-100 mb-3">Composite Circuit Stackup Overview</h3>
            <div class="space-y-2 mb-4 text-xs font-mono">
              <div class="flex justify-between gap-2 py-1.5 border-b border-slate-800/80"><span class="text-slate-400">Tool:</span><span id="card-tool" class="text-slate-100 font-semibold text-right">-</span></div>
              <div class="flex justify-between gap-2 py-1.5 border-b border-slate-800/80"><span class="text-slate-400">Depth:</span><span id="card-depth" class="text-amber-300">-</span></div>
              <div class="flex justify-between gap-2 py-1.5 border-b border-slate-800/80"><span class="text-slate-400">Feed Rate:</span><span id="card-feed" class="text-sky-400">-</span></div>
              <div class="flex justify-between gap-2 py-1.5 border-b border-slate-800/80"><span class="text-slate-400">Spindle RPM:</span><span id="card-rpm" class="text-emerald-400">-</span></div>
            </div>

            <div id="card-drill-breakdown" class="mb-4">
              <div class="flex items-center justify-between text-[11px] uppercase tracking-wider text-slate-400 mb-2">
                <span>Drill Bit Inventory</span>
                <span id="card-drill-total" class="font-mono text-sky-400"></span>
              </div>
              <div id="card-drill-chips" class="flex flex-wrap gap-1.5"></div>
            </div>

            <h4 class="text-[11px] font-semibold text-slate-400 uppercase tracking-wider mb-1.5">Process Notes</h4>
            <p id="card-instructions" class="text-xs text-slate-300 leading-relaxed bg-[#0f1523] p-3 rounded-lg border border-slate-800/90"></p>
          </div>

          <div class="pt-3 border-t border-slate-800/80 space-y-1.5 text-xs">
            <div class="flex items-center justify-between">
              <span class="text-slate-400">Stepping Tag:</span>
              <span class="text-amber-300 font-mono font-semibold">5mm Shortest Edge</span>
            </div>
            <div class="flex items-center justify-between">
              <span class="text-slate-400">Breakaway Mode:</span>
              <span class="text-emerald-300 font-mono font-semibold">{svg_escape(breakaway_cfg.label)}</span>
            </div>
          </div>
        </div>
      </div>
    </section>

    <!-- SECTION 2: Individual Manufactured Unit View (One-by-One + Sub-Menu) -->
    <section id="section-units" class="glass-panel p-5 sm:p-6 rounded-2xl scroll-mt-20">
      <div class="flex flex-col md:flex-row items-start md:items-center justify-between pb-4 mb-5 border-b border-slate-800/80 gap-4">
        <div>
          <h2 class="text-lg sm:text-xl font-bold text-slate-100 flex flex-wrap items-center gap-2">
            <span>2. Individual Manufactured Unit View</span>
            <span class="text-[11px] font-mono bg-amber-500/15 text-amber-300 border border-amber-500/30 px-2 py-0.5 rounded-md">{total_units} Unique Instance Units</span>
          </h2>
          <p class="text-xs text-slate-400 mt-1">Select any manufactured unit from the sub-menu to inspect its unique instance ID, Front Side vs Back Side layout, and bed half placement one by one.</p>
        </div>

        <div class="flex flex-wrap items-center gap-2" role="group" aria-label="Unit Batch Filter">
          <button type="button" onclick="filterUnitSubmenu('ALL')" id="unit-filter-ALL" aria-pressed="true" class="unit-filter-btn px-3.5 py-1.5 text-xs font-semibold rounded-lg bg-blue-600 text-white shadow-sm cursor-pointer transition">
            All Units ({total_units})
          </button>
          {"".join([f'''
          <button type="button" onclick="filterUnitSubmenu('{p.pcb_id}')" id="unit-filter-{p.pcb_id}" aria-pressed="false" class="unit-filter-btn px-3.5 py-1.5 text-xs font-semibold rounded-lg bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70 cursor-pointer transition">
            {svg_escape(p.pcb_id)} ({p.quantity})
          </button>
          ''' for p in unique_projects])}
        </div>
      </div>

      <div class="grid grid-cols-1 lg:grid-cols-12 gap-5">
        <!-- Unit Selector Sub-Menu -->
        <aside class="lg:col-span-4 bg-[#0b101b] rounded-xl border border-slate-800 p-3 flex flex-col">
          <div class="flex items-center justify-between px-1 pb-2.5 mb-2 border-b border-slate-800/80">
            <span class="text-xs font-semibold uppercase tracking-wider text-slate-300">Unit Sub-Menu (Unique IDs)</span>
            <span id="unit-submenu-count" class="text-[11px] font-mono text-slate-400"></span>
          </div>
          <div id="unit-submenu-list" class="space-y-1.5 max-h-[520px] overflow-y-auto pr-1"></div>
        </aside>

        <!-- Focused Single-Unit Viewport -->
        <div class="lg:col-span-8 bg-[#0b101b] rounded-xl border border-slate-800 p-4 sm:p-5 flex flex-col justify-between">
          <div class="flex flex-wrap items-center justify-between gap-3 pb-3 mb-4 border-b border-slate-800/80">
            <div class="flex flex-wrap items-center gap-2">
              <span id="unit-stage-pcb" class="px-2 py-0.5 text-xs font-mono font-bold rounded bg-amber-500/20 text-amber-300 border border-amber-500/40">-</span>
              <h3 id="unit-stage-title" class="text-base font-bold font-mono text-slate-100">-</h3>
              <span id="unit-stage-project" class="text-xs text-slate-400">-</span>
              <span id="unit-stage-side" class="px-2 py-0.5 text-[11px] font-mono rounded-md border">-</span>
              <span id="unit-stage-half" class="px-2 py-0.5 text-[11px] font-mono rounded-md border">-</span>
            </div>
            <div class="flex flex-wrap items-center gap-2">
              <div class="inline-flex rounded-lg bg-[#0a0e17] p-1 border border-slate-800" role="group" aria-label="Unit Side Selector">
                <button type="button" onclick="setUnitSide('front')" id="unit-side-front" aria-pressed="true" class="unit-side-btn px-3 py-1 text-xs font-semibold rounded-md bg-blue-600 text-white cursor-pointer transition">Front Side</button>
                <button type="button" onclick="setUnitSide('back')" id="unit-side-back" aria-pressed="false" class="unit-side-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition">Back Side</button>
              </div>
              <button type="button" onclick="toggleUnitMirror()" id="unit-mirror-btn" aria-pressed="false" class="px-3 py-1.5 text-xs font-semibold rounded-lg bg-slate-800 text-slate-300 border border-slate-700 hover:text-white cursor-pointer transition">Mirror: Disabled</button>
              <span id="unit-stage-pos" class="text-xs font-mono text-slate-400">-</span>
              <button type="button" onclick="stepUnit(-1)" class="px-2.5 py-1.5 text-xs font-semibold rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 cursor-pointer transition">&larr; Prev</button>
              <button type="button" onclick="stepUnit(1)" class="px-2.5 py-1.5 text-xs font-semibold rounded-lg bg-blue-600 hover:bg-blue-500 text-white cursor-pointer transition">Next &rarr;</button>
            </div>
          </div>

          <div class="blueprint-stage rounded-xl border border-slate-800/90 p-4 flex justify-center items-center min-h-[360px]">
            <img id="unit-stage-img" src="{img_prefix}Unit_{all_units[0].unit_id if all_units else ''}_FrontSide.svg" alt="Single Manufactured Unit Preview" class="max-h-[440px] w-auto object-contain rounded">
          </div>

          <div class="grid grid-cols-2 sm:grid-cols-4 gap-3 mt-4 pt-4 border-t border-slate-800/80 text-xs">
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Dimensions</span><strong id="unit-spec-size" class="font-mono text-slate-100">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Sheet / Bed Half</span><strong id="unit-spec-sheet" class="font-mono text-slate-100">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Orientation</span><strong id="unit-spec-rot" class="font-mono text-sky-400">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Panel Origin</span><strong id="unit-spec-origin" class="font-mono text-slate-200">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80 col-span-2"><span class="text-slate-400 block text-[10px] uppercase">Shortest-Edge Tag</span><strong id="unit-spec-tag" class="font-mono text-amber-300">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Seam Retention</span><strong id="unit-spec-seam" class="font-mono text-emerald-400">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Through Drills</span><strong id="unit-spec-drills" class="font-mono text-cyan-400">-</strong></div>
          </div>
        </div>
      </div>
    </section>

    <!-- SECTION 3: Dedicated Sheet-Wise Inspection View -->
    <section id="section-sheets" class="glass-panel p-5 sm:p-6 rounded-2xl scroll-mt-20">
      <div class="flex flex-col md:flex-row items-start md:items-center justify-between pb-4 mb-5 border-b border-slate-800/80 gap-4">
        <div>
          <h2 class="text-lg sm:text-xl font-bold text-slate-100 flex flex-wrap items-center gap-2">
            <span>3. Sheet-Wise Inspection View</span>
            <span class="text-[11px] font-mono bg-emerald-500/15 text-emerald-300 border border-emerald-500/30 px-2 py-0.5 rounded-md">{len(sheets)} A4 Sheet(s)</span>
          </h2>
          <p class="text-xs text-slate-400 mt-1">Inspect full A4 copper laminate sheets ({sheet_w:.0f}&times;{sheet_h:.0f} mm) with upright left-to-right text on Mirrored Back Side and common single-half bed M4 washer clamps.</p>
        </div>

        <div class="inline-flex items-center gap-1 rounded-lg bg-[#0a0e17] p-1 border border-slate-800" role="group" aria-label="Sheet Blueprint Zoom">
          <span class="text-[11px] text-slate-400 px-2">Zoom:</span>
          <button type="button" onclick="setSheetZoom('fit')" id="sheet-zoom-fit" aria-pressed="true" class="sheet-zoom-btn px-3 py-1 text-xs font-semibold rounded-md bg-blue-600 text-white cursor-pointer transition">Fit</button>
          <button type="button" onclick="setSheetZoom('full')" id="sheet-zoom-full" aria-pressed="false" class="sheet-zoom-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition">100%</button>
          <button type="button" onclick="setSheetZoom('zoom')" id="sheet-zoom-zoom" aria-pressed="false" class="sheet-zoom-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition">145%</button>
        </div>
      </div>

      <div class="flex flex-wrap gap-2 mb-5" role="tablist" aria-label="Select Manufacturing Sheet">
        {"".join(sheet_tabs_html)}
      </div>

      {"".join(sheet_panels_html)}

      <div class="mt-5 pt-4 border-t border-slate-800/80 grid grid-cols-2 sm:grid-cols-5 gap-2 text-xs">
        <div class="flex items-center gap-2"><span class="w-3 h-3 rounded bg-emerald-950 border border-emerald-500"></span><span class="text-slate-300">PCB Substrate</span></div>
        <div class="flex items-center gap-2"><span class="w-3 h-3 rounded bg-amber-500/40 border border-amber-500"></span><span class="text-slate-300">5mm Tag (Unique Unit ID)</span></div>
        <div class="flex items-center gap-2"><span class="w-3 h-3 rounded-full bg-red-500/40 border-2 border-red-400"></span><span class="text-slate-300">Common Bed Washer (No Drill)</span></div>
        <div class="flex items-center gap-2"><span class="w-3 h-3 rounded bg-emerald-500/60 border border-emerald-400"></span><span class="text-slate-300">2mm Retention Tabs</span></div>
        <div class="flex items-center gap-2"><span class="w-3 h-3 rounded bg-sky-500/30 border border-sky-400"></span><span class="text-slate-300">Guides &amp; V-Score</span></div>
      </div>
    </section>

    <!-- SECTION 4: CNC Bed & Dual-Sided Milling View -->
    <section id="section-cncbed" class="glass-panel p-5 sm:p-6 rounded-2xl scroll-mt-20">
      <div class="flex flex-col lg:flex-row items-start lg:items-center justify-between pb-4 mb-5 border-b border-slate-800/80 gap-4">
        <div>
          <h2 class="text-lg sm:text-xl font-bold text-slate-100 flex flex-wrap items-center gap-2">
            <span>4. CNC Bed &amp; Dual-Sided Milling View</span>
            <span class="text-[11px] font-mono bg-violet-500/20 text-violet-300 border border-violet-500/40 px-2 py-0.5 rounded-md">Bed {bed_x:.0f}&times;{bed_y:.0f} mm &bull; Dowel Lock: {'ON' if enable_dowel_pin_lock else 'DISABLED'}</span>
          </h2>
          <p class="text-xs text-slate-400 mt-1">Inspect each step-by-step tool setup on the CNC bed: fixed single-half M4 bed washer clamps, Z-Zero Limit Switch Danger Zone (X={z_switch_x:.3f}, Y={z_switch_y:.3f} mm), Y-axis shifts, and mirrored Back Side views.</p>
        </div>

        <div class="flex flex-wrap items-center gap-2">
          <div class="flex items-center gap-1.5" role="group" aria-label="Bed Sheet Selector">
            {"".join([f'''
            <button type="button" onclick="selectBedSheet({i + 1})" id="bed-sheet-{i + 1}" aria-pressed="{'true' if i==0 else 'false'}"
                    class="bed-sheet-btn px-3 py-1.5 text-xs font-semibold rounded-lg cursor-pointer transition {'bg-blue-600 text-white' if i==0 else 'bg-slate-800/90 text-slate-300'}">
              Sheet {i + 1:02d}
            </button>
            ''' for i in range(len(sheets))])}
          </div>
        </div>
      </div>

      <!-- Filter Strip: Side (Front vs Back Mirrored) + Half (Front Bed Half vs Back Bed Half) -->
      <div class="flex flex-wrap items-center justify-between gap-3 mb-5">
        <div class="flex flex-wrap items-center gap-2">
          <div class="inline-flex rounded-lg bg-[#0a0e17] p-1 border border-slate-800" role="group" aria-label="Board Side Filter">
            <button type="button" onclick="setBedSideFilter('ALL')" id="bed-side-ALL" aria-pressed="true" class="bed-side-btn px-3 py-1 text-xs font-semibold rounded-md bg-blue-600 text-white cursor-pointer transition">All Sides</button>
            <button type="button" onclick="setBedSideFilter('top')" id="bed-side-top" aria-pressed="false" class="bed-side-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition">Front Side</button>
            <button type="button" onclick="setBedSideFilter('bottom')" id="bed-side-bottom" aria-pressed="false" class="bed-side-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition">Back Side (Mirrored)</button>
          </div>

          <div class="inline-flex rounded-lg bg-[#0a0e17] p-1 border border-slate-800" role="group" aria-label="Sheet Half Filter">
            <button type="button" onclick="setBedHalfFilter('ALL')" id="bed-half-ALL" aria-pressed="true" class="bed-half-btn px-3 py-1 text-xs font-semibold rounded-md bg-emerald-600 text-white cursor-pointer transition">All Bed Halves</button>
            <button type="button" onclick="setBedHalfFilter('front')" id="bed-half-front" aria-pressed="false" class="bed-half-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition">Front Bed Half</button>
            <button type="button" onclick="setBedHalfFilter('back')" id="bed-half-back" aria-pressed="false" class="bed-half-btn px-3 py-1 text-xs font-semibold rounded-md text-slate-400 hover:text-white cursor-pointer transition">Back Bed Half (Y-Shifted)</button>
          </div>
        </div>

        <div class="flex items-center gap-2">
          <span id="bed-op-counter" class="text-xs font-mono text-slate-400">-</span>
          <button type="button" onclick="stepBedOp(-1)" class="px-2.5 py-1.5 text-xs font-semibold rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 cursor-pointer transition">&larr; Prev Op</button>
          <button type="button" onclick="stepBedOp(1)" class="px-2.5 py-1.5 text-xs font-semibold rounded-lg bg-blue-600 hover:bg-blue-500 text-white cursor-pointer transition">Next Op &rarr;</button>
        </div>
      </div>

      <div class="grid grid-cols-1 lg:grid-cols-12 gap-5">
        <!-- Chronological Tool Operation Sub-Menu -->
        <aside class="lg:col-span-4 bg-[#0b101b] rounded-xl border border-slate-800 p-3 flex flex-col">
          <div class="flex items-center justify-between px-1 pb-2.5 mb-2 border-b border-slate-800/80">
            <span class="text-xs font-semibold uppercase tracking-wider text-slate-300">CNC Operation Timeline</span>
            <span id="bed-ops-count" class="text-[11px] font-mono text-slate-400"></span>
          </div>
          <div id="bed-ops-submenu" class="space-y-1.5 max-h-[560px] overflow-y-auto pr-1"></div>
        </aside>

        <!-- Bed Toolpath Simulation Stage -->
        <div class="lg:col-span-8 bg-[#0b101b] rounded-xl border border-slate-800 p-4 sm:p-5 flex flex-col justify-between gap-4">
          <div class="flex flex-wrap items-center justify-between gap-2 pb-3 border-b border-slate-800/80">
            <div>
              <h3 id="bed-op-title" class="text-sm sm:text-base font-bold text-slate-100">-</h3>
              <p id="bed-op-action" class="text-xs font-mono text-amber-300 mt-0.5">-</p>
            </div>
            <span id="bed-mirror-pill" class="px-2.5 py-1 text-[11px] font-mono rounded-md">-</span>
          </div>

          <div class="blueprint-stage rounded-xl border border-slate-800/90 p-3 flex justify-center items-center min-h-[380px]">
            <img id="bed-op-img" src="" alt="CNC Bed Operation Toolpath View" class="max-h-[520px] w-full object-contain rounded">
          </div>

          <div class="grid grid-cols-1 sm:grid-cols-3 gap-3 text-xs">
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Tool Bit</span><strong id="bed-spec-tool" class="font-mono text-slate-100">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Cut Parameters</span><strong id="bed-spec-depth" class="font-mono text-amber-300">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Bed Y-Shift Datum</span><strong id="bed-spec-yshift" class="font-mono text-emerald-400">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Bed Bounds (&le;{bed_x:.0f}&times;{bed_y:.0f}mm)</span><strong id="bed-spec-bounds" class="font-mono text-sky-400">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80"><span class="text-slate-400 block text-[10px] uppercase">Operation Load</span><strong id="bed-spec-stats" class="font-mono text-slate-200">-</strong></div>
            <div class="bg-[#0f1523] p-2.5 rounded-lg border border-slate-800/80 flex items-center justify-between">
              <div class="overflow-hidden">
                <span class="text-slate-400 block text-[10px] uppercase">G-Code File</span>
                <strong id="bed-spec-gcode" class="font-mono text-slate-200 truncate block">-</strong>
              </div>
              <button type="button" onclick="sendExistingGCode(document.getElementById('bed-spec-gcode').innerText)"
                      title="Upload this G-code file directly to FluidNC"
                      class="ml-2 px-2 py-1 rounded-md bg-blue-600/90 hover:bg-blue-500 text-white font-medium text-[10px] flex items-center gap-1 shrink-0 cursor-pointer shadow-sm transition">
                <svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/></svg>
                <span>To CNC</span>
              </button>
            </div>
          </div>
        </div>
      </div>
    </section>

    <!-- SECTION 5: Interactive Fabrication Setup Checklist -->
    <section id="section-checklist" class="glass-panel p-5 sm:p-6 rounded-2xl scroll-mt-20">
      <div class="flex flex-wrap items-center justify-between pb-4 mb-4 border-b border-slate-800/80 gap-3">
        <div>
          <h2 class="text-lg sm:text-xl font-bold text-slate-100">5. Fabrication Setup Checklist</h2>
          <p class="text-xs text-slate-400 mt-0.5">Sequential shop-floor checklist: V-Bit sheet data &amp; reference spots &rarr; step-by-step drill tool changes with Limit Switch Z-Zero (X={z_switch_x:.3f}, Y={z_switch_y:.3f} mm Danger Zone) &rarr; Steps 2..7.</p>
        </div>
        <button type="button" onclick="resetChecklist()" class="px-3 py-1.5 text-xs font-semibold rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700 cursor-pointer transition">
          Reset Ticks
        </button>
      </div>

      <div class="flex flex-wrap gap-1.5 mb-4" role="group" aria-label="Checklist Categories">
        {"".join([f'''
        <button type="button" data-cat="{svg_escape(c)}" onclick="filterChecklistCat('{svg_escape(c)}')" aria-pressed="{'true' if idx==0 else 'false'}"
                class="chk-cat-btn px-3 py-1 text-xs font-semibold rounded-lg cursor-pointer transition {'bg-blue-600 text-white' if idx==0 else 'bg-slate-800/90 text-slate-300 hover:text-white border border-slate-700/70'}">
          {'All Categories' if c == 'ALL' else svg_escape(c)}
        </button>
        ''' for idx, c in enumerate(chk_categories)])}
      </div>

      <div class="mb-4">
        <div class="flex justify-between text-xs text-slate-400 mb-1.5">
          <span>Setup Progress</span>
          <span id="chk-progress-text" class="text-sky-400 font-mono font-semibold">0 / {len(chk_items)} Completed (0%)</span>
        </div>
        <div class="w-full bg-slate-800/90 h-2.5 rounded-full overflow-hidden">
          <div id="chk-progress-bar" class="bg-gradient-to-r from-blue-500 to-emerald-400 h-full rounded-full transition-all duration-300" style="width: 0%"></div>
        </div>
      </div>

      <div class="grid grid-cols-1 md:grid-cols-2 gap-3 max-h-[560px] overflow-y-auto pr-1">
        {"".join(chk_rows_html)}
      </div>
    </section>
  </main>

  <!-- Interactive G-Code Toolpath Viewer Modal -->
  <div id="gcode-viewer-modal" class="hidden fixed inset-0 z-50 bg-black/80 backdrop-blur-sm flex items-center justify-center p-3 sm:p-6">
    <div class="bg-[#0d111a] border border-slate-700/80 rounded-2xl shadow-2xl w-full max-w-6xl max-h-[92vh] flex flex-col overflow-hidden">
      <!-- Modal Header -->
      <div class="px-4 py-3 bg-slate-900/90 border-b border-slate-800 flex flex-wrap items-center justify-between gap-3">
        <div class="min-w-0">
          <div class="flex items-center gap-2">
            <span class="px-2 py-0.5 rounded bg-cyan-500/20 text-cyan-300 border border-cyan-500/40 font-mono text-[10px] font-bold">G-CODE VIEWER</span>
            <h3 id="gv-filename" class="font-mono text-sm font-bold text-slate-100 truncate">file.gcode</h3>
          </div>
          <div id="gv-filepath" class="text-[11px] font-mono text-slate-400 truncate mt-0.5"></div>
        </div>
        <div class="flex flex-wrap items-center gap-2">
          <div class="inline-flex rounded-lg border border-slate-700 overflow-hidden">
            <button type="button" id="gv-mode-2d" onclick="setGCodeViewMode('2d')" class="px-2.5 py-1 bg-cyan-600 text-white font-semibold text-xs cursor-pointer">2D View</button>
            <button type="button" id="gv-mode-3d" onclick="setGCodeViewMode('3d')" class="px-2.5 py-1 bg-slate-800 text-slate-300 hover:text-white font-semibold text-xs cursor-pointer">3D View</button>
          </div>
          <div id="gv-3d-presets" class="hidden inline-flex items-center gap-1">
            <button type="button" onclick="setGCode3DPreset('iso')" class="px-2 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700 text-[11px] cursor-pointer">Iso</button>
            <button type="button" onclick="setGCode3DPreset('front')" class="px-2 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700 text-[11px] cursor-pointer">Front</button>
            <button type="button" onclick="setGCode3DPreset('side')" class="px-2 py-1 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700 text-[11px] cursor-pointer">Side</button>
          </div>
          <button type="button" onclick="copyGCodePath(gcodeViewerState.filename, this)" class="px-2.5 py-1 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 text-xs font-medium cursor-pointer">📋 Copy Path</button>
          <button type="button" onclick="fitGCodeView()" class="px-2.5 py-1 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 text-xs font-medium cursor-pointer">⛶ Fit Bed</button>
          <button type="button" onclick="closeGCodeViewer()" class="px-2.5 py-1 rounded-lg bg-red-600/80 hover:bg-red-500 text-white text-xs font-semibold cursor-pointer">✕ Close</button>
        </div>
      </div>

      <!-- Fallback Banner if opened via file:// CORS -->
      <div id="gv-file-fallback" class="hidden px-4 py-2.5 bg-amber-950/60 border-b border-amber-500/40 flex flex-wrap items-center justify-between gap-2 text-xs text-amber-200">
        <span>Browser <code class="font-mono">file://</code> security blocked auto-loading. Run <code class="font-mono bg-black/40 px-1.5 py-0.5 rounded">python3 gcode_paralel_manufacture.py --serve</code> for 1-click loading, or select the file directly:</span>
        <label class="px-3 py-1 rounded-lg bg-amber-600 hover:bg-amber-500 text-white font-semibold cursor-pointer">
          📂 Load Local .gcode File
          <input type="file" accept=".gcode,.nc,.g,.ngc,.tap,.cnc" class="hidden" onchange="handleLocalGCodeFileSelect(this)">
        </label>
      </div>

      <!-- Stats & Simulation Speed Bar -->
      <div class="px-4 py-2 bg-slate-900/60 border-b border-slate-800/80 flex flex-wrap items-center justify-between gap-3 text-xs">
        <div id="gv-stats" class="flex flex-wrap items-center gap-1.5 font-mono text-[11px]"></div>
        <div class="flex flex-wrap items-center gap-2.5 text-slate-300">
          <label class="inline-flex items-center gap-1 cursor-pointer">
            <input type="checkbox" checked onchange="toggleGCodeBed(this)" class="accent-indigo-400">
            <span class="text-indigo-300">Bed</span>
          </label>
          <label class="inline-flex items-center gap-1 cursor-pointer">
            <input type="checkbox" checked onchange="toggleGCodeRapids(this)" class="accent-sky-400">
            <span class="text-sky-300">Rapids</span>
          </label>
          <label class="inline-flex items-center gap-1 cursor-pointer">
            <input type="checkbox" checked onchange="toggleGCodeDrills(this)" class="accent-amber-400">
            <span class="text-amber-300">Plunges</span>
          </label>
          <div class="flex items-center gap-1 border-l border-slate-700 pl-2.5">
            <span class="text-[11px] text-slate-400 mr-0.5">Speed:</span>
            <button type="button" data-pct="25" onclick="setGCodeSpeedPct(25)" class="gv-speed-btn px-2 py-0.5 rounded bg-slate-800 text-slate-300 border border-slate-700 text-[11px] cursor-pointer">25%</button>
            <button type="button" data-pct="50" onclick="setGCodeSpeedPct(50)" class="gv-speed-btn px-2 py-0.5 rounded bg-slate-800 text-slate-300 border border-slate-700 text-[11px] cursor-pointer">50%</button>
            <button type="button" data-pct="100" onclick="setGCodeSpeedPct(100)" class="gv-speed-btn px-2 py-0.5 rounded bg-amber-500 text-slate-950 font-bold text-[11px] cursor-pointer">100%</button>
            <button type="button" data-pct="200" onclick="setGCodeSpeedPct(200)" class="gv-speed-btn px-2 py-0.5 rounded bg-slate-800 text-slate-300 border border-slate-700 text-[11px] cursor-pointer">200%</button>
            <button type="button" data-pct="500" onclick="setGCodeSpeedPct(500)" class="gv-speed-btn px-2 py-0.5 rounded bg-slate-800 text-slate-300 border border-slate-700 text-[11px] cursor-pointer">500%</button>
            <button type="button" data-pct="1000" onclick="setGCodeSpeedPct(1000)" class="gv-speed-btn px-2 py-0.5 rounded bg-slate-800 text-slate-300 border border-slate-700 text-[11px] cursor-pointer">1000%</button>
            <button type="button" data-pct="2500" onclick="setGCodeSpeedPct(2500)" class="gv-speed-btn px-2 py-0.5 rounded bg-slate-800 text-slate-300 border border-slate-700 text-[11px] cursor-pointer">2500%</button>
          </div>
          <button type="button" id="gv-play-btn" onclick="toggleGCodePlayback()" class="px-2.5 py-1 rounded-lg bg-emerald-600 hover:bg-emerald-500 text-white font-semibold cursor-pointer">▶ Simulate</button>
          <button type="button" onclick="resetGCodeSimulation()" class="px-2 py-1 rounded-lg bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 font-medium cursor-pointer" title="Reset Simulation to Start">⏹ Reset</button>
        </div>
      </div>

      <!-- Main Viewer Area -->
      <div class="flex-1 grid grid-cols-1 lg:grid-cols-3 min-h-[380px] overflow-hidden">
        <div class="lg:col-span-2 relative bg-[#070a10] flex flex-col">
          <div id="gv-sim-hud" class="px-3 py-1 bg-slate-900/90 border-b border-slate-800/80 flex flex-wrap items-center gap-3 font-mono text-[11px] text-slate-300"></div>
          <canvas id="gv-canvas" class="w-full h-full min-h-[350px] cursor-grab active:cursor-grabbing"></canvas>
          <div class="px-4 py-2 bg-slate-900/80 border-t border-slate-800 flex items-center gap-3">
            <span class="text-[11px] font-mono text-slate-400 shrink-0">Scrub:</span>
            <input type="range" id="gv-progress-slider" min="0" max="100" value="100" oninput="scrubGCodeProgress(this.value)" class="w-full accent-cyan-400 cursor-pointer">
          </div>
        </div>
        <div id="gv-source-code" class="bg-slate-950 border-t lg:border-t-0 lg:border-l border-slate-800 overflow-y-auto font-mono text-[11px] max-h-[460px] p-1"></div>
      </div>
    </div>
  </div>

  <footer class="max-w-7xl mx-auto px-6 py-8 border-t border-slate-800/70 text-center text-xs text-slate-500">
    <p>A4 Copper Board CNC Panelization &amp; Dual-Sided CAM Pipeline &bull; FluidNC Integration</p>
  </footer>
</body>
</html>
"""
    return html


def generate_html_dashboard(sheets, holders_mgr: HolderManager, generated_files,
                            config: Dict, fluidnc_ip="192.168.8.109",
                            output_html="manufacture_guides/index.html",
                            bed_operations: Optional[Dict[int, List[Dict[str, Any]]]] = None,
                            layer_cache: Optional[LayerGeometryCache] = None,
                            sync_root_dashboard: bool = True):
    os.makedirs(os.path.dirname(output_html) or ".", exist_ok=True)

    html_guides = build_html_dashboard_content(
        sheets, holders_mgr, generated_files, config, fluidnc_ip,
        img_prefix="../manufacture_svg/", bed_operations=bed_operations, layer_cache=layer_cache
    )
    with open(output_html, "w", encoding="utf-8") as f:
        f.write(html_guides)
    print(f"[SUCCESS] Interactive Fabrication Dashboard written to: {output_html}")

    if sync_root_dashboard:
        html_root = build_html_dashboard_content(
            sheets, holders_mgr, generated_files, config, fluidnc_ip,
            img_prefix="manufacture_svg/", bed_operations=bed_operations, layer_cache=layer_cache
        )
        root_dash = "manufacture_dashboard.html"
        try:
            with open(root_dash, "w", encoding="utf-8") as f:
                f.write(html_root)
            print(f"[SUCCESS] Root dashboard synced to: {root_dash}")
        except Exception as e:
            print(f"[WARN] Failed writing root dashboard: {e}")


# ---------------------------------------------------------------------------
# Lightweight Local Web Server & FluidNC Network Bridge
# ---------------------------------------------------------------------------

def check_fluidnc_online(ip: str, timeout: float = 2.0) -> Tuple[bool, str]:
    """Test HTTP connection to FluidNC controller [ESP800] info endpoint."""
    try:
        url = f"http://{ip}/command?cmd=%5BESP800%5Djson=yes"
        req = urllib.request.Request(url, headers={"User-Agent": "PCBPanelizer/2.0"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            text = resp.read().decode("utf-8", errors="replace")
            return True, text.strip()
    except Exception:
        try:
            req = urllib.request.Request(f"http://{ip}/", headers={"User-Agent": "PCBPanelizer/2.0"})
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return (resp.status == 200), "HTTP 200 OK"
        except Exception as e:
            return False, str(e)


def upload_gcode_to_fluidnc(ip: str, filename: str, content: bytes, target: str = "sd", timeout: float = 15.0) -> Tuple[bool, str]:
    """Upload a G-code or config file to FluidNC via HTTP POST multipart/form-data matching FigUI."""
    endpoint = f"http://{ip}/upload" if target == "sd" else f"http://{ip}/files"
    remote_name = filename.lstrip("/")
    full_remote_path = f"/{remote_name}"
    boundary = "----FluidNC" + uuid.uuid4().hex[:16]
    body = bytearray()

    # 1. path field
    body.extend(f"--{boundary}\r\n".encode("utf-8"))
    body.extend(b'Content-Disposition: form-data; name="path"\r\n\r\n')
    body.extend(b"/\r\n")

    # 2. size field (<full_remote_path>S)
    body.extend(f"--{boundary}\r\n".encode("utf-8"))
    body.extend(f'Content-Disposition: form-data; name="{full_remote_path}S"\r\n\r\n'.encode("utf-8"))
    body.extend(f"{len(content)}\r\n".encode("utf-8"))

    # 3. myfile[] payload
    body.extend(f"--{boundary}\r\n".encode("utf-8"))
    body.extend(f'Content-Disposition: form-data; name="myfile[]"; filename="{full_remote_path}"\r\n'.encode("utf-8"))
    body.extend(b"Content-Type: application/octet-stream\r\n\r\n")
    body.extend(content)
    body.extend(f"\r\n--{boundary}--\r\n".encode("utf-8"))

    req = urllib.request.Request(
        endpoint,
        data=bytes(body),
        headers={
            "Content-Type": f"multipart/form-data; boundary={boundary}",
            "User-Agent": "PCBPanelizer/2.0"
        },
        method="POST"
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            msg = resp.read().decode("utf-8", errors="replace")
            if "No SD card" in msg:
                return False, "No SD card mounted in MKS-DLC32 TF slot"
            return True, msg or "Upload successful"
    except Exception as e:
        return False, str(e)


def run_gcode_on_fluidnc(ip: str, filename: str, target: str = "sd", timeout: float = 3.0) -> Tuple[bool, str]:
    """Trigger FluidNC to run an uploaded file on SD or LocalFS via WebSocket."""
    import socket, base64
    remote_name = filename.lstrip("/")
    cmd = f"$SD/Run=/{remote_name}\n" if target == "sd" else f"$LocalFS/Run=/{remote_name}\n"
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((ip, 80))
        key = base64.b64encode(os.urandom(16)).decode("utf-8")
        req = (
            f"GET /?independent_session=1 HTTP/1.1\r\n"
            f"Host: {ip}:80\r\n"
            f"Upgrade: websocket\r\n"
            f"Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Protocol: arduino\r\n"
            f"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        sock.sendall(req.encode("utf-8"))
        resp = sock.recv(2048).decode("latin1", errors="ignore")
        if "101 Switching Protocols" not in resp:
            sock.close()
            return False, "WebSocket handshake rejected"
        payload = cmd.encode("utf-8")
        mask = os.urandom(4)
        header = bytes([0x81, 0x80 | len(payload)]) + mask
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        sock.sendall(header + masked)
        time.sleep(0.25)
        try:
            reply = sock.recv(2048).decode("utf-8", errors="ignore")
        except Exception:
            reply = ""
        sock.close()
        if "error:" in reply:
            return False, reply.strip()
        return True, f"Started {cmd.strip()}"
    except Exception as e:
        return False, str(e)


class ChecklistHandler(http.server.SimpleHTTPRequestHandler):
    """
    HTTP request handler serving dashboard guides with CORS,
    JSON checklist persistence, and FluidNC live Drag & Drop upload bridge.
    """
    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Range, User-Agent")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path in ("/", ""):
            self.send_response(302)
            self.send_header("Location", "/manufacture_guides/index.html")
            self.end_headers()
            return

        if parsed.path == "/api/fluidnc_status":
            query = urllib.parse.parse_qs(parsed.query)
            ip = query.get("ip", ["192.168.8.109"])[0].strip() or "192.168.8.109"
            online, status_text = check_fluidnc_online(ip)
            resp = json.dumps({"online": online, "status": status_text, "ip": ip}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(resp)
            return

        super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/save_checklist":
            content_length = int(self.headers.get("Content-Length", 0))
            post_data = self.rfile.read(content_length)
            chk_file = "manufacture_guides/checklist_state.json"
            try:
                data = json.loads(post_data.decode("utf-8"))
                with open(chk_file, "w", encoding="utf-8") as f:
                    json.dump(data, f, indent=2)
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"status":"saved"}')
                print("[API] Saved updated checklist_state.json from browser.")
                return
            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(str(e).encode("utf-8"))
                return

        if parsed.path == "/api/upload_to_fluidnc":
            try:
                content_type = self.headers.get("Content-Type", "")
                content_length = int(self.headers.get("Content-Length", 0))
                post_data = self.rfile.read(content_length)

                msg = email.message_from_bytes(
                    b"Content-Type: " + content_type.encode("utf-8") + b"\r\n\r\n" + post_data,
                    policy=email.policy.default
                )
                fields = {}
                uploaded_files = []
                for part in msg.iter_parts():
                    name = part.get_param("name", header="content-disposition")
                    filename = part.get_filename()
                    if filename:
                        payload = part.get_payload(decode=True)
                        uploaded_files.append((filename, payload))
                    else:
                        fields[name] = str(part.get_content()).strip()

                ip = fields.get("ip", "192.168.8.109").strip() or "192.168.8.109"
                target = fields.get("target", "sd").strip() or "sd"
                auto_convert = fields.get("auto_convert", "true").lower() == "true"
                auto_send = fields.get("auto_send", "true").lower() == "true"

                job_id = f"job_{int(time.time())}_{uuid.uuid4().hex[:6]}"
                upload_base = os.path.join("manufacture_gcode", "uploads", job_id)
                raw_dir = os.path.join(upload_base, "raw")
                gcode_dir = os.path.join(upload_base, "gcode")
                os.makedirs(raw_dir, exist_ok=True)
                os.makedirs(gcode_dir, exist_ok=True)

                saved_raw_files = []
                for fname, fbytes in uploaded_files:
                    safe_name = os.path.basename(fname)
                    fpath = os.path.join(raw_dir, safe_name)
                    with open(fpath, "wb") as f:
                        f.write(fbytes)
                    saved_raw_files.append((safe_name, fpath, fbytes))

                # Processing & Conversion
                has_zip = any(fname.lower().endswith(".zip") for fname, _ in uploaded_files)
                gerber_exts = {".gtl", ".gbl", ".gko", ".drl", ".txt", ".gbr", ".ger", ".gm1", ".g1", ".g2"}
                has_gerber = any(os.path.splitext(fname.lower())[1] in gerber_exts for fname, _ in uploaded_files)

                if has_zip:
                    extract_dir = os.path.join(upload_base, "extracted")
                    os.makedirs(extract_dir, exist_ok=True)
                    for fname, fpath, _ in saved_raw_files:
                        if fname.lower().endswith(".zip"):
                            with zipfile.ZipFile(fpath, "r") as zf:
                                zf.extractall(extract_dir)

                    # Find folder containing gerber files
                    target_folder = extract_dir
                    for root, dirs, fnames in os.walk(extract_dir):
                        if any(os.path.splitext(f.lower())[1] in gerber_exts for f in fnames):
                            target_folder = root
                            break

                    if auto_convert:
                        cmd = [sys.executable, "gerber_to_gcode_easyeda.py", target_folder, gcode_dir, "--config", "tool_config.json"]
                        subprocess.run(cmd, check=False, capture_output=True, text=True)

                elif has_gerber and auto_convert:
                    if len(saved_raw_files) == 1:
                        # Single gerber file conversion
                        single_name, single_path, _ = saved_raw_files[0]
                        base_out = os.path.splitext(single_name)[0] + ".gcode"
                        out_path = os.path.join(gcode_dir, base_out)
                        cmd = [sys.executable, "gerber_to_gcode_easyeda.py", single_path, out_path, "--config", "tool_config.json"]
                        subprocess.run(cmd, check=False, capture_output=True, text=True)
                    else:
                        # Multiple gerber files in raw_dir
                        cmd = [sys.executable, "gerber_to_gcode_easyeda.py", raw_dir, gcode_dir, "--config", "tool_config.json"]
                        subprocess.run(cmd, check=False, capture_output=True, text=True)

                # Collect direct .gcode or .nc files
                for fname, fpath, fbytes in saved_raw_files:
                    if fname.lower().endswith((".gcode", ".nc", ".tap", ".ngc")):
                        dest = os.path.join(gcode_dir, fname)
                        if not os.path.exists(dest):
                            shutil.copy2(fpath, dest)

                # Read all output G-code files
                gcode_files = [f for f in os.listdir(gcode_dir) if f.lower().endswith((".gcode", ".nc", ".tap", ".ngc"))]

                # Check FluidNC status
                fluidnc_online, status_msg = check_fluidnc_online(ip)

                results = []
                for gfile in sorted(gcode_files):
                    gpath = os.path.join(gcode_dir, gfile)
                    with open(gpath, "rb") as gf:
                        gbytes = gf.read()

                    upload_ok = False
                    upload_msg = "Saved on server (auto_send disabled)"
                    if auto_send:
                        if fluidnc_online:
                            upload_ok, upload_msg = upload_gcode_to_fluidnc(ip, gfile, gbytes, target=target)
                        else:
                            upload_msg = f"FluidNC offline ({status_msg}). Saved on server."

                    results.append({
                        "filename": gfile,
                        "size": len(gbytes),
                        "uploaded": upload_ok,
                        "message": upload_msg,
                        "download_url": f"/manufacture_gcode/uploads/{job_id}/gcode/{gfile}"
                    })

                resp_obj = {
                    "success": True,
                    "job_id": job_id,
                    "ip": ip,
                    "target": target,
                    "fluidnc_online": fluidnc_online,
                    "status_msg": status_msg,
                    "files": results,
                    "total_gcode_count": len(results),
                    "summary": f"Generated {len(results)} G-code file(s). Uploaded: {sum(1 for r in results if r['uploaded'])}."
                }
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(resp_obj).encode("utf-8"))
                print(f"[API] Processed upload {job_id}: {len(results)} G-code files, uploaded: {sum(1 for r in results if r['uploaded'])}")
                return
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "error": str(e)}).encode("utf-8"))
                return

        if parsed.path == "/api/send_existing_gcode":
            try:
                content_length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(content_length)
                req_data = json.loads(body.decode("utf-8"))
                ip = req_data.get("ip", "192.168.8.109").strip() or "192.168.8.109"
                fname = os.path.basename(req_data.get("filename", ""))
                target = req_data.get("target", "sd")

                candidates = [
                    os.path.join("manufacture_gcode", fname),
                    os.path.join(".", fname),
                ]
                for root, _, files in os.walk("manufacture_gcode"):
                    if fname in files:
                        candidates.append(os.path.join(root, fname))
                        break

                target_path = None
                for c in candidates:
                    if os.path.isfile(c):
                        target_path = c
                        break

                if not target_path:
                    self.send_response(404)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"success": False, "error": f"File {fname} not found"}).encode("utf-8"))
                    return

                with open(target_path, "rb") as f:
                    content = f.read()

                ok, msg = upload_gcode_to_fluidnc(ip, fname, content, target=target)
                run_now = bool(req_data.get("run", False))
                if ok and run_now:
                    run_ok, run_msg = run_gcode_on_fluidnc(ip, fname, target=target)
                    ok = ok and run_ok
                    msg = f"{msg} | {run_msg}"
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"success": ok, "filename": fname, "message": msg}).encode("utf-8"))
                return
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"success": False, "error": str(e)}).encode("utf-8"))
                return

        super().do_POST()


def serve_dashboard(port=8088):
    """Run local HTTP server with automatic checklist JSON persistence."""
    print(f"\n[SERVER] Serving Manufacturing Dashboard at: http://localhost:{port}/manufacture_guides/")
    print(f"[SERVER] POST /api/save_checklist active -> writing to manufacture_guides/checklist_state.json")
    print("Press Ctrl+C to stop server.\n")
    with socketserver.TCPServer(("", port), ChecklistHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[SERVER] Server stopped.")


# ---------------------------------------------------------------------------
# Main Execution CLI Entrypoint
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="A4 Copper Board CNC PCB Panelizer & Multi-Step Manufacturing Pipeline")
    parser.add_argument("--config", default="manufacture_config.json", help="Path to manufacture_config.json.")
    parser.add_argument("--projects-dir", default=".", help="Root directory containing PCB project folders.")
    parser.add_argument("--holders", default="pcb_holders.json", help="Path to pcb_holders.json config.")
    parser.add_argument("--tool-config", default="tool_config.json", help="Path to tool_config.json.")
    parser.add_argument("--out-gcode", default="manufacture_gcode", help="Output directory for G-codes.")
    parser.add_argument("--out-svg", default="manufacture_svg", help="Output directory for SVGs.")
    parser.add_argument("--out-guides", default="manufacture_guides", help="Output directory for guides & checklist.")
    parser.add_argument("--fluidnc-ip", default="192.168.8.109", help="FluidNC controller IP address.")
    parser.add_argument("--breakaway-mode", default=None, help="Optional override for pcb_tag_strip.breakaway_mode.")
    parser.add_argument("--send-to-cnc", action="store_true", help="Send generated G-code files to FluidNC over network.")
    parser.add_argument("--serve", action="store_true", help="Run local web server on port 8088 to view dashboard and auto-sync checklist.")
    args = parser.parse_args()

    # Load configuration
    cfg = {}
    if os.path.exists(args.config):
        try:
            with open(args.config, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            print(f"[CONFIG] Loaded configuration from {args.config}")
        except Exception as e:
            print(f"[WARN] Failed loading {args.config}: {e}")

    if args.breakaway_mode:
        cfg.setdefault("pcb_tag_strip", {})["breakaway_mode"] = args.breakaway_mode

    sheet_w = float(cfg.get("copper_sheet", {}).get("sheet_width_mm", 210.0))
    sheet_h = float(cfg.get("copper_sheet", {}).get("sheet_height_mm", 297.0))
    bed_x = float(cfg.get("cnc_limits", {}).get("bed_travel_x_mm", 275.0))
    bed_y = float(cfg.get("cnc_limits", {}).get("bed_travel_y_mm", 170.0))
    y_split = float(cfg.get("cnc_limits", {}).get("y_split_threshold_mm", 145.0))
    tab_w = float(cfg.get("pcb_tag_strip", {}).get("tag_width_mm", 5.0))
    b_cfg = BreakawayConfig.from_config(cfg)

    print("================================================================================")
    print(" A4 Copper Board CNC PCB Panelizer & Multi-Step Manufacturing Pipeline")
    print(f" Bed Travel: {bed_x:.0f}x{bed_y:.0f} mm | Sheet Size: {sheet_w:.1f}x{sheet_h:.1f} mm | Y-Shift: {y_split:.1f} mm")
    print(f" Breakaway Mode: {b_cfg.label} | Shortest-Edge Stepping Tag")
    print("================================================================================\n")

    # 1. Scan Projects
    projects = scan_projects(args.projects_dir)
    if not projects:
        print("[ERROR] No PCB projects containing project_metadata.json were discovered.")
        sys.exit(1)
    print(f"[INFO] Discovered {len(projects)} PCB project(s):")
    for p in projects:
        print(f"  - {p.pcb_id} ({p.project_name}) | Dims: {p.width:.1f}x{p.height:.1f}mm | Qty: {p.quantity} | Mask: {p.solder_mask_color}")

    # 2. Load Holders (Single-Half Common Bed Locks & Z-Zero Limit Switch Danger Zone)
    holder_mgr = HolderManager(args.holders, y_split=y_split, bed_travel_y=bed_y, config=cfg)

    # 3. Layer Cache & Panelization
    layer_cache = LayerGeometryCache()
    panelizer = A4Panelizer(
        sheet_w=sheet_w, sheet_h=sheet_h,
        tab_w=tab_w, y_split=y_split,
        holder_mgr=holder_mgr, config=cfg
    )
    sheets = panelizer.pack_projects(projects, layer_cache)

    # 4. Run Manufacturing Pipeline (G-code, SVGs, Metadata)
    pipeline = ManufacturingPipeline(
        sheets=sheets, layer_cache=layer_cache, holders_mgr=holder_mgr,
        config=cfg, out_gcode_dir=args.out_gcode, out_svg_dir=args.out_svg,
        out_guides_dir=args.out_guides
    )
    generated_files = pipeline.run()

    # 5. Generate Interactive HTML Dashboard & Setup Checklist
    html_out = os.path.join(args.out_guides, "index.html")
    sync_root = (os.path.normpath(args.out_guides) == "manufacture_guides")
    generate_html_dashboard(
        sheets, holder_mgr, generated_files,
        config=cfg, fluidnc_ip=args.fluidnc_ip, output_html=html_out,
        bed_operations=pipeline.bed_operations, layer_cache=layer_cache,
        sync_root_dashboard=sync_root,
    )

    # 6. Direct Network Upload to FluidNC if requested
    if args.send_to_cnc:
        import urllib.request
        print(f"\n[NETWORK] Connecting to FluidNC controller at http://{args.fluidnc_ip} ...")
        try:
            req = urllib.request.Request(f"http://{args.fluidnc_ip}/status", headers={"User-Agent": "PCBPanelizer/2.0"})
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                status_text = resp.read().decode("utf-8")
                print(f"[NETWORK] FluidNC is ONLINE! Status: {status_text.strip()}")
        except Exception as e:
            print(f"[WARN] FluidNC at {args.fluidnc_ip} unreachable: {e}")

    print("\n[COMPLETE] Panelization and manufacturing workflow generated successfully.")

    if args.serve:
        serve_dashboard(port=8088)


if __name__ == "__main__":
    main()
