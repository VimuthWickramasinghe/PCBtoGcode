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
import sys
import glob
import math
import json
import argparse
import http.server
import socketserver
import urllib.parse
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional, Any

from shapely.geometry import Polygon, MultiPolygon, Point, box, LineString, MultiLineString
from shapely.affinity import translate, rotate
import shapely.ops as ops


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
    """Manages clamp geometries and collision keep-out zones."""
    def __init__(self, json_path="pcb_holders.json"):
        self.holders = []
        self.exclusion_polygons = []
        self.load(json_path)

    def load(self, json_path):
        if not os.path.exists(json_path):
            print(f"[WARN] Holders config {json_path} not found. Operating with zero fixture keepouts.")
            return
        try:
            with open(json_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.holders = data.get("holders", [])
            for h in self.holders:
                x = float(h.get("x", 0.0))
                y = float(h.get("y", 0.0))
                gap = float(h.get("holder_gap", 3.0))
                shape = h.get("shape", "circle").lower()
                
                if shape == "circle":
                    dia = float(h.get("diameter", 10.0))
                    rad = (dia / 2.0) + gap
                    poly = Point(x, y).buffer(rad)
                else:
                    w = float(h.get("width", 10.0))
                    l = float(h.get("length", 20.0))
                    hw = (w / 2.0) + gap
                    hl = (l / 2.0) + gap
                    poly = box(x - hw, y - hl, x + hw, y + hl)
                self.exclusion_polygons.append(poly)
            print(f"[INFO] Loaded {len(self.holders)} fixture holders from {json_path}.")
        except Exception as e:
            print(f"[ERROR] Failed parsing {json_path}: {e}")

    def collides_with(self, geom) -> bool:
        """Check if geometry intersects any physical clamp keep-out buffer."""
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

        # 6. Parse Drill Files with Standardized Tool Binning
        for drl_path in project.drill_files:
            try:
                dp = gerber_to_gcode.ExcellonParser(drl_path)
                dp.parse()
                for (tool_id, hx, hy) in dp.holes:
                    raw_dia = dp.tools.get(tool_id, 0.8)
                    norm_x = hx - ox
                    norm_y = hy - oy
                    
                    dia_key = self.standardize_tool_dia(raw_dia)
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
# Placed PCB Unit on A4 Panel (Supports 0° and 90° Rotations)
# ---------------------------------------------------------------------------

class PlacedUnit:
    """
    Represents a single PCB unit nested on an A4 sheet.
    The 5mm tag bar is strictly attached along the WIDTH side of the PCB design!
    When rotated 90°, the tag bar rotates with the PCB design along its width side.
    """
    def __init__(self, project: ProjectInfo, unit_id: str, x: float, y: float,
                 w: float, h: float, rotation_deg=0, tab_w=5.0, sheet_idx=0,
                 engrave_both_sides=False, engrave_arrow=True):
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
        
        self.pcb_w = project.width
        self.pcb_h = project.height
        
        # Bounding box of full unit (PCB + 5mm tag)
        self.full_box = box(x, y, x + w, y + h)
        
        if rotation_deg == 0:
            # Tag bar along PCB width side (length = pcb_w, thickness = tab_w) at bottom
            self.tab_box = box(x, y, x + self.pcb_w, y + tab_w)
            self.pcb_box = box(x, y + tab_w, x + self.pcb_w, y + tab_w + self.pcb_h)
            self.seam_line = [(x, y + tab_w), (x + self.pcb_w, y + tab_w)]
            
            # Mouse bites along horizontal seam
            self.mouse_bites = []
            bite_count = int(self.pcb_w // 1.5)
            for i in range(1, bite_count):
                self.mouse_bites.append((x + i * 1.5, y + tab_w))

            # Directional arrow in tag bar pointing +Y into PCB body
            arrow_cx = x + self.pcb_w / 2.0
            arrow_tip_y = y + tab_w - 0.8
            arrow_base_y = y + 1.2
            self.arrow_lines = [
                [(arrow_cx, arrow_base_y), (arrow_cx, arrow_tip_y)],
                [(arrow_cx - 2.5, arrow_tip_y - 2.0), (arrow_cx, arrow_tip_y)],
                [(arrow_cx + 2.5, arrow_tip_y - 2.0), (arrow_cx, arrow_tip_y)]
            ] if engrave_arrow else []

            # Unit ID text in tag bar
            text_x = arrow_cx + 3.0
            text_y = y + 1.4
            self.id_text_lines = text_to_polylines(self.unit_id, text_x, text_y, char_h=2.2, char_w=1.4)

        elif rotation_deg == 90:
            # PCB is rotated 90° counter-clockwise.
            # Its width side (length pcb_w) is oriented along the Y axis!
            # Tag bar is attached along this width side: thickness = tab_w along X, length = pcb_w along Y.
            self.tab_box = box(x, y, x + tab_w, y + self.pcb_w)
            self.pcb_box = box(x + tab_w, y, x + tab_w + self.pcb_h, y + self.pcb_w)
            self.seam_line = [(x + tab_w, y), (x + tab_w, y + self.pcb_w)]

            # Mouse bites along vertical seam
            self.mouse_bites = []
            bite_count = int(self.pcb_w // 1.5)
            for i in range(1, bite_count):
                self.mouse_bites.append((x + tab_w, y + i * 1.5))

            # Directional arrow in tag bar pointing +X into PCB body
            arrow_cy = y + self.pcb_w / 2.0
            arrow_tip_x = x + tab_w - 0.8
            arrow_base_x = x + 1.2
            self.arrow_lines = [
                [(arrow_base_x, arrow_cy), (arrow_tip_x, arrow_cy)],
                [(arrow_tip_x - 2.0, arrow_cy - 2.5), (arrow_tip_x, arrow_cy)],
                [(arrow_tip_x - 2.0, arrow_cy + 2.5), (arrow_tip_x, arrow_cy)]
            ] if engrave_arrow else []

            # Unit ID text engraved inside tag bar along Y axis
            text_x = x + 1.4
            text_y = arrow_cy + 3.0
            raw_text = text_to_polylines(self.unit_id, 0.0, 0.0, char_h=2.2, char_w=1.4)
            self.id_text_lines = []
            for stroke in raw_text:
                # Rotate 90 degrees: (px, py) -> (-py, px) translated to (text_x, text_y)
                rot_stroke = [(text_x + p[1], text_y + p[0]) for p in stroke]
                self.id_text_lines.append(rot_stroke)

    def to_sheet_coords(self, px: float, py: float) -> Tuple[float, float]:
        """Maps local normalized PCB coordinates [0, pcb_w] x [0, pcb_h] to A4 sheet coordinates."""
        if self.rotation_deg == 0:
            return self.x + px, self.y + self.tab_w + py
        elif self.rotation_deg == 90:
            rot_x = self.pcb_h - py
            rot_y = px
            return self.x + self.tab_w + rot_x, self.y + rot_y
        return self.x + px, self.y + py


# ---------------------------------------------------------------------------
# A4 Copper Board Panelizer (Zero-Gap Tight Packing & Solder Mask Color Grouping)
# ---------------------------------------------------------------------------

class A4Panelizer:
    """
    Panelizes multiple PCB projects onto A4 copper sheets (210x297mm):
    - Prioritizes grouping boards by solder mask color.
    - Rotates rectangular PCBs (0° or 90°) to maximize boards per sheet.
    - Packs tightly with zero gap between adjacent designs for single-pass perimeter cutting.
    - Partitions strictly into Front Half (Y <= 141mm) and Back Half (Y >= 150mm).
    """
    def __init__(self, sheet_w=210.0, sheet_h=297.0, tab_w=5.0, y_split=145.0,
                 holder_mgr: Optional[HolderManager] = None, config: Optional[Dict] = None):
        self.sheet_w = sheet_w
        self.sheet_h = sheet_h
        self.tab_w = tab_w
        self.y_split = y_split
        self.holder_mgr = holder_mgr
        self.cfg = config or {}
        
        cnc = self.cfg.get("cnc_limits", {})
        self.left_margin = float(cnc.get("left_margin_guide_width_mm", 18.0))
        self.front_max_y = float(cnc.get("front_half_max_y_mm", 141.0))
        self.back_min_y = float(cnc.get("back_half_min_y_mm", 150.0))
        self.back_max_y = float(cnc.get("back_half_max_y_mm", 292.0))
        
        panel_cfg = self.cfg.get("panelization", {})
        self.zero_gap = bool(panel_cfg.get("zero_gap_tight_packing", True))
        self.allow_rot = bool(panel_cfg.get("allow_rotation_to_maximize_space", True))
        self.group_by_color = bool(panel_cfg.get("group_by_solder_mask_color", True))
        self.strict_color = bool(panel_cfg.get("strict_solder_mask_color", False))
        
        tag_cfg = self.cfg.get("pcb_tag_strip", {})
        self.engrave_both = bool(tag_cfg.get("engrave_text_both_sides", False))
        self.engrave_arrow = bool(tag_cfg.get("engrave_arrow", True))

    def _get_dims_for_rot(self, p: ProjectInfo, rot: int) -> Tuple[float, float]:
        if rot == 90:
            return p.height + self.tab_w, p.width
        return p.width, p.height + self.tab_w

    def pack_projects(self, projects: List[ProjectInfo], layer_cache: LayerGeometryCache) -> List[List[PlacedUnit]]:
        # Sort & group projects by solder mask color
        if self.group_by_color:
            projects_sorted = sorted(projects, key=lambda p: (p.solder_mask_color, -p.quantity))
        else:
            projects_sorted = sorted(projects, key=lambda p: -p.quantity)

        # Expand into individual unit requests
        units_to_pack = []
        for p in projects_sorted:
            for u_id in p.unit_ids:
                units_to_pack.append({"project": p, "unit_id": u_id})

        sheets: List[List[PlacedUnit]] = []
        idx = 0
        right_margin = 5.0
        usable_w = self.sheet_w - self.left_margin - right_margin
        front_min_y = 8.0

        while idx < len(units_to_pack):
            sheet_units: List[PlacedUnit] = []
            sheet_idx = len(sheets)

            # Check active solder mask color for this sheet
            active_color = units_to_pack[idx]["project"].solder_mask_color

            # 1. Pack Front Half
            curr_x = self.left_margin
            while idx < len(units_to_pack):
                item = units_to_pack[idx]
                p = item["project"]

                if self.strict_color and p.solder_mask_color != active_color:
                    break

                # Determine best rotation: 90 deg preferred for rectangular to fit two wide
                rot = 90 if (self.allow_rot and p.shape == "rectangular") else 0
                u_w, u_h = self._get_dims_for_rot(p, rot)

                if (curr_x + u_w <= self.left_margin + usable_w) and (front_min_y + u_h <= self.front_max_y):
                    u = PlacedUnit(
                        project=p, unit_id=item["unit_id"],
                        x=curr_x, y=front_min_y, w=u_w, h=u_h,
                        rotation_deg=rot, tab_w=self.tab_w, sheet_idx=sheet_idx,
                        engrave_both_sides=self.engrave_both, engrave_arrow=self.engrave_arrow
                    )
                    sheet_units.append(u)
                    curr_x += u_w  # ZERO GAP!
                    idx += 1
                else:
                    break

            # 2. Pack Back Half
            curr_x = self.left_margin
            while idx < len(units_to_pack):
                item = units_to_pack[idx]
                p = item["project"]

                if self.strict_color and p.solder_mask_color != active_color:
                    break

                rot = 90 if (self.allow_rot and p.shape == "rectangular") else 0
                u_w, u_h = self._get_dims_for_rot(p, rot)

                if (curr_x + u_w <= self.left_margin + usable_w) and (self.back_min_y + u_h <= self.back_max_y):
                    u = PlacedUnit(
                        project=p, unit_id=item["unit_id"],
                        x=curr_x, y=self.back_min_y, w=u_w, h=u_h,
                        rotation_deg=rot, tab_w=self.tab_w, sheet_idx=sheet_idx,
                        engrave_both_sides=self.engrave_both, engrave_arrow=self.engrave_arrow
                    )
                    sheet_units.append(u)
                    curr_x += u_w  # ZERO GAP!
                    idx += 1
                else:
                    break

            if sheet_units:
                sheets.append(sheet_units)
            else:
                print(f"[ERROR] Could not fit unit {units_to_pack[idx]['unit_id']} on an A4 sheet. Check board dimensions.")
                idx += 1

        print(f"[INFO] Panelized {len(units_to_pack)} units across {len(sheets)} A4 copper sheet(s).")
        return sheets


# ---------------------------------------------------------------------------
# Left Alignment Guides & Sheet Tag Generator
# ---------------------------------------------------------------------------

def generate_left_guides(sheet_w=210.0, sheet_h=297.0, margin_w=18.0, y_shift=145.0):
    """
    Step 0: Left Margin Alignment Guides, Millimeter Scale & Shift Datum Line.
    Provides precise mechanical registration for 2-step CNC operations.
    """
    lines = []
    texts = []
    pin_holes = []

    # 1. Left border guide line
    x_guide = 12.0
    lines.append([(x_guide, 0.0), (x_guide, sheet_h)])

    # 2. Millimeter ruler ticks
    for y in range(0, int(sheet_h) + 1, 5):
        tick_len = 5.0 if (y % 50 == 0) else (3.5 if (y % 10 == 0) else 2.0)
        lines.append([(x_guide, float(y)), (x_guide - tick_len, float(y))])
        if y % 50 == 0 and y > 0:
            txt_strokes = text_to_polylines(f"{y}", 2.0, float(y) - 1.0, char_h=2.0, char_w=1.2)
            texts.extend(txt_strokes)

    # 3. Y=145.0mm Shift Datum Line
    lines.append([(0.0, y_shift), (margin_w, y_shift)])
    shift_lbl = text_to_polylines("Y145 SHIFT", 1.5, y_shift + 2.0, char_h=2.2, char_w=1.3)
    texts.extend(shift_lbl)

    # 4. Reference Dowel Pin Holes (3.0mm diameter)
    pin_holes.append((x_guide / 2.0, 25.0))
    pin_holes.append((x_guide / 2.0, y_shift + 25.0))

    return {
        "lines": lines,
        "texts": texts,
        "pin_holes": pin_holes,
        "pin_dia": 3.0
    }


def generate_sheet_tag(sheet_num: int, total_sheets: int, units_count: int,
                       solder_mask_color: str = "green",
                       x: float = 30.0, y: float = 286.0, w: float = 160.0, h: float = 8.0):
    """
    Engraves a dedicated Sheet Tag on each copper sheet with sheet number,
    total sheets, nested units count, solder mask color, and orientation arrow.
    """
    lines = []
    # Outer bounding border
    lines.append([(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)])
    
    # Tag Text
    label = f"SHEET {sheet_num:02d}/{total_sheets:02d} | UNITS: {units_count} | MASK: {solder_mask_color.upper()} | A4 210X297"
    text_strokes = text_to_polylines(label, x + 3.0, y + 2.5, char_h=2.8, char_w=1.7)
    
    # Orientation Arrow (pointing in +Y machine bed direction)
    arrow_x = x + w - 18.0
    arrow_y_base = y + 1.8
    arrow_y_tip = y + h - 1.8
    arrow_strokes = [
        [(arrow_x, arrow_y_base), (arrow_x, arrow_y_tip)],
        [(arrow_x - 2.5, arrow_y_tip - 2.5), (arrow_x, arrow_y_tip)],
        [(arrow_x + 2.5, arrow_y_tip - 2.5), (arrow_x, arrow_y_tip)]
    ]
    arrow_label = text_to_polylines("BED Y+", arrow_x + 3.5, y + 2.5, char_h=2.2, char_w=1.3)
    
    return {
        "border": lines,
        "text": text_strokes + arrow_label,
        "arrow": arrow_strokes
    }


# ---------------------------------------------------------------------------
# Individual Circuit Gerber SVG Visualizer (Step-by-Step Inspector)
# ---------------------------------------------------------------------------

class CircuitSvgBuilder:
    """Generates high-resolution vector SVGs for each manufacturing step of a circuit."""
    def __init__(self, project: ProjectInfo, p_data: Dict, scale: float = 4.5, margin: float = 15.0, tab_w: float = 5.0):
        self.project = project
        self.p_data = p_data
        self.w = float(project.width)
        self.h = float(project.height)
        self.tab_w = float(tab_w)
        self.margin = float(margin)
        self.scale = float(scale)
        
        self.total_w = self.w + 2 * self.margin
        self.total_h = self.h + self.tab_w + 2 * self.margin
        
        self.px_w = self.total_w * self.scale
        self.px_h = self.total_h * self.scale
        
        self.norm_layers = {
            k: p_data.get(k) for k in [
                "outline", "copper_top", "copper_bottom", "mask_top", "mask_bottom", "silkscreen_top", "silkscreen_bottom"
            ]
        }
        self.drills = p_data.get("drills", {})

    def _tr(self, x, y):
        sx = (self.margin + x) * self.scale
        sy = ((self.total_h - self.margin) - y) * self.scale
        return sx, sy

    def _geom_to_path_d(self, geom, y_offset=0.0):
        if geom is None or geom.is_empty:
            return ""
        polys = geom.geoms if hasattr(geom, "geoms") else [geom]
        d_parts = []
        for p in polys:
            if hasattr(p, "exterior") and p.exterior is not None:
                ext_coords = [self._tr(pt[0], pt[1] + y_offset) for pt in p.exterior.coords]
                if ext_coords:
                    d_parts.append(f"M {ext_coords[0][0]:.1f},{ext_coords[0][1]:.1f} " +
                                   " ".join(f"L {pt[0]:.1f},{pt[1]:.1f}" for pt in ext_coords[1:]) + " Z")
                for interior in p.interiors:
                    int_coords = [self._tr(pt[0], pt[1] + y_offset) for pt in interior.coords]
                    if int_coords:
                        d_parts.append(f"M {int_coords[0][0]:.1f},{int_coords[0][1]:.1f} " +
                                       " ".join(f"L {pt[0]:.1f},{pt[1]:.1f}" for pt in int_coords[1:]) + " Z")
        return " ".join(d_parts)

    def draw_background(self):
        return f'<rect width="{self.px_w}" height="{self.px_h}" fill="#0a0e17"/>'

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

    def draw_dimensions(self):
        elems = []
        x0, y_top = self._tr(0, self.tab_w + self.h + 4.0)
        x1, _ = self._tr(self.w, self.tab_w + self.h + 4.0)
        elems.append(f'<line x1="{x0}" y1="{y_top}" x2="{x1}" y2="{y_top}" stroke="#64748b" stroke-width="1.2" marker-start="url(#arr-dim)" marker-end="url(#arr-dim)"/>')
        elems.append(f'<text x="{(x0+x1)/2.0}" y="{y_top - 4}" fill="#94a3b8" font-family="monospace" font-size="11" text-anchor="middle">{self.w:.1f} mm</text>')

        x_r, y_pcb_b = self._tr(self.w + 4.0, self.tab_w)
        _, y_pcb_t = self._tr(self.w + 4.0, self.tab_w + self.h)
        elems.append(f'<line x1="{x_r}" y1="{y_pcb_b}" x2="{x_r}" y2="{y_pcb_t}" stroke="#64748b" stroke-width="1.2"/>')
        elems.append(f'<text x="{x_r + 14}" y="{(y_pcb_b+y_pcb_t)/2.0}" fill="#94a3b8" font-family="monospace" font-size="11" text-anchor="middle" transform="rotate(-90 {x_r+14} {(y_pcb_b+y_pcb_t)/2.0})">{self.h:.1f} mm</text>')

        x_tab_r, y_tab_b = self._tr(self.w + 4.0, 0)
        _, y_tab_t = self._tr(self.w + 4.0, self.tab_w)
        elems.append(f'<line x1="{x_tab_r}" y1="{y_tab_b}" x2="{x_tab_r}" y2="{y_tab_t}" stroke="#f59e0b" stroke-width="1.2"/>')
        elems.append(f'<text x="{x_tab_r + 14}" y="{(y_tab_b+y_tab_t)/2.0 + 3}" fill="#f59e0b" font-family="monospace" font-size="9" text-anchor="middle" transform="rotate(-90 {x_tab_r+14} {(y_tab_b+y_tab_t)/2.0 + 3})">5mm Tab</text>')
        return "".join(elems)

    def draw_substrate(self):
        x0, y1 = self._tr(0, self.tab_w + self.h)
        w_px = self.w * self.scale
        h_px = self.h * self.scale
        return (f'<rect x="{x0:.1f}" y="{y1:.1f}" width="{w_px:.1f}" height="{h_px:.1f}" '
                f'fill="#0b2416" stroke="#1b4d2e" stroke-width="1.5" rx="2"/>')

    def draw_snap_tab(self, label=None):
        elems = []
        lbl = label or f"{self.project.pcb_id} (5mm SNAP-OFF TAB)"
        x0, y1 = self._tr(0, self.tab_w)
        w_px = self.w * self.scale
        th_px = self.tab_w * self.scale
        
        # 5mm Tab along width
        elems.append(f'<rect x="{x0:.1f}" y="{y1:.1f}" width="{w_px:.1f}" height="{th_px:.1f}" '
                     f'fill="rgba(217, 119, 6, 0.15)" stroke="#d97706" stroke-width="1.2" stroke-dasharray="4,2"/>')

        # Mouse bites along seam (0.8mm drill, 1.5mm pitch)
        bite_count = int(self.w // 1.5)
        for i in range(1, bite_count):
            bx = i * 1.5
            sx, sy = self._tr(bx, self.tab_w)
            elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{0.4 * self.scale:.1f}" fill="#0f172a" stroke="#f59e0b" stroke-width="0.6"/>')

        # Directional arrow pointing up into PCB body
        mid_x = self.w / 2.0
        ax, ay_tip = self._tr(mid_x, self.tab_w - 0.8)
        _, ay_base = self._tr(mid_x, 1.2)
        ax_l, _ = self._tr(mid_x - 3.0, self.tab_w - 2.5)
        ax_r, _ = self._tr(mid_x + 3.0, self.tab_w - 2.5)
        elems.append(f'<polygon points="{ax:.1f},{ay_tip:.1f} {ax_l:.1f},{ay_base:.1f} {ax_r:.1f},{ay_base:.1f}" fill="#f59e0b"/>')
        elems.append(f'<text x="{ax + 20}" y="{ay_tip + 8}" fill="#f59e0b" font-family="monospace" font-weight="bold" font-size="10">▲ PCB BODY</text>')
        elems.append(f'<text x="{ax - 20}" y="{ay_tip + 8}" fill="#fbbf24" font-family="monospace" font-weight="bold" font-size="10" text-anchor="end">{lbl}</text>')

        return "".join(elems)

    def draw_drills_layer(self):
        elems = []
        color_map = {
            0.3: "#06b6d4", 0.7: "#8b5cf6", 0.75: "#a855f7", 0.8: "#c084fc",
            0.9: "#fbbf24", 1.0: "#f59e0b", 1.1: "#f97316", 1.2: "#ea580c",
            1.3: "#dc2626", 1.5: "#ef4444", 1.8: "#f43f5e", 3.0: "#ec4899",
        }
        for dia, pts in self.drills.items():
            col = color_map.get(dia, "#38bdf8")
            r_px = max(1.2, (dia / 2.0) * self.scale)
            for (hx, hy) in pts:
                sx, sy = self._tr(hx, self.tab_w + hy)
                elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{r_px:.1f}" fill="{col}" stroke="#ffffff" stroke-width="0.5" opacity="0.95"/>')
                elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{max(0.6, r_px * 0.5):.1f}" fill="#0a0e17"/>')
        return "".join(elems)

    def build_step1_drills_svg(self):
        body = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(),
            self.draw_snap_tab(), self.draw_dimensions(), self.draw_drills_layer(),
            f'<text x="{self.margin*self.scale}" y="22" fill="#38bdf8" font-family="monospace" font-weight="bold" font-size="14">STEP 1: MULTI-DIAMETER TOP DRILLING</text>',
            f'<text x="{self.margin*self.scale}" y="38" fill="#64748b" font-family="monospace" font-size="11">Total Holes: {sum(len(pts) for pts in self.drills.values())} | Distinct Bit Sizes: {len(self.drills)}</text>'
        ]
        return self._wrap_svg(body)

    def build_step2_vias_svg(self):
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(),
            self.draw_snap_tab(), self.draw_dimensions()
        ]
        via_count = len(self.drills.get(0.3, []))
        for (hx, hy) in self.drills.get(0.3, []):
            sx, sy = self._tr(hx, self.tab_w + hy)
            elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{1.2 * self.scale:.1f}" fill="rgba(6, 182, 212, 0.25)" stroke="#06b6d4" stroke-width="1.2"/>')
            elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{0.4 * self.scale:.1f}" fill="#06b6d4"/>')
            elems.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{0.15 * self.scale:.1f}" fill="#000000"/>')

        elems.append(f'<text x="{self.margin*self.scale}" y="22" fill="#06b6d4" font-family="monospace" font-weight="bold" font-size="14">STEP 2: CHEMICAL THROUGH-HOLE VIA METALLIZATION</text>')
        elems.append(f'<text x="{self.margin*self.scale}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">Via Count: {via_count} holes (0.30mm dia) | Target Barrel Plating: 25µm Cu</text>')
        return self._wrap_svg(elems)

    def build_step3_top_copper_svg(self):
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(),
            self.draw_snap_tab(), self.draw_dimensions()
        ]
        d_cu = self._geom_to_path_d(self.norm_layers.get("copper_top"), y_offset=self.tab_w)
        if d_cu:
            elems.append(f'<path d="{d_cu}" fill="#eab308" fill-rule="evenodd" opacity="0.9" stroke="#ca8a04" stroke-width="0.5"/>')
        elems.append(self.draw_drills_layer())
        elems.append(f'<text x="{self.margin*self.scale}" y="22" fill="#eab308" font-family="monospace" font-weight="bold" font-size="14">STEP 3: TOP COPPER ISOLATION ENGRAVING</text>')
        elems.append(f'<text x="{self.margin*self.scale}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">Tool: 30° / 0.1mm V-Bit | Cut Depth: -0.04mm</text>')
        return self._wrap_svg(elems)

    def build_step4_bottom_copper_svg(self):
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(),
            self.draw_snap_tab(label=f"{self.project.pcb_id} (BOTTOM VIEW)"), self.draw_dimensions()
        ]
        d_cu = self._geom_to_path_d(self.norm_layers.get("copper_bottom"), y_offset=self.tab_w)
        if d_cu:
            elems.append(f'<path d="{d_cu}" fill="#d97706" fill-rule="evenodd" opacity="0.9" stroke="#b45309" stroke-width="0.5"/>')
        elems.append(self.draw_drills_layer())
        elems.append(f'<text x="{self.margin*self.scale}" y="22" fill="#f59e0b" font-family="monospace" font-weight="bold" font-size="14">STEP 4: BOTTOM COPPER ISOLATION ENGRAVING</text>')
        elems.append(f'<text x="{self.margin*self.scale}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">Board Orientation: Flipped horizontally along Y-axis | Tool: 30° V-Bit (-0.04mm)</text>')
        return self._wrap_svg(elems)

    def build_step5_solder_mask_svg(self):
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(),
            self.draw_snap_tab(), self.draw_dimensions()
        ]
        x0, y1 = self._tr(0, self.tab_w + self.h)
        elems.append(f'<rect x="{x0:.1f}" y="{y1:.1f}" width="{self.w*self.scale:.1f}" height="{self.h*self.scale:.1f}" fill="#14532d" opacity="0.75" rx="2"/>')
        d_mask = self._geom_to_path_d(self.norm_layers.get("mask_top"), y_offset=self.tab_w)
        if d_mask:
            elems.append(f'<path d="{d_mask}" fill="#fbbf24" fill-rule="evenodd" stroke="#f59e0b" stroke-width="0.8"/>')
        elems.append(self.draw_drills_layer())
        elems.append(f'<text x="{self.margin*self.scale}" y="22" fill="#22c55e" font-family="monospace" font-weight="bold" font-size="14">STEP 5: SOLDER MASK PAD CLEARING</text>')
        elems.append(f'<text x="{self.margin*self.scale}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">Z-Height Compensation: +35µm | M0 Pause for surface re-touch</text>')
        return self._wrap_svg(elems)

    def build_step6_silkscreen_svg(self):
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(),
            self.draw_snap_tab(), self.draw_dimensions()
        ]
        d_silk = self._geom_to_path_d(self.norm_layers.get("silkscreen_top"), y_offset=self.tab_w)
        if d_silk:
            elems.append(f'<path d="{d_silk}" fill="#ffffff" fill-rule="evenodd" opacity="0.95"/>')
        elems.append(self.draw_drills_layer())
        elems.append(f'<text x="{self.margin*self.scale}" y="22" fill="#f8fafc" font-family="monospace" font-weight="bold" font-size="14">STEP 6: SILKSCREEN LEGEND PRINTING</text>')
        elems.append(f'<text x="{self.margin*self.scale}" y="38" fill="#94a3b8" font-family="monospace" font-size="11"># TODO: Future Flatbed Inkjet DoD Printhead | UV Curable White Ink</text>')
        return self._wrap_svg(elems)

    def build_step7_snap_cutout_svg(self):
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(),
            self.draw_snap_tab(), self.draw_dimensions()
        ]
        x0, y_tab = self._tr(0, self.tab_w)
        _, y_top = self._tr(0, self.tab_w + self.h)
        x1, _ = self._tr(self.w, self.tab_w)
        
        mid_y = self.tab_w + self.h / 2.0
        _, y_mid_b = self._tr(0, mid_y - 1.5)
        _, y_mid_t = self._tr(0, mid_y + 1.5)

        elems.append(f'<line x1="{x0}" y1="{y_tab}" x2="{x1}" y2="{y_tab}" stroke="#ef4444" stroke-width="2.5" stroke-dasharray="8,4"/>')
        elems.append(f'<line x1="{x0}" y1="{y_top}" x2="{x1}" y2="{y_top}" stroke="#ef4444" stroke-width="2.5"/>')
        elems.append(f'<line x1="{x0}" y1="{y_tab}" x2="{x0}" y2="{y_mid_b}" stroke="#ef4444" stroke-width="2.5"/>')
        elems.append(f'<line x1="{x0}" y1="{y_mid_t}" x2="{x0}" y2="{y_top}" stroke="#ef4444" stroke-width="2.5"/>')
        elems.append(f'<line x1="{x1}" y1="{y_tab}" x2="{x1}" y2="{y_mid_b}" stroke="#ef4444" stroke-width="2.5"/>')
        elems.append(f'<line x1="{x1}" y1="{y_mid_t}" x2="{x1}" y2="{y_top}" stroke="#ef4444" stroke-width="2.5"/>')

        elems.append(f'<text x="{self.margin*self.scale}" y="22" fill="#ef4444" font-family="monospace" font-weight="bold" font-size="14">STEP 7: DUAL-SIDED SNAPABLE CUTOUT & 5MM BREAKAWAY</text>')
        elems.append(f'<text x="{self.margin*self.scale}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">Dual-Side Scoring: -0.85mm Top + -0.85mm Bottom | Bridge Tabs: 3.0mm Retention</text>')
        return self._wrap_svg(elems)

    def build_composite_stackup_svg(self):
        elems = [
            self.draw_background(), self.draw_grid(), self.draw_substrate(),
            self.draw_snap_tab(), self.draw_dimensions()
        ]
        d_cu_b = self._geom_to_path_d(self.norm_layers.get("copper_bottom"), y_offset=self.tab_w)
        if d_cu_b:
            elems.append(f'<path d="{d_cu_b}" fill="#b45309" fill-rule="evenodd" opacity="0.45"/>')
        d_cu = self._geom_to_path_d(self.norm_layers.get("copper_top"), y_offset=self.tab_w)
        if d_cu:
            elems.append(f'<path d="{d_cu}" fill="#eab308" fill-rule="evenodd" opacity="0.85" stroke="#ca8a04" stroke-width="0.3"/>')
        d_mask = self._geom_to_path_d(self.norm_layers.get("mask_top"), y_offset=self.tab_w)
        if d_mask:
            elems.append(f'<path d="{d_mask}" fill="none" stroke="#22c55e" stroke-width="0.8" opacity="0.7"/>')
        d_silk = self._geom_to_path_d(self.norm_layers.get("silkscreen_top"), y_offset=self.tab_w)
        if d_silk:
            elems.append(f'<path d="{d_silk}" fill="#ffffff" fill-rule="evenodd" opacity="0.95"/>')
        elems.append(self.draw_drills_layer())
        elems.append(f'<text x="{self.margin*self.scale}" y="22" fill="#38bdf8" font-family="monospace" font-weight="bold" font-size="14">COMPOSITE CIRCUIT STACKUP OVERVIEW</text>')
        elems.append(f'<text x="{self.margin*self.scale}" y="38" fill="#94a3b8" font-family="monospace" font-size="11">{self.project.project_name} ({self.project.pcb_id}) | {self.w:.1f} x {self.h:.1f} mm</text>')
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
# SVG Sheet Blueprint Renderer
# ---------------------------------------------------------------------------

class SvgVisualizer:
    """Renders detailed SVG previews for full A4 panel layouts and individual steps."""
    def __init__(self, width=210.0, height=297.0, scale=3.5):
        self.width = width
        self.height = height
        self.scale = scale
        self.elements = []

    def _tr(self, x, y):
        sx = x * self.scale
        sy = (self.height - y) * self.scale
        return sx, sy

    def draw_board(self):
        w = self.width * self.scale
        h = self.height * self.scale
        self.elements.append(
            f'<rect x="0" y="0" width="{w}" height="{h}" fill="#161b22" stroke="#d29922" stroke-width="2"/>'
        )
        for x in range(0, int(self.width), 10):
            sx, _ = self._tr(x, 0)
            col = "#30363d" if (x % 50 != 0) else "#484f58"
            self.elements.append(f'<line x1="{sx}" y1="0" x2="{sx}" y2="{h}" stroke="{col}" stroke-width="0.8"/>')
        for y in range(0, int(self.height), 10):
            _, sy = self._tr(0, y)
            col = "#30363d" if (y % 50 != 0) else "#484f58"
            self.elements.append(f'<line x1="0" y1="{sy}" x2="{w}" y2="{sy}" stroke="{col}" stroke-width="0.8"/>')

    def draw_holders(self, holders):
        for h in holders:
            x = float(h.get("x", 0))
            y = float(h.get("y", 0))
            gap = float(h.get("holder_gap", 3.0))
            shape = h.get("shape", "circle").lower()
            
            if shape == "circle":
                dia = float(h.get("diameter", 10.0))
                rad = (dia / 2.0)
                tot_r = rad + gap
                sx, sy = self._tr(x, y)
                self.elements.append(
                    f'<circle cx="{sx}" cy="{sy}" r="{tot_r * self.scale}" fill="rgba(248, 81, 73, 0.25)" stroke="#f85149" stroke-width="1.2" stroke-dasharray="3,3"/>'
                )
                self.elements.append(
                    f'<circle cx="{sx}" cy="{sy}" r="{rad * self.scale}" fill="#da3633" stroke="#f85149" stroke-width="1.5"/>'
                )
            else:
                w = float(h.get("width", 10.0))
                l = float(h.get("length", 20.0))
                sx, sy = self._tr(x - w/2.0 - gap, y + l/2.0 + gap)
                sw = (w + 2*gap) * self.scale
                sh = (l + 2*gap) * self.scale
                self.elements.append(
                    f'<rect x="{sx}" y="{sy}" width="{sw}" height="{sh}" fill="rgba(248, 81, 73, 0.25)" stroke="#f85149" stroke-width="1.2" stroke-dasharray="3,3"/>'
                )
                sx2, sy2 = self._tr(x - w/2.0, y + l/2.0)
                self.elements.append(
                    f'<rect x="{sx2}" y="{sy2}" width="{w * self.scale}" height="{l * self.scale}" fill="#da3633" stroke="#f85149" stroke-width="1.5"/>'
                )

    def draw_unit(self, unit: PlacedUnit, layer_cache: Optional[LayerGeometryCache] = None):
        # 1. PCB main body substrate
        pcb_b = unit.pcb_box.bounds
        px, py = self._tr(pcb_b[0], pcb_b[3])
        pw = (pcb_b[2] - pcb_b[0]) * self.scale
        ph = (pcb_b[3] - pcb_b[1]) * self.scale
        self.elements.append(
            f'<rect x="{px:.1f}" y="{py:.1f}" width="{pw:.1f}" height="{ph:.1f}" fill="#082012" stroke="#104e28" stroke-width="1.2"/>'
        )

        # 2. Render actual circuit traces and drill holes
        if layer_cache:
            p_data = layer_cache.get_project_layers(unit.project)
            cu = p_data.get("copper_top")
            if cu:
                polys = cu.geoms if hasattr(cu, "geoms") else [cu]
                for p in polys:
                    if hasattr(p, "exterior") and p.exterior is not None:
                        ext_pts = " ".join([f"{self._tr(*unit.to_sheet_coords(pt[0], pt[1]))[0]:.1f},{self._tr(*unit.to_sheet_coords(pt[0], pt[1]))[1]:.1f}" for pt in p.exterior.coords])
                        d = f"M {ext_pts}"
                        for interior in p.interiors:
                            int_pts = " ".join([f"{self._tr(*unit.to_sheet_coords(pt[0], pt[1]))[0]:.1f},{self._tr(*unit.to_sheet_coords(pt[0], pt[1]))[1]:.1f}" for pt in interior.coords])
                            d += f" M {int_pts}"
                        self.elements.append(f'<path d="{d}" fill="#d4af37" fill-rule="evenodd" opacity="0.9"/>')

            for dia, pts in p_data.get("drills", {}).items():
                rad = max(0.4, (dia / 2.0) * self.scale)
                for (hx, hy) in pts:
                    sx, sy = self._tr(*unit.to_sheet_coords(hx, hy))
                    self.elements.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{rad:.1f}" fill="#c084fc" stroke="#581c87" stroke-width="0.3"/>')

        # 3. 5mm Snap-off ID Tab
        tab_b = unit.tab_box.bounds
        tx, ty = self._tr(tab_b[0], tab_b[3])
        tw = (tab_b[2] - tab_b[0]) * self.scale
        th = (tab_b[3] - tab_b[1]) * self.scale
        self.elements.append(
            f'<rect x="{tx:.1f}" y="{ty:.1f}" width="{tw:.1f}" height="{th:.1f}" fill="rgba(210, 153, 34, 0.35)" stroke="#d29922" stroke-width="1.2"/>'
        )

        # 4. Mouse-bite perforations
        for (bx, by) in unit.mouse_bites:
            sx, sy = self._tr(bx, by)
            self.elements.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{0.4 * self.scale:.1f}" fill="#0d1117" stroke="#d29922" stroke-width="0.5"/>')

        # 5. Directional Arrow
        for stroke in unit.arrow_lines:
            pts = " ".join([f"{self._tr(px, py)[0]:.1f},{self._tr(px, py)[1]:.1f}" for (px, py) in stroke])
            self.elements.append(f'<polyline points="{pts}" fill="none" stroke="#58a6ff" stroke-width="1.8"/>')

        # 6. ID text strokes
        for stroke in unit.id_text_lines:
            pts = " ".join([f"{self._tr(px, py)[0]:.1f},{self._tr(px, py)[1]:.1f}" for (px, py) in stroke])
            self.elements.append(f'<polyline points="{pts}" fill="none" stroke="#f0883e" stroke-width="1.2"/>')

    def draw_y_split(self, y_split):
        _, sy = self._tr(0, y_split)
        w = self.width * self.scale
        self.elements.append(
            f'<line x1="0" y1="{sy}" x2="{w}" y2="{sy}" stroke="#ff7b72" stroke-width="2" stroke-dasharray="6,4"/>'
        )
        self.elements.append(
            f'<text x="{w - 180}" y="{sy - 8}" fill="#ff7b72" font-family="monospace" font-size="12" font-weight="bold">Y-SHIFT BOUNDARY ({y_split:.1f}mm)</text>'
        )

    def draw_polylines(self, polylines, color="#388bfd", width=1.0):
        for poly in polylines:
            if not poly:
                continue
            pts = " ".join([f"{self._tr(p[0], p[1])[0]:.1f},{self._tr(p[0], p[1])[1]:.1f}" for p in poly])
            self.elements.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="{width}"/>')

    def draw_circles(self, points, color="#ff7b72", radius=1.5):
        r_px = radius * self.scale
        for pt in points:
            sx, sy = self._tr(pt[0], pt[1])
            self.elements.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="{r_px:.1f}" fill="none" stroke="{color}" stroke-width="1.2"/>')

    def render(self):
        w = self.width * self.scale
        h = self.height * self.scale
        content = "\n".join(self.elements)
        return f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w} {h}" width="{w}" height="{h}">\n{content}\n</svg>'


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

class ManufacturingPipeline:
    """Executes multi-step G-code, SVG, and metadata generation."""
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
        
        self.sheet_w = float(config.get("copper_sheet", {}).get("sheet_width_mm", 210.0))
        self.sheet_h = float(config.get("copper_sheet", {}).get("sheet_height_mm", 297.0))
        self.y_split = float(config.get("cnc_limits", {}).get("y_split_threshold_mm", 145.0))
        
        os.makedirs(self.out_gcode_dir, exist_ok=True)
        os.makedirs(self.out_svg_dir, exist_ok=True)
        os.makedirs(self.out_guides_dir, exist_ok=True)
        
        self.generated_files = []

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

    def run(self):
        self._clean_old_files()

        # 1. Generate Individual Circuit Gerber SVGs (Step-by-Step Inspector)
        self._generate_circuit_gerber_svgs()

        # 2. Process each Sheet
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

        # 3. Output manufacture_metadata.json
        self._generate_manufacture_metadata_json()

        print(f"\n[SUCCESS] All files generated under '{self.out_gcode_dir}', '{self.out_svg_dir}', and '{self.out_guides_dir}'.")
        return self.generated_files

    def _generate_circuit_gerber_svgs(self):
        """Generates all 8 step-by-step SVGs for each unique circuit project."""
        seen = set()
        for sheet in self.sheets:
            for u in sheet:
                p = u.project
                if p.pcb_id in seen:
                    continue
                seen.add(p.pcb_id)
                p_data = self.layer_cache.get_project_layers(p)
                builder = CircuitSvgBuilder(p, p_data, tab_w=u.tab_w)
                
                step_svgs = [
                    (f"Circuit_{p.pcb_id}_Composite_Stackup.svg", builder.build_composite_stackup_svg()),
                    (f"Circuit_{p.pcb_id}_Step1_Drills.svg", builder.build_step1_drills_svg()),
                    (f"Circuit_{p.pcb_id}_Step2_Vias.svg", builder.build_step2_vias_svg()),
                    (f"Circuit_{p.pcb_id}_Step3_Top_Copper.svg", builder.build_step3_top_copper_svg()),
                    (f"Circuit_{p.pcb_id}_Step4_Bottom_Copper.svg", builder.build_step4_bottom_copper_svg()),
                    (f"Circuit_{p.pcb_id}_Step5_Solder_Mask.svg", builder.build_step5_solder_mask_svg()),
                    (f"Circuit_{p.pcb_id}_Step6_Silkscreen.svg", builder.build_step6_silkscreen_svg()),
                    (f"Circuit_{p.pcb_id}_Step7_Cutout_SnapTab.svg", builder.build_step7_snap_cutout_svg()),
                ]
                for fname, svg_content in step_svgs:
                    fpath = os.path.join(self.out_svg_dir, fname)
                    with open(fpath, "w", encoding="utf-8") as f:
                        f.write(svg_content)
                print(f"  [+] Generated 8 circuit SVGs for {p.pcb_id} ({p.project_name})")

    def _generate_composite_svg(self, sheet_num, units):
        svg = SvgVisualizer(self.sheet_w, self.sheet_h)
        svg.draw_board()
        svg.draw_holders(self.holders_mgr.holders)
        svg.draw_y_split(self.y_split)
        
        # Left guides
        guides = generate_left_guides(self.sheet_w, self.sheet_h, y_shift=self.y_split)
        svg.draw_polylines(guides["lines"], color="#58a6ff", width=1.0)
        svg.draw_polylines(guides["texts"], color="#79c0ff", width=0.8)
        svg.draw_circles(guides["pin_holes"], color="#ff7b72", radius=1.5)

        # Sheet Tag
        sm_color = units[0].project.solder_mask_color if units else "green"
        s_tag = generate_sheet_tag(sheet_num, len(self.sheets), len(units), solder_mask_color=sm_color)
        svg.draw_polylines(s_tag["border"], color="#d29922", width=1.2)
        svg.draw_polylines(s_tag["text"], color="#e3b341", width=1.0)
        svg.draw_polylines(s_tag["arrow"], color="#58a6ff", width=1.5)

        # Units
        for u in units:
            svg.draw_unit(u, self.layer_cache)

        fname = f"Sheet{sheet_num:02d}_Overview_Panel_Blueprint.svg"
        fpath = os.path.join(self.out_svg_dir, fname)
        with open(fpath, "w", encoding="utf-8") as f:
            f.write(svg.render())
        print(f"  [+] Saved panel blueprint SVG: {fname}")

    def _generate_step0_alignment_guides(self, sheet_num, units):
        guides = generate_left_guides(self.sheet_w, self.sheet_h, y_shift=self.y_split)
        sm_color = units[0].project.solder_mask_color if units else "green"
        s_tag = generate_sheet_tag(sheet_num, len(self.sheets), len(units), solder_mask_color=sm_color)
        
        builder = GcodeBuilder(f"Sheet {sheet_num} - Step 0 Left Alignment Guides & Sheet Tag", feed=250.0, cut_z=-0.04)
        builder.header([
            "Step 0: Left Margin Alignment Guides, Millimeter Scale, Datum Pins & Sheet Tag",
            "Tool: 30° / 0.1mm V-Bit for engraving; 3.0mm drill for dowel holes",
            "Purpose: Enables 2-step CNC Y-relocation without Z-axis rotation."
        ])
        builder.spindle_start()
        
        for stroke in guides["lines"]:
            builder.engrave_polyline(stroke)
        for stroke in guides["texts"]:
            builder.engrave_polyline(stroke)
            
        # Engrave Sheet Tag
        for stroke in s_tag["border"]:
            builder.engrave_polyline(stroke)
        for stroke in s_tag["text"]:
            builder.engrave_polyline(stroke)
        for stroke in s_tag["arrow"]:
            builder.engrave_polyline(stroke)

        for (px, py) in guides["pin_holes"]:
            builder.peck_drill(px, py, depth=-1.8, step=0.4)
            
        builder.footer()

        fname = f"Sheet{sheet_num:02d}_Step0_Left_Alignment_Guides.gcode"
        fpath = os.path.join(self.out_gcode_dir, fname)
        with open(fpath, "w", encoding="utf-8") as f:
            f.write(builder.get_text())
        self.generated_files.append(fpath)
        print(f"  [+] Step 0: {fname}")

    def _generate_step1_drilling(self, sheet_num, units):
        tool_drills_front = {}
        tool_drills_back = {}

        for u in units:
            u_data = self.layer_cache.get_project_layers(u.project)
            for dia, pts in u_data.get("drills", {}).items():
                for (hx, hy) in pts:
                    sx, sy = u.to_sheet_coords(hx, hy)
                    if sy <= self.y_split:
                        tool_drills_front.setdefault(dia, []).append((sx, sy))
                    else:
                        tool_drills_back.setdefault(dia, []).append((sx, sy - self.y_split))

            # 5mm Snap tab mouse bites (0.8mm drill)
            for (bx, by) in u.mouse_bites:
                if by <= self.y_split:
                    tool_drills_front.setdefault(0.8, []).append((bx, by))
                else:
                    tool_drills_back.setdefault(0.8, []).append((bx, by - self.y_split))

        all_tools = sorted(list(set(list(tool_drills_front.keys()) + list(tool_drills_back.keys()))))

        for dia in all_tools:
            # Step 1A: Front Half
            f_pts = tool_drills_front.get(dia, [])
            if f_pts:
                b_front = GcodeBuilder(f"Sheet {sheet_num} - Step 1A Front Top Drills {dia:.2f}mm", feed=150.0, plunge=80.0, spindle_rpm=10000)
                b_front.header([
                    f"Step 1A: FRONT HALF Top Layer Drilling - Tool: {dia:.2f}mm Carbide Drill",
                    f"Hole Count: {len(f_pts)}",
                    "Cycle: Peck drilling (0.4mm depth increments)"
                ])
                b_front.spindle_start()
                for (x, y) in f_pts:
                    b_front.peck_drill(x, y, depth=-1.8, step=0.4)
                b_front.footer()

                fname = f"Sheet{sheet_num:02d}_Step1A_Front_Top_Drills_{dia:.2f}mm.gcode"
                fpath = os.path.join(self.out_gcode_dir, fname)
                with open(fpath, "w", encoding="utf-8") as f:
                    f.write(b_front.get_text())
                self.generated_files.append(fpath)

            # Step 1B: Back Half
            b_pts = tool_drills_back.get(dia, [])
            if b_pts:
                b_back = GcodeBuilder(f"Sheet {sheet_num} - Step 1B Back Top Drills {dia:.2f}mm", feed=150.0, plunge=80.0, spindle_rpm=10000)
                b_back.header([
                    f"Step 1B: BACK HALF Top Layer Drilling - Tool: {dia:.2f}mm Carbide Drill",
                    f"*** ACTION: Slide sheet forward by {self.y_split:.1f} mm along Y-axis! ***",
                    f"Hole Count: {len(b_pts)}"
                ])
                b_back.spindle_start()
                for (x, y) in b_pts:
                    b_back.peck_drill(x, y, depth=-1.8, step=0.4)
                b_back.footer()

                fname = f"Sheet{sheet_num:02d}_Step1B_Back_Top_Drills_{dia:.2f}mm.gcode"
                fpath = os.path.join(self.out_gcode_dir, fname)
                with open(fpath, "w", encoding="utf-8") as f:
                    f.write(b_back.get_text())
                self.generated_files.append(fpath)

        print(f"  [+] Step 1: Generated drill files for {len(all_tools)} standardized diameters.")

    def _generate_step2_vias_guide(self, sheet_num):
        content = (
            f"=== Sheet {sheet_num} - Step 2: Chemical Through-Hole Via Metallization Guide ===\n"
            "This step is performed chemically after drilling and before copper isolation.\n"
            "Process sequence:\n"
            "  1. Ultrasonic clean & acid desmear\n"
            "  2. Conductive carbon-black / palladium catalyst activation\n"
            "  3. Micro-etching bare copper\n"
            "  4. Acid copper electroplating bath (CuSO4 + H2SO4 @ 1.5A/dm² for 45 min = 25µm Cu barrel)\n"
            "  5. DI water rinse and oven dry\n"
        )
        fname = f"Sheet{sheet_num:02d}_Step2_Chemical_Vias_Guide.txt"
        fpath = os.path.join(self.out_gcode_dir, fname)
        with open(fpath, "w", encoding="utf-8") as f:
            f.write(content)
        self.generated_files.append(fpath)
        print("  [+] Step 2: Via Chemical Process Guide written.")

    def _generate_step3_top_isolation(self, sheet_num, units):
        import gerber_to_gcode
        front_polylines = []
        back_polylines = []

        for u in units:
            # 5mm Snap-off ID text and arrow
            unit_strokes = u.arrow_lines + u.id_text_lines
            for stroke in unit_strokes:
                avg_y = sum(p[1] for p in stroke) / len(stroke)
                if avg_y <= self.y_split:
                    front_polylines.append(stroke)
                else:
                    back_polylines.append([(p[0], p[1] - self.y_split) for p in stroke])

            # Top copper isolation
            u_data = self.layer_cache.get_project_layers(u.project)
            cu = u_data.get("copper_top")
            if cu:
                try:
                    mb = (0.0, 0.0, u_data["width"], u_data["height"])
                    gen = gerber_to_gcode.ToolpathGenerator(cu, master_bounds=mb, tool_dia=0.2, passes=1)
                    iso_paths, _, _ = gen.generate()
                    for poly in iso_paths:
                        shifted = [u.to_sheet_coords(p[0], p[1]) for p in poly]
                        avg_y = sum(p[1] for p in shifted) / len(shifted)
                        if avg_y <= self.y_split:
                            front_polylines.append(shifted)
                        else:
                            back_polylines.append([(p[0], p[1] - self.y_split) for p in shifted])
                except Exception as e:
                    print(f"[WARN] Error isolating top copper {u.unit_id}: {e}")

        if front_polylines:
            b_3a = GcodeBuilder(f"Sheet {sheet_num} - Step 3A Front Top Copper Isolation", feed=300.0, cut_z=-0.04)
            b_3a.header(["Step 3A: FRONT HALF Top Copper Layer Isolation Routing", "Tool: 30° / 0.1mm V-Bit"])
            b_3a.spindle_start()
            for poly in front_polylines:
                b_3a.engrave_polyline(poly)
            b_3a.footer()
            fname_3a = f"Sheet{sheet_num:02d}_Step3A_Front_Top_Copper_Isolation.gcode"
            fpath_3a = os.path.join(self.out_gcode_dir, fname_3a)
            with open(fpath_3a, "w", encoding="utf-8") as f:
                f.write(b_3a.get_text())
            self.generated_files.append(fpath_3a)

        if back_polylines:
            b_3b = GcodeBuilder(f"Sheet {sheet_num} - Step 3B Back Top Copper Isolation", feed=300.0, cut_z=-0.04)
            b_3b.header([
                "Step 3B: BACK HALF Top Copper Layer Isolation Routing",
                f"*** ACTION: Slide sheet forward by {self.y_split:.1f} mm along Y-axis! ***"
            ])
            b_3b.spindle_start()
            for poly in back_polylines:
                b_3b.engrave_polyline(poly)
            b_3b.footer()
            fname_3b = f"Sheet{sheet_num:02d}_Step3B_Back_Top_Copper_Isolation.gcode"
            fpath_3b = os.path.join(self.out_gcode_dir, fname_3b)
            with open(fpath_3b, "w", encoding="utf-8") as f:
                f.write(b_3b.get_text())
            self.generated_files.append(fpath_3b)

        print("  [+] Step 3: Top isolation routing generated.")

    def _generate_step4_bottom_isolation(self, sheet_num, units):
        import gerber_to_gcode
        front_polylines = []
        back_polylines = []

        for u in units:
            u_data = self.layer_cache.get_project_layers(u.project)
            cu_b = u_data.get("copper_bottom")
            if cu_b:
                try:
                    mb = (0.0, 0.0, u_data["width"], u_data["height"])
                    gen = gerber_to_gcode.ToolpathGenerator(cu_b, master_bounds=mb, tool_dia=0.2, passes=1)
                    iso_paths, _, _ = gen.generate()
                    for poly in iso_paths:
                        # Bottom layer is flipped along X: X' = sheet_w - X_top
                        shifted = [(self.sheet_w - pt[0], pt[1]) for pt in [u.to_sheet_coords(p[0], p[1]) for p in poly]]
                        avg_y = sum(p[1] for p in shifted) / len(shifted)
                        if avg_y <= self.y_split:
                            front_polylines.append(shifted)
                        else:
                            back_polylines.append([(p[0], p[1] - self.y_split) for p in shifted])
                except Exception as e:
                    print(f"[WARN] Error isolating bottom copper: {e}")

        if front_polylines:
            b_4a = GcodeBuilder(f"Sheet {sheet_num} - Step 4A Front Bottom Copper Isolation", feed=300.0, cut_z=-0.04)
            b_4a.header(["Step 4A: FRONT HALF Bottom Copper Isolation", "Board Orientation: FLIPPED horizontally across Y-axis"])
            b_4a.spindle_start()
            for poly in front_polylines:
                b_4a.engrave_polyline(poly)
            b_4a.footer()
            fname_4a = f"Sheet{sheet_num:02d}_Step4A_Front_Bottom_Copper_Isolation.gcode"
            fpath_4a = os.path.join(self.out_gcode_dir, fname_4a)
            with open(fpath_4a, "w", encoding="utf-8") as f:
                f.write(b_4a.get_text())
            self.generated_files.append(fpath_4a)

        if back_polylines:
            b_4b = GcodeBuilder(f"Sheet {sheet_num} - Step 4B Back Bottom Copper Isolation", feed=300.0, cut_z=-0.04)
            b_4b.header([
                "Step 4B: BACK HALF Bottom Copper Isolation",
                f"*** ACTION: Slide flipped sheet forward by {self.y_split:.1f} mm along Y-axis! ***"
            ])
            b_4b.spindle_start()
            for poly in back_polylines:
                b_4b.engrave_polyline(poly)
            b_4b.footer()
            fname_4b = f"Sheet{sheet_num:02d}_Step4B_Back_Bottom_Copper_Isolation.gcode"
            fpath_4b = os.path.join(self.out_gcode_dir, fname_4b)
            with open(fpath_4b, "w", encoding="utf-8") as f:
                f.write(b_4b.get_text())
            self.generated_files.append(fpath_4b)

        print("  [+] Step 4: Bottom isolation routing generated.")

    def _generate_step5_solder_mask(self, sheet_num, units):
        import gerber_to_gcode
        # Top mask
        front_top_pads = []
        back_top_pads = []
        # Bottom mask
        front_bot_pads = []
        back_bot_pads = []

        for u in units:
            u_data = self.layer_cache.get_project_layers(u.project)
            # Top Mask
            m_top = u_data.get("mask_top")
            if m_top:
                try:
                    mb = (0.0, 0.0, u_data["width"], u_data["height"])
                    gen = gerber_to_gcode.ToolpathGenerator(m_top, master_bounds=mb, tool_dia=0.2, passes=1)
                    paths, _, _ = gen.generate()
                    for poly in paths:
                        shifted = [u.to_sheet_coords(p[0], p[1]) for p in poly]
                        avg_y = sum(p[1] for p in shifted) / len(shifted)
                        if avg_y <= self.y_split:
                            front_top_pads.append(shifted)
                        else:
                            back_top_pads.append([(p[0], p[1] - self.y_split) for p in shifted])
                except Exception as e:
                    print(f"[WARN] Error in top solder mask: {e}")

            # Bottom Mask
            m_bot = u_data.get("mask_bottom")
            if m_bot:
                try:
                    mb = (0.0, 0.0, u_data["width"], u_data["height"])
                    gen = gerber_to_gcode.ToolpathGenerator(m_bot, master_bounds=mb, tool_dia=0.2, passes=1)
                    paths, _, _ = gen.generate()
                    for poly in paths:
                        shifted = [(self.sheet_w - pt[0], pt[1]) for pt in [u.to_sheet_coords(p[0], p[1]) for p in poly]]
                        avg_y = sum(p[1] for p in shifted) / len(shifted)
                        if avg_y <= self.y_split:
                            front_bot_pads.append(shifted)
                        else:
                            back_bot_pads.append([(p[0], p[1] - self.y_split) for p in shifted])
                except Exception as e:
                    print(f"[WARN] Error in bottom solder mask: {e}")

        # Step 5A: Front Top Mask
        if front_top_pads:
            b_5a = GcodeBuilder(f"Sheet {sheet_num} - Step 5A Front Top Mask Clear", feed=300.0, cut_z=-0.06)
            b_5a.header([
                "Step 5A: FRONT HALF Top Solder Mask Pad Clearing",
                "CRITICAL Z-HEIGHT NOTICE: Solder mask adds ~35µm. Re-zero Z onto mask surface!",
                "M0 (PAUSE: Verify Z0 touch-off on cured solder mask film)"
            ])
            b_5a.lines.append("M0 ; Operator pause for Z-probe verification")
            b_5a.spindle_start()
            for pad in front_top_pads:
                b_5a.engrave_polyline(pad)
            b_5a.footer()
            fpath_5a = os.path.join(self.out_gcode_dir, f"Sheet{sheet_num:02d}_Step5A_Front_Top_Mask_Clear.gcode")
            with open(fpath_5a, "w", encoding="utf-8") as f:
                f.write(b_5a.get_text())
            self.generated_files.append(fpath_5a)

        # Step 5B: Back Top Mask
        if back_top_pads:
            b_5b = GcodeBuilder(f"Sheet {sheet_num} - Step 5B Back Top Mask Clear", feed=300.0, cut_z=-0.06)
            b_5b.header([
                "Step 5B: BACK HALF Top Solder Mask Pad Clearing",
                f"*** ACTION: Slide sheet forward by {self.y_split:.1f} mm along Y-axis! ***",
                "M0 (PAUSE: Verify Z0 touch-off on cured solder mask film)"
            ])
            b_5b.lines.append("M0 ; Operator pause for Z-probe verification")
            b_5b.spindle_start()
            for pad in back_top_pads:
                b_5b.engrave_polyline(pad)
            b_5b.footer()
            fpath_5b = os.path.join(self.out_gcode_dir, f"Sheet{sheet_num:02d}_Step5B_Back_Top_Mask_Clear.gcode")
            with open(fpath_5b, "w", encoding="utf-8") as f:
                f.write(b_5b.get_text())
            self.generated_files.append(fpath_5b)

        # Step 5C: Front Bottom Mask
        if front_bot_pads:
            b_5c = GcodeBuilder(f"Sheet {sheet_num} - Step 5C Front Bottom Mask Clear", feed=300.0, cut_z=-0.06)
            b_5c.header([
                "Step 5C: FRONT HALF Bottom Solder Mask Pad Clearing",
                "Board Orientation: FLIPPED horizontally across Y-axis.",
                "M0 (PAUSE: Verify Z0 touch-off on cured bottom solder mask)"
            ])
            b_5c.lines.append("M0 ; Operator pause for Z-probe verification")
            b_5c.spindle_start()
            for pad in front_bot_pads:
                b_5c.engrave_polyline(pad)
            b_5c.footer()
            fpath_5c = os.path.join(self.out_gcode_dir, f"Sheet{sheet_num:02d}_Step5C_Front_Bottom_Mask_Clear.gcode")
            with open(fpath_5c, "w", encoding="utf-8") as f:
                f.write(b_5c.get_text())
            self.generated_files.append(fpath_5c)

        # Step 5D: Back Bottom Mask
        if back_bot_pads:
            b_5d = GcodeBuilder(f"Sheet {sheet_num} - Step 5D Back Bottom Mask Clear", feed=300.0, cut_z=-0.06)
            b_5d.header([
                "Step 5D: BACK HALF Bottom Solder Mask Pad Clearing",
                f"*** ACTION: Slide flipped sheet forward by {self.y_split:.1f} mm along Y-axis! ***",
                "M0 (PAUSE: Verify Z0 touch-off on cured bottom solder mask)"
            ])
            b_5d.lines.append("M0 ; Operator pause for Z-probe verification")
            b_5d.spindle_start()
            for pad in back_bot_pads:
                b_5d.engrave_polyline(pad)
            b_5d.footer()
            fpath_5d = os.path.join(self.out_gcode_dir, f"Sheet{sheet_num:02d}_Step5D_Back_Bottom_Mask_Clear.gcode")
            with open(fpath_5d, "w", encoding="utf-8") as f:
                f.write(b_5d.get_text())
            self.generated_files.append(fpath_5d)

        print("  [+] Step 5: Solder mask pad clearing generated (Top & Bottom, Front & Back).")

    def _generate_step6_silkscreen(self, sheet_num):
        content = (
            f"=== Sheet {sheet_num} - Step 6: Silkscreen Printing Roadmap ===\n"
            "# TODO: Future flatbed drop-on-demand (DoD) inkjet printer integration.\n"
            "Silkscreen component legends (R1, C1, U1, pin-1 markers) do not require CNC engraving.\n"
            "Will be deposited directly using UV-curable white pigment ink with LED-395nm pinning.\n"
        )
        fname = f"Sheet{sheet_num:02d}_Step6_Silkscreen_TODO.txt"
        fpath = os.path.join(self.out_gcode_dir, fname)
        with open(fpath, "w", encoding="utf-8") as f:
            f.write(content)
        self.generated_files.append(fpath)
        print("  [+] Step 6: Silkscreen TODO comment and roadmap documented.")

    def _generate_step7_snapable_cutout(self, sheet_num, units):
        """
        Step 7: Dual-Sided Snapable Perimeter Cutout.
        - Routes partial depth (-0.85mm top & bottom) with 3.0mm retention bridge tabs.
        - Uses single-pass cut along shared boundaries between tightly connected PCB designs!
        """
        front_cuts = []
        back_cuts = []
        depth_per_side = -0.85
        tab_len = 3.0

        # Separate units by half
        front_units = [u for u in units if u.y + u.h <= self.y_split]
        back_units = [u for u in units if u.y >= self.y_split]

        def build_half_cutouts(half_units, is_back=False):
            segs = []
            if not half_units:
                return segs
            
            # Find bounds of cluster
            min_x = min(u.x for u in half_units)
            max_x = max(u.x + u.w for u in half_units)
            min_y = min(u.y for u in half_units)
            max_y = max(u.y + u.h for u in half_units)
            y_off = self.y_split if is_back else 0.0

            # 1. Outer cluster bottom edge with bridge tab
            mid_x = (min_x + max_x) / 2.0
            segs.append([(min_x, min_y - y_off), (mid_x - tab_len/2.0, min_y - y_off)])
            segs.append([(mid_x + tab_len/2.0, min_y - y_off), (max_x, min_y - y_off)])

            # 2. Outer cluster top edge with bridge tab
            segs.append([(min_x, max_y - y_off), (mid_x - tab_len/2.0, max_y - y_off)])
            segs.append([(mid_x + tab_len/2.0, max_y - y_off), (max_x, max_y - y_off)])

            # 3. Outer left edge with bridge tab
            mid_y = (min_y + max_y) / 2.0
            segs.append([(min_x, min_y - y_off), (min_x, mid_y - tab_len/2.0 - y_off)])
            segs.append([(min_x, mid_y + tab_len/2.0 - y_off), (min_x, max_y - y_off)])

            # 4. Outer right edge with bridge tab
            segs.append([(max_x, min_y - y_off), (max_x, mid_y - tab_len/2.0 - y_off)])
            segs.append([(max_x, mid_y + tab_len/2.0 - y_off), (max_x, max_y - y_off)])

            # 5. Shared internal boundary cuts (Single-pass cut between adjacent units!)
            shared_xs = sorted(list(set(u.x + u.w for u in half_units if u.x + u.w < max_x - 0.1)))
            for sx in shared_xs:
                # Single pass cut along the shared seam with a retention tab
                segs.append([(sx, min_y - y_off), (sx, mid_y - tab_len/2.0 - y_off)])
                segs.append([(sx, mid_y + tab_len/2.0 - y_off), (sx, max_y - y_off)])

            return segs

        front_cuts = build_half_cutouts(front_units, is_back=False)
        back_cuts = build_half_cutouts(back_units, is_back=True)

        # 7A: Front Top Cutout
        if front_cuts:
            b_7a = GcodeBuilder(f"Sheet {sheet_num} - Step 7A Front Top Snapable Cutout", feed=200.0, cut_z=depth_per_side)
            b_7a.header([
                "Step 7A: FRONT HALF Top Cutout Partial Depth (-0.85mm)",
                "Tightly Packed Zero-Gap: Single-pass cut along shared boundaries!",
                "Bridge Tabs Active: Prevents loose boards from flying away during spindle rotation!"
            ])
            b_7a.spindle_start()
            for seg in front_cuts:
                b_7a.engrave_polyline(seg)
            b_7a.footer()
            fname_7a = f"Sheet{sheet_num:02d}_Step7A_Front_Top_Snap_Cutout.gcode"
            fpath_7a = os.path.join(self.out_gcode_dir, fname_7a)
            with open(fpath_7a, "w", encoding="utf-8") as f:
                f.write(b_7a.get_text())
            self.generated_files.append(fpath_7a)

        # 7B: Back Top Cutout
        if back_cuts:
            b_7b = GcodeBuilder(f"Sheet {sheet_num} - Step 7B Back Top Snapable Cutout", feed=200.0, cut_z=depth_per_side)
            b_7b.header([
                "Step 7B: BACK HALF Top Cutout Partial Depth (-0.85mm)",
                f"*** ACTION: Shift sheet forward by {self.y_split:.1f} mm along Y-axis! ***"
            ])
            b_7b.spindle_start()
            for seg in back_cuts:
                b_7b.engrave_polyline(seg)
            b_7b.footer()
            fname_7b = f"Sheet{sheet_num:02d}_Step7B_Back_Top_Snap_Cutout.gcode"
            fpath_7b = os.path.join(self.out_gcode_dir, fname_7b)
            with open(fpath_7b, "w", encoding="utf-8") as f:
                f.write(b_7b.get_text())
            self.generated_files.append(fpath_7b)

        # 7C & 7D: Bottom Side Cutout (Flipped Board)
        mirrored_front = [[(self.sheet_w - p[0], p[1]) for p in seg] for seg in front_cuts]
        mirrored_back = [[(self.sheet_w - p[0], p[1]) for p in seg] for seg in back_cuts]

        if mirrored_front:
            b_7c = GcodeBuilder(f"Sheet {sheet_num} - Step 7C Front Bottom Snapable Cutout", feed=200.0, cut_z=depth_per_side)
            b_7c.header([
                "Step 7C: FRONT HALF Bottom Cutout Partial Depth (-0.85mm)",
                "Board Orientation: FLIPPED horizontally along Y-axis.",
                "Completes dual-sided scoring (-0.85mm top + -0.85mm bottom = 1.70mm total cut through 1.6mm FR4)"
            ])
            b_7c.spindle_start()
            for seg in mirrored_front:
                b_7c.engrave_polyline(seg)
            b_7c.footer()
            fname_7c = f"Sheet{sheet_num:02d}_Step7C_Front_Bottom_Snap_Cutout.gcode"
            fpath_7c = os.path.join(self.out_gcode_dir, fname_7c)
            with open(fpath_7c, "w", encoding="utf-8") as f:
                f.write(b_7c.get_text())
            self.generated_files.append(fpath_7c)

        if mirrored_back:
            b_7d = GcodeBuilder(f"Sheet {sheet_num} - Step 7D Back Bottom Snapable Cutout", feed=200.0, cut_z=depth_per_side)
            b_7d.header([
                "Step 7D: BACK HALF Bottom Cutout Partial Depth (-0.85mm)",
                f"*** ACTION: Shift flipped sheet forward by {self.y_split:.1f} mm along Y-axis! ***"
            ])
            b_7d.spindle_start()
            for seg in mirrored_back:
                b_7d.engrave_polyline(seg)
            b_7d.footer()
            fname_7d = f"Sheet{sheet_num:02d}_Step7D_Back_Bottom_Snap_Cutout.gcode"
            fpath_7d = os.path.join(self.out_gcode_dir, fname_7d)
            with open(fpath_7d, "w", encoding="utf-8") as f:
                f.write(b_7d.get_text())
            self.generated_files.append(fpath_7d)

        print("  [+] Step 7: Dual-sided snapable cutout files generated.")

    def _generate_manufacture_metadata_json(self):
        """Generates comprehensive manufacture_metadata.json detailing all boards and sheets."""
        sheets_data = []
        total_units = sum(len(s) for s in self.sheets)

        for s_idx, u_list in enumerate(self.sheets):
            s_num = s_idx + 1
            sm_color = u_list[0].project.solder_mask_color if u_list else "green"
            sheet_tag_lbl = f"SHEET {s_num:02d}/{len(self.sheets):02d} | UNITS: {len(u_list)} | MASK: {sm_color.upper()}"
            
            u_data = []
            for u in u_list:
                p_layers = self.layer_cache.get_project_layers(u.project)
                drl_count = sum(len(pts) for pts in p_layers.get("drills", {}).values())
                half = "front" if (u.y + u.h <= self.y_split) else "back"
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
                        "attached_edge": "width",
                        "has_directional_arrow": u.engrave_arrow,
                        "has_mouse_bites": True
                    },
                    "half": half,
                    "solder_mask_color": u.project.solder_mask_color,
                    "drill_holes_count": drl_count
                })

            sheets_data.append({
                "sheet_number": s_num,
                "sheet_tag": sheet_tag_lbl,
                "solder_mask_color": sm_color,
                "unit_count": len(u_list),
                "units": u_data
            })

        metadata = {
            "$schema_version": "1.0.0",
            "production_summary": {
                "batch_name": "A4_PCB_PANEL_BATCH",
                "total_sheets": len(self.sheets),
                "total_units": total_units,
                "sheet_dimensions_mm": {"width": self.sheet_w, "height": self.sheet_h},
                "cnc_bed_travel_mm": {"x": 275.0, "y": 170.0},
                "y_split_threshold_mm": self.y_split,
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


# ---------------------------------------------------------------------------
# Interactive HTML Dashboard & Setup Checklist Generator
# ---------------------------------------------------------------------------

def build_html_dashboard_content(sheets, holders_mgr: HolderManager, generated_files,
                                 config: Dict, fluidnc_ip="192.168.8.109",
                                 img_prefix="../manufacture_svg/"):
    """Builds the HTML markup for the interactive fabrication dashboard."""
    total_units = sum(len(s) for s in sheets)

    # Unique projects for the Circuit Gerber Inspector
    projects_dict = {}
    for s in sheets:
        for u in s:
            if u.project.pcb_id not in projects_dict:
                projects_dict[u.project.pcb_id] = u.project
    unique_projects = list(projects_dict.values())

    sheet_tabs_html = []
    sheet_panels_html = []
    for s_idx, u_list in enumerate(sheets):
        s_num = s_idx + 1
        active_class = "border-blue-500 text-blue-400 bg-blue-500/10" if s_idx == 0 else "border-transparent text-gray-400 hover:text-white"
        display_style = "block" if s_idx == 0 else "none"
        sm_color = u_list[0].project.solder_mask_color.upper() if u_list else "GREEN"

        sheet_tabs_html.append(
            f'<button onclick="selectSheet({s_num})" id="tab-btn-{s_num}" '
            f'class="px-4 py-2 text-sm font-semibold border-b-2 transition {active_class}">'
            f'Sheet {s_num:02d} ({len(u_list)} Units | {sm_color})'
            f'</button>'
        )

        units_rows = "".join([
            f'<tr class="border-b border-gray-800 hover:bg-gray-800/30 font-mono text-xs">'
            f'<td class="py-2 px-3 text-yellow-400 font-bold">{u.unit_id}</td>'
            f'<td class="py-2 px-3 text-gray-300">{u.project.project_name}</td>'
            f'<td class="py-2 px-3 text-gray-400">{u.pcb_w:.1f} x {u.pcb_h:.1f} mm</td>'
            f'<td class="py-2 px-3 text-blue-400">{u.rotation_deg}° (Along Width)</td>'
            f'<td class="py-2 px-3 text-gray-400">X: {u.x:.1f}, Y: {u.y:.1f}</td>'
            f'<td class="py-2 px-3"><span class="px-2 py-0.5 rounded bg-green-500/20 text-green-300">5mm Tab + Arrow (0mm Gap)</span></td>'
            f'</tr>' for u in u_list
        ])

        sheet_panels_html.append(f'''
        <div id="sheet-panel-{s_num}" style="display: {display_style};" class="sheet-content">
          <div class="flex items-center justify-between mb-4">
            <h3 class="font-bold text-lg text-white flex items-center space-x-2">
              <span>Sheet {s_num:02d} Panel Layout</span>
              <span class="text-xs bg-gray-800 text-gray-300 px-2 py-0.5 rounded">210 x 297 mm</span>
              <span class="text-xs bg-green-900/40 text-green-400 border border-green-700/40 px-2 py-0.5 rounded">{sm_color} SOLDERMASK</span>
            </h3>
            <span class="text-xs text-yellow-500 bg-yellow-500/10 px-2.5 py-1 rounded-full border border-yellow-500/20 font-mono">
              Y-Split: 145.0 mm | {len(u_list)} Units Nested (Zero Gap Single Cut)
            </span>
          </div>

          <div class="svg-container bg-black p-3 rounded-xl border border-gray-800 flex justify-center items-center">
            <img src="{img_prefix}Sheet{s_num:02d}_Overview_Panel_Blueprint.svg" alt="Sheet {s_num} Blueprint" class="max-h-[680px] object-contain rounded">
          </div>

          <div class="mt-4 overflow-x-auto">
            <h4 class="text-xs font-bold text-gray-400 uppercase tracking-wider mb-2">Nested Boards on Sheet {s_num:02d}</h4>
            <table class="w-full text-left">
              <thead class="text-gray-400 border-b border-gray-800 bg-gray-950/40 text-xs">
                <tr>
                  <th class="py-2 px-3">Unit ID</th>
                  <th class="py-2 px-3">Project</th>
                  <th class="py-2 px-3">Dimensions</th>
                  <th class="py-2 px-3">Rotation</th>
                  <th class="py-2 px-3">Panel Origin</th>
                  <th class="py-2 px-3">Snap-off Feature</th>
                </tr>
              </thead>
              <tbody>
                {units_rows}
              </tbody>
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
    chk_items = checklist_data.get("items", {})

    # Build Checklist HTML rows
    chk_rows_html = []
    for item_id, item in chk_items.items():
        checked_attr = "checked" if item.get("checked", False) else ""
        checked_class = "line-through text-gray-500" if item.get("checked", False) else "text-white"
        chk_rows_html.append(f'''
        <div class="flex items-start space-x-3 p-3 bg-gray-950/60 rounded-xl border border-gray-800 hover:border-blue-500/40 transition">
          <input type="checkbox" id="{item_id}" {checked_attr} onchange="toggleChecklistItem('{item_id}', this.checked)"
                 class="mt-1 w-4 h-4 rounded text-blue-600 bg-gray-900 border-gray-700 focus:ring-blue-500 focus:ring-offset-gray-900 cursor-pointer">
          <div class="flex-1">
            <div class="flex items-center justify-between">
              <label for="{item_id}" id="label-{item_id}" class="text-sm font-semibold cursor-pointer {checked_class}">
                {item.get("label", item_id)}
              </label>
              <span class="text-[10px] uppercase font-mono px-2 py-0.5 rounded bg-gray-800 text-gray-400 border border-gray-700">
                {item.get("category", "Setup")}
              </span>
            </div>
            <p class="text-xs text-gray-400 mt-0.5">{item.get("description", "")}</p>
          </div>
        </div>
        ''')

    # Practical Issues Matrix
    issues = [
        {
            "badge": "Solved via Probing & Clamps",
            "title": "1. FR4 Board Bowing & Z-Height Variations",
            "desc": "A4 copper laminate sheets (210x297mm) exhibit up to 0.3mm bowing across the bed, causing trace disconnects or snapped 0.1mm V-bits.",
            "sol": "Built-in conservative cut depth (-0.04mm), autoleveling height map integration (G38.2 probe grid), and high-rigidity low-profile corner clamps from pcb_holders.json."
        },
        {
            "badge": "Solved via Step 0 Guides",
            "title": "2. Y-Axis Shift Repeatability (2-Step 170mm Bed)",
            "desc": "Manually shifting the A4 sheet 145mm forward can introduce skew / angular tilt, misaligning front and back half features.",
            "sol": "Dedicated Left-Edge Step 0 Alignment Guides: Engraved millimeter scale, shift datum line at Y=145.0mm, and dual 3.0mm reference dowel pin holes for rigid mechanical registration."
        },
        {
            "badge": "Solved via Dual-Side Scoring",
            "title": "3. Dangerous Flying Boards & Endmill Snapping",
            "desc": "Full perimeter through-cutting in 1.6mm FR4 causes loose boards to vibrate into the 10,000 RPM endmill, breaking bits.",
            "sol": "Dual-Sided Snapable Cutout: Routing partial depth (-0.85mm) top AND bottom sides, maintaining retention bridge tabs and mouse-bite perforations. Zero flying boards; snap out by hand."
        },
        {
            "badge": "Solved via Mouse Bites",
            "title": "4. 5mm Board ID Tag Breakaway Cleanliness",
            "desc": "Snapping off ID tags can tear copper traces or delaminate the FR4 board edge.",
            "sol": "Automated 0.8mm mouse-bite drill line with 1.5mm pitch along the PCB width seam + engraved directional arrows pointing directly into the PCB body."
        },
        {
            "badge": "Solved via Re-Zero Prompt",
            "title": "5. Solder Mask Surface Height Discrepancy",
            "desc": "Cured UV solder mask adds 25-50µm of thickness, causing shallow pad openings if bare copper Z0 is reused.",
            "sol": "Step 5 G-code incorporates an explicit operator prompt (M0) and dedicated solder mask clearance depth (-0.06mm) to guarantee clean, exposed pads."
        },
        {
            "badge": "Solved via Keep-Out Checks",
            "title": "6. Fixture Clamp Collisions & Machine Alarms",
            "desc": "Rapid machine moves hitting clamps or commanding Y > 170mm causing FluidNC soft-limit alarms.",
            "sol": "Full collision avoidance engine enforcing holder_gap from pcb_holders.json, and strict coordinate clamping (Y' = Y - 145mm <= 152mm)."
        }
    ]

    issues_html = "".join([f'''
    <div class="bg-gray-900 border border-gray-800 p-5 rounded-2xl flex flex-col justify-between">
      <div>
        <div class="flex items-center justify-between mb-2">
          <span class="text-xs font-semibold text-blue-400 bg-blue-900/40 px-2.5 py-0.5 rounded-full">{iss["badge"]}</span>
        </div>
        <h3 class="font-bold text-white text-base mb-1">{iss["title"]}</h3>
        <p class="text-xs text-gray-400 mb-3">{iss["desc"]}</p>
      </div>
      <div class="p-3 bg-gray-950 rounded-xl border border-gray-800/80 text-xs text-gray-300">
        <strong class="text-blue-400 font-semibold">Solution:</strong> {iss["sol"]}
      </div>
    </div>
    ''' for iss in issues])

    first_proj_id = unique_projects[0].pcb_id if unique_projects else "PCB-AC-01"

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>CNC PCB Manufacturing & Panelization Dashboard</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <style>
    body {{ background-color: #0b0e14; color: #e6edf3; font-family: ui-sans-serif, system-ui, sans-serif; }}
    .svg-container svg, .svg-container img {{ width: 100%; height: auto; max-height: 700px; border-radius: 0.5rem; }}
    .step-btn.active {{ border-color: #3b82f6; background-color: rgba(59, 130, 246, 0.15); color: #60a5fa; }}
  </style>
  <script>
    let imgPrefix = '{img_prefix}';
    let currentProject = '{first_proj_id}';
    let currentStep = 'Composite_Stackup';

    const stepInfo = {{
      'Composite_Stackup': {{
        title: 'Composite Circuit Stackup Overview',
        tool: 'All layers combined',
        depth: 'N/A', feed: 'N/A', rpm: 'N/A',
        instructions: 'Visual inspection of FR4 substrate, copper traces, mask apertures, silkscreen legends, drills, and the 5mm breakaway tab attached along the PCB width side.'
      }},
      'Step1_Drills': {{
        title: 'Step 1: Multi-Diameter CNC Drilling',
        tool: 'Carbide Drills (0.3mm to 3.0mm)',
        depth: '-1.80 mm', feed: '150 mm/min', rpm: '10,000 RPM',
        instructions: 'Peck drilling (0.4mm depth increments) to prevent chip packing and bit breakage. Drills via holes, component pins, and 5mm tab mouse-bite perforations.'
      }},
      'Step2_Vias': {{
        title: 'Step 2: Chemical Through-Hole Via Metallization',
        tool: 'Chemical Electroless / Electroplating Bath',
        depth: 'N/A', feed: 'N/A', rpm: 'N/A',
        instructions: 'Desmear -> Carbon-black / Palladium seeding -> Microetch -> 45 min Copper Electroplating @ 1.5A/dm² = 25µm copper barrel deposition in all 0.3mm via holes.'
      }},
      'Step3_Top_Copper': {{
        title: 'Step 3: Top Copper Isolation Engraving',
        tool: '30° / 0.1mm V-Bit or 0.2mm Endmill',
        depth: '-0.04 mm', feed: '300 mm/min', rpm: '10,000 RPM',
        instructions: 'Isolation routing for signal traces and ground pours. Engraves Board ID and directional arrow pointing into PCB body in the 5mm breakaway tab.'
      }},
      'Step4_Bottom_Copper': {{
        title: 'Step 4: Bottom Copper Isolation Engraving',
        tool: '30° / 0.1mm V-Bit',
        depth: '-0.04 mm', feed: '300 mm/min', rpm: '10,000 RPM',
        instructions: 'Board flipped horizontally along Y-axis without Z-rotation. Milled using X-mirrored coordinates registered against dowel pins and left margin scale.'
      }},
      'Step5_Solder_Mask': {{
        title: 'Step 5: Solder Mask Pad Clearing',
        tool: '0.4mm Flat Endmill or V-Bit',
        depth: '-0.06 mm', feed: '300 mm/min', rpm: '10,000 RPM',
        instructions: 'CRITICAL Z-HEIGHT NOTICE: Solder mask adds ~35µm. Operator must re-touch Z0 on the cured mask surface when prompted by M0 pause before cycle start.'
      }},
      'Step6_Silkscreen': {{
        title: 'Step 6: Silkscreen Component Legend Printing',
        tool: '# TODO: Flatbed Inkjet DoD Printhead',
        depth: 'N/A', feed: 'N/A', rpm: 'N/A',
        instructions: 'Component designators (R1, C1, U1, pin 1 marks). Deposited using UV-curable white pigment ink with LED-395nm pinning. No CNC spindle engraving needed.'
      }},
      'Step7_Cutout_SnapTab': {{
        title: 'Step 7: Dual-Sided Snapable Cutout & 5mm Tab',
        tool: '1.0mm Corn Endmill or V-Bit',
        depth: '-0.85 mm (Each Side)', feed: '200 mm/min', rpm: '10,000 RPM',
        instructions: 'Dual-sided partial scoring (-0.85mm top + -0.85mm bottom = 1.70mm total cut through 1.6mm FR4). Single-pass cut along shared boundaries with 3mm retention bridge tabs.'
      }}
    }};

    function selectCircuitProject(pcbId) {{
      currentProject = pcbId;
      document.querySelectorAll('.proj-btn').forEach(btn => {{
        btn.classList.remove('bg-blue-600', 'text-white');
        btn.classList.add('bg-gray-800', 'text-gray-300');
      }});
      const activeBtn = document.getElementById('proj-btn-' + pcbId);
      if (activeBtn) {{
        activeBtn.classList.remove('bg-gray-800', 'text-gray-300');
        activeBtn.classList.add('bg-blue-600', 'text-white');
      }}
      updateCircuitView();
    }}

    function selectCircuitStep(stepName) {{
      currentStep = stepName;
      document.querySelectorAll('.step-btn').forEach(btn => btn.classList.remove('active'));
      const activeBtn = document.getElementById('step-btn-' + stepName);
      if (activeBtn) activeBtn.classList.add('active');
      updateCircuitView();
    }}

    function updateCircuitView() {{
      const svgPath = `${{imgPrefix}}Circuit_${{currentProject}}_${{currentStep}}.svg`;
      const img = document.getElementById('circuit-svg-img');
      if (img) img.src = svgPath;

      const info = stepInfo[currentStep] || {{}};
      document.getElementById('card-step-title').innerText = info.title || currentStep;
      document.getElementById('card-tool').innerText = info.tool || '-';
      document.getElementById('card-depth').innerText = info.depth || '-';
      document.getElementById('card-feed').innerText = info.feed || '-';
      document.getElementById('card-rpm').innerText = info.rpm || '-';
      document.getElementById('card-instructions').innerText = info.instructions || '-';
    }}

    function selectSheet(num) {{
      document.querySelectorAll('.sheet-content').forEach(el => el.style.display = 'none');
      document.querySelectorAll('[id^="tab-btn-"]').forEach(el => {{
        el.className = el.className.replace('border-blue-500 text-blue-400 bg-blue-500/10', 'border-transparent text-gray-400 hover:text-white');
      }});
      const panel = document.getElementById('sheet-panel-' + num);
      if (panel) panel.style.display = 'block';
      const btn = document.getElementById('tab-btn-' + num);
      if (btn) btn.className = 'px-4 py-2 text-sm font-semibold border-b-2 transition border-blue-500 text-blue-400 bg-blue-500/10';
    }}

    // Checklist interactive state saving
    let checklistState = {json.dumps(chk_items)};

    function toggleChecklistItem(itemId, isChecked) {{
      if (checklistState[itemId]) {{
        checklistState[itemId].checked = isChecked;
      }}
      const lbl = document.getElementById('label-' + itemId);
      if (lbl) {{
        if (isChecked) {{
          lbl.classList.add('line-through', 'text-gray-500');
          lbl.classList.remove('text-white');
        }} else {{
          lbl.classList.remove('line-through', 'text-gray-500');
          lbl.classList.add('text-white');
        }}
      }}
      updateChecklistProgress();
      saveChecklistToDisk();
    }}

    function updateChecklistProgress() {{
      const total = Object.keys(checklistState).length;
      const checkedCount = Object.values(checklistState).filter(item => item.checked).length;
      const pct = Math.round((checkedCount / total) * 100);
      document.getElementById('chk-progress-bar').style.width = pct + '%';
      document.getElementById('chk-progress-text').innerText = `${{checkedCount}} / ${{total}} Completed (${{pct}}%)`;
    }}

    async function saveChecklistToDisk() {{
      const payload = {{
        updated_at: new Date().toISOString(),
        checklist_version: "1.0",
        items: checklistState
      }};
      try {{
        // Save to browser localStorage
        localStorage.setItem('pcb_checklist_state', JSON.stringify(payload));
        // Try posting to local server if running
        fetch('/api/save_checklist', {{
          method: 'POST',
          headers: {{ 'Content-Type': 'application/json' }},
          body: JSON.stringify(payload)
        }}).catch(() => {{}});
      }} catch (e) {{}}
    }}

    function downloadChecklistJSON() {{
      const payload = {{
        updated_at: new Date().toISOString(),
        checklist_version: "1.0",
        items: checklistState
      }};
      const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(payload, null, 2));
      const a = document.createElement('a');
      a.setAttribute("href", dataStr);
      a.setAttribute("download", "checklist_state.json");
      document.body.appendChild(a);
      a.click();
      a.remove();
    }}

    window.onload = function() {{
      updateCircuitView();
      updateChecklistProgress();
    }};
  </script>
</head>
<body class="p-6 max-w-7xl mx-auto">
  <!-- Header -->
  <header class="flex flex-col md:flex-row items-start md:items-center justify-between pb-6 mb-8 border-b border-gray-800">
    <div>
      <div class="flex items-center space-x-3">
        <span class="p-2 bg-blue-600 rounded-lg text-white font-bold text-xl">CNC</span>
        <h1 class="text-2xl font-bold tracking-tight">A4 PCB Multi-Step Fabrication Dashboard</h1>
      </div>
      <p class="text-gray-400 text-sm mt-1">2-Step Y-Axis CNC Milling Pipeline | FluidNC Local Controller ({fluidnc_ip})</p>
    </div>
    <div class="mt-4 md:mt-0 flex items-center space-x-3">
      <div class="px-4 py-2 bg-gray-900 border border-gray-700 rounded-lg text-sm">
        <span class="text-gray-400">Total Sheets:</span> <strong class="text-white">{len(sheets)}</strong>
      </div>
      <div class="px-4 py-2 bg-gray-900 border border-gray-700 rounded-lg text-sm">
        <span class="text-gray-400">Total Units:</span> <strong class="text-green-400">{total_units}</strong>
      </div>
      <div class="px-4 py-2 bg-gray-900 border border-gray-700 rounded-lg text-sm">
        <span class="text-gray-400">Clamps Active:</span> <strong class="text-red-400">{len(holders_mgr.holders)}</strong>
      </div>
    </div>
  </header>

  <!-- FluidNC Status Alert -->
  <div class="mb-8 p-4 bg-gray-900 border border-blue-500/30 rounded-xl flex items-center justify-between">
    <div class="flex items-center space-x-3">
      <div class="w-3 h-3 rounded-full bg-green-500 animate-pulse"></div>
      <div>
        <h4 class="text-sm font-semibold text-white">FluidNC Machine Target: <code class="text-blue-400">{fluidnc_ip}</code></h4>
        <p class="text-xs text-gray-400">G-code ready for direct network upload & execution via Web/REST API.</p>
      </div>
    </div>
    <a href="http://{fluidnc_ip}" target="_blank" class="px-4 py-1.5 bg-blue-600 hover:bg-blue-500 text-white text-xs font-semibold rounded-lg transition">
      Open FluidNC WebUI &rarr;
    </a>
  </div>

  <!-- SECTION 1: Interactive Step-by-Step Circuit Gerber Inspector -->
  <section class="mb-12 bg-gray-900 border border-gray-800 p-6 rounded-2xl">
    <div class="flex flex-col md:flex-row items-start md:items-center justify-between pb-4 mb-6 border-b border-gray-800">
      <div>
        <h2 class="text-xl font-bold text-white flex items-center space-x-2">
          <span>Step-by-Step Circuit Gerber Inspector</span>
          <span class="text-xs bg-blue-500/20 text-blue-300 border border-blue-500/40 px-2 py-0.5 rounded">Interactive Artwork Viewer</span>
        </h2>
        <p class="text-xs text-gray-400 mt-0.5">Inspect vector artwork, drill schedules, copper isolation, solder mask, and the 5mm breakaway tab for each circuit.</p>
      </div>

      <!-- Circuit Project Switcher -->
      <div class="mt-4 md:mt-0 flex items-center space-x-2">
        {"".join([f'''
        <button onclick="selectCircuitProject('{p.pcb_id}')" id="proj-btn-{p.pcb_id}"
                class="proj-btn px-3 py-1.5 text-xs font-semibold rounded-lg transition {'bg-blue-600 text-white' if idx==0 else 'bg-gray-800 text-gray-300 hover:text-white'}">
          {p.project_name} ({p.pcb_id})
        </button>
        ''' for idx, p in enumerate(unique_projects)])}
      </div>
    </div>

    <!-- Stepper Buttons -->
    <div class="grid grid-cols-2 sm:grid-cols-4 lg:grid-cols-8 gap-2 mb-6">
      <button onclick="selectCircuitStep('Composite_Stackup')" id="step-btn-Composite_Stackup" class="step-btn active p-2.5 rounded-xl border border-gray-800 bg-gray-950 text-left text-xs transition">
        <span class="block font-bold text-gray-300">★ Overview</span>
        <span class="text-[10px] text-gray-400">Full Stackup</span>
      </button>
      <button onclick="selectCircuitStep('Step1_Drills')" id="step-btn-Step1_Drills" class="step-btn p-2.5 rounded-xl border border-gray-800 bg-gray-950 text-left text-xs transition">
        <span class="block font-bold text-gray-300">Step 1</span>
        <span class="text-[10px] text-gray-400">CNC Drills</span>
      </button>
      <button onclick="selectCircuitStep('Step2_Vias')" id="step-btn-Step2_Vias" class="step-btn p-2.5 rounded-xl border border-gray-800 bg-gray-950 text-left text-xs transition">
        <span class="block font-bold text-gray-300">Step 2</span>
        <span class="text-[10px] text-gray-400">Chemical Vias</span>
      </button>
      <button onclick="selectCircuitStep('Step3_Top_Copper')" id="step-btn-Step3_Top_Copper" class="step-btn p-2.5 rounded-xl border border-gray-800 bg-gray-950 text-left text-xs transition">
        <span class="block font-bold text-gray-300">Step 3</span>
        <span class="text-[10px] text-gray-400">Top Copper</span>
      </button>
      <button onclick="selectCircuitStep('Step4_Bottom_Copper')" id="step-btn-Step4_Bottom_Copper" class="step-btn p-2.5 rounded-xl border border-gray-800 bg-gray-950 text-left text-xs transition">
        <span class="block font-bold text-gray-300">Step 4</span>
        <span class="text-[10px] text-gray-400">Bottom Copper</span>
      </button>
      <button onclick="selectCircuitStep('Step5_Solder_Mask')" id="step-btn-Step5_Solder_Mask" class="step-btn p-2.5 rounded-xl border border-gray-800 bg-gray-950 text-left text-xs transition">
        <span class="block font-bold text-gray-300">Step 5</span>
        <span class="text-[10px] text-gray-400">Solder Mask</span>
      </button>
      <button onclick="selectCircuitStep('Step6_Silkscreen')" id="step-btn-Step6_Silkscreen" class="step-btn p-2.5 rounded-xl border border-gray-800 bg-gray-950 text-left text-xs transition">
        <span class="block font-bold text-gray-300">Step 6</span>
        <span class="text-[10px] text-gray-400">Silkscreen</span>
      </button>
      <button onclick="selectCircuitStep('Step7_Cutout_SnapTab')" id="step-btn-Step7_Cutout_SnapTab" class="step-btn p-2.5 rounded-xl border border-gray-800 bg-gray-950 text-left text-xs transition">
        <span class="block font-bold text-gray-300">Step 7</span>
        <span class="text-[10px] text-gray-400">Snap Cutout</span>
      </button>
    </div>

    <!-- 2-Column Inspector Layout -->
    <div class="grid grid-cols-1 lg:grid-cols-12 gap-6">
      <!-- Left: SVG Artwork Viewport -->
      <div class="lg:col-span-8 bg-black p-4 rounded-xl border border-gray-800 flex flex-col justify-center items-center svg-container">
        <img id="circuit-svg-img" src="{img_prefix}Circuit_{first_proj_id}_Composite_Stackup.svg" alt="Circuit Step View" class="max-h-[520px] object-contain">
      </div>

      <!-- Right: Technical Step Card -->
      <div class="lg:col-span-4 bg-gray-950 p-5 rounded-xl border border-gray-800 flex flex-col justify-between">
        <div>
          <h3 id="card-step-title" class="font-bold text-base text-white mb-3">Composite Circuit Stackup Overview</h3>
          
          <div class="space-y-2 mb-4 text-xs font-mono">
            <div class="flex justify-between py-1 border-b border-gray-800">
              <span class="text-gray-400">Tool / Bit:</span>
              <span id="card-tool" class="text-white font-bold">All layers</span>
            </div>
            <div class="flex justify-between py-1 border-b border-gray-800">
              <span class="text-gray-400">Cut Depth:</span>
              <span id="card-depth" class="text-yellow-400">N/A</span>
            </div>
            <div class="flex justify-between py-1 border-b border-gray-800">
              <span class="text-gray-400">Feed Rate:</span>
              <span id="card-feed" class="text-blue-400">N/A</span>
            </div>
            <div class="flex justify-between py-1 border-b border-gray-800">
              <span class="text-gray-400">Spindle Speed:</span>
              <span id="card-rpm" class="text-green-400">N/A</span>
            </div>
          </div>

          <h4 class="text-xs font-bold text-gray-400 uppercase tracking-wider mb-1.5">Fabrication Advice & Instructions</h4>
          <p id="card-instructions" class="text-xs text-gray-300 leading-relaxed bg-gray-900 p-3 rounded-lg border border-gray-800">
            Visual inspection of FR4 substrate, copper traces, mask apertures, silkscreen legends, drills, and the 5mm breakaway tab attached along the PCB width side.
          </p>
        </div>

        <div class="mt-4 pt-4 border-t border-gray-800 flex items-center justify-between text-xs">
          <span class="text-gray-400">Attached Tag Bar:</span>
          <span class="text-amber-400 font-mono font-bold">5mm along PCB Width</span>
        </div>
      </div>
    </div>
  </section>

  <!-- SECTION 2: Sheet Selector Tabs & Blueprint Viewer -->
  <div class="flex border-b border-gray-800 mb-6 space-x-2">
    {"".join(sheet_tabs_html)}
  </div>

  <div class="grid grid-cols-1 lg:grid-cols-12 gap-8 mb-12">
    <!-- Left Column: Interactive Panel Preview -->
    <div class="lg:col-span-7 bg-gray-900 border border-gray-800 p-5 rounded-2xl">
      {"".join(sheet_panels_html)}

      <!-- Blueprint Legend -->
      <div class="mt-6 pt-4 border-t border-gray-800 grid grid-cols-2 sm:grid-cols-4 gap-2 text-xs">
        <div class="flex items-center space-x-1.5">
          <div class="w-3 h-3 rounded bg-green-500/30 border border-green-500"></div>
          <span class="text-gray-300">PCB Body</span>
        </div>
        <div class="flex items-center space-x-1.5">
          <div class="w-3 h-3 rounded bg-yellow-500/40 border border-yellow-500"></div>
          <span class="text-gray-300">5mm Snap ID Tag</span>
        </div>
        <div class="flex items-center space-x-1.5">
          <div class="w-3 h-3 rounded bg-red-500/30 border border-red-500"></div>
          <span class="text-gray-300">Clamp Keepout</span>
        </div>
        <div class="flex items-center space-x-1.5">
          <div class="w-3 h-3 rounded bg-blue-500/30 border border-blue-500"></div>
          <span class="text-gray-300">Left Guide Margin</span>
        </div>
      </div>
    </div>

    <!-- Right Column: Interactive Setup Checklist -->
    <div class="lg:col-span-5 bg-gray-900 border border-gray-800 p-5 rounded-2xl flex flex-col justify-between">
      <div>
        <div class="flex items-center justify-between pb-3 mb-3 border-b border-gray-800">
          <div>
            <h3 class="font-bold text-base text-white">Fabrication Setup Checklist</h3>
            <p class="text-xs text-gray-400">Tick items as you execute setup. State auto-saves to <code class="text-blue-400 font-mono">checklist_state.json</code>.</p>
          </div>
          <button onclick="downloadChecklistJSON()" class="px-2.5 py-1 text-xs bg-gray-800 hover:bg-gray-700 text-gray-200 rounded border border-gray-700">
            Export JSON
          </button>
        </div>

        <!-- Progress Bar -->
        <div class="mb-4">
          <div class="flex justify-between text-xs text-gray-400 mb-1">
            <span>Setup Progress</span>
            <span id="chk-progress-text" class="text-blue-400 font-mono font-bold">0 / 20 Completed (0%)</span>
          </div>
          <div class="w-full bg-gray-800 h-2 rounded-full overflow-hidden">
            <div id="chk-progress-bar" class="bg-blue-600 h-full rounded-full transition-all duration-300" style="width: 0%"></div>
          </div>
        </div>

        <!-- Checklist Item Rows -->
        <div class="space-y-2.5 max-h-[580px] overflow-y-auto pr-1">
          {"".join(chk_rows_html)}
        </div>
      </div>
    </div>
  </div>

  <!-- SECTION 3: Practical Issues & Built-in Solutions Matrix -->
  <section class="mt-8 mb-12">
    <h2 class="text-xl font-bold text-white mb-4">Practical CNC Machining Challenges & Automated Solutions</h2>
    <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-5">
      {issues_html}
    </div>
  </section>

  <!-- Footer -->
  <footer class="pt-6 border-t border-gray-800 text-center text-xs text-gray-400">
    <p>A4 Copper Board CNC Panelization & Manufacturing Pipeline &bull; FluidNC CNC Direct Integration</p>
  </footer>
</body>
</html>
"""
    return html


def generate_html_dashboard(sheets, holders_mgr: HolderManager, generated_files,
                            config: Dict, fluidnc_ip="192.168.8.109",
                            output_html="manufacture_guides/index.html"):
    os.makedirs(os.path.dirname(output_html) or ".", exist_ok=True)
    
    # 1. Write manufacture_guides/index.html (img_prefix="../manufacture_svg/")
    html_guides = build_html_dashboard_content(
        sheets, holders_mgr, generated_files, config, fluidnc_ip, img_prefix="../manufacture_svg/"
    )
    with open(output_html, "w", encoding="utf-8") as f:
        f.write(html_guides)
    print(f"[SUCCESS] Interactive Fabrication Dashboard written to: {output_html}")
    
    # 2. Write root manufacture_dashboard.html (img_prefix="manufacture_svg/")
    html_root = build_html_dashboard_content(
        sheets, holders_mgr, generated_files, config, fluidnc_ip, img_prefix="manufacture_svg/"
    )
    root_dash = "manufacture_dashboard.html"
    try:
        with open(root_dash, "w", encoding="utf-8") as f:
            f.write(html_root)
        print(f"[SUCCESS] Root dashboard synced to: {root_dash}")
    except Exception as e:
        print(f"[WARN] Failed writing root dashboard: {e}")


# ---------------------------------------------------------------------------
# Lightweight Local Web Server for Checklist Auto-Sync
# ---------------------------------------------------------------------------

class ChecklistHandler(http.server.SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path == "/api/save_checklist":
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

    sheet_w = float(cfg.get("copper_sheet", {}).get("sheet_width_mm", 210.0))
    sheet_h = float(cfg.get("copper_sheet", {}).get("sheet_height_mm", 297.0))
    y_split = float(cfg.get("cnc_limits", {}).get("y_split_threshold_mm", 145.0))
    tab_w = float(cfg.get("pcb_tag_strip", {}).get("tag_width_mm", 5.0))

    print("================================================================================")
    print(" A4 Copper Board CNC PCB Panelizer & Multi-Step Manufacturing Pipeline")
    print(f" Bed Travel: 275x170 mm | Sheet Size: {sheet_w:.1f}x{sheet_h:.1f} mm | Y-Shift: {y_split:.1f} mm")
    print("================================================================================\n")

    # 1. Scan Projects
    projects = scan_projects(args.projects_dir)
    if not projects:
        print("[ERROR] No PCB projects containing project_metadata.json were discovered.")
        sys.exit(1)
    print(f"[INFO] Discovered {len(projects)} PCB project(s):")
    for p in projects:
        print(f"  - {p.pcb_id} ({p.project_name}) | Dims: {p.width:.1f}x{p.height:.1f}mm | Qty: {p.quantity} | Mask: {p.solder_mask_color}")

    # 2. Load Holders
    holder_mgr = HolderManager(args.holders)

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
    generate_html_dashboard(
        sheets, holder_mgr, generated_files,
        config=cfg, fluidnc_ip=args.fluidnc_ip, output_html=html_out
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
