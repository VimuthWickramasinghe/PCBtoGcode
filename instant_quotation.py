#!/usr/bin/env python3
"""
=============================================================================
 Instant PCB Quotation Calculator (KiCad & EasyEDA)
=============================================================================
 Automatically inspects and analyzes Gerber (RS-274X) and Excellon drill files
 in a selected folder, extracts board dimensions, layer count, drill hole
 statistics, non-circular slots, solder mask, and silkscreen options, and
 calculates an itemized manufacturing quotation based on quotation_config.json.

 Usage:
     python instant_quotation.py ./project01
     python instant_quotation.py ./project02 --qty 10 --mask-color black
     python instant_quotation.py ./project01 --config quotation_config.json
     python instant_quotation.py ./project01 --json
     python instant_quotation.py ./project01 --output quotation.json
=============================================================================
"""

import sys
import os
import glob
import re
import math
import json
import argparse
import time

try:
    from shapely.geometry import Polygon, LineString, MultiLineString, Point
    from shapely.ops import unary_union
    HAVE_SHAPELY = True
except ImportError:
    HAVE_SHAPELY = False

# Import Gerber and Excellon parsers from gerber_to_gcode if available
try:
    from gerber_to_gcode import GerberParser, ExcellonParser
    HAVE_PARSERS = True
except ImportError:
    HAVE_PARSERS = False


# -----------------------------------------------------------------------------
# Configuration Loader
# -----------------------------------------------------------------------------

DEFAULT_CONFIG = {
    "currency": {
        "code": "LKR",
        "symbol": "Rs. ",
        "format": "{symbol}{amount:,.2f}",
        "conversion_rate_usd": 300.0
    },
    "general": {
        "default_quantity": 5,
        "default_lead_time_days": 3,
        "show_cnc_comparison": True,
        "tax_rate_percent": 0.0
    },
    "fabrication": {
        "base_setup_fee": 1200.00,
        "engineering_fee": 450.00,
        "standard_promo": {
            "enabled": True,
            "max_width_mm": 100.0,
            "max_height_mm": 100.0,
            "max_quantity": 5,
            "flat_price": 1500.00,
            "description": "Standard 100x100mm (5 pcs) Special Promotion Tier (Rs. 1,500)"
        },
        "area_pricing": {
            "price_per_cm2": 10.50,
            "minimum_charge_area_cm2": 25.0
        },
        "layer_multipliers": {
            "1": 0.90,
            "2": 1.00,
            "4": 2.60,
            "6": 4.80
        },
        "quantity_discounts": {
            "1": 1.00,
            "5": 1.00,
            "10": 0.88,
            "20": 0.80,
            "30": 0.74,
            "50": 0.65,
            "100": 0.52,
            "200": 0.45,
            "500": 0.38
        },
        "drilling": {
            "free_holes": 500,
            "fee_per_extra_100_holes": 180.00,
            "min_diameter_limits": {
                "standard_min_mm": 0.30,
                "micro_drill_surcharge": 2400.00,
                "high_density_min_mm": 0.20,
                "high_density_surcharge": 6600.00
            },
            "tool_count_threshold": 10,
            "extra_tool_fee": 150.00
        },
        "slots": {
            "slot_base_fee": 600.00,
            "fee_per_slot": 90.00
        },
        "board_shape": {
            "complex_shape_surcharge": 900.00
        },
        "solder_mask": {
            "colors": {
                "green": 0.00,
                "red": 750.00,
                "yellow": 750.00,
                "blue": 750.00,
                "white": 1050.00,
                "black": 1050.00,
                "purple": 1500.00,
                "matte_black": 1800.00,
                "matte_green": 1800.00
            },
            "double_sided_included": True
        },
        "silkscreen": {
            "colors": {
                "white": 0.00,
                "black": 450.00,
                "none": 0.00
            },
            "double_sided_extra": 450.00
        },
        "surface_finish": {
            "HASL": 0.00,
            "LeadFree_HASL": 900.00,
            "ENIG": 5400.00
        },
        "board_thickness_mm": {
            "1.6": 0.00,
            "0.8": 750.00,
            "1.0": 450.00,
            "1.2": 240.00,
            "2.0": 1350.00
        },
        "copper_weight": {
            "1oz": 0.00,
            "2oz": 3600.00
        },
        "producer_mark": {
            "remove_mark_fee": 600.00
        }
    },
    "cnc_prototype": {
        "enabled": True,
        "setup_fee": 2400.00,
        "material_fr1_per_cm2": 7.50,
        "material_fr4_per_cm2": 12.00,
        "machine_hourly_rate": 4500.00,
        "isolation_milling_feed_mm_min": 300.0,
        "estimated_copper_density": 0.35,
        "drill_time_per_hole_sec": 3.5,
        "tool_change_time_sec": 25.0,
        "slot_milling_rate_per_mm": 15.00,
        "cutout_time_per_board_min": 4.0,
        "tool_wear_cost_per_board": 450.00
    }
}


def load_quotation_config(config_path=None, folder_path=None):
    """
    Search for and load quotation_config.json.
    Priority:
      1. Explicit config_path argument
      2. folder_path/quotation_config.json
      3. ./quotation_config.json
      4. Script directory quotation_config.json
    """
    candidates = []
    if config_path:
        candidates.append(config_path)
    if folder_path and os.path.isdir(folder_path):
        candidates.extend([
            os.path.join(folder_path, "quotation_config.json"),
            os.path.join(folder_path, "pricing_config.json")
        ])
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates.extend([
        os.path.join(os.getcwd(), "quotation_config.json"),
        os.path.join(script_dir, "quotation_config.json"),
        os.path.join(os.getcwd(), "pricing_config.json"),
        os.path.join(script_dir, "pricing_config.json"),
    ])

    loaded_path = None
    loaded_config = {}
    for c in candidates:
        if c and os.path.isfile(c) and os.path.getsize(c) > 0:
            try:
                with open(c, "r", encoding="utf-8") as f:
                    loaded_config = json.load(f)
                loaded_path = os.path.abspath(c)
                break
            except Exception as e:
                print(f"Warning: Failed to parse config file '{c}': {e}", file=sys.stderr)

    # Deep merge with defaults
    merged = json.loads(json.dumps(DEFAULT_CONFIG))
    if loaded_config:
        def deep_update(d, u):
            for k, v in u.items():
                if isinstance(v, dict) and k in d and isinstance(d[k], dict):
                    deep_update(d[k], v)
                else:
                    d[k] = v
        deep_update(merged, loaded_config)

    return merged, loaded_path


# -----------------------------------------------------------------------------
# Metadata and Job File Parsers
# -----------------------------------------------------------------------------

def parse_project_metadata(folder_path):
    """
    Inspect project folder for metadata files (e.g., project_metadata.json, metadata.json).
    Supports both valid JSON files and loose key-value text files:
      amount = 10
      pcb_unit_solder_mask_colors = ["black"]
      without_producer_mark
      silkscreen_color = white
    """
    candidates = [
        os.path.join(folder_path, "project_metadata.json"),
        os.path.join(folder_path, "metadata.json"),
        os.path.join(folder_path, "order_metadata.json"),
        os.path.join(folder_path, "pcb_metadata.json"),
    ]

    target_file = None
    for c in candidates:
        if os.path.isfile(c) and os.path.getsize(c) > 0:
            target_file = c
            break

    if not target_file:
        return {}

    with open(target_file, "r", encoding="utf-8") as f:
        content = f.read()

    # Try standard JSON parsing
    try:
        data = json.loads(content)
        if isinstance(data, dict):
            return data
    except Exception:
        pass

    # Fallback to key-value line parser
    meta = {}
    for line in content.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("//"):
            continue

        # Check for key = value or key: value
        m = re.match(r"^([a-zA-Z0-9_\-]+)\s*[:=]\s*(.*)$", line)
        if m:
            key = m.group(1).strip().lower()
            val_str = m.group(2).strip()

            if not val_str:
                meta[key] = None
                continue

            # Try parsing integer or float
            if re.match(r"^-?\d+$", val_str):
                meta[key] = int(val_str)
                continue
            if re.match(r"^-?\d+\.\d+$", val_str):
                meta[key] = float(val_str)
                continue

            # Boolean check
            if val_str.lower() in ("true", "yes", "1"):
                meta[key] = True
                continue
            if val_str.lower() in ("false", "no", "0"):
                meta[key] = False
                continue

            # JSON array check e.g. ["black", "green"]
            if val_str.startswith("[") and "]" in val_str:
                array_part = val_str[:val_str.index("]") + 1]
                try:
                    meta[key] = json.loads(array_part)
                    continue
                except Exception:
                    pass

            # String value (strip quotes)
            clean_val = val_str.strip("\"'")
            if key == "amount":
                # If it's a constraint template like '1<=100', do not parse as a single integer
                if "<" in clean_val or ">" in clean_val or "etc" in clean_val:
                    meta[key] = None
                else:
                    amt_m = re.match(r"^(\d+)\b", clean_val)
                    if amt_m:
                        meta[key] = int(amt_m.group(1))
                    else:
                        meta[key] = None
            else:
                meta[key] = clean_val
        else:
            # Standalone flag e.g. 'without_producer_mark'
            bare_key = line.strip().lower()
            # If line is accompanied by template options like 'rectangular or other', don't treat as true flag
            if re.match(r"^[a-zA-Z0-9_\-]+$", bare_key):
                # Only treat as true if it's explicitly a boolean-intent flag and not template text
                meta[bare_key] = False

    return meta


def parse_kicad_gbrjob(folder_path):
    """Parse KiCad .gbrjob file if present."""
    job_files = glob.glob(os.path.join(folder_path, "*.gbrjob"))
    if not job_files:
        return {}

    try:
        with open(job_files[0], "r", encoding="utf-8") as f:
            data = json.load(f)
        specs = data.get("GeneralSpecs", {})
        header = data.get("Header", {})
        return {
            "app": header.get("GenerationSoftware", {}).get("Application", "KiCad"),
            "size_x": specs.get("Size", {}).get("X"),
            "size_y": specs.get("Size", {}).get("Y"),
            "layer_number": specs.get("LayerNumber"),
            "board_thickness": specs.get("BoardThickness"),
            "finish": specs.get("Finish"),
            "rules": data.get("DesignRules", [])
        }
    except Exception:
        return {}


# -----------------------------------------------------------------------------
# Gerber & Drill Layer Identification
# -----------------------------------------------------------------------------

def categorize_project_files(folder_path):
    """
    Examine all files in the folder and categorize them by function:
      - copper: top, bottom, inner layers
      - outline: edge cuts / board outline
      - drill: plated, non-plated, vias
      - mask: top, bottom solder mask
      - silkscreen: top, bottom silkscreen
      - paste: top, bottom solder paste
    """
    files = sorted(glob.glob(os.path.join(folder_path, "*")))
    cats = {
        "top_copper": [],
        "bottom_copper": [],
        "inner_copper": [],
        "outline": [],
        "drill": [],
        "top_mask": [],
        "bottom_mask": [],
        "top_silk": [],
        "bottom_silk": [],
        "paste": [],
        "job": [],
        "other": []
    }

    for f in files:
        if os.path.isdir(f):
            continue
        fn = os.path.basename(f).lower()
        ext = os.path.splitext(fn)[1]

        # Ignore non-CAD metadata and gcode/svg files
        if ext in (".gcode", ".svg", ".json", ".md", ".txt") and not ext in (".drl", ".txt"):
            if fn.endswith(".json") and "metadata" in fn:
                continue
            if ext in (".gcode", ".svg", ".md"):
                continue

        # 1. Drill Files
        if ext in (".drl", ".xln", ".exc") or (ext == ".txt" and ("drill" in fn or "drl" in fn)):
            cats["drill"].append(f)
            continue

        # 2. Board Outline / Edge Cuts
        if "edge_cuts" in fn or "edge.cuts" in fn or "boardoutline" in fn or "outline" in fn or ext in (".gm1", ".gko", ".gml"):
            cats["outline"].append(f)
            continue

        # 3. Job file
        if ext == ".gbrjob":
            cats["job"].append(f)
            continue

        # 4. Solder Mask
        if "f_mask" in fn or "f.mask" in fn or "topsoldermask" in fn or ext == ".gts":
            cats["top_mask"].append(f)
            continue
        if "b_mask" in fn or "b.mask" in fn or "bottomsoldermask" in fn or ext == ".gbs":
            cats["bottom_mask"].append(f)
            continue

        # 5. Silkscreen
        if "f_silk" in fn or "f.silk" in fn or "topsilk" in fn or ext == ".gto":
            cats["top_silk"].append(f)
            continue
        if "b_silk" in fn or "b.silk" in fn or "bottomsilk" in fn or ext == ".gbo":
            cats["bottom_silk"].append(f)
            continue

        # 6. Solder Paste / Stencil
        if "paste" in fn or ext in (".gtp", ".gbp"):
            cats["paste"].append(f)
            continue

        # 7. Copper Layers
        if "f_cu" in fn or "f.cu" in fn or "toplayer" in fn or ext == ".gtl":
            cats["top_copper"].append(f)
            continue
        if "b_cu" in fn or "b.cu" in fn or "bottomlayer" in fn or ext == ".gbl":
            cats["bottom_copper"].append(f)
            continue
        if "in" in fn and "cu" in fn or ext in (".g1", ".g2", ".g3", ".g4"):
            cats["inner_copper"].append(f)
            continue

        # Generic gerber fallback
        if ext == ".gbr":
            if "top" in fn:
                cats["top_copper"].append(f)
            elif "bot" in fn:
                cats["bottom_copper"].append(f)
            else:
                cats["other"].append(f)
        else:
            cats["other"].append(f)

    return cats


# -----------------------------------------------------------------------------
# PCB Metric Extraction (Dimensions, Area, Holes, Slots, Shape)
# -----------------------------------------------------------------------------

def analyze_board_metrics(folder_path, categorized_files, gbrjob_data=None):
    """
    Extract accurate physical metrics of the PCB:
      - Dimensions: Width, Height (mm & inches)
      - Area: cm2, sq in
      - Perimeter: mm
      - Shape: Rectangular vs Complex / Circular
      - Drill Statistics: Total holes, unique tools, min diameter, micro-vias
      - Slot Statistics: Count, total length
      - Copper layer count
    """
    metrics = {
        "eda_detected": "Unknown",
        "width_mm": 0.0,
        "height_mm": 0.0,
        "area_cm2": 0.0,
        "perimeter_mm": 0.0,
        "shape": "Rectangular",
        "copper_layers": 2,
        "has_top_copper": bool(categorized_files["top_copper"]),
        "has_bottom_copper": bool(categorized_files["bottom_copper"]),
        "inner_layers_count": len(categorized_files["inner_copper"]),
        "has_top_mask": bool(categorized_files["top_mask"]),
        "has_bottom_mask": bool(categorized_files["bottom_mask"]),
        "has_top_silk": bool(categorized_files["top_silk"]),
        "has_bottom_silk": bool(categorized_files["bottom_silk"]),
        "total_holes": 0,
        "unique_drill_diameters": [],
        "min_drill_dia_mm": 999.0,
        "micro_via_count": 0,
        "total_slots": 0,
        "slot_lengths_mm": [],
        "outline_source": None
    }

    # Detect EDA source
    all_files = [os.path.basename(f) for sub in categorized_files.values() for f in sub]
    all_str = " ".join(all_files)
    if gbrjob_data or any("f_cu" in f.lower() or "b_cu" in f.lower() for f in all_files):
        metrics["eda_detected"] = "KiCad"
    elif any(f.endswith((".GTL", ".GBL", ".GKO", ".DRL")) for f in all_files):
        metrics["eda_detected"] = "EasyEDA"
    elif any(f.endswith((".gtl", ".gbl", ".gm1")) for f in all_files):
        metrics["eda_detected"] = "Altium / Protel"

    # 1. Determine Copper Layer Count
    copper_count = 0
    if metrics["has_top_copper"]: copper_count += 1
    if metrics["has_bottom_copper"]: copper_count += 1
    copper_count += metrics["inner_layers_count"]
    if gbrjob_data and gbrjob_data.get("layer_number"):
        copper_count = max(copper_count, int(gbrjob_data["layer_number"]))
    metrics["copper_layers"] = max(1, copper_count)

    # 2. Extract Board Dimensions & Outline
    outline_files = categorized_files["outline"]
    raw_bounds = None
    raw_paths = []

    if outline_files and HAVE_PARSERS:
        try:
            gp = GerberParser(outline_files[0])
            gp.parse()
            if gp.raw_bounds:
                raw_bounds = gp.raw_bounds
                raw_paths = gp.raw_paths
                metrics["outline_source"] = os.path.basename(outline_files[0])
        except Exception as e:
            print(f"Warning: Failed to parse outline file '{outline_files[0]}': {e}", file=sys.stderr)

    # If outline file bounds not found, fallback to KiCad gbrjob or copper layers bounds
    if not raw_bounds and gbrjob_data and gbrjob_data.get("size_x") and gbrjob_data.get("size_y"):
        w = float(gbrjob_data["size_x"])
        h = float(gbrjob_data["size_y"])
        metrics["width_mm"] = w
        metrics["height_mm"] = h
        metrics["area_cm2"] = (w * h) / 100.0
        metrics["perimeter_mm"] = 2.0 * (w + h)
        metrics["outline_source"] = "KiCad .gbrjob"
    elif raw_bounds:
        minx, miny, maxx, maxy = raw_bounds
        w = max(0.1, maxx - minx)
        h = max(0.1, maxy - miny)
        metrics["width_mm"] = round(w, 2)
        metrics["height_mm"] = round(h, 2)
        metrics["area_cm2"] = round((w * h) / 100.0, 2)

        # Compute perimeter and evaluate board shape from raw paths
        total_perim = 0.0
        poly_geom = None
        for p in raw_paths:
            if len(p) >= 2:
                for idx in range(len(p) - 1):
                    x1, y1 = p[idx]
                    x2, y2 = p[idx + 1]
                    total_perim += math.hypot(x2 - x1, y2 - y1)
                if len(p) >= 3 and HAVE_SHAPELY:
                    try:
                        poly_candidate = Polygon(p)
                        if poly_candidate.is_valid and poly_candidate.area > 0:
                            poly_geom = poly_candidate
                    except Exception:
                        pass

        metrics["perimeter_mm"] = round(total_perim if total_perim > 0 else 2.0 * (w + h), 2)

        # Shape Classification: Rectangular vs Complex / Circular
        if poly_geom and HAVE_SHAPELY:
            actual_poly_area = poly_geom.area
            bbox_area = w * h
            fill_ratio = actual_poly_area / bbox_area if bbox_area > 0 else 1.0
            vertex_count = len(poly_geom.exterior.coords) - 1

            if fill_ratio >= 0.95 and vertex_count <= 8:
                metrics["shape"] = "Rectangular"
            elif 0.75 <= fill_ratio <= 0.82 and abs(w - h) < 2.0:
                metrics["shape"] = "Circular"
            else:
                metrics["shape"] = "Complex Polygon / Irregular"
        else:
            # Check segment count
            total_points = sum(len(p) for p in raw_paths)
            if total_points <= 8:
                metrics["shape"] = "Rectangular"
            else:
                metrics["shape"] = "Complex Polygon / Irregular"
    else:
        # Fallback: estimate from copper files
        copper_files = categorized_files["top_copper"] + categorized_files["bottom_copper"]
        if copper_files and HAVE_PARSERS:
            all_minx, all_miny, all_maxx, all_maxy = [], [], [], []
            for cf in copper_files:
                try:
                    gp = GerberParser(cf)
                    gp.parse()
                    if gp.raw_bounds:
                        all_minx.append(gp.raw_bounds[0])
                        all_miny.append(gp.raw_bounds[1])
                        all_maxx.append(gp.raw_bounds[2])
                        all_maxy.append(gp.raw_bounds[3])
                except Exception:
                    pass
            if all_minx:
                w = max(0.1, max(all_maxx) - min(all_minx))
                h = max(0.1, max(all_maxy) - min(all_miny))
                metrics["width_mm"] = round(w, 2)
                metrics["height_mm"] = round(h, 2)
                metrics["area_cm2"] = round((w * h) / 100.0, 2)
                metrics["perimeter_mm"] = round(2.0 * (w + h), 2)
                metrics["outline_source"] = "Estimated from Copper Layers"

    # Default fallback if board size is still unknown
    if metrics["width_mm"] == 0.0:
        metrics["width_mm"] = 100.0
        metrics["height_mm"] = 100.0
        metrics["area_cm2"] = 100.0
        metrics["perimeter_mm"] = 400.0
        metrics["outline_source"] = "Default (100x100mm)"

    # 3. Extract Drill Holes and Slots
    drill_files = categorized_files["drill"]
    dedup_holes = set()
    unique_dias = set()
    all_slots = []
    min_dia = 999.0

    if drill_files and HAVE_PARSERS:
        for df in drill_files:
            try:
                ep = ExcellonParser(df)
                ddata = ep.parse()
                for tid, dia in ddata["tools"].items():
                    c_dia = round(dia, 4)
                    unique_dias.add(c_dia)
                    if c_dia < min_dia:
                        min_dia = c_dia

                for tid, hx, hy in ddata["holes"]:
                    dia = ddata["tools"].get(tid, 0.8)
                    dedup_holes.add((round(hx, 3), round(hy, 3), round(dia, 3)))

                all_slots.extend(ddata["slots"])
            except Exception as e:
                print(f"Warning: Failed to parse drill file '{df}': {e}", file=sys.stderr)

    metrics["total_holes"] = len(dedup_holes)
    metrics["unique_drill_diameters"] = sorted(list(unique_dias))
    metrics["min_drill_dia_mm"] = min_dia if min_dia < 999.0 else 0.8
    metrics["micro_via_count"] = sum(1 for h in dedup_holes if h[2] < 0.30)
    metrics["total_slots"] = len(all_slots)

    slot_lengths = []
    for s in all_slots:
        # s is (tid, x1, y1, x2, y2)
        sx1, sy1, sx2, sy2 = s[1], s[2], s[3], s[4]
        slot_lengths.append(math.hypot(sx2 - sx1, sy2 - sy1))
    metrics["slot_lengths_mm"] = slot_lengths

    return metrics


# -----------------------------------------------------------------------------
# Quotation Engine
# -----------------------------------------------------------------------------

def calculate_quotation(metrics, config, user_options):
    """
    Compute itemized quotation for standard PCB fabrication and desktop CNC milling.
    """
    fab_cfg = config.get("fabrication", {})
    cnc_cfg = config.get("cnc_prototype", {})
    curr = config.get("currency", {})
    sym = curr.get("symbol", "$")

    qty = max(1, int(user_options.get("quantity") or config.get("general", {}).get("default_quantity", 5)))
    w_mm = metrics["width_mm"]
    h_mm = metrics["height_mm"]
    area_cm2 = metrics["area_cm2"]
    layers = metrics["copper_layers"]
    shape = user_options.get("shape") or metrics["shape"]

    # Solder mask & Silkscreen options
    mask_color = str(user_options.get("mask_color") or "green").lower().strip()
    silkscreen_color = str(user_options.get("silkscreen_color") or "white").lower().strip()
    surface_finish = str(user_options.get("surface_finish") or "HASL").strip()
    board_thickness = str(user_options.get("board_thickness") or "1.6").strip()
    copper_weight = str(user_options.get("copper_weight") or "1oz").strip()
    remove_mark = bool(user_options.get("without_producer_mark", False))

    fab_items = []

    # 1. Promo vs Standard Area Base Cost
    promo = fab_cfg.get("standard_promo", {})
    is_promo_eligible = (
        promo.get("enabled", False)
        and w_mm <= promo.get("max_width_mm", 100.0)
        and h_mm <= promo.get("max_height_mm", 100.0)
        and qty <= promo.get("max_quantity", 5)
        and layers <= 2
        and mask_color == "green"
    )

    if is_promo_eligible:
        promo_flat = float(promo.get("flat_price", 5.00))
        fab_items.append({
            "category": "Base PCB Fabrication",
            "item": f"Special Promo ({qty} pcs, <=100x100mm, 2-Layer)",
            "details": f"{w_mm:.1f} x {h_mm:.1f} mm, {qty} boards",
            "unit_price": promo_flat / qty,
            "quantity": qty,
            "cost": promo_flat
        })
    else:
        # Base setup & engineering
        base_setup = float(fab_cfg.get("base_setup_fee", 4.00))
        eng_fee = float(fab_cfg.get("engineering_fee", 1.50))
        fab_items.append({
            "category": "Setup & Engineering",
            "item": "CAM Engineering & Tooling Setup",
            "details": f"Gerber inspection & panelization ({layers} layers)",
            "unit_price": base_setup + eng_fee,
            "quantity": 1,
            "cost": base_setup + eng_fee
        })

        # Area pricing
        area_cfg = fab_cfg.get("area_pricing", {})
        price_per_cm2 = float(area_cfg.get("price_per_cm2", 0.035))
        min_area = float(area_cfg.get("minimum_charge_area_cm2", 25.0))
        charged_area = max(area_cm2, min_area)

        layer_mult = float(fab_cfg.get("layer_multipliers", {}).get(str(layers), 1.0))

        # Quantity volume discount tier
        q_discounts = fab_cfg.get("quantity_discounts", {})
        applicable_disc = 1.0
        for threshold_str in sorted(q_discounts.keys(), key=lambda x: int(x), reverse=True):
            if qty >= int(threshold_str):
                applicable_disc = float(q_discounts[threshold_str])
                break

        board_base_rate = charged_area * price_per_cm2 * layer_mult * applicable_disc
        total_area_cost = board_base_rate * qty

        desc = f"{area_cm2:.1f} cm² ({w_mm:.1f} x {h_mm:.1f} mm), {layers}-Layer"
        if applicable_disc < 1.0:
            desc += f", {(1.0 - applicable_disc)*100:.0f}% volume discount"

        fab_items.append({
            "category": "Board Substrate",
            "item": f"FR-4 Base Material ({qty} pcs)",
            "details": desc,
            "unit_price": board_base_rate,
            "quantity": qty,
            "cost": total_area_cost
        })

    # 2. Drilling Surcharges
    drill_cfg = fab_cfg.get("drilling", {})
    total_holes_order = metrics["total_holes"] * qty
    free_holes = int(drill_cfg.get("free_holes", 500))
    extra_hole_rate = float(drill_cfg.get("fee_per_extra_100_holes", 0.60))

    if total_holes_order > free_holes:
        extra_holes = total_holes_order - free_holes
        batches = math.ceil(extra_holes / 100.0)
        hole_cost = batches * extra_hole_rate
        fab_items.append({
            "category": "Drilling",
            "item": f"High Hole Density ({total_holes_order} holes)",
            "details": f"{extra_holes} holes over {free_holes} free quota",
            "unit_price": extra_hole_rate,
            "quantity": batches,
            "cost": hole_cost
        })

    min_limits = drill_cfg.get("min_diameter_limits", {})
    min_dia = metrics["min_drill_dia_mm"]
    if min_dia < float(min_limits.get("high_density_min_mm", 0.20)):
        hd_fee = float(min_limits.get("high_density_surcharge", 22.00))
        fab_items.append({
            "category": "Drilling",
            "item": f"High-Density Interconnect Micro-Vias ({min_dia:.3f}mm)",
            "details": "Requires specialized laser or high-speed carbide micro-drill",
            "unit_price": hd_fee,
            "quantity": 1,
            "cost": hd_fee
        })
    elif min_dia < float(min_limits.get("standard_min_mm", 0.30)):
        micro_fee = float(min_limits.get("micro_drill_surcharge", 8.00))
        fab_items.append({
            "category": "Drilling",
            "item": f"Small Hole Surcharge ({min_dia:.3f}mm < 0.30mm)",
            "details": "Precision micro-drill tooling tier",
            "unit_price": micro_fee,
            "quantity": 1,
            "cost": micro_fee
        })

    # Tool count threshold
    tool_thresh = int(drill_cfg.get("tool_count_threshold", 10))
    unique_tools = len(metrics["unique_drill_diameters"])
    if unique_tools > tool_thresh:
        extra_tools = unique_tools - tool_thresh
        extra_t_fee = float(drill_cfg.get("extra_tool_fee", 0.50)) * extra_tools
        fab_items.append({
            "category": "Drilling",
            "item": f"Excess Drill Tool Sizes ({unique_tools} tools)",
            "details": f"{extra_tools} tools above standard {tool_thresh} tool limit",
            "unit_price": drill_cfg.get("extra_tool_fee", 0.50),
            "quantity": extra_tools,
            "cost": extra_t_fee
        })

    # 3. Slots / Milling Surcharges
    slot_cfg = fab_cfg.get("slots", {})
    if metrics["total_slots"] > 0:
        base_slot_fee = float(slot_cfg.get("slot_base_fee", 2.00))
        per_slot_fee = float(slot_cfg.get("fee_per_slot", 0.30))
        total_slot_cost = base_slot_fee + (metrics["total_slots"] * qty * per_slot_fee)
        fab_items.append({
            "category": "Routing & Slots",
            "item": f"Plated/Non-Plated Slots ({metrics['total_slots']} slots)",
            "details": f"CNC endmill slot milling ({metrics['total_slots']} slots x {qty} pcs)",
            "unit_price": total_slot_cost,
            "quantity": 1,
            "cost": total_slot_cost
        })

    # 4. Complex Shape Surcharge
    shape_cfg = fab_cfg.get("board_shape", {})
    if shape != "Rectangular":
        c_fee = float(shape_cfg.get("complex_shape_surcharge", 3.00))
        fab_items.append({
            "category": "Routing & Slots",
            "item": f"Non-Rectangular Board Outline ({shape})",
            "details": "Custom multi-axis CNC contour routing",
            "unit_price": c_fee,
            "quantity": 1,
            "cost": c_fee
        })

    # 5. Solder Mask Color Surcharge
    mask_cfg = fab_cfg.get("solder_mask", {})
    mask_fee = float(mask_cfg.get("colors", {}).get(mask_color, 0.00))
    if mask_fee > 0.00:
        fab_items.append({
            "category": "Surface Finishing",
            "item": f"Solder Mask Color ({mask_color.capitalize()})",
            "details": "Special photoimageable solder resist pigment",
            "unit_price": mask_fee,
            "quantity": 1,
            "cost": mask_fee
        })

    # 6. Silkscreen Color / Double-Sided Surcharge
    silk_cfg = fab_cfg.get("silkscreen", {})
    silk_color_fee = float(silk_cfg.get("colors", {}).get(silkscreen_color, 0.00))
    if silk_color_fee > 0.00:
        fab_items.append({
            "category": "Surface Finishing",
            "item": f"Silkscreen Legend Color ({silkscreen_color.capitalize()})",
            "details": "Custom silkscreen ink formulation",
            "unit_price": silk_color_fee,
            "quantity": 1,
            "cost": silk_color_fee
        })

    if metrics["has_top_silk"] and metrics["has_bottom_silk"]:
        double_silk_fee = float(silk_cfg.get("double_sided_extra", 1.50))
        if double_silk_fee > 0.00:
            fab_items.append({
                "category": "Surface Finishing",
                "item": "Double-Sided Silkscreen Legend",
                "details": "Silkscreen printing on both Top and Bottom copper sides",
                "unit_price": double_silk_fee,
                "quantity": 1,
                "cost": double_silk_fee
            })

    # 7. Surface Finish Surcharge (HASL, ENIG, etc.)
    finish_cfg = fab_cfg.get("surface_finish", {})
    finish_fee = float(finish_cfg.get(surface_finish, 0.00))
    if finish_fee > 0.00:
        fab_items.append({
            "category": "Surface Finishing",
            "item": f"Surface Finish ({surface_finish})",
            "details": f"Chemical plating / surface treatment ({surface_finish})",
            "unit_price": finish_fee,
            "quantity": 1,
            "cost": finish_fee
        })

    # 8. Board Thickness Surcharge
    thick_cfg = fab_cfg.get("board_thickness_mm", {})
    thick_fee = float(thick_cfg.get(board_thickness, 0.00))
    if thick_fee > 0.00:
        fab_items.append({
            "category": "Board Substrate",
            "item": f"Non-Standard Thickness ({board_thickness}mm)",
            "details": f"Custom laminate core thickness ({board_thickness} mm)",
            "unit_price": thick_fee,
            "quantity": 1,
            "cost": thick_fee
        })

    # 9. Copper Weight Surcharge
    cu_cfg = fab_cfg.get("copper_weight", {})
    cu_fee = float(cu_cfg.get(copper_weight, 0.00))
    if cu_fee > 0.00:
        fab_items.append({
            "category": "Board Substrate",
            "item": f"Heavy Copper Weight ({copper_weight})",
            "details": f"70µm / 2oz heavy copper foil plating",
            "unit_price": cu_fee,
            "quantity": 1,
            "cost": cu_fee
        })

    # 10. Remove Producer Mark Surcharge
    mark_cfg = fab_cfg.get("producer_mark", {})
    if remove_mark:
        rm_fee = float(mark_cfg.get("remove_mark_fee", 2.00))
        fab_items.append({
            "category": "Customization",
            "item": "Remove Producer / Manufacturer Marking",
            "details": "No internal serial code or fab logo etched on board",
            "unit_price": rm_fee,
            "quantity": 1,
            "cost": rm_fee
        })

    # Calculate fabrication totals
    fab_subtotal = sum(it["cost"] for it in fab_items)
    tax_rate = float(config.get("general", {}).get("tax_rate_percent", 0.0))
    tax_amount = fab_subtotal * (tax_rate / 100.0)
    fab_total = fab_subtotal + tax_amount
    fab_unit_price = fab_total / qty

    # -------------------------------------------------------------------------
    # Desktop CNC Prototype Milling Quotation (Dual Comparison)
    # -------------------------------------------------------------------------
    cnc_quote = None
    if cnc_cfg.get("enabled", True):
        cnc_items = []
        c_setup = float(cnc_cfg.get("setup_fee", 8.00))
        cnc_items.append({
            "item": "CNC Machine Setup & Alignment",
            "details": "Stock fixturing, probing, zeroing to lower-left origin",
            "cost": c_setup
        })

        # Material blank cost (FR1 or FR4 single/double sided)
        mat_rate = float(cnc_cfg.get("material_fr1_per_cm2", 0.025))
        if board_thickness != "1.6" or "fr4" in str(user_options.get("material", "")).lower():
            mat_rate = float(cnc_cfg.get("material_fr4_per_cm2", 0.040))
        mat_cost = area_cm2 * mat_rate * qty
        cnc_items.append({
            "item": f"Copper-Clad PCB Blanks ({qty} pcs)",
            "details": f"{area_cm2:.1f} cm² per board x {qty} pcs",
            "cost": mat_cost
        })

        # Machine cutting time estimation
        hourly_rate = float(cnc_cfg.get("machine_hourly_rate", 15.00))
        m_rate_min = hourly_rate / 60.0

        # Estimated isolation cut distance: approx 3.5m per 100cm2
        est_iso_dist_mm = (area_cm2 * 35.0) * (2 if layers == 2 else 1)
        feed_mm_min = float(cnc_cfg.get("isolation_milling_feed_mm_min", 300.0))
        iso_time_min = (est_iso_dist_mm / feed_mm_min) * qty

        # Drill time: holes * time_per_hole + tools * change_time
        sec_per_hole = float(cnc_cfg.get("drill_time_per_hole_sec", 3.5))
        sec_per_tool = float(cnc_cfg.get("tool_change_time_sec", 25.0))
        drill_time_min = ((metrics["total_holes"] * sec_per_hole * qty) + (len(metrics["unique_drill_diameters"]) * sec_per_tool)) / 60.0

        # Cutout time: perimeter * passes / feed
        cutout_time_min = float(cnc_cfg.get("cutout_time_per_board_min", 4.0)) * qty

        total_machining_min = iso_time_min + drill_time_min + cutout_time_min
        machining_cost = total_machining_min * m_rate_min

        cnc_items.append({
            "item": f"CNC Spindle Time ({total_machining_min:.1f} minutes)",
            "details": f"Isolation milling, drilling {metrics['total_holes']} holes, cutout routing",
            "cost": machining_cost
        })

        # Tool wear
        wear_cost = float(cnc_cfg.get("tool_wear_cost_per_board", 1.50)) * qty
        cnc_items.append({
            "item": f"Tooling & Carbide Bit Wear ({qty} pcs)",
            "details": f"V-bit isolation cutters, micro-drills, 1.0mm endmill wear",
            "cost": wear_cost
        })

        cnc_total = sum(i["cost"] for i in cnc_items)
        cnc_quote = {
            "items": cnc_items,
            "total_machining_minutes": round(total_machining_min, 1),
            "total": round(cnc_total, 2),
            "unit_price": round(cnc_total / qty, 2)
        }

    return {
        "quantity": qty,
        "currency_symbol": sym,
        "currency_code": curr.get("code", "USD"),
        "lead_time_days": config.get("general", {}).get("default_lead_time_days", 3),
        "options": {
            "mask_color": mask_color,
            "silkscreen_color": silkscreen_color,
            "surface_finish": surface_finish,
            "board_thickness": board_thickness,
            "copper_weight": copper_weight,
            "without_producer_mark": remove_mark,
            "shape": shape
        },
        "fabrication": {
            "items": fab_items,
            "subtotal": round(fab_subtotal, 2),
            "tax_amount": round(tax_amount, 2),
            "total": round(fab_total, 2),
            "unit_price": round(fab_unit_price, 2)
        },
        "cnc_prototype": cnc_quote
    }


# -----------------------------------------------------------------------------
# Terminal Formatting and Reports
# -----------------------------------------------------------------------------

def print_terminal_quotation(folder_path, metrics, quote_data, config_path, metadata):
    """Render a clean, high-contrast, professional CLI report."""
    sym = quote_data["currency_symbol"]
    qty = quote_data["quantity"]
    opts = quote_data["options"]
    fab = quote_data["fabrication"]
    cnc = quote_data.get("cnc_prototype")

    W = 86

    print("\n" + "=" * W)
    print(f" {'PCB INSTANT QUOTATION & MANUFACTURING ESTIMATE':^{W-2}} ")
    print("=" * W)
    print(f" Source Project : {folder_path}")
    print(f" Detected Format: {metrics['eda_detected']} ({metrics['outline_source'] or 'Standard'})")
    if config_path:
        print(f" Pricing Config : {config_path}")
    if metadata.get("order_id") or metadata.get("user_id"):
        print(f" Order Reference: ID={metadata.get('order_id', '-')} | User={metadata.get('user_id', '-')} | Email={metadata.get('email', '-')}")
    print(f" Quote Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("-" * W)

    # 1. Physical Specifications
    print(f" {'PCB PHYSICAL SPECIFICATIONS':<{W-2}}")
    print("-" * W)
    print(f"  Dimensions    : {metrics['width_mm']:.2f} mm x {metrics['height_mm']:.2f} mm  ({metrics['width_mm']/25.4:.2f}\" x {metrics['height_mm']/25.4:.2f}\")")
    print(f"  Total Area    : {metrics['area_cm2']:.2f} cm²  ({metrics['area_cm2']/6.4516:.2f} sq in)")
    print(f"  Perimeter     : {metrics['perimeter_mm']:.2f} mm")
    print(f"  Board Shape   : {opts['shape']}")
    print(f"  Layer Count   : {metrics['copper_layers']}-Layer ({'Top + Bottom Copper' if metrics['copper_layers']==2 else f'{metrics['copper_layers']} Copper Layers'})")
    print(f"  Hole Count    : {metrics['total_holes']} holes ({len(metrics['unique_drill_diameters'])} distinct tool sizes)")
    print(f"  Drill Sizes   : {', '.join(f'{d:.3f}mm' for d in metrics['unique_drill_diameters'][:8])}{' ...' if len(metrics['unique_drill_diameters']) > 8 else ''}")
    print(f"  Min Hole Dia  : {metrics['min_drill_dia_mm']:.3f} mm {'(Micro-via)' if metrics['min_drill_dia_mm'] < 0.30 else '(Standard)'}")
    print(f"  Slots / Mills : {metrics['total_slots']} non-circular slots")
    print(f"  Solder Mask   : {opts['mask_color'].capitalize()} ({'2-Sided' if metrics['has_bottom_mask'] else '1-Sided'})")
    print(f"  Silkscreen    : {opts['silkscreen_color'].capitalize()} ({'2-Sided' if metrics['has_bottom_silk'] else '1-Sided'})")
    print(f"  Surface Finish: {opts['surface_finish']}")
    print(f"  Board Core    : {opts['board_thickness']} mm FR-4, {opts['copper_weight']} Copper")
    print(f"  Producer Mark : {'Remove Manufacturer Mark' if opts['without_producer_mark'] else 'Standard Manufacturer Mark'}")

    # 2. Fabrication Itemized Quotation
    print("\n" + "=" * W)
    print(f" {'STANDARD PCB FABRICATION QUOTATION (Quantity: ' + str(qty) + ' pcs)':<{W-2}}")
    print("=" * W)
    print(f"  {'Item Description':<44} | {'Details / Basis':<24} | {'Amount':>10}")
    print("-" * W)

    for it in fab["items"]:
        item_title = it["item"][:43]
        details = it["details"][:24]
        cost_str = f"{sym}{it['cost']:7.2f}"
        print(f"  {item_title:<44} | {details:<24} | {cost_str:>10}")

    print("-" * W)
    if fab["tax_amount"] > 0:
        print(f"  {'Subtotal':<71} : {sym}{fab['subtotal']:7.2f}")
        print(f"  {'Tax':<71} : {sym}{fab['tax_amount']:7.2f}")
    print(f"  {'TOTAL FABRICATION PRICE (' + str(qty) + ' pcs)':<71} : {sym}{fab['total']:7.2f}")
    print(f"  {'PRICE PER BOARD':<71} : {sym}{fab['unit_price']:7.2f} / pc")
    print(f"  {'Estimated Production Lead Time':<71} : {quote_data['lead_time_days']} business days")
    print("=" * W)

    # 3. CNC Prototype Comparison (if available)
    if cnc:
        print(f"\n {'ALTERNATIVE: DESKTOP CNC PROTOTYPE ISOLATION MILLING':<{W-2}}")
        print("-" * W)
        for ci in cnc["items"]:
            item_t = ci["item"][:43]
            det_t = ci["details"][:24]
            print(f"  {item_t:<44} | {det_t:<24} | {sym}{ci['cost']:7.2f}")
        print("-" * W)
        print(f"  {'TOTAL CNC PROTOTYPE PRICE (' + str(qty) + ' pcs)':<71} : {sym}{cnc['total']:7.2f}")
        print(f"  {'CNC Price Per Board':<71} : {sym}{cnc['unit_price']:7.2f} / pc")
        print(f"  {'Estimated Total Machining Time':<71} : {cnc['total_machining_minutes']} minutes")
        print("=" * W)

    print("\n [Tip] Update pricing rates easily anytime in quotation_config.json")
    print(" [Tip] Pass --qty <N>, --mask-color <color>, or --json for automated ordering.\n")


# -----------------------------------------------------------------------------
# Main CLI Entry Point
# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Instant PCB Manufacturing Quotation Calculator (KiCad & EasyEDA)"
    )
    parser.add_argument("folder", help="Path to project folder containing Gerber and Drill files (e.g., ./project01)")
    parser.add_argument("--config", type=str, default=None,
                        help="Path to custom quotation_config.json file")
    parser.add_argument("-n", "--qty", "--quantity", type=int, default=None,
                        help="Number of PCB units to quote (defaults to 5 or metadata amount)")
    parser.add_argument("--mask-color", type=str, default=None,
                        help="Solder mask color (green, red, yellow, blue, white, black, purple)")
    parser.add_argument("--silkscreen-color", type=str, default=None,
                        help="Silkscreen legend color (white, black, none)")
    parser.add_argument("--surface-finish", type=str, default=None,
                        help="Surface finish (HASL, LeadFree_HASL, ENIG)")
    parser.add_argument("--board-thickness", type=str, default=None,
                        help="Board thickness in mm (1.6, 0.8, 1.0, 1.2, 2.0)")
    parser.add_argument("--copper-weight", type=str, default=None,
                        help="Copper weight (1oz, 2oz)")
    parser.add_argument("--without-producer-mark", action="store_true", default=None,
                        help="Remove manufacturer / producer identification mark")
    parser.add_argument("--shape", type=str, default=None,
                        help="Board outline shape (Rectangular or Complex Polygon)")
    parser.add_argument("--mode", choices=["fab", "cnc", "both"], default="both",
                        help="Quotation mode: fab (standard manufacturing), cnc (desktop milling), both")
    parser.add_argument("--json", action="store_true",
                        help="Output quotation in JSON format")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Optional file path to save quotation result (.json or .txt)")

    args = parser.parse_args()

    folder_path = os.path.normpath(args.folder)
    if not os.path.exists(folder_path):
        print(f"Error: Folder '{folder_path}' does not exist.", file=sys.stderr)
        sys.exit(1)
    if not os.path.isdir(folder_path):
        folder_path = os.path.dirname(folder_path) or "."

    # 1. Load Configuration
    config, config_path = load_quotation_config(args.config, folder_path)
    if args.mode == "fab":
        config["general"]["show_cnc_comparison"] = False
        if "cnc_prototype" in config:
            config["cnc_prototype"]["enabled"] = False
    elif args.mode == "cnc":
        config["general"]["show_cnc_comparison"] = True
        if "cnc_prototype" in config:
            config["cnc_prototype"]["enabled"] = True

    # 2. Inspect Metadata & Job Files
    metadata = parse_project_metadata(folder_path)
    gbrjob_data = parse_kicad_gbrjob(folder_path)

    # 3. Categorize Project Files & Extract Metrics
    categorized = categorize_project_files(folder_path)
    metrics = analyze_board_metrics(folder_path, categorized, gbrjob_data)

    # 4. Determine Effective Options (CLI overrides > Metadata > Config Defaults)
    # Quantity
    effective_qty = args.qty
    if effective_qty is None and metadata.get("amount"):
        try:
            effective_qty = int(metadata["amount"])
        except (ValueError, TypeError):
            pass
    if effective_qty is None:
        effective_qty = config.get("general", {}).get("default_quantity", 5)

    # Solder Mask Color
    effective_mask = args.mask_color
    if effective_mask is None and metadata.get("pcb_unit_solder_mask_colors"):
        raw_colors = metadata["pcb_unit_solder_mask_colors"]
        if isinstance(raw_colors, list) and raw_colors:
            cand = str(raw_colors[0]).strip().lower()
            if "/" not in cand and "etc" not in cand:
                effective_mask = cand
        elif isinstance(raw_colors, str):
            if "/" not in raw_colors and "etc" not in raw_colors:
                for c in ("black", "green", "red", "yellow", "blue", "white", "purple", "matte_black", "matte_green"):
                    if c in raw_colors.lower():
                        effective_mask = c
                        break
    if effective_mask is None:
        effective_mask = "green"

    # Silkscreen Color
    effective_silk = args.silkscreen_color
    if effective_silk is None and metadata.get("silkscreen_color"):
        effective_silk = str(metadata["silkscreen_color"]).strip().lower()
    if effective_silk is None:
        effective_silk = "white"

    # Surface Finish
    effective_finish = args.surface_finish
    if effective_finish is None and gbrjob_data.get("finish") and gbrjob_data["finish"] != "None":
        effective_finish = str(gbrjob_data["finish"])
    if effective_finish is None and metadata.get("finish"):
        effective_finish = str(metadata["finish"])
    if effective_finish is None:
        effective_finish = "HASL"

    # Board Thickness
    effective_thick = args.board_thickness
    if effective_thick is None and gbrjob_data.get("board_thickness"):
        effective_thick = str(gbrjob_data["board_thickness"])
    if effective_thick is None and metadata.get("board_thickness"):
        effective_thick = str(metadata["board_thickness"])
    if effective_thick is None:
        effective_thick = "1.6"

    # Copper Weight
    effective_cu = args.copper_weight
    if effective_cu is None and metadata.get("copper_weight"):
        effective_cu = str(metadata["copper_weight"])
    if effective_cu is None:
        effective_cu = "1oz"

    # Without Producer Mark
    effective_rm_mark = args.without_producer_mark
    if effective_rm_mark is None:
        raw_rm = metadata.get("without_producer_mark")
        if raw_rm is True or str(raw_rm).lower() in ("true", "yes", "1"):
            effective_rm_mark = True
        else:
            effective_rm_mark = False

    # Shape override
    effective_shape = args.shape
    if effective_shape is None and metadata.get("pcb_shape"):
        raw_shape = str(metadata["pcb_shape"]).lower()
        if "rect" in raw_shape:
            effective_shape = "Rectangular"
        elif "other" in raw_shape or "complex" in raw_shape:
            effective_shape = "Complex Polygon / Irregular"
    if effective_shape is None:
        effective_shape = metrics["shape"]

    user_options = {
        "quantity": effective_qty,
        "mask_color": effective_mask,
        "silkscreen_color": effective_silk,
        "surface_finish": effective_finish,
        "board_thickness": effective_thick,
        "copper_weight": effective_cu,
        "without_producer_mark": effective_rm_mark,
        "shape": effective_shape
    }

    # 5. Calculate Quotation
    quote_data = calculate_quotation(metrics, config, user_options)

    # 6. Output Results
    full_output = {
        "project_folder": os.path.abspath(folder_path),
        "config_loaded": config_path,
        "metadata_found": bool(metadata),
        "metadata": metadata,
        "metrics": metrics,
        "quotation": quote_data
    }

    if args.json:
        json_str = json.dumps(full_output, indent=2)
        print(json_str)
        if args.output:
            with open(args.output, "w", encoding="utf-8") as f:
                f.write(json_str)
            print(f"\nQuotation JSON saved to: {args.output}", file=sys.stderr)
    else:
        print_terminal_quotation(folder_path, metrics, quote_data, config_path, metadata)
        if args.output:
            if args.output.endswith(".json"):
                with open(args.output, "w", encoding="utf-8") as f:
                    json.dump(full_output, f, indent=2)
            else:
                # Save text report
                import io
                buf = io.StringIO()
                old_stdout = sys.stdout
                sys.stdout = buf
                print_terminal_quotation(folder_path, metrics, quote_data, config_path, metadata)
                sys.stdout = old_stdout
                with open(args.output, "w", encoding="utf-8") as f:
                    f.write(buf.getvalue())
            print(f"Quotation report saved to: {args.output}")


if __name__ == "__main__":
    main()
