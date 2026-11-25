#!/usr/bin/env python3
"""
PCBtoGcode Configuration Studio (config_studio.py)
===================================================
A 100% standalone, zero-dependency configuration suite and Web UI for the
PCBtoGcode manufacturing pipeline.

Key Capabilities:
  1. Quick-Init Preset Hub (Home screen with one-click presets & diff previews).
  2. Single Source of Truth Config Management:
     - Master Machine/Sheet/Z-Limit-Switch: manufacture_config.json
     - Clamps only (stripped of duplicate bed/Z-switch settings): pcb_holders.json
     - Tooling (Feeds/Speeds/Depths): tool_config.json
     - UI & Hardware Transport: ui_config.json
     - Quotation & Pricing: quotation_config.json
     - Bed Pocket Milling: Bed_mods/pocket_config.json
  3. Pure-Python Gerber/Excellon/ZIP Scanner (generates project_metadata.json).
  4. Automatic Timestamped Backups (.config_backups/*.bak) with one-click restore.
  5. Live Cross-File Safety & Bounds Validator.
  6. Interactive 2D CNC Bed & Fixture SVG Preview Canvas.

Run:
  python3 config_studio.py
"""

import os
import sys
import json
import re
import glob
import time
import shutil
import zipfile
import datetime
import argparse
import webbrowser
import socket
import http.server
import socketserver
import urllib.parse
from typing import Dict, Any, List, Tuple, Optional

# Default file paths relative to root
FILE_PATHS = {
    "manufacture": "manufacture_config.json",
    "holders": "pcb_holders.json",
    "tool": "tool_config.json",
    "ui": "ui_config.json",
    "quotation": "quotation_config.json",
    "pocket": os.path.join("Bed_mods", "pocket_config.json")
}

# Canonical Default Schemas
FILE_DEFAULTS: Dict[str, Dict[str, Any]] = {
    "manufacture": {
        "selected_projects": ["project01", "project02", "project03"],
        "copper_sheet": {
            "sheet_width_mm": 206.0,
            "sheet_height_mm": 300.0,
            "thickness_mm": 1.6,
            "copper_weight_oz": 1.0
        },
        "cnc_limits": {
            "bed_travel_x_mm": 275.0,
            "bed_milling_x_mm": 260.0,
            "bed_travel_y_mm": 170.0,
            "bed_milling_y_mm": 165.0,
            "auto_y_split": True,
            "y_split_threshold_mm": 165.0,
            "min_y_split_mm": 132.0,
            "max_y_split_mm": 165.0,
            "front_half_min_y_mm": 8.5,
            "front_half_max_y_mm": 160.0,
            "back_half_min_y_mm": 135.0,
            "back_half_max_y_mm": 292.0,
            "left_margin_guide_width_mm": 18.0,
            "right_margin_mm": 5.0
        },
        "panelization": {
            "zero_gap_tight_packing": False,
            "board_gap_mm": 2.5,
            "allow_rotation_to_maximize_space": True,
            "allow_any_angle_rectangular": False,
            "allow_any_angle_non_rectangular": True,
            "rectangular_allowed_rotations_deg": [0, 90],
            "group_by_solder_mask_color": True,
            "strict_solder_mask_color": False
        },
        "pcb_tag_strip": {
            "include_pcb_tag_strip": True,
            "tag_width_mm": 5.0,
            "attach_along_shortest_edge": True,
            "breakaway_mode": "v_bit_and_milling",
            "available_breakaway_modes": [
                "drilling_only", "milling_only", "v_bit_only",
                "drilling_and_milling", "v_bit_and_milling"
            ],
            "retention_tab_count": 2,
            "retention_tab_width_mm": 2.0,
            "smooth_edge_finishing": True,
            "smooth_edge_recess_mm": 0.15,
            "engrave_text_both_sides": False,
            "engrave_arrow": True,
            "mouse_bite_drill_dia_mm": 0.8,
            "mouse_bite_pitch_mm": 1.5,
            "milling_tool_dia_mm": 1.0,
            "milling_cut_depth_per_side_mm": -0.85,
            "v_bit_angle_deg": 30.0,
            "v_bit_score_depth_per_side_mm": -0.65
        },
        "sheet_tag_strip": {
            "enable_sheet_tag_strip": True,
            "tag_position": "front_right",
            "engrave_sheet_number": True,
            "engrave_metadata_summary": True
        },
        "guides_and_fixturing": {
            "add_guide_lines": True,
            "enable_left_margin_guides": True,
            "enable_dowel_pin_lock": False,
            "alignment_pin_hole_dia_mm": 3.0,
            "holders_config_file": "pcb_holders.json",
            "z_probe_limit_switch": {
                "enabled": True,
                "x_mm": 272.001,
                "y_mm": 135.002,
                "height_above_bed_mm": 5.5,
                "width_mm": 7.0,
                "length_mm": 20.0,
                "danger_margin_mm": 6.0,
                "label": "Z-ZERO LIMIT SWITCH (DANGER ZONE)"
            },
            "startup_verification": {
                "enabled": True,
                "verify_x_mm": 20.0,
                "verify_y_mm": 20.0,
                "verify_z_mm": 2.0
            }
        },
        "inkjet_printer": {
            "enabled": True,
            "connection_type": "stm32_usb_cdc",
            "port": "auto",
            "baudrate": 115200,
            "dpi": 600,
            "uv_cure_enabled": True,
            "uv_led_power_pct": 85,
            "pass_speed_mm_s": 40.0,
            "x_offset_mm": 0.0,
            "y_offset_mm": 0.0
        },
        "output_paths": {
            "gcode_dir": "manufacture_gcode",
            "svg_dir": "manufacture_svg",
            "guides_dir": "manufacture_guides",
            "metadata_json": "manufacture_metadata.json",
            "checklist_json": "manufacture_guides/checklist_state.json",
            "html_dashboard": "manufacture_guides/index.html"
        }
    },
    "holders": {
        "version": "1.2.0",
        "description": "Common Single-Half CNC Bed Locking Washer Clamps (Single Source of Truth: Bed Limits & Z-Switch live in manufacture_config.json)",
        "units": "mm",
        "coordinate_frame": "cnc_bed_common_one_half",
        "washer_clamping_only": True,
        "drill_through_pcb": False,
        "holders": [
            {
                "id": "lock_bed_front_left",
                "name": "Front-Left Common M4 Washer Clamp",
                "shape": "circle",
                "x": -5.0,
                "y": 8.0,
                "diameter": 30.0,
                "holder_gap": 3.0,
                "step_zone": "bed_common",
                "common_for_both_halves": True,
                "drill_through_pcb": False,
                "anchor_type": "washer_clamp",
                "description": "Common bed locking screw with M4 washer clamping over PCB margin (no through-PCB anchor hole)"
            },
            {
                "id": "lock_bed_front_right",
                "name": "Front-Right Common M4 Washer Clamp",
                "shape": "circle",
                "x": 210.0,
                "y": 0.0,
                "diameter": 30.0,
                "holder_gap": 3.0,
                "step_zone": "bed_common",
                "common_for_both_halves": True,
                "drill_through_pcb": False,
                "anchor_type": "washer_clamp",
                "description": "Common bed locking screw with M4 washer clamping over PCB margin (no through-PCB anchor hole)"
            },
            {
                "id": "lock_bed_rear_right",
                "name": "Rear-Right Common Edge Washer Clamp",
                "shape": "circle",
                "x": 205.0,
                "y": 160.0,
                "diameter": 30.0,
                "holder_gap": 3.0,
                "step_zone": "bed_common",
                "common_for_both_halves": True,
                "drill_through_pcb": False,
                "anchor_type": "washer_clamp",
                "description": "Common bed locking screw with M4 washer clamping over PCB margin (no through-PCB anchor hole)"
            },
            {
                "id": "lock_bed_rear_left",
                "name": "Rear-Left Common Edge Washer Clamp",
                "shape": "circle",
                "x": -3.0,
                "y": 160.0,
                "diameter": 30.0,
                "holder_gap": 3.0,
                "step_zone": "bed_common",
                "common_for_both_halves": True,
                "drill_through_pcb": False,
                "anchor_type": "washer_clamp",
                "description": "Common bed locking screw with M4 washer clamping over PCB margin (no through-PCB anchor hole)"
            }
        ]
    },
    "tool": {
        "isolation": {
            "tool_dia": 0.2,
            "cut_depth": -0.04,
            "feed_rate": 300.0,
            "plunge_rate": 80.0,
            "spindle_speed": 10000,
            "passes": 1,
            "stepover": 0.7,
            "safe_z": 5.0,
            "engrave_edge": True
        },
        "drilling": {
            "drill_depth": -1.8,
            "drill_feed": 150.0,
            "safe_z": 5.0,
            "tool_change_z": 20.0,
            "spindle_speed": 10000
        },
        "slot_milling": {
            "tool_dia": 1.0,
            "cut_depth": -1.8,
            "feed_rate": 200.0,
            "plunge_rate": 80.0,
            "safe_z": 5.0,
            "spindle_speed": 10000,
            "passes": 2
        },
        "cutout": {
            "tool_dia": 1.0,
            "cutout_depth": -1.8,
            "cutout_passes": 3,
            "cutout_feed": 200.0,
            "plunge_rate": 80.0,
            "safe_z": 5.0,
            "spindle_speed": 10000
        },
        "general": {
            "margin": 0.0
        }
    },
    "ui": {
        "touch_probe_enabled": False,
        "show_macros_in_dispatch": True,
        "enable_homing_btn": True,
        "enable_unlock_btn": True,
        "enable_zero_work_btn": True,
        "enable_pcb_z_offset": True,
        "enable_heightmap_leveling": True,
        "cnc_transport": "auto",
        "cnc_serial_port": "auto",
        "cnc_baudrate": 115200
    },
    "quotation": {
        "_description": "PCB Instant Quotation Pricing Configuration (in Sri Lankan Rupees - LKR).",
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
                "flat_price": 2800.00,
                "description": "Standard 100x100mm (5 pcs) 2-Layer Prototype Promo (Rs. 2,800)"
            },
            "area_pricing": {
                "raw_a4_board_cost_lkr": 1500.00,
                "raw_cost_per_cm2": 2.405,
                "profit_margin_percent": 60.0,
                "price_per_cm2": 6.00,
                "minimum_charge_area_cm2": 25.0
            },
            "layer_multipliers": {
                "1": 0.70, "2": 1.00, "4": 2.80, "6": 5.00
            },
            "quantity_discounts": {
                "1": 1.00, "5": 1.00, "10": 0.88, "20": 0.80, "30": 0.74,
                "50": 0.65, "100": 0.52, "200": 0.45, "500": 0.38
            },
            "drilling": {
                "via_plating_method": "Electroless Copper Seed Layer + Acid Copper Electroplating",
                "free_holes": 500,
                "fee_per_extra_100_holes": 180.00,
                "min_diameter_limits": {
                    "standard_min_mm": 0.30,
                    "micro_drill_surcharge": 2500.00,
                    "high_density_min_mm": 0.20,
                    "high_density_surcharge": 6800.00
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
                    "green": 0.00, "red": 650.00, "yellow": 650.00, "blue": 650.00,
                    "white": 900.00, "black": 900.00, "purple": 1200.00,
                    "matte_black": 1500.00, "matte_green": 1500.00
                },
                "double_sided_included": True
            },
            "silkscreen": {
                "colors": {
                    "white": 0.00, "black": 450.00, "none": 0.00
                },
                "double_sided_extra": 450.00
            },
            "surface_finish": {
                "none": 0.00, "HASL": 250.00, "LeadFree_HASL": 850.00,
                "Immersion_Tin": 1200.00, "ENIG": 5400.00
            },
            "board_thickness_mm": {
                "1.6": 0.00, "0.8": 750.00, "1.0": 450.00, "1.2": 240.00, "2.0": 1350.00
            },
            "copper_weight": {
                "1oz": 0.00, "2oz": 3600.00
            },
            "producer_mark": {
                "remove_mark_fee": 600.00
            }
        },
        "cnc_prototype": {
            "enabled": True,
            "setup_fee": 2400.00,
            "material_fr1_per_cm2": 2.00,
            "material_fr4_per_cm2": 4.00,
            "machine_hourly_rate": 4500.00,
            "isolation_milling_feed_mm_min": 300.0,
            "estimated_copper_density": 0.35,
            "drill_time_per_hole_sec": 3.5,
            "tool_change_time_sec": 25.0,
            "slot_milling_rate_per_mm": 15.00,
            "cutout_time_per_board_min": 4.0,
            "tool_wear_cost_per_board": 400.00
        }
    },
    "pocket": {
        "version": "1.0.0",
        "description": "Universal CNC Pocket Milling Configuration - Any Shape, Any Location, Any Depth",
        "shape": {
            "type": "rectangle",
            "width_mm": 5.9,
            "length_mm": 19.5,
            "corner_radius_mm": 0.0,
            "rotation_deg": 90.0,
            "clearance_mm": 0.0
        },
        "position": {
            "reference": "middle_of_bed",
            "custom_center_x_mm": 137.5,
            "custom_center_y_mm": 85.0
        },
        "depth": {
            "total_depth_mm": 10.0,
            "stepdown_mm": 0.5
        },
        "tool": {
            "type": "v_bit",
            "v_bit": {
                "angle_deg": 30.0,
                "tip_dia_mm": 0.2,
                "shank_dia_mm": 3.0
            },
            "endmill": {
                "tool_dia_mm": 3.0
            },
            "stepover_ratio": 0.55
        },
        "feeds_and_speeds": {
            "feed_xy_mm_min": 350.0,
            "plunge_rate_mm_min": 80.0,
            "spindle_speed_rpm": 10000,
            "safe_z_mm": 5.0,
            "clearance_z_mm": 20.0
        },
        "machine_limits": {
            "bed_travel_x_mm": 275.0,
            "bed_travel_y_mm": 170.0,
            "edge_safety_margin_mm": 0.5
        },
        "output": {
            "gcode_filename": "center_switch_pocket_rotated90.gcode",
            "svg_filename": "center_switch_pocket_rotated90.svg"
        }
    }
}

# Quick-Init Preset Bundles
PRESET_BUNDLES: Dict[str, Dict[str, Any]] = {
    "a4_doublesided_standard": {
        "name": "A4 Dual-Sided FR4 Standard (Recommended)",
        "badge": "Standard Production",
        "color": "emerald",
        "description": "Full A4 (206 x 300 mm) double-sided panelization with Y-split at 165mm, both V-Bit & Milling breakaway tags, 4 common bed washer clamps, and 300 mm/min feed.",
        "configs": {
            "manufacture": {
                "copper_sheet": {"sheet_width_mm": 206.0, "sheet_height_mm": 300.0, "thickness_mm": 1.6, "copper_weight_oz": 1.0},
                "cnc_limits": {
                    "bed_travel_x_mm": 275.0, "bed_milling_x_mm": 260.0, "bed_travel_y_mm": 170.0, "bed_milling_y_mm": 165.0,
                    "auto_y_split": True, "y_split_threshold_mm": 165.0, "min_y_split_mm": 132.0, "max_y_split_mm": 165.0,
                    "front_half_min_y_mm": 8.5, "front_half_max_y_mm": 160.0, "back_half_min_y_mm": 135.0, "back_half_max_y_mm": 292.0,
                    "left_margin_guide_width_mm": 18.0, "right_margin_mm": 5.0
                },
                "pcb_tag_strip": {"breakaway_mode": "v_bit_and_milling", "tag_width_mm": 5.0, "retention_tab_count": 2, "smooth_edge_finishing": True}
            },
            "tool": {
                "isolation": {"tool_dia": 0.2, "cut_depth": -0.04, "feed_rate": 300.0, "spindle_speed": 10000, "passes": 1},
                "drilling": {"drill_depth": -1.8, "drill_feed": 150.0, "spindle_speed": 10000},
                "cutout": {"tool_dia": 1.0, "cutout_depth": -1.8, "cutout_passes": 3, "cutout_feed": 200.0}
            },
            "ui": {"cnc_transport": "auto"}
        }
    },
    "a4_singlesided_fast": {
        "name": "A4 Single-Sided FR1 / FR4 (Fast Prototype)",
        "badge": "High Speed",
        "color": "cyan",
        "description": "Rapid single-layer routing without bottom copper passes. Pure milling breakaway, aggressive 350 mm/min isolation feed, and zero-gap packing enabled.",
        "configs": {
            "manufacture": {
                "copper_sheet": {"sheet_width_mm": 206.0, "sheet_height_mm": 300.0, "thickness_mm": 1.6, "copper_weight_oz": 1.0},
                "cnc_limits": {
                    "bed_travel_x_mm": 275.0, "bed_milling_x_mm": 260.0, "bed_travel_y_mm": 170.0, "bed_milling_y_mm": 165.0,
                    "auto_y_split": True, "y_split_threshold_mm": 165.0
                },
                "panelization": {"zero_gap_tight_packing": True, "board_gap_mm": 2.0},
                "pcb_tag_strip": {"breakaway_mode": "milling_only", "tag_width_mm": 4.0, "retention_tab_count": 2, "smooth_edge_finishing": False}
            },
            "tool": {
                "isolation": {"tool_dia": 0.2, "cut_depth": -0.045, "feed_rate": 350.0, "spindle_speed": 10000, "passes": 1},
                "drilling": {"drill_depth": -1.8, "drill_feed": 180.0, "spindle_speed": 10000},
                "cutout": {"tool_dia": 1.0, "cutout_depth": -1.8, "cutout_passes": 2, "cutout_feed": 240.0}
            }
        }
    },
    "a5_half_sheet_no_split": {
        "name": "Half-A4 / A5 Small Sheet (Zero Y-Shift)",
        "badge": "No Y-Shift",
        "color": "amber",
        "description": "Direct single-half manufacturing on half-sheet FR4 (206 x 150 mm). auto_y_split is disabled so all machining completes in one fixture without sliding stock.",
        "configs": {
            "manufacture": {
                "copper_sheet": {"sheet_width_mm": 206.0, "sheet_height_mm": 150.0, "thickness_mm": 1.6, "copper_weight_oz": 1.0},
                "cnc_limits": {
                    "bed_travel_x_mm": 275.0, "bed_milling_x_mm": 260.0, "bed_travel_y_mm": 170.0, "bed_milling_y_mm": 165.0,
                    "auto_y_split": False, "y_split_threshold_mm": 165.0, "front_half_min_y_mm": 5.0, "front_half_max_y_mm": 145.0
                },
                "panelization": {"board_gap_mm": 2.5}
            }
        }
    },
    "fine_pitch_smd": {
        "name": "Fine-Pitch SMD & High Precision (0.1mm V-Bit)",
        "badge": "High Precision",
        "color": "violet",
        "description": "Ultra-sharp 0.1mm 30° V-Bit isolation with 2 passes (0.55 stepover), shallow -0.035mm depth, conservative 220 mm/min feed, and strict smooth-edge finishing.",
        "configs": {
            "tool": {
                "isolation": {"tool_dia": 0.1, "cut_depth": -0.035, "feed_rate": 220.0, "plunge_rate": 60.0, "spindle_speed": 10000, "passes": 2, "stepover": 0.55},
                "drilling": {"drill_depth": -1.8, "drill_feed": 110.0, "spindle_speed": 10000},
                "cutout": {"tool_dia": 1.0, "cutout_depth": -1.8, "cutout_passes": 4, "cutout_feed": 160.0}
            },
            "manufacture": {
                "pcb_tag_strip": {"smooth_edge_finishing": True, "smooth_edge_recess_mm": 0.20, "v_bit_angle_deg": 30.0, "v_bit_score_depth_per_side_mm": -0.70}
            }
        }
    },
    "safe_beginner_conservative": {
        "name": "Safe Beginner & Machine Calibration",
        "badge": "Ultra Safe",
        "color": "blue",
        "description": "Slow, extra-safe feeds (200 mm/min isolation, 100 mm/min drill), 4 cutout passes, enlarged 8.0mm Z-probe danger margin, and startup verification enabled.",
        "configs": {
            "tool": {
                "isolation": {"tool_dia": 0.2, "cut_depth": -0.038, "feed_rate": 200.0, "plunge_rate": 60.0, "spindle_speed": 9000, "passes": 1, "safe_z": 6.0},
                "drilling": {"drill_depth": -1.8, "drill_feed": 100.0, "safe_z": 6.0, "spindle_speed": 9000},
                "cutout": {"tool_dia": 1.0, "cutout_depth": -1.8, "cutout_passes": 4, "cutout_feed": 150.0, "safe_z": 6.0}
            },
            "manufacture": {
                "panelization": {"board_gap_mm": 4.0},
                "guides_and_fixturing": {
                    "startup_verification": {"enabled": True, "verify_x_mm": 20.0, "verify_y_mm": 20.0, "verify_z_mm": 2.0},
                    "z_probe_limit_switch": {"danger_margin_mm": 8.0}
                }
            }
        }
    }
}


# ---------------------------------------------------------------------------
# Backup Manager
# ---------------------------------------------------------------------------

class BackupManager:
    """Manages automatic timestamped .config_backups/*.bak before write operations."""
    def __init__(self, root_dir: str = "."):
        self.root_dir = os.path.abspath(root_dir)
        self.backup_dir = os.path.join(self.root_dir, ".config_backups")
        os.makedirs(self.backup_dir, exist_ok=True)

    def backup_file(self, rel_path: str) -> Optional[str]:
        src = os.path.join(self.root_dir, rel_path)
        if not os.path.isfile(src):
            return None
        safe_name = rel_path.replace(os.sep, "_").replace("/", "_")
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        bak_filename = f"{safe_name}.{timestamp}.bak"
        dst = os.path.join(self.backup_dir, bak_filename)
        shutil.copy2(src, dst)
        return bak_filename

    def list_backups(self) -> List[Dict[str, Any]]:
        if not os.path.isdir(self.backup_dir):
            return []
        items = []
        for fn in sorted(os.listdir(self.backup_dir), reverse=True):
            if fn.endswith(".bak"):
                fp = os.path.join(self.backup_dir, fn)
                try:
                    stat = os.stat(fp)
                    items.append({
                        "filename": fn,
                        "size_bytes": stat.st_size,
                        "created_at": datetime.datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
                    })
                except Exception:
                    pass
        return items

    def restore_backup(self, bak_filename: str) -> Tuple[bool, str]:
        src = os.path.join(self.backup_dir, os.path.basename(bak_filename))
        if not os.path.isfile(src):
            return False, f"Backup file {bak_filename} not found."
        
        # Parse original target filename: e.g. "manufacture_config.json.20261124_221000.bak" or "Bed_mods_pocket_config.json.20261124_221000.bak"
        parts = bak_filename.split(".")
        if len(parts) < 3:
            return False, "Invalid backup filename format."
        
        orig_base = parts[0]
        ext = parts[1]
        orig_rel = f"{orig_base}.{ext}".replace("_pocket_config", os.sep + "pocket_config")
        target_path = os.path.join(self.root_dir, orig_rel)
        
        # Take safety backup of current target before restore
        self.backup_file(orig_rel)
        
        os.makedirs(os.path.dirname(target_path), exist_ok=True)
        shutil.copy2(src, target_path)
        return True, f"Successfully restored {orig_rel} from {bak_filename}."


# ---------------------------------------------------------------------------
# Cross-File Schema & Safety Validator
# ---------------------------------------------------------------------------

class ConfigValidator:
    """Validates parameters across all configs against physical CNC safety limits."""
    @staticmethod
    def validate_all(configs: Dict[str, Any], root_dir: str = ".") -> Dict[str, Any]:
        errors: List[str] = []
        warnings: List[str] = []

        mcfg = configs.get("manufacture") or {}
        hcfg = configs.get("holders") or {}
        tcfg = configs.get("tool") or {}
        ucfg = configs.get("ui") or {}

        # 1. Sheet & Bed Dimensions
        sheet = mcfg.get("copper_sheet", {})
        sw = float(sheet.get("sheet_width_mm", 0))
        sh = float(sheet.get("sheet_height_mm", 0))
        st = float(sheet.get("thickness_mm", 0))
        if sw <= 10.0 or sh <= 10.0:
            errors.append(f"Sheet dimensions invalid: width={sw}mm, height={sh}mm must be > 10mm.")
        if st <= 0.2 or st > 5.0:
            warnings.append(f"Unusual PCB thickness: {st}mm (typical FR4 is 0.8, 1.2, or 1.6mm).")

        limits = mcfg.get("cnc_limits", {})
        tx = float(limits.get("bed_travel_x_mm", 275.0))
        ty = float(limits.get("bed_travel_y_mm", 170.0))
        mx = float(limits.get("bed_milling_x_mm", 260.0))
        my = float(limits.get("bed_milling_y_mm", 165.0))
        if mx > tx or my > ty:
            errors.append(f"Milling area ({mx}x{my}mm) exceeds machine bed travel ({tx}x{ty}mm).")

        # 2. Y-Split Safety
        if limits.get("auto_y_split", True):
            ysplit = float(limits.get("y_split_threshold_mm", 165.0))
            if ysplit > ty:
                errors.append(f"Y-split threshold ({ysplit}mm) exceeds bed travel ({ty}mm).")
            f_max = float(limits.get("front_half_max_y_mm", 160.0))
            if f_max > ty:
                errors.append(f"Front half max Y ({f_max}mm) exceeds bed travel ({ty}mm).")

        # 3. Z-Probe Limit Switch & Fixture Overlap Check
        z_sw = mcfg.get("guides_and_fixturing", {}).get("z_probe_limit_switch", {})
        if z_sw.get("enabled", True):
            zx = float(z_sw.get("x_mm", 272.001))
            zy = float(z_sw.get("y_mm", 135.002))
            zw = float(z_sw.get("width_mm", 7.0)) / 2.0 + float(z_sw.get("danger_margin_mm", 6.0))
            zl = float(z_sw.get("length_mm", 20.0)) / 2.0 + float(z_sw.get("danger_margin_mm", 6.0))
            z_min_x, z_max_x = zx - zw, zx + zw
            z_min_y, z_max_y = zy - zl, zy + zl

            # Verify clamps in pcb_holders.json do not sit in limit switch danger zone
            holders = hcfg.get("holders", [])
            for h in holders:
                hx = float(h.get("x", 0))
                hy = float(h.get("y", 0))
                hr = (float(h.get("diameter", 30.0)) / 2.0) if h.get("shape", "circle") == "circle" else (float(h.get("width", 20.0)) / 2.0)
                # Check Front Half
                if (z_min_x <= hx <= z_max_x) and (z_min_y <= hy <= z_max_y):
                    errors.append(f"Clamp '{h.get('name', h.get('id'))}' at ({hx}, {hy}) overlaps Bed Z-Probe Switch Danger Zone!")
                # Check Back Half (shifted)
                if limits.get("auto_y_split", True):
                    hy_back = hy + float(limits.get("y_split_threshold_mm", 165.0))
                    if (z_min_x <= hx <= z_max_x) and (z_min_y <= hy_back <= z_max_y):
                        errors.append(f"Shifted Back-Half clamp '{h.get('name')}' at Y={hy_back:.1f}mm overlaps Z-Probe Switch Danger Zone!")

        # 4. Tool Depths & Feeds
        iso = tcfg.get("isolation", {})
        if iso:
            c_depth = float(iso.get("cut_depth", -0.04))
            if c_depth >= 0:
                errors.append(f"Isolation cut_depth must be negative (got {c_depth} mm).")
            elif c_depth < -0.15:
                warnings.append(f"Deep isolation depth ({c_depth} mm) may erode thin traces with 30° V-bit.")
            if float(iso.get("feed_rate", 300)) > 600:
                warnings.append(f"High isolation feed ({iso.get('feed_rate')} mm/min) risk breaking fine 0.1/0.2mm V-bits.")

        drill = tcfg.get("drilling", {})
        if drill:
            d_depth = float(drill.get("drill_depth", -1.8))
            if d_depth >= 0:
                errors.append(f"Drill depth must be negative (got {d_depth} mm).")

        cutout = tcfg.get("cutout", {})
        if cutout:
            co_depth = float(cutout.get("cutout_depth", -1.8))
            if co_depth >= 0:
                errors.append(f"Cutout depth must be negative (got {co_depth} mm).")

        # 5. Selected Projects Existence
        sel_projs = mcfg.get("selected_projects", [])
        if not sel_projs:
            warnings.append("No projects selected in manufacture_config.json -> selected_projects.")
        else:
            for p in sel_projs:
                p_dir = os.path.join(root_dir, p)
                meta_file = os.path.join(p_dir, "project_metadata.json")
                if not os.path.isdir(p_dir):
                    warnings.append(f"Selected project folder '{p}' does not exist on disk.")
                elif not os.path.isfile(meta_file):
                    warnings.append(f"Selected project '{p}' is missing project_metadata.json (use Projects tab to scan it).")

        # 6. Duplication Check (pcb_holders.json)
        if "bed_limits" in hcfg or "z_probe_limit_switch" in hcfg:
            warnings.append("Legacy duplicated keys ('bed_limits' or 'z_probe_limit_switch') detected in pcb_holders.json. Run 'Clean Up Duplicates' to maintain single source of truth.")

        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "warnings": warnings,
            "checked_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }


# ---------------------------------------------------------------------------
# Pure-Python Gerber / Excellon / ZIP Project Scanner
# ---------------------------------------------------------------------------

class ProjectScanner:
    """Pure-Python parser for Gerber, Excellon, and ZIP packages without third-party libraries."""
    @staticmethod
    def scan_folder(folder_path: str) -> Dict[str, Any]:
        folder_path = os.path.abspath(folder_path)
        folder_name = os.path.basename(folder_path)
        all_files = os.listdir(folder_path) if os.path.isdir(folder_path) else []

        # Find candidate layer files
        layers: Dict[str, Any] = {
            "outline": {"file": None, "type": "gerber", "role": "edge_cuts", "present": False},
            "copper_top": {"file": None, "type": "gerber", "role": "f_cu", "present": False},
            "copper_bottom": {"file": None, "type": "gerber", "role": "b_cu", "present": False},
            "mask_top": {"file": None, "type": "gerber", "role": "f_mask", "present": False},
            "mask_bottom": {"file": None, "type": "gerber", "role": "b_mask", "present": False},
            "silkscreen_top": {"file": None, "type": "gerber", "role": "f_silkscreen", "present": False},
            "silkscreen_bottom": {"file": None, "type": "gerber", "role": "b_silkscreen", "present": False},
            "paste_top": {"file": None, "type": "gerber", "role": "f_paste", "present": False},
            "paste_bottom": {"file": None, "type": "gerber", "role": "b_paste", "present": False}
        }
        drills: Dict[str, Any] = {}

        ecad = "KiCad"
        patterns = {
            "outline": [r".*edge[._-]cuts.*\.gbr$", r".*boardoutline.*\.gko$", r".*\.gko$", r".*\.gm1$"],
            "copper_top": [r".*f[._-]cu.*\.gbr$", r".*toplayer.*\.gtl$", r".*\.gtl$"],
            "copper_bottom": [r".*b[._-]cu.*\.gbr$", r".*bottomlayer.*\.gbl$", r".*\.gbl$"],
            "mask_top": [r".*f[._-]mask.*\.gbr$", r".*topsoldermask.*\.gts$", r".*\.gts$"],
            "mask_bottom": [r".*b[._-]mask.*\.gbr$", r".*bottomsoldermask.*\.gbs$", r".*\.gbs$"],
            "silkscreen_top": [r".*f[._-]silk.*\.gbr$", r".*topsilkscreen.*\.gto$", r".*\.gto$"],
            "silkscreen_bottom": [r".*b[._-]silk.*\.gbr$", r".*bottomsilkscreen.*\.gbo$", r".*\.gbo$"],
            "paste_top": [r".*f[._-]paste.*\.gbr$", r".*toppaste.*\.gtp$", r".*\.gtp$"],
            "paste_bottom": [r".*b[._-]paste.*\.gbr$", r".*bottompaste.*\.gbp$", r".*\.gbp$"]
        }

        for fn in all_files:
            fn_lower = fn.lower()
            if "easyeda" in fn_lower or fn_lower.startswith("gerber_"):
                ecad = "EasyEDA"
            for role, regexes in patterns.items():
                if not layers[role]["present"]:
                    for r in regexes:
                        if re.match(r, fn_lower):
                            layers[role]["file"] = fn
                            layers[role]["present"] = True
                            break

            # Excellon drill check
            if fn_lower.endswith(".drl") or (fn_lower.endswith(".txt") and "drill" in fn_lower):
                role = "npth" if "npth" in fn_lower else "pth"
                drills[role] = {
                    "file": fn,
                    "type": "excellon",
                    "plated": (role == "pth"),
                    "present": True
                }

        # Parse outline dimensions
        dims = {"width": 50.0, "height": 50.0, "thickness": 1.6}
        if layers["outline"]["present"] and layers["outline"]["file"]:
            ol_path = os.path.join(folder_path, layers["outline"]["file"])
            parsed_dims = ProjectScanner._parse_gerber_bounds(ol_path)
            if parsed_dims:
                dims.update(parsed_dims)

        # Parse drill stats
        drill_stats = {"total_holes": 0, "unique_drill_diameters_mm": [], "min_drill_dia_mm": 0.8, "total_slots": 0}
        for dkey, dval in drills.items():
            if dval.get("file"):
                dp = os.path.join(folder_path, dval["file"])
                ds = ProjectScanner._parse_excellon_stats(dp)
                drill_stats["total_holes"] += ds["total_holes"]
                drill_stats["total_slots"] += ds["total_slots"]
                drill_stats["unique_drill_diameters_mm"].extend(ds["unique_drill_diameters_mm"])
        drill_stats["unique_drill_diameters_mm"] = sorted(list(set(drill_stats["unique_drill_diameters_mm"])))
        if drill_stats["unique_drill_diameters_mm"]:
            drill_stats["min_drill_dia_mm"] = min(drill_stats["unique_drill_diameters_mm"])

        # Construct full metadata structure
        calc_area = round((dims["width"] * dims["height"]) / 100.0, 2)
        calc_perim = round(2 * (dims["width"] + dims["height"]), 2)
        pcb_id = f"PCB-{folder_name.upper()[:6]}"

        metadata: Dict[str, Any] = {
            "$schema_version": "1.0.0",
            "order_info": {
                "order_id": f"ORD-{datetime.datetime.now().strftime('%Y%m%d')}-001",
                "job_id": f"JOB-{int(time.time()) % 1000000}",
                "created_at": datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S+05:30"),
                "status": "pending_production"
            },
            "customer_info": {
                "user_id": "local-workshop-user",
                "email": "maker@local.lab",
                "contact_no": "+94000000000"
            },
            "pcb_info": {
                "pcb_id": pcb_id,
                "project_name": folder_name,
                "revision": "v1.0",
                "ecad_software": ecad,
                "without_producer_mark": False,
                "batch": {
                    "quantity": 1,
                    "unit_id_format": "{pcb_id}-{index:03d}",
                    "pcb_unit_ids": [f"{pcb_id}-001"]
                }
            },
            "physical_specs": {
                "dimensions_mm": dims,
                "shape": "rectangular",
                "calculated_area_cm2": calc_area,
                "perimeter_mm": calc_perim,
                "layer_count": 2 if layers["copper_bottom"]["present"] else 1,
                "copper_weight_oz": 1.0,
                "surface_finish": "HASL_lead_free"
            },
            "visual_specs": {
                "solder_mask": {
                    "top_color": "green",
                    "bottom_color": "green",
                    "custom_unit_colors": {"enabled": False, "selected_colors": ["green"], "color_surcharge_applicable": False}
                },
                "silkscreen": {
                    "top_color": "white",
                    "bottom_color": "white" if layers["silkscreen_bottom"]["present"] else "none"
                }
            },
            "pcb_files": {
                "local_dir": "./",
                "archive_file": f"{folder_name}.zip" if f"{folder_name}.zip" in all_files else None,
                "layers": layers,
                "drills": drills
            },
            "drill_analysis": drill_stats,
            "cam_and_manufacturing": {
                "local_gcode_output_dir": f"./{folder_name}_gcode",
                "milling_strategy": "isolation_routing",
                "origin_zero": "lower_left_corner"
            }
        }
        return metadata

    @staticmethod
    def _parse_gerber_bounds(file_path: str) -> Optional[Dict[str, float]]:
        """Extract rectangular bounding box from Gerber Edge_Cuts coordinates."""
        try:
            with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
                content = f.read()
        except Exception:
            return None

        # Determine scale (e.g., %FSLAX46Y46*% or %FSLAX24Y24*%)
        scale = 1.0
        is_inch = "%MOIN" in content
        fs_m = re.search(r"%FS[A-Z]*X(\d)(\d)Y", content)
        if fs_m:
            decimals = int(fs_m.group(2))
            scale = 10.0 ** (-decimals)
        elif "%FS" in content:
            scale = 0.000001

        xs: List[float] = []
        ys: List[float] = []
        coord_matches = re.finditer(r"X(-?\d+)Y(-?\d+)", content)
        for m in coord_matches:
            vx = float(m.group(1)) * scale
            vy = float(m.group(2)) * scale
            if is_inch:
                vx *= 25.4
                vy *= 25.4
            xs.append(vx)
            ys.append(vy)

        if xs and ys:
            w = round(max(xs) - min(xs), 2)
            h = round(max(ys) - min(ys), 2)
            if w > 0 and h > 0:
                return {"width": w, "height": h, "thickness": 1.6}
        return None

    @staticmethod
    def _parse_excellon_stats(file_path: str) -> Dict[str, Any]:
        """Extract tool diameters and hole counts from Excellon drill file."""
        total_holes = 0
        total_slots = 0
        dias: List[float] = []
        try:
            with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
                lines = f.readlines()
        except Exception:
            return {"total_holes": 0, "total_slots": 0, "unique_drill_diameters_mm": []}

        is_inch = True
        tool_dias: Dict[str, float] = {}
        for line in lines:
            line = line.strip()
            if "METRIC" in line or "mm" in line.lower():
                is_inch = False
            elif "INCH" in line:
                is_inch = True

            # Tool definition, e.g. T01C0.800 or T1F00S00C0.0312
            tm = re.search(r"T(\d+)[^C]*C([0-9.]+)", line)
            if tm:
                t_idx = tm.group(1)
                dia = float(tm.group(2))
                if is_inch:
                    dia = round(dia * 25.4, 3)
                tool_dias[t_idx] = dia
                dias.append(dia)

            # Hole coordinate
            if re.match(r"^X-?\d+Y-?\d+", line):
                total_holes += 1
            if "G85" in line or "slot" in line.lower():
                total_slots += 1

        return {
            "total_holes": total_holes,
            "total_slots": total_slots,
            "unique_drill_diameters_mm": sorted(list(set(dias)))
        }

    @staticmethod
    def extract_zip(zip_path: str, dest_folder: str) -> Tuple[bool, str]:
        """Safely extract ZIP file protecting against Zip-Slip path traversal."""
        try:
            os.makedirs(dest_folder, exist_ok=True)
            with zipfile.ZipFile(zip_path, 'r') as zf:
                for member in zf.infolist():
                    target_path = os.path.abspath(os.path.join(dest_folder, member.filename))
                    if not target_path.startswith(os.path.abspath(dest_folder)):
                        return False, "Security violation: Zip-Slip path traversal detected."
                    zf.extract(member, dest_folder)
            return True, f"Extracted {len(zf.namelist())} files into {dest_folder}."
        except Exception as e:
            return False, f"Failed extracting ZIP: {e}"


# ---------------------------------------------------------------------------
# Studio Workspace State Manager
# ---------------------------------------------------------------------------

class StudioState:
    """Loads, saves, and synchronizes all configs across the workspace."""
    def __init__(self, root_dir: str = "."):
        self.root_dir = os.path.abspath(root_dir)
        self.backup_mgr = BackupManager(self.root_dir)

    def load_all(self) -> Dict[str, Any]:
        configs: Dict[str, Any] = {}
        for key, rel_path in FILE_PATHS.items():
            full_path = os.path.join(self.root_dir, rel_path)
            if os.path.isfile(full_path):
                try:
                    with open(full_path, "r", encoding="utf-8") as f:
                        configs[key] = json.load(f)
                except Exception as e:
                    configs[key] = {"_error": f"Failed parsing JSON: {e}"}
            else:
                configs[key] = None

        # Scan for existing projects
        projects = []
        for d in sorted(os.listdir(self.root_dir)):
            full_d = os.path.join(self.root_dir, d)
            if os.path.isdir(full_d) and not d.startswith(".") and d not in ("pcb_env", "manufacture_gcode", "manufacture_svg", "manufacture_guides", "__pycache__"):
                meta_p = os.path.join(full_d, "project_metadata.json")
                if os.path.isfile(meta_p):
                    try:
                        with open(meta_p, "r", encoding="utf-8") as mf:
                            mdata = json.load(mf)
                        pinfo = mdata.get("pcb_info", {})
                        pspec = mdata.get("physical_specs", {})
                        projects.append({
                            "folder": d,
                            "pcb_id": pinfo.get("pcb_id", d),
                            "project_name": pinfo.get("project_name", d),
                            "dimensions": pspec.get("dimensions_mm", {}),
                            "solder_mask": mdata.get("visual_specs", {}).get("solder_mask", {}).get("top_color", "green"),
                            "quantity": pinfo.get("batch", {}).get("quantity", 1),
                            "ecad": pinfo.get("ecad_software", "Unknown"),
                            "has_metadata": True
                        })
                    except Exception:
                        projects.append({"folder": d, "has_metadata": False, "error": "Corrupt metadata JSON"})
                else:
                    # Folder without metadata (can be scanned)
                    gbrs = [f for f in os.listdir(full_d) if f.lower().endswith(('.gbr', '.gtl', '.gbl', '.drl', '.zip'))]
                    if gbrs:
                        projects.append({
                            "folder": d,
                            "pcb_id": d,
                            "project_name": d,
                            "has_metadata": False,
                            "candidate_files": len(gbrs)
                        })

        val_report = ConfigValidator.validate_all(configs, self.root_dir)
        backups = self.backup_mgr.list_backups()

        return {
            "root_dir": self.root_dir,
            "configs": configs,
            "projects": projects,
            "validation": val_report,
            "backups": backups,
            "file_paths": FILE_PATHS
        }

    def save_config(self, key: str, data: Dict[str, Any]) -> Tuple[bool, str]:
        if key not in FILE_PATHS:
            return False, f"Unknown config key: {key}"
        
        rel_path = FILE_PATHS[key]
        full_path = os.path.join(self.root_dir, rel_path)

        # Single source of truth enforcement for pcb_holders.json
        if key == "holders":
            data.pop("bed_limits", None)
            data.pop("z_probe_limit_switch", None)

        # Automatic backup before write
        self.backup_mgr.backup_file(rel_path)

        os.makedirs(os.path.dirname(full_path), exist_ok=True)
        try:
            with open(full_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            return True, f"Saved {rel_path} successfully."
        except Exception as e:
            return False, f"Failed saving {rel_path}: {e}"

    def apply_preset(self, preset_id: str, dry_run: bool = False) -> Dict[str, Any]:
        preset = PRESET_BUNDLES.get(preset_id)
        if not preset:
            return {"success": False, "error": f"Unknown preset: {preset_id}"}

        changes: Dict[str, Any] = {}
        for key, pdata in preset["configs"].items():
            rel_path = FILE_PATHS.get(key)
            if not rel_path:
                continue
            
            # Start from default or existing
            full_path = os.path.join(self.root_dir, rel_path)
            curr = {}
            if os.path.isfile(full_path):
                try:
                    with open(full_path, "r", encoding="utf-8") as f:
                        curr = json.load(f)
                except Exception:
                    curr = {}
            else:
                curr = json.loads(json.dumps(FILE_DEFAULTS.get(key, {})))

            # Deep merge updates
            updated = json.loads(json.dumps(curr))
            self._deep_update(updated, pdata)
            changes[key] = {
                "file": rel_path,
                "current": curr,
                "updated": updated
            }

        if dry_run:
            return {"success": True, "dry_run": True, "preset_id": preset_id, "changes": changes}

        # Apply and backup
        applied = []
        for key, info in changes.items():
            self.save_config(key, info["updated"])
            applied.append(info["file"])

        return {
            "success": True,
            "dry_run": False,
            "preset_id": preset_id,
            "applied_files": applied,
            "message": f"Applied preset '{preset['name']}' to {len(applied)} file(s)."
        }

    def reset_file(self, key: str) -> Tuple[bool, str]:
        if key not in FILE_DEFAULTS:
            return False, f"No default template for '{key}'."
        default_data = json.loads(json.dumps(FILE_DEFAULTS[key]))
        return self.save_config(key, default_data)

    def cleanup_duplicate_keys(self) -> Tuple[bool, str]:
        """Strip bed_limits and z_probe_limit_switch from pcb_holders.json."""
        rel_path = FILE_PATHS["holders"]
        full_path = os.path.join(self.root_dir, rel_path)
        if not os.path.isfile(full_path):
            return False, "pcb_holders.json does not exist."
        try:
            with open(full_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            had_bed = "bed_limits" in data
            had_z = "z_probe_limit_switch" in data
            if not had_bed and not had_z:
                return True, "pcb_holders.json is already clean (no duplicate keys)."
            data.pop("bed_limits", None)
            data.pop("z_probe_limit_switch", None)
            self.save_config("holders", data)
            return True, "Successfully cleaned up legacy duplicate keys from pcb_holders.json."
        except Exception as e:
            return False, f"Failed cleaning pcb_holders.json: {e}"

    def init_missing(self) -> List[str]:
        """Initialize any missing configuration file using its default schema."""
        created = []
        for key, rel_path in FILE_PATHS.items():
            full_path = os.path.join(self.root_dir, rel_path)
            if not os.path.isfile(full_path) and key in FILE_DEFAULTS:
                self.save_config(key, json.loads(json.dumps(FILE_DEFAULTS[key])))
                created.append(rel_path)
        return created

    @staticmethod
    def _deep_update(base: Dict[str, Any], updates: Dict[str, Any]) -> None:
        for k, v in updates.items():
            if isinstance(v, dict) and k in base and isinstance(base[k], dict):
                StudioState._deep_update(base[k], v)
            else:
                base[k] = v


# ---------------------------------------------------------------------------
# HTTP API & Web Request Handler
# ---------------------------------------------------------------------------

class StudioRequestHandler(http.server.BaseHTTPRequestHandler):
    """Stdlib HTTP request handler serving REST JSON API and embedded Web UI."""
    server_version = "ConfigStudioHTTP/1.0"

    def log_message(self, format, *args):
        # Keep terminal output clean
        pass

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        path = url.path.rstrip("/")
        if path == "" or path == "/index.html":
            self._serve_html()
            return
        
        state_mgr: StudioState = self.server.state_mgr

        if path == "/api/state":
            state = state_mgr.load_all()
            self._send_json(200, state)
            return

        if path == "/api/presets":
            self._send_json(200, {
                "presets": PRESET_BUNDLES,
                "file_defaults": {k: FILE_PATHS[k] for k in FILE_DEFAULTS}
            })
            return

        if path == "/api/backups":
            self._send_json(200, {"backups": state_mgr.backup_mgr.list_backups()})
            return

        self._send_json(404, {"error": "Not Found"})

    def do_POST(self):
        url = urllib.parse.urlparse(self.path)
        path = url.path.rstrip("/")
        body = self._read_json_body()
        state_mgr: StudioState = self.server.state_mgr

        if path == "/api/save_config":
            key = body.get("key")
            data = body.get("data")
            if not key or data is None:
                self._send_json(400, {"success": False, "error": "Missing key or data"})
                return
            ok, msg = state_mgr.save_config(key, data)
            self._send_json(200 if ok else 500, {"success": ok, "message": msg, "state": state_mgr.load_all()})
            return

        if path == "/api/init_preset":
            preset_id = body.get("preset_id")
            dry_run = bool(body.get("dry_run", False))
            res = state_mgr.apply_preset(preset_id, dry_run=dry_run)
            if not dry_run and res.get("success"):
                res["state"] = state_mgr.load_all()
            self._send_json(200 if res.get("success") else 400, res)
            return

        if path == "/api/reset_file":
            key = body.get("key")
            ok, msg = state_mgr.reset_file(key)
            self._send_json(200 if ok else 500, {"success": ok, "message": msg, "state": state_mgr.load_all()})
            return

        if path == "/api/init_missing":
            created = state_mgr.init_missing()
            self._send_json(200, {"success": True, "created": created, "state": state_mgr.load_all()})
            return

        if path == "/api/cleanup_duplicates":
            ok, msg = state_mgr.cleanup_duplicate_keys()
            self._send_json(200 if ok else 500, {"success": ok, "message": msg, "state": state_mgr.load_all()})
            return

        if path == "/api/validate":
            configs = body.get("configs", {})
            val = ConfigValidator.validate_all(configs, state_mgr.root_dir)
            self._send_json(200, val)
            return

        if path == "/api/projects/scan":
            target = body.get("folder")
            if not target:
                self._send_json(400, {"error": "Target folder or path required"})
                return
            full_target = os.path.join(state_mgr.root_dir, target)
            if not os.path.isdir(full_target):
                self._send_json(404, {"error": f"Folder '{target}' not found."})
                return
            meta = ProjectScanner.scan_folder(full_target)
            self._send_json(200, {"success": True, "metadata": meta, "folder": target})
            return

        if path == "/api/projects/save_metadata":
            target_folder = body.get("folder")
            metadata = body.get("metadata")
            add_to_selected = bool(body.get("add_to_selected", True))
            if not target_folder or not metadata:
                self._send_json(400, {"error": "Target folder and metadata required"})
                return
            full_dir = os.path.join(state_mgr.root_dir, target_folder)
            os.makedirs(full_dir, exist_ok=True)
            meta_path = os.path.join(full_dir, "project_metadata.json")
            
            # Backup if existing
            state_mgr.backup_mgr.backup_file(os.path.join(target_folder, "project_metadata.json"))
            with open(meta_path, "w", encoding="utf-8") as f:
                json.dump(metadata, f, indent=2)

            # Optionally include in manufacture_config.json
            if add_to_selected:
                m_path = os.path.join(state_mgr.root_dir, FILE_PATHS["manufacture"])
                if os.path.isfile(m_path):
                    try:
                        with open(m_path, "r", encoding="utf-8") as mf:
                            mcfg = json.load(mf)
                        sel = mcfg.setdefault("selected_projects", [])
                        if target_folder not in sel:
                            sel.append(target_folder)
                            state_mgr.save_config("manufacture", mcfg)
                    except Exception:
                        pass

            self._send_json(200, {"success": True, "message": f"Saved {target_folder}/project_metadata.json", "state": state_mgr.load_all()})
            return

        if path == "/api/backups/restore":
            bak_name = body.get("filename")
            if not bak_name:
                self._send_json(400, {"error": "Backup filename required"})
                return
            ok, msg = state_mgr.backup_mgr.restore_backup(bak_name)
            self._send_json(200 if ok else 500, {"success": ok, "message": msg, "state": state_mgr.load_all()})
            return

        self._send_json(404, {"error": "Not Found"})

    def _read_json_body(self) -> Dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length > 0:
                raw = self.rfile.read(length).decode("utf-8")
                return json.loads(raw)
        except Exception:
            pass
        return {}

    def _send_json(self, status: int, data: Any):
        payload = json.dumps(data, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(payload)

    def _serve_html(self):
        payload = HTML_TEMPLATE.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

# ---------------------------------------------------------------------------
# Embedded Single-Page Engineering Web UI
# ---------------------------------------------------------------------------

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en" class="dark">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>PCBtoGcode Config Studio</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <script>
    tailwind.config = {
      darkMode: 'class',
      theme: {
        extend: {
          colors: {
            brand: { 500: '#10b981', 600: '#059669', 700: '#047857' }
          }
        }
      }
    }
  </script>
  <style>
    /* Custom scrollbar and engineering grid */
    ::-webkit-scrollbar { width: 6px; height: 6px; }
    ::-webkit-scrollbar-track { background: #0f172a; }
    ::-webkit-scrollbar-thumb { background: #334155; border-radius: 3px; }
    ::-webkit-scrollbar-thumb:hover { background: #475569; }
    .bg-grid {
      background-size: 20px 20px;
      background-image: linear-gradient(to right, rgba(255, 255, 255, 0.03) 1px, transparent 1px),
                        linear-gradient(to bottom, rgba(255, 255, 255, 0.03) 1px, transparent 1px);
    }
    input[type=number]::-webkit-inner-spin-button,
    input[type=number]::-webkit-outer-spin-button { -webkit-appearance: none; margin: 0; }
  </style>
</head>
<body class="bg-slate-950 text-slate-100 min-h-screen flex flex-col font-sans antialiased selection:bg-emerald-500 selection:text-white">

  <!-- Top Navigation & Safety Status Bar -->
  <header class="border-b border-slate-800 bg-slate-900/90 backdrop-blur sticky top-0 z-40 px-6 py-3 flex items-center justify-between">
    <div class="flex items-center space-x-4">
      <div class="flex items-center space-x-2">
        <span class="inline-block w-3 h-3 rounded-full bg-emerald-500 shadow-[0_0_12px_#10b981]"></span>
        <h1 class="text-lg font-bold tracking-tight text-white flex items-center gap-2">
          <span>PCBtoGcode</span>
          <span class="text-xs font-mono uppercase bg-emerald-950/80 text-emerald-400 border border-emerald-800/80 px-2 py-0.5 rounded">Studio</span>
        </h1>
      </div>
      <span class="text-xs font-mono text-slate-400 border-l border-slate-800 pl-4 hidden md:inline truncate max-w-sm" id="lbl-workspace">...</span>
    </div>

    <!-- Health & Global Actions -->
    <div class="flex items-center space-x-3 text-xs">
      <span id="badge-health" class="px-2.5 py-1 rounded font-medium bg-emerald-950 text-emerald-400 border border-emerald-800 flex items-center gap-1.5">
        <span class="w-1.5 h-1.5 rounded-full bg-emerald-400"></span> 0 Issues
      </span>
      <button onclick="toggleBackupsModal(true)" class="px-3 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 font-medium border border-slate-700 transition flex items-center gap-1.5">
        <svg class="w-3.5 h-3.5 text-slate-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z"></path></svg>
        Backups (<span id="lbl-backup-count">0</span>)
      </button>
      <button onclick="saveAllActive()" id="btn-save-active" class="px-3 py-1.5 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-semibold shadow-sm transition flex items-center gap-1.5">
        <svg class="w-3.5 h-3.5" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M5 13l4 4L19 7"></path></svg>
        Save Changes
      </button>
    </div>
  </header>

  <!-- Navigation Tabs -->
  <div class="bg-slate-900 border-b border-slate-800 px-6 flex space-x-1 overflow-x-auto text-sm select-none">
    <button onclick="switchTab('tab-home')" id="nav-tab-home" class="tab-btn px-4 py-2.5 font-medium border-b-2 border-emerald-500 text-emerald-400 transition flex items-center gap-2">
      <svg class="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M3 12l2-2m0 0l7-7 7 7M5 10v10a1 1 0 001 1h3m10-11l2 2m-2-2v10a1 1 0 01-1 1h-3m-6 0a1 1 0 001-1v-4a1 1 0 011-1h2a1 1 0 011 1v4a1 1 0 001 1m-6 0h6"></path></svg>
      Quick-Init Presets
    </button>
    <button onclick="switchTab('tab-manufacture')" id="nav-tab-manufacture" class="tab-btn px-4 py-2.5 font-medium border-b-2 border-transparent text-slate-400 hover:text-slate-200 transition flex items-center gap-2">
      Machine & Sheet (manufacture_config)
    </button>
    <button onclick="switchTab('tab-holders')" id="nav-tab-holders" class="tab-btn px-4 py-2.5 font-medium border-b-2 border-transparent text-slate-400 hover:text-slate-200 transition flex items-center gap-2">
      Fixture Clamps (pcb_holders)
    </button>
    <button onclick="switchTab('tab-projects')" id="nav-tab-projects" class="tab-btn px-4 py-2.5 font-medium border-b-2 border-transparent text-slate-400 hover:text-slate-200 transition flex items-center gap-2">
      Projects & Scanner (<span id="lbl-nav-proj-count">0</span>)
    </button>
    <button onclick="switchTab('tab-tool')" id="nav-tab-tool" class="tab-btn px-4 py-2.5 font-medium border-b-2 border-transparent text-slate-400 hover:text-slate-200 transition flex items-center gap-2">
      CNC Tooling (tool_config)
    </button>
    <button onclick="switchTab('tab-ui')" id="nav-tab-ui" class="tab-btn px-4 py-2.5 font-medium border-b-2 border-transparent text-slate-400 hover:text-slate-200 transition flex items-center gap-2">
      Dashboard & Transport (ui_config)
    </button>
    <button onclick="switchTab('tab-quotation')" id="nav-tab-quotation" class="tab-btn px-4 py-2.5 font-medium border-b-2 border-transparent text-slate-400 hover:text-slate-200 transition flex items-center gap-2">
      Quotation (quotation_config)
    </button>
    <button onclick="switchTab('tab-pocket')" id="nav-tab-pocket" class="tab-btn px-4 py-2.5 font-medium border-b-2 border-transparent text-slate-400 hover:text-slate-200 transition flex items-center gap-2">
      Bed Pocket (pocket_config)
    </button>
  </div>

  <!-- Main Container -->
  <main class="flex-1 p-6 max-w-7xl w-full mx-auto">

    <!-- TAB 1: HOME & QUICK-INIT PRESETS HUB -->
    <div id="tab-home" class="tab-pane space-y-8">
      <div>
        <h2 class="text-xl font-bold text-white mb-1">Quick-Init Configuration Presets</h2>
        <p class="text-sm text-slate-400">Click any preset card to review and initialize standard production parameters with automatic backup.</p>
      </div>

      <!-- Preset Cards Grid -->
      <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-5" id="preset-cards-container">
        <!-- Rendered dynamically via JS -->
      </div>

      <!-- Quick Setup & Maintenance Actions -->
      <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-6 space-y-4">
        <h3 class="text-sm font-semibold uppercase tracking-wider text-slate-400">Quick Workshop Tools</h3>
        <div class="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
          <button onclick="switchTab('tab-projects')" class="p-4 rounded-lg bg-slate-800/60 hover:bg-slate-800 border border-slate-700/80 text-left transition flex flex-col justify-between">
            <span class="font-medium text-white flex items-center gap-2">
              <svg class="w-4 h-4 text-emerald-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 4v16m8-8H4"></path></svg>
              Scan New Project
            </span>
            <span class="text-xs text-slate-400 mt-2">Parse Gerber/ZIP files into project_metadata.json</span>
          </button>
          <button onclick="cleanupDuplicates()" class="p-4 rounded-lg bg-slate-800/60 hover:bg-slate-800 border border-slate-700/80 text-left transition flex flex-col justify-between">
            <span class="font-medium text-white flex items-center gap-2">
              <svg class="w-4 h-4 text-cyan-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16"></path></svg>
              Clean Up Duplicates
            </span>
            <span class="text-xs text-slate-400 mt-2">Remove legacy duplicate keys from pcb_holders.json</span>
          </button>
          <button onclick="initMissingConfigs()" class="p-4 rounded-lg bg-slate-800/60 hover:bg-slate-800 border border-slate-700/80 text-left transition flex flex-col justify-between">
            <span class="font-medium text-white flex items-center gap-2">
              <svg class="w-4 h-4 text-amber-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z"></path></svg>
              Init Missing Files
            </span>
            <span class="text-xs text-slate-400 mt-2">Create any missing JSON config using clean defaults</span>
          </button>
          <button onclick="quickSetTransport('usb')" class="p-4 rounded-lg bg-slate-800/60 hover:bg-slate-800 border border-slate-700/80 text-left transition flex flex-col justify-between">
            <span class="font-medium text-white flex items-center gap-2">
              <svg class="w-4 h-4 text-violet-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M13 10V3L4 14h7v7l9-11h-7z"></path></svg>
              CNC USB Mode
            </span>
            <span class="text-xs text-slate-400 mt-2">Set ui_config.json transport to USB Serial (115200)</span>
          </button>
        </div>
      </div>

      <!-- Live Config Inventory -->
      <div class="space-y-4">
        <h3 class="text-sm font-semibold uppercase tracking-wider text-slate-400">Active Workspace Configuration Files</h3>
        <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4" id="inventory-cards-container">
          <!-- Rendered dynamically -->
        </div>
      </div>
    </div>

    <!-- TAB 2: MACHINE, SHEET & PIPELINE -->
    <div id="tab-manufacture" class="tab-pane hidden space-y-6">
      <div class="flex items-center justify-between">
        <div>
          <h2 class="text-xl font-bold text-white">Machine, Sheet & Pipeline Configuration</h2>
          <p class="text-xs font-mono text-slate-400">manufacture_config.json &bull; Single Source of Truth for Bed Limits & Z-Limit Switch</p>
        </div>
        <button onclick="saveActiveSection('manufacture')" class="px-4 py-1.5 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-medium text-xs transition">Save manufacture_config.json</button>
      </div>

      <div class="grid grid-cols-1 lg:grid-cols-12 gap-6">
        <!-- Form Fields Column -->
        <div class="lg:col-span-7 space-y-6">

          <!-- Projects Multi-Select -->
          <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-3">
            <h3 class="text-sm font-semibold text-emerald-400 uppercase tracking-wider">Selected Projects for Panelization</h3>
            <div id="mfg-selected-projects-list" class="space-y-2 max-h-48 overflow-y-auto pr-2">
              <!-- Dynamically populated -->
            </div>
          </div>

          <!-- Copper Sheet -->
          <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-4">
            <h3 class="text-sm font-semibold text-emerald-400 uppercase tracking-wider">Raw Copper Stock Sheet</h3>
            <div class="grid grid-cols-2 sm:grid-cols-4 gap-4 text-xs">
              <div>
                <label class="block text-slate-400 mb-1">Width (mm)</label>
                <input type="number" step="0.1" id="mfg_sheet_w" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Height (mm)</label>
                <input type="number" step="0.1" id="mfg_sheet_h" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Thickness (mm)</label>
                <input type="number" step="0.05" id="mfg_sheet_thick" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Weight (oz)</label>
                <input type="number" step="0.5" id="mfg_copper_oz" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
            </div>
          </div>

          <!-- CNC Bed Limits & Y-Split -->
          <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-4">
            <h3 class="text-sm font-semibold text-cyan-400 uppercase tracking-wider">CNC Travel Limits & Y-Split Partitioning</h3>
            <div class="grid grid-cols-2 sm:grid-cols-4 gap-4 text-xs">
              <div>
                <label class="block text-slate-400 mb-1">Bed Travel X (mm)</label>
                <input type="number" step="1" id="mfg_bed_x" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Bed Travel Y (mm)</label>
                <input type="number" step="1" id="mfg_bed_y" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Max Milling X (mm)</label>
                <input type="number" step="1" id="mfg_mill_x" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Max Milling Y (mm)</label>
                <input type="number" step="1" id="mfg_mill_y" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
            </div>
            <div class="grid grid-cols-1 sm:grid-cols-3 gap-4 text-xs pt-2 border-t border-slate-800">
              <div class="flex items-center space-x-2 pt-4">
                <input type="checkbox" id="mfg_auto_ysplit" onchange="renderBedPreview()" class="w-4 h-4 rounded text-emerald-600 bg-slate-800 border-slate-700">
                <label for="mfg_auto_ysplit" class="text-white font-medium">Auto Y-Split Sheet</label>
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Y-Split Shift (mm)</label>
                <input type="number" step="1" id="mfg_ysplit_thresh" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Left Margin Guide (mm)</label>
                <input type="number" step="1" id="mfg_left_margin" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
            </div>
          </div>

          <!-- Fixed Bed Z-Probe Limit Switch (Single Source of Truth) -->
          <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-4">
            <div class="flex items-center justify-between">
              <h3 class="text-sm font-semibold text-rose-400 uppercase tracking-wider">Fixed Bed Z-Probe Switch (Danger Zone)</h3>
              <span class="text-[10px] bg-rose-950/80 text-rose-300 border border-rose-800 px-2 py-0.5 rounded font-mono">Collision Avoidance</span>
            </div>
            <div class="grid grid-cols-2 sm:grid-cols-4 gap-4 text-xs">
              <div>
                <label class="block text-slate-400 mb-1">Switch Machine X (mm)</label>
                <input type="number" step="0.001" id="mfg_z_sw_x" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Switch Machine Y (mm)</label>
                <input type="number" step="0.001" id="mfg_z_sw_y" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Height Above Bed (mm)</label>
                <input type="number" step="0.05" id="mfg_z_sw_h" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Safety Margin (mm)</label>
                <input type="number" step="0.5" id="mfg_z_sw_margin" oninput="renderBedPreview()" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
            </div>
          </div>

          <!-- Breakaway & Tag Strip -->
          <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-4">
            <h3 class="text-sm font-semibold text-amber-400 uppercase tracking-wider">Breakaway Snapping & PCB Tag Strip</h3>
            <div class="grid grid-cols-1 sm:grid-cols-3 gap-4 text-xs">
              <div>
                <label class="block text-slate-400 mb-1">Breakaway Mode</label>
                <select id="mfg_breakaway_mode" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-medium">
                  <option value="v_bit_and_milling">V-Bit and Milling (Cleanest)</option>
                  <option value="milling_only">Milling Only</option>
                  <option value="v_bit_only">V-Bit Scoring Only</option>
                  <option value="drilling_and_milling">Drilling and Milling</option>
                  <option value="drilling_only">Drilling Only</option>
                </select>
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Tag Width (mm)</label>
                <input type="number" step="0.5" id="mfg_tag_w" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
              <div>
                <label class="block text-slate-400 mb-1">Tab Count</label>
                <input type="number" step="1" id="mfg_tab_count" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
            </div>
          </div>

        </div>

        <!-- Sticky 2D CNC Bed & Sheet Canvas -->
        <div class="lg:col-span-5 space-y-4">
          <div class="sticky top-24 border border-slate-800 rounded-xl bg-slate-900/80 p-5 space-y-3">
            <div class="flex items-center justify-between">
              <h3 class="text-sm font-bold text-white flex items-center gap-2">
                <span class="w-2 h-2 rounded-full bg-cyan-400"></span> 2D CNC Bed & Fixture Visualizer
              </h3>
              <span class="text-[10px] font-mono text-slate-400">Live Geometry Preview</span>
            </div>
            <div class="w-full aspect-[4/3] bg-slate-950 rounded border border-slate-800 flex items-center justify-center overflow-hidden p-2">
              <svg id="svg-bed-preview" class="w-full h-full" viewBox="-20 -20 320 210">
                <!-- SVG dynamically rendered -->
              </svg>
            </div>
            <div class="grid grid-cols-2 gap-2 text-[11px] font-mono text-slate-400 pt-2 border-t border-slate-800/80">
              <div class="flex items-center gap-2"><span class="w-2.5 h-2.5 rounded-sm bg-slate-700 border border-slate-500"></span> CNC Travel Boundary</div>
              <div class="flex items-center gap-2"><span class="w-2.5 h-2.5 rounded-sm bg-amber-600/60 border border-amber-500"></span> Raw Copper Sheet</div>
              <div class="flex items-center gap-2"><span class="w-2.5 h-2.5 rounded-full bg-blue-500/80"></span> M4 Washer Clamps</div>
              <div class="flex items-center gap-2"><span class="w-2.5 h-2.5 rounded-sm bg-rose-500/80 border border-rose-400"></span> Z-Zero Switch Zone</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- TAB 3: FIXTURE CLAMPS (pcb_holders.json) -->
    <div id="tab-holders" class="tab-pane hidden space-y-6">
      <div class="flex items-center justify-between">
        <div>
          <h2 class="text-xl font-bold text-white">Common CNC Bed Washer Clamps</h2>
          <p class="text-xs font-mono text-slate-400">pcb_holders.json &bull; Common Single-Half Clamps Reused on Shifted Back Half</p>
        </div>
        <div class="flex space-x-2">
          <button onclick="addHolderRow()" class="px-3 py-1.5 rounded bg-slate-800 hover:bg-slate-700 text-white font-medium text-xs border border-slate-700 transition">+ Add Clamp</button>
          <button onclick="saveActiveSection('holders')" class="px-4 py-1.5 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-medium text-xs transition">Save pcb_holders.json</button>
        </div>
      </div>

      <div class="border border-slate-800 rounded-xl bg-slate-900/60 overflow-hidden">
        <div class="overflow-x-auto">
          <table class="w-full text-left text-xs">
            <thead class="bg-slate-800/80 text-slate-400 uppercase font-mono border-b border-slate-800">
              <tr>
                <th class="p-3">Clamp Name & ID</th>
                <th class="p-3">Shape</th>
                <th class="p-3">Bed X (mm)</th>
                <th class="p-3">Bed Y (mm)</th>
                <th class="p-3">Diameter (mm)</th>
                <th class="p-3">Safety Gap (mm)</th>
                <th class="p-3">Both Halves</th>
                <th class="p-3 text-right">Actions</th>
              </tr>
            </thead>
            <tbody id="holders-table-body" class="divide-y divide-slate-800 text-slate-200">
              <!-- Rendered dynamically -->
            </tbody>
          </table>
        </div>
      </div>
    </div>

    <!-- TAB 4: PROJECTS & GERBER SCANNER -->
    <div id="tab-projects" class="tab-pane hidden space-y-6">
      <div class="flex items-center justify-between">
        <div>
          <h2 class="text-xl font-bold text-white">Discovered PCB Projects & Metadata</h2>
          <p class="text-xs font-mono text-slate-400">Per-Project project_metadata.json &bull; Pure-Python Gerber/Excellon/ZIP Parser</p>
        </div>
      </div>

      <!-- Projects Grid -->
      <div class="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-5" id="projects-grid-container">
        <!-- Rendered dynamically -->
      </div>

      <!-- Scan & Add Modal / Box -->
      <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-6 space-y-4">
        <h3 class="text-sm font-semibold uppercase tracking-wider text-emerald-400 flex items-center gap-2">
          <svg class="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M9 5H7a2 2 0 00-2 2v12a2 2 0 002 2h10a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2"></path></svg>
          Scan Folder for Gerber / Drill Layers
        </h3>
        <p class="text-xs text-slate-400">Select an existing folder in the workspace to automatically inspect layer files, compute board bounds, and generate a validated project_metadata.json.</p>
        <div class="flex flex-col sm:flex-row gap-3">
          <input type="text" id="scan_folder_input" placeholder="e.g. project01 or project04" class="flex-1 bg-slate-800 border border-slate-700 rounded p-2 text-xs text-white font-mono">
          <button onclick="scanProjectFolder()" class="px-4 py-2 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-medium text-xs transition">Analyze Folder</button>
        </div>

        <!-- Scan Result Preview -->
        <div id="scan-result-box" class="hidden border border-slate-800 rounded-lg bg-slate-950 p-4 space-y-3">
          <div class="flex items-center justify-between">
            <span class="text-xs font-mono text-emerald-400 font-semibold" id="scan-result-title">Scan Results</span>
            <button onclick="saveScannedMetadata()" class="px-3 py-1 bg-emerald-600 hover:bg-emerald-500 rounded text-xs font-semibold text-white">Save project_metadata.json</button>
          </div>
          <pre id="scan-result-json" class="text-[11px] font-mono text-slate-300 max-h-60 overflow-y-auto bg-slate-900 p-3 rounded"></pre>
        </div>
      </div>
    </div>

    <!-- TAB 5: CNC TOOLING (tool_config.json) -->
    <div id="tab-tool" class="tab-pane hidden space-y-6">
      <div class="flex items-center justify-between">
        <div>
          <h2 class="text-xl font-bold text-white">CNC Tooling, Feeds & Speeds</h2>
          <p class="text-xs font-mono text-slate-400">tool_config.json &bull; Isolation, Peck Drilling, Slot Milling & Perimeter Cutout</p>
        </div>
        <button onclick="saveActiveSection('tool')" class="px-4 py-1.5 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-medium text-xs transition">Save tool_config.json</button>
      </div>

      <div class="grid grid-cols-1 md:grid-cols-2 gap-6" id="tool-config-grid">
        <!-- Rendered dynamically -->
      </div>
    </div>

    <!-- TAB 6: DASHBOARD & TRANSPORT (ui_config.json) -->
    <div id="tab-ui" class="tab-pane hidden space-y-6">
      <div class="flex items-center justify-between">
        <div>
          <h2 class="text-xl font-bold text-white">Dashboard Features & CNC Transport</h2>
          <p class="text-xs font-mono text-slate-400">ui_config.json &bull; FluidNC Network / USB Serial & Dispatch Controls</p>
        </div>
        <button onclick="saveActiveSection('ui')" class="px-4 py-1.5 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-medium text-xs transition">Save ui_config.json</button>
      </div>

      <div class="grid grid-cols-1 md:grid-cols-2 gap-6">
        <!-- CNC Transport Settings -->
        <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-4">
          <h3 class="text-sm font-semibold text-cyan-400 uppercase tracking-wider">Controller Bridge & Transport</h3>
          <div class="space-y-3 text-xs">
            <div>
              <label class="block text-slate-400 mb-1">Active Transport Mode</label>
              <select id="ui_transport" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-medium">
                <option value="auto">Auto (Detect Network or USB)</option>
                <option value="ws">WebSocket Proxy (FluidNC Wi-Fi)</option>
                <option value="usb">USB Serial Direct (MKS DLC32)</option>
                <option value="telnet">Telnet Port 23</option>
              </select>
            </div>
            <div>
              <label class="block text-slate-400 mb-1">Serial Port Device</label>
              <input type="text" id="ui_serial_port" placeholder="auto or /dev/ttyUSB0" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
            </div>
            <div>
              <label class="block text-slate-400 mb-1">Serial Baudrate</label>
              <input type="number" id="ui_baudrate" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
            </div>
          </div>
        </div>

        <!-- Dashboard UI Features -->
        <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-4">
          <h3 class="text-sm font-semibold text-emerald-400 uppercase tracking-wider">Dashboard UI Toggles</h3>
          <div class="space-y-3 text-xs" id="ui-toggles-container">
            <!-- Dynamically populated checkboxes -->
          </div>
        </div>
      </div>
    </div>

    <!-- TAB 7: QUOTATION (quotation_config.json) -->
    <div id="tab-quotation" class="tab-pane hidden space-y-6">
      <div class="flex items-center justify-between">
        <div>
          <h2 class="text-xl font-bold text-white">PCB Instant Quotation Pricing</h2>
          <p class="text-xs font-mono text-slate-400">quotation_config.json &bull; Substrate Area Rates, Chemical Vias & Promo Tiers</p>
        </div>
        <button onclick="saveActiveSection('quotation')" class="px-4 py-1.5 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-medium text-xs transition">Save quotation_config.json</button>
      </div>
      <div id="quotation-form-container" class="space-y-6">
        <!-- Rendered dynamically -->
      </div>
    </div>

    <!-- TAB 8: BED POCKET (Bed_mods/pocket_config.json) -->
    <div id="tab-pocket" class="tab-pane hidden space-y-6">
      <div class="flex items-center justify-between">
        <div>
          <h2 class="text-xl font-bold text-white">Universal CNC Bed Pocket Milling</h2>
          <p class="text-xs font-mono text-slate-400">Bed_mods/pocket_config.json &bull; Bed Cavities, Fixture Pockets & Z-Switch Wells</p>
        </div>
        <button onclick="saveActiveSection('pocket')" class="px-4 py-1.5 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-medium text-xs transition">Save pocket_config.json</button>
      </div>
      <div id="pocket-form-container" class="space-y-6">
        <!-- Rendered dynamically -->
      </div>
    </div>

  </main>

  <!-- Diff & Preset Confirmation Modal -->
  <div id="modal-diff" class="fixed inset-0 bg-black/80 backdrop-blur-sm z-50 hidden flex items-center justify-center p-4">
    <div class="bg-slate-900 border border-slate-700 rounded-xl max-w-2xl w-full max-h-[85vh] flex flex-col shadow-2xl">
      <div class="p-5 border-b border-slate-800 flex items-center justify-between">
        <h3 class="text-sm font-bold text-white flex items-center gap-2">
          <span class="w-2.5 h-2.5 rounded-full bg-emerald-400"></span>
          Preset Configuration Preview
        </h3>
        <button onclick="closeDiffModal()" class="text-slate-400 hover:text-white">&times;</button>
      </div>
      <div class="p-5 overflow-y-auto space-y-4 flex-1 text-xs" id="diff-content">
        <!-- Diff preview content -->
      </div>
      <div class="p-4 border-t border-slate-800 bg-slate-950 flex justify-end space-x-3">
        <button onclick="closeDiffModal()" class="px-4 py-2 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 font-medium text-xs">Cancel</button>
        <button onclick="executeApplyPreset()" id="btn-execute-preset" class="px-4 py-2 rounded bg-emerald-600 hover:bg-emerald-500 text-white font-semibold text-xs shadow">Apply Preset (With Backup)</button>
      </div>
    </div>
  </div>

  <!-- Backups & Restore Modal -->
  <div id="modal-backups" class="fixed inset-0 bg-black/80 backdrop-blur-sm z-50 hidden flex items-center justify-center p-4">
    <div class="bg-slate-900 border border-slate-700 rounded-xl max-w-2xl w-full max-h-[80vh] flex flex-col shadow-2xl">
      <div class="p-5 border-b border-slate-800 flex items-center justify-between">
        <h3 class="text-sm font-bold text-white flex items-center gap-2">
          <svg class="w-4 h-4 text-emerald-400" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z"></path></svg>
          Configuration Snapshots (.config_backups)
        </h3>
        <button onclick="toggleBackupsModal(false)" class="text-slate-400 hover:text-white">&times;</button>
      </div>
      <div class="p-5 overflow-y-auto space-y-2 flex-1 text-xs" id="backups-list-container">
        <!-- Backups list -->
      </div>
    </div>
  </div>

  <!-- Client-side Studio Logic -->
  <script>
    let G_STATE = {};
    let G_PRESETS = {};
    let G_ACTIVE_PRESET_ID = null;
    let G_SCANNED_META = null;

    async function init() {
      await loadState();
      await loadPresets();
    }

    async function loadState() {
      try {
        const res = await fetch('/api/state');
        G_STATE = await res.json();
        document.getElementById('lbl-workspace').textContent = G_STATE.root_dir;
        document.getElementById('lbl-backup-count').textContent = (G_STATE.backups || []).length;
        document.getElementById('lbl-nav-proj-count').textContent = (G_STATE.projects || []).length;
        renderHealthBadge(G_STATE.validation);
        renderHomeInventory();
        renderManufactureForm();
        renderHoldersTable();
        renderProjectsGrid();
        renderToolForm();
        renderUiForm();
        renderQuotationForm();
        renderPocketForm();
        renderBedPreview();
      } catch (err) {
        console.error("Failed loading state:", err);
      }
    }

    async function loadPresets() {
      try {
        const res = await fetch('/api/presets');
        const data = await res.json();
        G_PRESETS = data.presets || {};
        renderPresetCards();
      } catch (err) {
        console.error("Failed loading presets:", err);
      }
    }

    function switchTab(tabId) {
      document.querySelectorAll('.tab-pane').forEach(el => el.classList.add('hidden'));
      document.querySelectorAll('.tab-btn').forEach(el => {
        el.classList.remove('border-emerald-500', 'text-emerald-400');
        el.classList.add('border-transparent', 'text-slate-400');
      });
      document.getElementById(tabId).classList.remove('hidden');
      const activeNav = document.getElementById('nav-' + tabId);
      if (activeNav) {
        activeNav.classList.add('border-emerald-500', 'text-emerald-400');
        activeNav.classList.remove('border-transparent', 'text-slate-400');
      }
    }

    function renderHealthBadge(val) {
      const el = document.getElementById('badge-health');
      if (!val) return;
      if (val.valid && (!val.warnings || val.warnings.length === 0)) {
        el.className = "px-2.5 py-1 rounded font-medium bg-emerald-950 text-emerald-400 border border-emerald-800 flex items-center gap-1.5";
        el.innerHTML = '<span class="w-1.5 h-1.5 rounded-full bg-emerald-400"></span> 0 Issues (Clean)';
      } else if (!val.valid) {
        el.className = "px-2.5 py-1 rounded font-medium bg-rose-950 text-rose-300 border border-rose-800 flex items-center gap-1.5 cursor-pointer";
        el.innerHTML = `<span class="w-1.5 h-1.5 rounded-full bg-rose-400 animate-pulse"></span> ${val.errors.length} Error(s)`;
        el.title = val.errors.join('\\n');
      } else {
        el.className = "px-2.5 py-1 rounded font-medium bg-amber-950 text-amber-300 border border-amber-800 flex items-center gap-1.5 cursor-pointer";
        el.innerHTML = `<span class="w-1.5 h-1.5 rounded-full bg-amber-400"></span> ${val.warnings.length} Warning(s)`;
        el.title = val.warnings.join('\\n');
      }
    }

    function renderPresetCards() {
      const c = document.getElementById('preset-cards-container');
      c.innerHTML = '';
      for (const [pid, p] of Object.entries(G_PRESETS)) {
        const div = document.createElement('div');
        div.className = "border border-slate-800 rounded-xl bg-slate-900/60 p-5 flex flex-col justify-between hover:border-slate-700 transition space-y-3";
        div.innerHTML = `
          <div>
            <div class="flex items-center justify-between mb-2">
              <span class="text-[10px] font-mono uppercase px-2 py-0.5 rounded bg-emerald-950 text-emerald-400 border border-emerald-800">${p.badge}</span>
              <span class="text-xs font-mono text-slate-500">${Object.keys(p.configs).length} files</span>
            </div>
            <h4 class="font-bold text-white text-sm mb-1">${p.name}</h4>
            <p class="text-xs text-slate-400 leading-relaxed">${p.description}</p>
          </div>
          <button onclick="previewPreset('${pid}')" class="w-full mt-3 py-2 px-3 rounded bg-slate-800 hover:bg-emerald-600 hover:text-white text-slate-200 text-xs font-medium border border-slate-700 transition flex items-center justify-center gap-2">
            <span>Apply Preset</span>
            <svg class="w-3.5 h-3.5" fill="none" stroke="currentColor" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M14 5l7 7m0 0l-7 7m7-7H3"></path></svg>
          </button>
        `;
        c.appendChild(div);
      }
    }

    function renderHomeInventory() {
      const c = document.getElementById('inventory-cards-container');
      c.innerHTML = '';
      const targets = [
        { key: 'manufacture', title: 'manufacture_config.json', tab: 'tab-manufacture', desc: 'CNC limits, sheet dims, Y-split & Z-probe switch' },
        { key: 'holders', title: 'pcb_holders.json', tab: 'tab-holders', desc: 'Bed washer clamps & collision keep-out zones' },
        { key: 'tool', title: 'tool_config.json', tab: 'tab-tool', desc: 'Speeds, feeds, cut depths & safe retracts' },
        { key: 'ui', title: 'ui_config.json', tab: 'tab-ui', desc: 'Hardware transport & dashboard toggle switches' },
        { key: 'quotation', title: 'quotation_config.json', tab: 'tab-quotation', desc: 'Itemized fabrication pricing & promos' },
        { key: 'pocket', title: 'Bed_mods/pocket_config.json', tab: 'tab-pocket', desc: 'CNC bed alignment pockets & fixtures' }
      ];

      targets.forEach(t => {
        const exists = G_STATE.configs && G_STATE.configs[t.key] && !G_STATE.configs[t.key]._error;
        const div = document.createElement('div');
        div.className = "border border-slate-800/80 rounded-xl bg-slate-900/40 p-4 flex flex-col justify-between space-y-3";
        div.innerHTML = `
          <div>
            <div class="flex items-center justify-between mb-1.5">
              <span class="text-xs font-mono font-semibold text-white truncate max-w-[200px]">${t.title}</span>
              <span class="text-[10px] px-2 py-0.5 rounded font-mono ${exists ? 'bg-emerald-950 text-emerald-400 border border-emerald-800' : 'bg-rose-950 text-rose-400 border border-rose-800'}">
                ${exists ? 'Ready' : 'Missing'}
              </span>
            </div>
            <p class="text-xs text-slate-400">${t.desc}</p>
          </div>
          <div class="flex items-center justify-end space-x-2 pt-2 border-t border-slate-800/60">
            <button onclick="resetSingleFile('${t.key}')" class="text-[11px] text-slate-400 hover:text-rose-400 font-mono transition">Reset</button>
            <button onclick="switchTab('${t.tab}')" class="text-[11px] px-2.5 py-1 bg-slate-800 hover:bg-slate-700 text-slate-200 rounded font-medium transition">Edit &rarr;</button>
          </div>
        `;
        c.appendChild(div);
      });
    }

    async function previewPreset(pid) {
      G_ACTIVE_PRESET_ID = pid;
      const res = await fetch('/api/init_preset', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ preset_id: pid, dry_run: true })
      });
      const data = await res.json();
      if (!data.success) {
        alert("Error previewing preset: " + data.error);
        return;
      }
      const c = document.getElementById('diff-content');
      c.innerHTML = `<p class="text-slate-300 mb-3">The following target configuration files will be safely updated (with timestamped automatic backups in <code>.config_backups/</code>):</p>`;
      for (const [k, ch] of Object.entries(data.changes)) {
        const block = document.createElement('div');
        block.className = "border border-slate-800 bg-slate-950 rounded-lg p-3 space-y-1";
        block.innerHTML = `
          <div class="font-mono font-semibold text-emerald-400">${ch.file}</div>
          <pre class="text-[10px] text-slate-300 max-h-40 overflow-y-auto bg-slate-900 p-2 rounded mt-1">${JSON.stringify(ch.updated, null, 2)}</pre>
        `;
        c.appendChild(block);
      }
      document.getElementById('modal-diff').classList.remove('hidden');
    }

    async function executeApplyPreset() {
      if (!G_ACTIVE_PRESET_ID) return;
      const res = await fetch('/api/init_preset', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ preset_id: G_ACTIVE_PRESET_ID, dry_run: false })
      });
      const data = await res.json();
      closeDiffModal();
      if (data.success) {
        await loadState();
        alert(data.message);
      } else {
        alert("Error applying preset: " + data.error);
      }
    }

    function closeDiffModal() {
      document.getElementById('modal-diff').classList.add('hidden');
      G_ACTIVE_PRESET_ID = null;
    }

    async function resetSingleFile(key) {
      if (!confirm(`Reset ${key} to factory standard defaults? A safety backup will be created.`)) return;
      const res = await fetch('/api/reset_file', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ key: key })
      });
      const data = await res.json();
      if (data.success) {
        await loadState();
        alert(data.message);
      } else {
        alert("Reset failed: " + data.error);
      }
    }

    async function cleanupDuplicates() {
      const res = await fetch('/api/cleanup_duplicates', { method: 'POST' });
      const data = await res.json();
      await loadState();
      alert(data.message);
    }

    async function initMissingConfigs() {
      const res = await fetch('/api/init_missing', { method: 'POST' });
      const data = await res.json();
      await loadState();
      if (data.created.length > 0) {
        alert("Initialized missing files:\\n" + data.created.join("\\n"));
      } else {
        alert("All configuration files already exist.");
      }
    }

    async function quickSetTransport(mode) {
      const ucfg = G_STATE.configs && G_STATE.configs.ui ? G_STATE.configs.ui : {};
      ucfg.cnc_transport = mode;
      if (mode === 'usb') {
        ucfg.cnc_serial_port = 'auto';
        ucfg.cnc_baudrate = 115200;
      }
      const res = await fetch('/api/save_config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ key: 'ui', data: ucfg })
      });
      await loadState();
      alert(`CNC Transport set to: ${mode}`);
    }

    // Manufacture Form Binding
    function renderManufactureForm() {
      const m = G_STATE.configs && G_STATE.configs.manufacture ? G_STATE.configs.manufacture : {};
      const s = m.copper_sheet || {};
      document.getElementById('mfg_sheet_w').value = s.sheet_width_mm || 206.0;
      document.getElementById('mfg_sheet_h').value = s.sheet_height_mm || 300.0;
      document.getElementById('mfg_sheet_thick').value = s.thickness_mm || 1.6;
      document.getElementById('mfg_copper_oz').value = s.copper_weight_oz || 1.0;

      const l = m.cnc_limits || {};
      document.getElementById('mfg_bed_x').value = l.bed_travel_x_mm || 275.0;
      document.getElementById('mfg_bed_y').value = l.bed_travel_y_mm || 170.0;
      document.getElementById('mfg_mill_x').value = l.bed_milling_x_mm || 260.0;
      document.getElementById('mfg_mill_y').value = l.bed_milling_y_mm || 165.0;
      document.getElementById('mfg_auto_ysplit').checked = l.auto_y_split !== false;
      document.getElementById('mfg_ysplit_thresh').value = l.y_split_threshold_mm || 165.0;
      document.getElementById('mfg_left_margin').value = l.left_margin_guide_width_mm || 18.0;

      const zw = (m.guides_and_fixturing || {}).z_probe_limit_switch || {};
      document.getElementById('mfg_z_sw_x').value = zw.x_mm || 272.001;
      document.getElementById('mfg_z_sw_y').value = zw.y_mm || 135.002;
      document.getElementById('mfg_z_sw_h').value = zw.height_above_bed_mm || 5.5;
      document.getElementById('mfg_z_sw_margin').value = zw.danger_margin_mm || 6.0;

      const tag = m.pcb_tag_strip || {};
      document.getElementById('mfg_breakaway_mode').value = tag.breakaway_mode || "v_bit_and_milling";
      document.getElementById('mfg_tag_w').value = tag.tag_width_mm || 5.0;
      document.getElementById('mfg_tab_count').value = tag.retention_tab_count || 2;

      // Project checkboxes
      const selList = document.getElementById('mfg-selected-projects-list');
      selList.innerHTML = '';
      const selected = m.selected_projects || [];
      const projs = G_STATE.projects || [];
      projs.forEach(p => {
        const isSel = selected.includes(p.folder);
        const row = document.createElement('label');
        row.className = "flex items-center space-x-3 text-xs text-slate-300 p-2 bg-slate-800/40 rounded hover:bg-slate-800 cursor-pointer";
        row.innerHTML = `
          <input type="checkbox" value="${p.folder}" ${isSel ? 'checked' : ''} class="mfg-proj-chk w-4 h-4 rounded text-emerald-600 bg-slate-900 border-slate-700">
          <span class="font-mono font-semibold text-white">${p.folder}</span>
          <span class="text-slate-500">(${p.project_name || p.pcb_id})</span>
        `;
        selList.appendChild(row);
      });
    }

    function collectManufactureData() {
      const cur = (G_STATE.configs && G_STATE.configs.manufacture) ? JSON.parse(JSON.stringify(G_STATE.configs.manufacture)) : {};
      cur.copper_sheet = cur.copper_sheet || {};
      cur.copper_sheet.sheet_width_mm = parseFloat(document.getElementById('mfg_sheet_w').value);
      cur.copper_sheet.sheet_height_mm = parseFloat(document.getElementById('mfg_sheet_h').value);
      cur.copper_sheet.thickness_mm = parseFloat(document.getElementById('mfg_sheet_thick').value);
      cur.copper_sheet.copper_weight_oz = parseFloat(document.getElementById('mfg_copper_oz').value);

      cur.cnc_limits = cur.cnc_limits || {};
      cur.cnc_limits.bed_travel_x_mm = parseFloat(document.getElementById('mfg_bed_x').value);
      cur.cnc_limits.bed_travel_y_mm = parseFloat(document.getElementById('mfg_bed_y').value);
      cur.cnc_limits.bed_milling_x_mm = parseFloat(document.getElementById('mfg_mill_x').value);
      cur.cnc_limits.bed_milling_y_mm = parseFloat(document.getElementById('mfg_mill_y').value);
      cur.cnc_limits.auto_y_split = document.getElementById('mfg_auto_ysplit').checked;
      cur.cnc_limits.y_split_threshold_mm = parseFloat(document.getElementById('mfg_ysplit_thresh').value);
      cur.cnc_limits.left_margin_guide_width_mm = parseFloat(document.getElementById('mfg_left_margin').value);

      cur.guides_and_fixturing = cur.guides_and_fixturing || {};
      cur.guides_and_fixturing.z_probe_limit_switch = cur.guides_and_fixturing.z_probe_limit_switch || {};
      cur.guides_and_fixturing.z_probe_limit_switch.x_mm = parseFloat(document.getElementById('mfg_z_sw_x').value);
      cur.guides_and_fixturing.z_probe_limit_switch.y_mm = parseFloat(document.getElementById('mfg_z_sw_y').value);
      cur.guides_and_fixturing.z_probe_limit_switch.height_above_bed_mm = parseFloat(document.getElementById('mfg_z_sw_h').value);
      cur.guides_and_fixturing.z_probe_limit_switch.danger_margin_mm = parseFloat(document.getElementById('mfg_z_sw_margin').value);

      cur.pcb_tag_strip = cur.pcb_tag_strip || {};
      cur.pcb_tag_strip.breakaway_mode = document.getElementById('mfg_breakaway_mode').value;
      cur.pcb_tag_strip.tag_width_mm = parseFloat(document.getElementById('mfg_tag_w').value);
      cur.pcb_tag_strip.retention_tab_count = parseInt(document.getElementById('mfg_tab_count').value);

      const chks = document.querySelectorAll('.mfg-proj-chk');
      const sel = [];
      chks.forEach(ck => { if (ck.checked) sel.push(ck.value); });
      cur.selected_projects = sel;
      return cur;
    }

    // Holders Table Binding
    function renderHoldersTable() {
      const hcfg = (G_STATE.configs && G_STATE.configs.holders) || {};
      const tbody = document.getElementById('holders-table-body');
      tbody.innerHTML = '';
      const holders = hcfg.holders || [];
      holders.forEach((h, idx) => {
        const tr = document.createElement('tr');
        tr.innerHTML = `
          <td class="p-3 font-mono font-medium text-white">${h.name || h.id}</td>
          <td class="p-3 font-mono text-slate-400">${h.shape}</td>
          <td class="p-3 font-mono">${h.x}</td>
          <td class="p-3 font-mono">${h.y}</td>
          <td class="p-3 font-mono">${h.diameter || h.width || 30.0}</td>
          <td class="p-3 font-mono">${h.holder_gap || 3.0}</td>
          <td class="p-3">${h.common_for_both_halves !== false ? '<span class="text-emerald-400">&check;</span>' : '<span class="text-slate-500">&times;</span>'}</td>
          <td class="p-3 text-right">
            <button onclick="deleteHolderRow(${idx})" class="text-rose-400 hover:text-rose-300 font-medium">Delete</button>
          </td>
        `;
        tbody.appendChild(tr);
      });
    }

    function addHolderRow() {
      const name = prompt("Clamp Name:", "Custom M4 Washer Clamp");
      if (!name) return;
      const x = parseFloat(prompt("Bed X Coordinate (mm):", "0.0") || "0");
      const y = parseFloat(prompt("Bed Y Coordinate (mm):", "0.0") || "0");
      const dia = parseFloat(prompt("Washer Diameter (mm):", "30.0") || "30");
      const hcfg = (G_STATE.configs && G_STATE.configs.holders) || { holders: [] };
      hcfg.holders = hcfg.holders || [];
      hcfg.holders.push({
        id: "clamp_" + Date.now(),
        name: name,
        shape: "circle",
        x: x,
        y: y,
        diameter: dia,
        holder_gap: 3.0,
        common_for_both_halves: true,
        anchor_type: "washer_clamp"
      });
      renderHoldersTable();
      renderBedPreview();
    }

    function deleteHolderRow(idx) {
      const hcfg = (G_STATE.configs && G_STATE.configs.holders) || { holders: [] };
      if (hcfg.holders && hcfg.holders[idx]) {
        hcfg.holders.splice(idx, 1);
        renderHoldersTable();
        renderBedPreview();
      }
    }

    // Projects Grid Binding
    function renderProjectsGrid() {
      const c = document.getElementById('projects-grid-container');
      c.innerHTML = '';
      const projs = G_STATE.projects || [];
      projs.forEach(p => {
        const div = document.createElement('div');
        div.className = "border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-3";
        const dims = p.dimensions || {};
        div.innerHTML = `
          <div class="flex items-center justify-between">
            <span class="text-xs font-mono font-bold text-white">${p.folder}</span>
            <span class="text-[10px] font-mono px-2 py-0.5 rounded ${p.has_metadata ? 'bg-emerald-950 text-emerald-400 border border-emerald-800' : 'bg-amber-950 text-amber-400 border border-amber-800'}">
              ${p.has_metadata ? p.ecad : 'Needs Scan'}
            </span>
          </div>
          <p class="text-xs text-slate-300 font-medium">${p.project_name || p.pcb_id}</p>
          <div class="text-[11px] font-mono text-slate-400 space-y-1">
            <div>Dimensions: ${dims.width ? dims.width + ' x ' + dims.height + ' mm' : 'N/A'}</div>
            <div>Solder Mask: <span class="capitalize text-slate-200">${p.solder_mask || 'N/A'}</span> &bull; Qty: ${p.quantity || 1}</div>
          </div>
          <button onclick="document.getElementById('scan_folder_input').value='${p.folder}'; scanProjectFolder();" class="w-full mt-2 py-1.5 px-3 rounded bg-slate-800 hover:bg-slate-700 text-slate-200 text-xs font-medium border border-slate-700 transition">
            ${p.has_metadata ? 'Re-Scan & Review' : 'Scan Layers Now'}
          </button>
        `;
        c.appendChild(div);
      });
    }

    async function scanProjectFolder() {
      const folder = document.getElementById('scan_folder_input').value.trim();
      if (!folder) {
        alert("Please enter a folder name.");
        return;
      }
      const res = await fetch('/api/projects/scan', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ folder: folder })
      });
      const data = await res.json();
      if (!data.success) {
        alert("Scan failed: " + data.error);
        return;
      }
      G_SCANNED_META = data.metadata;
      document.getElementById('scan-result-title').textContent = `Scanned Metadata for ${data.folder}`;
      document.getElementById('scan-result-json').textContent = JSON.stringify(data.metadata, null, 2);
      document.getElementById('scan-result-box').classList.remove('hidden');
    }

    async function saveScannedMetadata() {
      const folder = document.getElementById('scan_folder_input').value.trim();
      if (!folder || !G_SCANNED_META) return;
      const res = await fetch('/api/projects/save_metadata', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ folder: folder, metadata: G_SCANNED_META, add_to_selected: true })
      });
      const data = await res.json();
      if (data.success) {
        await loadState();
        document.getElementById('scan-result-box').classList.add('hidden');
        alert(data.message);
      } else {
        alert("Error saving metadata: " + data.error);
      }
    }

    // Tool Form Binding
    function renderToolForm() {
      const t = (G_STATE.configs && G_STATE.configs.tool) || {};
      const c = document.getElementById('tool-config-grid');
      c.innerHTML = '';

      const sections = [
        { key: 'isolation', title: 'Copper Isolation (V-Bit)', fields: ['tool_dia', 'cut_depth', 'feed_rate', 'plunge_rate', 'spindle_speed', 'passes', 'stepover'] },
        { key: 'drilling', title: 'Carbide Peck Drilling', fields: ['drill_depth', 'drill_feed', 'safe_z', 'tool_change_z', 'spindle_speed'] },
        { key: 'slot_milling', title: 'Endmill Slot Milling', fields: ['tool_dia', 'cut_depth', 'feed_rate', 'plunge_rate', 'passes', 'spindle_speed'] },
        { key: 'cutout', title: 'Breakaway Perimeter Cutout', fields: ['tool_dia', 'cutout_depth', 'cutout_passes', 'cutout_feed', 'spindle_speed'] }
      ];

      sections.forEach(s => {
        const secData = t[s.key] || {};
        const div = document.createElement('div');
        div.className = "border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-3";
        div.innerHTML = `<h3 class="text-sm font-semibold text-emerald-400 uppercase tracking-wider">${s.title}</h3>`;
        const innerGrid = document.createElement('div');
        innerGrid.className = "grid grid-cols-2 gap-3 text-xs";
        s.fields.forEach(f => {
          innerGrid.innerHTML += `
            <div>
              <label class="block text-slate-400 mb-1 font-mono">${f}</label>
              <input type="number" step="any" id="tool_${s.key}_${f}" value="${secData[f] !== undefined ? secData[f] : ''}" class="w-full bg-slate-800 border border-slate-700 rounded p-1.5 text-white font-mono">
            </div>
          `;
        });
        div.appendChild(innerGrid);
        c.appendChild(div);
      });
    }

    function collectToolData() {
      const cur = (G_STATE.configs && G_STATE.configs.tool) ? JSON.parse(JSON.stringify(G_STATE.configs.tool)) : {};
      const sections = ['isolation', 'drilling', 'slot_milling', 'cutout'];
      sections.forEach(s => {
        cur[s] = cur[s] || {};
        for (const [k, v] of Object.entries(cur[s])) {
          const el = document.getElementById(`tool_${s}_${k}`);
          if (el) cur[s][k] = parseFloat(el.value);
        }
      });
      return cur;
    }

    // UI & Transport Form Binding
    function renderUiForm() {
      const u = (G_STATE.configs && G_STATE.configs.ui) || {};
      document.getElementById('ui_transport').value = u.cnc_transport || "auto";
      document.getElementById('ui_serial_port').value = u.cnc_serial_port || "auto";
      document.getElementById('ui_baudrate').value = u.cnc_baudrate || 115200;

      const tc = document.getElementById('ui-toggles-container');
      tc.innerHTML = '';
      const toggles = [
        { k: 'touch_probe_enabled', l: 'Touch Probe Calibration' },
        { k: 'show_macros_in_dispatch', l: 'Show Quick Macros in Dispatch Bar' },
        { k: 'enable_homing_btn', l: 'Show Homing ($H) Action Button' },
        { k: 'enable_unlock_btn', l: 'Show Unlock ($X) Action Button' },
        { k: 'enable_zero_work_btn', l: 'Show Zero Work Coordinates Button' },
        { k: 'enable_pcb_z_offset', l: 'Allow Software PCB Z-Offset' },
        { k: 'enable_heightmap_leveling', l: 'Enable Bed Heightmap Probing & Leveling' }
      ];
      toggles.forEach(item => {
        const checked = u[item.k] !== false;
        tc.innerHTML += `
          <label class="flex items-center space-x-3 text-slate-300 p-2 bg-slate-800/40 rounded hover:bg-slate-800 cursor-pointer">
            <input type="checkbox" id="ui_${item.k}" ${checked ? 'checked' : ''} class="w-4 h-4 rounded text-emerald-600 bg-slate-900 border-slate-700">
            <span class="text-xs text-white">${item.l}</span>
          </label>
        `;
      });
    }

    function collectUiData() {
      const cur = (G_STATE.configs && G_STATE.configs.ui) ? JSON.parse(JSON.stringify(G_STATE.configs.ui)) : {};
      cur.cnc_transport = document.getElementById('ui_transport').value;
      cur.cnc_serial_port = document.getElementById('ui_serial_port').value;
      cur.cnc_baudrate = parseInt(document.getElementById('ui_baudrate').value);
      const toggles = [
        'touch_probe_enabled', 'show_macros_in_dispatch', 'enable_homing_btn',
        'enable_unlock_btn', 'enable_zero_work_btn', 'enable_pcb_z_offset', 'enable_heightmap_leveling'
      ];
      toggles.forEach(k => {
        const el = document.getElementById(`ui_${k}`);
        if (el) cur[k] = el.checked;
      });
      return cur;
    }

    // Quotation Form Binding
    function renderQuotationForm() {
      const q = (G_STATE.configs && G_STATE.configs.quotation) || {};
      const c = document.getElementById('quotation-form-container');
      c.innerHTML = `
        <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-4">
          <h3 class="text-sm font-semibold text-emerald-400 uppercase tracking-wider">Currency & Promotional Promo</h3>
          <div class="grid grid-cols-2 sm:grid-cols-4 gap-4 text-xs">
            <div>
              <label class="block text-slate-400 mb-1">Currency Code</label>
              <input type="text" id="qc_curr_code" value="${(q.currency || {}).code || 'LKR'}" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
            </div>
            <div>
              <label class="block text-slate-400 mb-1">Currency Symbol</label>
              <input type="text" id="qc_curr_sym" value="${(q.currency || {}).symbol || 'Rs. '}" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
            </div>
            <div>
              <label class="block text-slate-400 mb-1">Base Setup Fee</label>
              <input type="number" id="qc_setup_fee" value="${(q.fabrication || {}).base_setup_fee || 1200}" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
            </div>
            <div>
              <label class="block text-slate-400 mb-1">Promo Flat Price</label>
              <input type="number" id="qc_promo_flat" value="${((q.fabrication || {}).standard_promo || {}).flat_price || 2800}" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
            </div>
          </div>
        </div>
      `;
    }

    function collectQuotationData() {
      const cur = (G_STATE.configs && G_STATE.configs.quotation) ? JSON.parse(JSON.stringify(G_STATE.configs.quotation)) : {};
      cur.currency = cur.currency || {};
      cur.currency.code = document.getElementById('qc_curr_code').value;
      cur.currency.symbol = document.getElementById('qc_curr_sym').value;
      cur.fabrication = cur.fabrication || {};
      cur.fabrication.base_setup_fee = parseFloat(document.getElementById('qc_setup_fee').value);
      cur.fabrication.standard_promo = cur.fabrication.standard_promo || {};
      cur.fabrication.standard_promo.flat_price = parseFloat(document.getElementById('qc_promo_flat').value);
      return cur;
    }

    // Bed Pocket Form Binding
    function renderPocketForm() {
      const p = (G_STATE.configs && G_STATE.configs.pocket) || {};
      const sh = p.shape || {};
      const pos = p.position || {};
      const dp = p.depth || {};
      const c = document.getElementById('pocket-form-container');
      c.innerHTML = `
        <div class="border border-slate-800 rounded-xl bg-slate-900/60 p-5 space-y-4">
          <h3 class="text-sm font-semibold text-emerald-400 uppercase tracking-wider">Pocket Milling Dimensions</h3>
          <div class="grid grid-cols-2 sm:grid-cols-4 gap-4 text-xs">
            <div>
              <label class="block text-slate-400 mb-1">Width (mm)</label>
              <input type="number" step="0.1" id="pk_w" value="${sh.width_mm || 5.9}" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
            </div>
            <div>
              <label class="block text-slate-400 mb-1">Length (mm)</label>
              <input type="number" step="0.1" id="pk_l" value="${sh.length_mm || 19.5}" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
            </div>
            <div>
              <label class="block text-slate-400 mb-1">Depth (mm)</label>
              <input type="number" step="0.1" id="pk_d" value="${dp.total_depth_mm || 10.0}" class="w-full bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
            </div>
            <div>
              <label class="block text-slate-400 mb-1">Center X / Y (mm)</label>
              <div class="flex gap-2">
                <input type="number" id="pk_cx" value="${pos.custom_center_x_mm || 137.5}" class="w-1/2 bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
                <input type="number" id="pk_cy" value="${pos.custom_center_y_mm || 85.0}" class="w-1/2 bg-slate-800 border border-slate-700 rounded p-2 text-white font-mono">
              </div>
            </div>
          </div>
        </div>
      `;
    }

    function collectPocketData() {
      const cur = (G_STATE.configs && G_STATE.configs.pocket) ? JSON.parse(JSON.stringify(G_STATE.configs.pocket)) : {};
      cur.shape = cur.shape || {};
      cur.shape.width_mm = parseFloat(document.getElementById('pk_w').value);
      cur.shape.length_mm = parseFloat(document.getElementById('pk_l').value);
      cur.depth = cur.depth || {};
      cur.depth.total_depth_mm = parseFloat(document.getElementById('pk_d').value);
      cur.position = cur.position || {};
      cur.position.custom_center_x_mm = parseFloat(document.getElementById('pk_cx').value);
      cur.position.custom_center_y_mm = parseFloat(document.getElementById('pk_cy').value);
      return cur;
    }

    // Save Active Section
    async function saveActiveSection(secKey) {
      let data = null;
      if (secKey === 'manufacture') data = collectManufactureData();
      else if (secKey === 'holders') data = (G_STATE.configs && G_STATE.configs.holders) || {};
      else if (secKey === 'tool') data = collectToolData();
      else if (secKey === 'ui') data = collectUiData();
      else if (secKey === 'quotation') data = collectQuotationData();
      else if (secKey === 'pocket') data = collectPocketData();

      if (!data) return;
      const res = await fetch('/api/save_config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ key: secKey, data: data })
      });
      const resData = await res.json();
      if (resData.success) {
        await loadState();
        alert(resData.message);
      } else {
        alert("Error saving: " + resData.error);
      }
    }

    async function saveAllActive() {
      await saveActiveSection('manufacture');
    }

    // Interactive 2D CNC Bed & Sheet Canvas
    function renderBedPreview() {
      const svg = document.getElementById('svg-bed-preview');
      if (!svg) return;

      const tx = parseFloat(document.getElementById('mfg_bed_x').value) || 275.0;
      const ty = parseFloat(document.getElementById('mfg_bed_y').value) || 170.0;
      const sw = parseFloat(document.getElementById('mfg_sheet_w').value) || 206.0;
      const sh = parseFloat(document.getElementById('mfg_sheet_h').value) || 300.0;
      const ysplit = parseFloat(document.getElementById('mfg_ysplit_thresh').value) || 165.0;
      const autoY = document.getElementById('mfg_auto_ysplit').checked;
      const leftMargin = parseFloat(document.getElementById('mfg_left_margin').value) || 18.0;

      const zx = parseFloat(document.getElementById('mfg_z_sw_x').value) || 272.001;
      const zy = parseFloat(document.getElementById('mfg_z_sw_y').value) || 135.002;
      const zmargin = parseFloat(document.getElementById('mfg_z_sw_margin').value) || 6.0;

      svg.setAttribute('viewBox', `-15 -15 ${tx + 30} ${ty + 30}`);
      let html = '';

      // Machine Bed boundary
      html += `<rect x="0" y="0" width="${tx}" height="${ty}" fill="#0f172a" stroke="#334155" stroke-width="1.5" />`;

      // Grid lines
      for (let x = 50; x < tx; x += 50) {
        html += `<line x1="${x}" y1="0" x2="${x}" y2="${ty}" stroke="#1e293b" stroke-width="0.8" stroke-dasharray="2 2" />`;
      }
      for (let y = 50; y < ty; y += 50) {
        html += `<line x1="0" y1="${y}" x2="${tx}" y2="${y}" stroke="#1e293b" stroke-width="0.8" stroke-dasharray="2 2" />`;
      }

      // Copper Sheet (Front Half)
      const frontH = autoY ? ysplit : Math.min(sh, ty);
      html += `<rect x="0" y="0" width="${sw}" height="${frontH}" fill="rgba(217, 119, 6, 0.15)" stroke="#d97706" stroke-width="1" />`;
      html += `<line x1="${leftMargin}" y1="0" x2="${leftMargin}" y2="${frontH}" stroke="#f59e0b" stroke-width="0.8" stroke-dasharray="3 3" />`;

      // Y-Split line
      if (autoY) {
        html += `<line x1="0" y1="${ysplit}" x2="${tx}" y2="${ysplit}" stroke="#06b6d4" stroke-width="1.2" stroke-dasharray="4 2" />`;
        html += `<text x="${tx - 8}" y="${ysplit - 3}" fill="#06b6d4" font-size="6" font-family="monospace" text-anchor="end">Y-SPLIT: ${ysplit}mm</text>`;
      }

      // Z-Probe Switch Danger Zone
      const zw = 7.0 / 2.0 + zmargin;
      const zl = 20.0 / 2.0 + zmargin;
      html += `<rect x="${zx - zw}" y="${zy - zl}" width="${zw * 2}" height="${zl * 2}" fill="rgba(244, 63, 94, 0.25)" stroke="#f43f5e" stroke-width="1" />`;
      html += `<circle cx="${zx}" cy="${zy}" r="1.5" fill="#f43f5e" />`;
      html += `<text x="${zx}" y="${zy - zl - 2}" fill="#f43f5e" font-size="5" font-family="monospace" text-anchor="middle">Z-SWITCH</text>`;

      // Fixture Clamps
      const hcfg = (G_STATE.configs && G_STATE.configs.holders) || {};
      (hcfg.holders || []).forEach(h => {
        const rad = (h.diameter || 30.0) / 2.0;
        html += `<circle cx="${h.x}" cy="${h.y}" r="${rad}" fill="rgba(59, 130, 246, 0.3)" stroke="#3b82f6" stroke-width="1" />`;
        html += `<circle cx="${h.x}" cy="${h.y}" r="2" fill="#3b82f6" />`;
        if (autoY && h.common_for_both_halves !== false) {
          const backY = h.y + ysplit;
          if (backY <= ty) {
            html += `<circle cx="${h.x}" cy="${backY}" r="${rad}" fill="rgba(59, 130, 246, 0.15)" stroke="#3b82f6" stroke-width="0.8" stroke-dasharray="2 2" />`;
          }
        }
      });

      svg.innerHTML = html;
    }

    // Backups Modal
    function toggleBackupsModal(show) {
      const modal = document.getElementById('modal-backups');
      if (show) {
        modal.classList.remove('hidden');
        renderBackupsList();
      } else {
        modal.classList.add('hidden');
      }
    }

    function renderBackupsList() {
      const c = document.getElementById('backups-list-container');
      c.innerHTML = '';
      const list = G_STATE.backups || [];
      if (list.length === 0) {
        c.innerHTML = `<p class="text-slate-500 py-4 text-center">No snapshots found in <code>.config_backups/</code>.</p>`;
        return;
      }
      list.forEach(b => {
        const div = document.createElement('div');
        div.className = "flex items-center justify-between p-3 rounded bg-slate-950 border border-slate-800 hover:border-slate-700 transition";
        div.innerHTML = `
          <div>
            <div class="font-mono font-medium text-white">${b.filename}</div>
            <div class="text-[10px] text-slate-400">${b.created_at} &bull; ${b.size_bytes} bytes</div>
          </div>
          <button onclick="restoreBackup('${b.filename}')" class="px-3 py-1 bg-slate-800 hover:bg-emerald-600 hover:text-white rounded text-slate-300 font-medium transition text-xs">
            Restore
          </button>
        `;
        c.appendChild(div);
      });
    }

    async function restoreBackup(filename) {
      if (!confirm(`Restore ${filename}? Current file will be preserved before restoring.`)) return;
      const res = await fetch('/api/backups/restore', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ filename: filename })
      });
      const data = await res.json();
      if (data.success) {
        await loadState();
        toggleBackupsModal(false);
        alert(data.message);
      } else {
        alert("Restore failed: " + data.error);
      }
    }

    window.onload = init;
  </script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# CLI & Standalone Server Entrypoint
# ---------------------------------------------------------------------------

class ThreadingStudioServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def find_free_port(start_port: int = 8090, max_attempts: int = 20) -> int:
    for port in range(start_port, start_port + max_attempts):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(('127.0.0.1', port)) != 0:
                return port
    return start_port


def main():
    parser = argparse.ArgumentParser(description="PCBtoGcode Configuration Studio & Web UI")
    parser.add_argument("--root", default=".", help="Root workspace directory containing PCB configs.")
    parser.add_argument("--port", type=int, default=8090, help="Port to serve Configuration Studio Web UI.")
    parser.add_argument("--host", default="127.0.0.1", help="Host interface to bind HTTP server.")
    parser.add_argument("--no-browser", action="store_true", help="Do not automatically launch browser.")
    args = parser.parse_args()

    root_dir = os.path.abspath(args.root)
    state_mgr = StudioState(root_dir)
    port = find_free_port(args.port)

    server = ThreadingStudioServer((args.host, port), StudioRequestHandler)
    server.state_mgr = state_mgr

    url = f"http://{args.host}:{port}/"
    print("=" * 72)
    print("  PCBtoGcode Configuration Studio (Standalone Web UI)")
    print("=" * 72)
    print(f"  Workspace Root : {root_dir}")
    print(f"  Server URL     : {url}")
    print(f"  Backups Folder : {os.path.join(root_dir, '.config_backups')}")
    print("  Press Ctrl+C to stop server.\n")

    if not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[STUDIO] Server stopped.")


if __name__ == "__main__":
    main()
