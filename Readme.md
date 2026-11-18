# PCB to G-Code Converter (KiCad & EasyEDA)

A Python toolchain built from scratch to convert Gerber (RS-274X) and Excellon drill files to precision CNC G-code and visual SVG previews.

Supports both **KiCad** and **EasyEDA** project exports, single-layer and double-sided boards, board outline engraving, isolated hole drilling per bit diameter, pre-drilling of slots, and dedicated endmill slot milling.

---

## Quick Start

### 1. KiCad Projects
```bash
python gerber_to_gcode.py project01
# Or pass a custom configuration:
python gerber_to_gcode.py project01 --config tool_config.json
```

### 2. EasyEDA Projects
```bash
python gerber_to_gcode_easyeda.py project02
# Or pass a custom configuration:
python gerber_to_gcode_easyeda.py project02 --config tool_config.json
```

### 3. Single File Conversion
Convert any single Gerber copper layer or Excellon drill file:
```bash
python gerber_to_gcode.py project01/AirCooler-F_Cu.gbr output_front.gcode
python gerber_to_gcode_easyeda.py project02/Drill_PTH_Through.DRL output_drill.gcode
```

---

## Tool Configuration (`tool_config.json`)

Both KiCad and EasyEDA scripts share a single, unified configuration file [`tool_config.json`](file:///home/vnews/Projects/PCBtoGcode/tool_config.json):

```json
{
  "isolation": {
    "tool_dia": 0.2,
    "cut_depth": -0.04,
    "feed_rate": 300.0,
    "plunge_rate": 80.0,
    "spindle_speed": 10000,
    "passes": 1,
    "stepover": 0.7,
    "safe_z": 5.0,
    "engrave_edge": true
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
    "passes": 2,
    "safe_z": 5.0,
    "spindle_speed": 10000
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
}
```

Config lookup order:
1. Explicit `--config <path>` flag if supplied.
2. Target project folder: `<folder>/tool_config.json` or `config.json`.
3. Current working directory / script directory: `tool_config.json` or `config.json`.

---

## Key Features

### 1. Isolated Drill Programs per Diameter
- **No empty / placeholder files**: A G-code drill file is generated **only** if holes of that diameter actually exist in the project.
- **Dedicated single-bit files**: Files are named `<prefix>_drill_<dia>mm.gcode` and `<prefix>_drill_<dia>mm-mirrored.gcode`.
- **Zero tool change pauses mid-program**: Load your bit, zero Z, and run without interruption.
- Mirrored variants allow drilling from the bottom side without coordinate disorientation.

### 2. Two-Stage Machining for Non-Circular Holes / Slots
Carbide PCB drill bits are brittle and break under lateral cutting loads (`G01 X/Y`). To solve this cleanly:
1. **Pre-Drill Phase (Drill Bit)**:
   - During the drilling operation, the drill bit performs pure vertical peck/relief plunges (`G01 Z...` then `G00 Z{safe_z}`) along the slot centerline spaced at half tool diameter ($s \approx 0.5 \cdot \text{dia}$).
   - **Zero lateral movement** occurs while the drill bit is at cutting depth, preventing bit deflection and breakage.
2. **Slot Milling Phase (Endmill)**:
   - A dedicated slot milling G-code program (`<prefix>_slot_milling_<dia>mm.gcode` and `-mirrored.gcode`) is generated.
   - Uses a milling endmill with multi-pass stepdown (`passes: 2`) and bi-directional finishing passes to mill out remaining scallops cleanly and accurately.

### 3. Board Edge Engraving
- The board outline from Edge.Cuts (KiCad) or BoardOutline (EasyEDA) is engraved at isolation depth (`-0.04 mm`) directly into Front Copper (`*_F_Cu.gcode`, `*TopLayer.gcode`) and Back Copper (`*_B_Cu.gcode`, `*BottomLayer.gcode`).
- This marks exact board boundaries on both copper sides before cutout.

### 4. Perfect Origin Zeroing
- All generated G-code programs and SVG previews are zero-aligned to the lower-left corner of the board outline at `(0.0000, 0.0000)`.
- Mirrored bottom-side operations maintain the identical bottom-left workpiece origin `(0, 0)` so stock fixtures never require re-zeroing.

### 5. SVG Visual Previews for Every File
- Every G-code file (`.gcode`) is paired with an identical `.svg` vector preview.
- Toolpaths, peck points, slot capsules, and board outlines are visually rendered in high-contrast color coding:
  - Copper contours: `#3b82f6` (Cyan/Blue)
  - Isolation toolpaths: `#e2e8f0` (White/Silver)
  - Drill holes: `#22c55e` (Emerald Green)
  - Pre-drilled slot relief pecks: `#eab308` (Yellow)
  - Slot milling paths / capsules: `#ec4899` / `#38bdf8` (Magenta / Light Blue)
  - Board outline / cutout: `#f97316` (Orange)

---

## Generated Output Files Example

For KiCad `project01` (`project01_gcode/`):
- `AirCooler-B_Cu.gcode` & `.svg` (Back copper isolation + outline engraving, mirrored)
- `AirCooler-F_Cu.gcode` & `.svg` (Front copper isolation + outline engraving)
- `AirCooler_drill_0.300mm.gcode` & `.svg` (+ mirrored)
- `AirCooler_drill_0.700mm.gcode` & `.svg` (+ mirrored, includes vertical slot pre-drill pecks)
- `AirCooler_slot_milling_0.700mm.gcode` & `.svg` (+ mirrored, endmill slot finishing)
- `AirCooler_drill_0.750mm.gcode` & `.svg` (+ mirrored)
- `AirCooler_drill_0.800mm.gcode` & `.svg` (+ mirrored)
- `AirCooler_drill_0.900mm.gcode` & `.svg` (+ mirrored)
- `AirCooler_drill_1.000mm.gcode` & `.svg` (+ mirrored)
- `AirCooler_drill_1.100mm.gcode` & `.svg` (+ mirrored)
- `AirCooler_drill_1.300mm.gcode` & `.svg` (+ mirrored)
- `AirCooler_drill_1.524mm.gcode` & `.svg` (+ mirrored)
- `AirCooler-Edge_Cuts.gcode` & `.svg` (+ mirrored, 3-pass cutout)

For EasyEDA `project02` (`project02_gcode/`):
- `Gerber_BottomLayer.gcode` & `.svg` (Bottom copper isolation + outline engraving, mirrored)
- `Gerber_TopLayer.gcode` & `.svg` (Top copper isolation + outline engraving)
- `Gerber_drill_0.306mm.gcode` & `.svg` (+ mirrored)
- `Gerber_drill_0.915mm.gcode` & `.svg` (+ mirrored)
- `Gerber_drill_0.930mm.gcode` & `.svg` (+ mirrored)
- `Gerber_drill_1.016mm.gcode` & `.svg` (+ mirrored, includes vertical slot pre-drill pecks)
- `Gerber_slot_milling_1.016mm.gcode` & `.svg` (+ mirrored, endmill slot finishing)
- `Gerber_drill_1.200mm.gcode` & `.svg` (+ mirrored)
- `Gerber_drill_1.800mm.gcode` & `.svg` (+ mirrored)
- `Gerber_drill_3.000mm.gcode` & `.svg` (+ mirrored)
- `Gerber_BoardOutlineLayer.gcode` & `.svg` (+ mirrored, 3-pass cutout)

---

## Instant PCB Quotation Calculator

`instant_quotation.py` automatically analyzes Gerber and drill files in any selected folder, extracts board metrics, and calculates an itemized manufacturing quotation based on [`quotation_config.json`](file:///home/vnews/Projects/PCBtoGcode/quotation_config.json).

### Usage:
```bash
# Calculate quotation for a project folder:
python instant_quotation.py project01
python instant_quotation.py project02

# Specify quantity and solder mask color:
python instant_quotation.py project01 --qty 10 --mask-color black

# Remove manufacturer mark and select surface finish:
python instant_quotation.py project01 --qty 20 --without-producer-mark --surface-finish ENIG

# Output as JSON:
python instant_quotation.py project02 --json

# Save report to a file:
python instant_quotation.py project01 -o quotation.txt
python instant_quotation.py project02 --json -o quotation.json
```

### Configurable Pricing (`quotation_config.json`):
All pricing rules, fees, discounts, and machine rates can be updated anytime in [`quotation_config.json`](file:///home/vnews/Projects/PCBtoGcode/quotation_config.json):
- **Currency**: Symbol, code, exchange rate.
- **Base Fees**: Engineering CAM setup fee.
- **Standard Promo Tier**: Special flat rate (e.g. 5 pcs $\le 100\times 100$ mm for \$5.00).
- **Area Pricing**: Base substrate rate per $\text{cm}^2$ and layer multipliers.
- **Quantity Discounts**: Tiered volume discount percentages (5, 10, 20, 50, 100+ pcs).
- **Drill Surcharges**: Free hole quota, per-100 overage fee, micro-via ($<0.30$ mm) precision tier.
- **Slots & Shape**: Milling fees for non-circular slots and complex polygonal outlines.
- **Finishes & Colors**: Solder mask colors (black, red, yellow, blue, white, purple), silkscreen, HASL/ENIG, copper weight (1oz, 2oz).
- **Desktop CNC Prototype Comparison**: Stock material blank cost, spindle cutting time rate, and tool wear estimate.

### Material Cost Database (`material_cost.json`):
Direct market procurement pricing from [Tronic.lk](https://tronic.lk), Daraz.lk, and AliExpress is cataloged in [`material_cost.json`](file:///home/vnews/Projects/PCBtoGcode/material_cost.json), including:
- Dual-sided and single-sided FR-4 copper clad boards (A4 cost basis: Rs. 1,500).
- Electroless & electro copper via plating chemicals ($\text{CuSO}_4$, $\text{H}_2\text{SO}_4$, $\text{HCl}$, $\text{NaOH}$, Formaldehyde, Palladium via activator, and copper anodes).
- UV-curable solder mask inks (Green, Black, Red, White) and silkscreen inks.
- CNC carbide drill bits (0.1–1.0mm sets), V-shape engraving bits, and corn-teeth router bits.
- Safety protective equipment (Nitrile gloves, eye protection goggles, UV glasses, and dual-cartridge respirators).


