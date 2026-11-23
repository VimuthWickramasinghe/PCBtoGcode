# DIY CNC PCB Design Guidelines & Minimum Fabrication Rules

*(Derived from Gerber/Excellon benchmark analysis of `project02` vs `project01` & `project03` on the `PCBtoGcode` dual-sided FR-4 CNC milling pipeline)*

---

## 1. Why `project01` & `project03` Lost Copper (Root-Cause Analysis)

Industrial PCB fab houses use photo-lithography and chemical etching down to `0.10 mm`. On a **CNC router using a 30° V-bit (`0.1–0.2 mm` tip at `-0.04 mm` depth), carbide twist drills, and manual/chemical vias without industrial electroplated copper barrels**, mechanical forces and tool geometry impose much stricter limits:

1. **Thin Tracks (`0.50–0.60 mm`) Get Eroded by V-Bit Kerf & Z-Depth Variance**
   - A `30° / 0.2 mm` V-bit removes **`>= 0.20 mm`** of copper even at nominal depth. If board warp or Z-zero variance pushes depth to `-0.08 mm` to `-0.12 mm`, cut width widens to `0.24–0.28 mm`.
   - With spindle runout (`~0.03–0.05 mm`), a `0.50 mm` track (`project03`) or `0.60 mm` track (`project01`) shrinks to a narrow foil strip (`~0.25–0.35 mm`) that easily lifts or breaks under mechanical cutting heat.
   - `project02` uses **`1.000 mm` (`~39.4 mil`) tracks everywhere**, leaving **`~0.75 mm`** of sturdy, well-bonded copper after isolation.

2. **Small Annular Rings (`<= 0.20 mm`) Peel Off During Drilling & Engraving**
   - On bare CNC FR-4, pads do not have heavy electroplated barrel anchors inside the hole wall prior to milling.
   - `project01` contains pads down to `1.05 × 1.50 mm` with `0.75 mm` holes (**`0.150 mm` radial annular ring**) and `0.60 mm` via pads with `0.30 mm` holes (**`0.150 mm` ring**).
   - Combining drill runout (`±0.10 mm`) with `0.20 mm` isolation cut width completely wipes out or delaminates `<= 0.20 mm` rings.
   - `project02` keeps component annular rings at **`0.300–0.650 mm`** (`1.524–3.000 mm` pads around `0.915–1.800 mm` holes).

3. **Tight Copper Clearances (`<= 0.20 mm`) Leave Zero Bit Margin**
   - `project01` drops to **`0.200 mm`** copper-to-copper gap — identical to the `0.20 mm` isolation bit diameter — leaving no tolerance for backlash, alignment error, or V-bit kerf growth.
   - `project02` enforces **`>= 0.302 mm`** minimum clearance everywhere.

4. **Zero Solder Mask Expansion (`0.00 mm`) Causes Pad Clipping**
   - `project01` and `project03` exported KiCad Gerbers with **`0.000 mm` solder mask expansion** (mask opening = exact pad size). Any slight XY registration offset during Step 5 mask clearing trims pad copper edges or leaves cured UV mask covering pad edges.
   - `project02` uses **`+0.101 mm` (`~4 mil`) radial solder mask expansion** (`+0.203 mm` opening diameter), cleanly exposing the entire copper pad.

5. **Small Micro-Holes (`<= 0.75 mm`) Wander & Break Easily**
   - `project01` and `project03` mix micro-drills (`0.30, 0.70, 0.75 mm`) across many diameter changes. Small carbide bits break easily and deflect on glass-fiber weave.
   - `project02` uses **`0.915–1.200 mm`** standard component holes (`0.915 / 1.016 / 1.200 mm`), making drilling rigid and reliable. *(Note: `project02` did include 8 tiny `0.306 mm` vias on `0.610 mm` pads — for DIY CNC vias, enlarge vias to **`0.70–0.80 mm` hole / `1.60–1.80 mm` pad**.)*

---

## 2. Measured Benchmark Comparison (`project02` vs `project01` & `project03`)

| Design Parameter | `project02` (`Board-01`, EasyEDA — Good) | `project01` (`AirCooler`, KiCad — Fragile) | `project03` (`Micro Probe`, KiCad — Fragile) |
| :--- | :--- | :--- | :--- |
| **Track Width (Signal)** | **`1.000 mm`** (all 106 trace segments) | `0.600 mm` (`2.0 mm` bottom partial) | `0.500 mm` (bottom traces) |
| **Min Copper Clearance** | **`0.302 mm`** (`~12 mil`) | `0.200 mm` (bottom) / `0.220 mm` (top) | `0.500 mm` |
| **Component Drill Sizes** | `0.915, 0.930, 1.016, 1.200, 1.800 mm` *(+ `3.0 mm` NPTH)* | `0.70, 0.75, 0.80, 0.90, 1.00, 1.10, 1.30, 1.524 mm` | `0.70, 0.80, 0.90, 1.00, 1.02, 1.20, 3.00 mm` |
| **Via Drill / Pad Size** | `0.306 mm` hole / `0.610 mm` pad *(recommend enlarging for DIY)* | `0.300 mm` hole / `0.600 mm` pad | `3.000 mm` / `4.000 mm` |
| **Component Pad Sizes** | `1.524, 1.778, 1.8×2.0, 2.2, 2.5, 3.0 mm` | `1.05×1.50` up to `2.80 mm` | `1.40, 1.73×2.03, 1.80, 2.40 mm` |
| **Min Component Annular Ring** | **`0.300 mm`** (`0.300 – 0.650 mm` across all pins) | **`0.150 mm`** (`0.75 mm` hole in `1.05 mm` pad) | `0.349 mm` (pads) but thin `0.50 mm` tracks |
| **Copper Pour (`G36` Zones)** | **None** (point-to-point `1.0 mm` routing) | Full copper zones (top & bottom) | Full copper zones (top & bottom) |
| **Solder Mask Expansion** | **`+0.101 mm` per side** (`+0.203 mm` dia) | `0.000 mm` (`1:1` mask = pad size) | `0.000 mm` (`1:1` mask = pad size) |

---

## 3. Master CNC PCB Fabrication Design Rules (DRC Specification)

Use **Recommended** values by default; never go below **Absolute Minimum** limits.

| Rule / Parameter | Absolute Minimum (Hard Floor) | Recommended for Reliable CNC Milling | Power / High-Current / Mounting |
| :--- | :--- | :--- | :--- |
| **Track Width** | `0.80 mm` (`31.5 mil`) | **`1.00 mm`** (`39.4 mil`, matches `project02`) | `1.50 – 2.50 mm` |
| **Copper-to-Copper Clearance** | `0.30 mm` (`11.8 mil`) | **`0.40 mm`** (`15.7 mil`) | `0.50 – 0.80 mm` |
| **Annular Ring (Radial Width)** | `0.30 mm` (`11.8 mil`) | **`0.40 – 0.50 mm`** (`15.7 – 19.7 mil`) | `>= 0.60 mm` |
| **Through-Hole Pad Formula** | `Hole Dia + 0.65 mm` | **`Hole Dia + 0.80 to 1.00 mm`** | `Hole Dia + 1.20 mm` |
| **Component Drill Diameter** | `0.80 mm` | **`0.90 – 1.05 mm`** (use fewest distinct sizes) | `1.20 – 1.80 mm` |
| **Via Drill Diameter** | `0.60 mm` *(avoid `0.30 mm` micro-vias)* | **`0.80 mm`** (easy wire-stitch / rivet / chemical plating) | `1.00 mm` |
| **Via Pad Diameter** | `1.40 mm` | **`1.80 – 2.00 mm`** (`0.50–0.60 mm` ring) | `2.20 mm` |
| **SMD Pad Width / Gap** | `0.70 mm` pad / `0.30 mm` gap | **`>= 0.90 mm` pad / `>= 0.40 mm` gap** | Prefer `1206` / `0805` / `SOIC` (`>= 1.27 mm` pitch) |
| **Copper Pour (Ground Zone)** | Avoid if possible; else `0.40 mm` clearance | **No full pour** (like `project02`) or **`0.50 mm` gap + `0.60 mm` spoke** | Keep thermal spokes `>= 0.60 mm` |
| **Solder Mask Expansion (Radial)** | `+0.08 mm` (`3 mil`) | **`+0.10 mm`** (`4 mil`, matches `project02`) | `+0.15 mm` |
| **Copper-to-Board-Edge Clearance** | `0.80 mm` | **`1.50 – 2.00 mm`** | `>= 2.50 mm` near snap tabs / clamps |
| **Silkscreen Line Width / Height** | `0.20 mm` / `1.00 mm` | `0.25 mm` / `1.20 mm` | — |

### Standard Drill ↔ Pad Reference Table (Use Directly in Footprints)

| Lead / Pin Type | Drill Diameter | Minimum Pad Diameter (`>= 0.30 mm` ring) | Recommended CNC Pad Diameter (`>= 0.40 mm` ring) |
| :--- | :--- | :--- | :--- |
| **DIY Through-Hole Via** | `0.80 mm` | `1.45 mm` | **`1.80 mm`** (or `2.00 mm`) |
| **Resistors / Caps / Small Diodes** | `0.90 mm` (`35 mil`) | `1.52 mm` (`project02` uses `1.524 mm`) | **`1.80 mm`** |
| **2.54 mm Pin Headers / ICs (DIP)** | `1.00 mm` (`40 mil`) | `1.65 mm` (`project02` uses `1.778 mm`) | **`1.80 mm` circle or `1.78 × 2.20 mm` oval** *(keeps `0.76 mm` clearance between 2.54 mm pins!)* |
| **Power Diodes / Screw Terminals** | `1.20 mm` | `1.85 mm` (`project02` uses `1.8×2.0 / 2.5 mm`) | **`2.20 – 2.50 mm`** |
| **Heavy Connectors / Barrel Jacks** | `1.80 mm` | `2.50 mm` | **`3.00 mm`** (`project02` uses `3.00 mm`) |
| **M3 Mounting Holes (NPTH)** | `3.20 mm` (or `3.00 mm`) | Unplated (`NPTH`), `5.5 mm` keepout | **`5.5 – 6.0 mm`** courtyard clearance |

> **Tip on Consolidating Drill Sizes:** Limiting a design to just **3–4 drill diameters** (e.g., `0.8 mm` vias, `1.0 mm` headers/parts, `1.2 mm` terminals, `3.0 mm` mounting) drastically cuts manual drill changes in Step 1!

---

## 4. Tool-Specific Setup Instructions

### A. KiCad (KiCad 7 / 8 / 9 / 10)

KiCad's factory defaults are aimed at commercial fabrication (`0.20 mm` track, `0.20 mm` clearance, `0.00 mm` mask expansion, `0.13 mm` min annular width), which directly caused the copper loss in `project01` and `project03`. Configure these before routing:

1. **Board Setup → Design Rules → Constraints** (`File → Board Setup`):
   - **Minimum clearance**: `0.35 mm` *(hard floor `0.30 mm`)*
   - **Minimum track width**: `0.80 mm` *(recommended `1.00 mm`)*
   - **Minimum connection width**: `0.60 mm`
   - **Minimum annular width**: **`0.35 mm`** *(CRITICAL — prevents KiCad DRC from allowing `0.15 mm` rings like `project01`!)*
   - **Minimum via diameter**: **`1.80 mm`**
   - **Minimum via hole**: **`0.80 mm`**
   - **Minimum through hole**: **`0.80 mm`**
   - **Copper to hole clearance**: `0.40 mm`
   - **Copper to edge clearance**: `1.50 mm`
2. **Board Setup → Design Rules → Net Classes**:
   - Set **Default** class: Clearance = `0.35 mm` (or `0.40 mm`), Track Width = **`1.00 mm`**, Via Size = **`1.80 mm`**, Via Hole = **`0.80 mm`**.
   - Add a **Power** class: Track Width = **`1.50 – 2.00 mm`**, Clearance = `0.50 mm`.
3. **Board Setup → Design Rules → Pre-defined Sizes** (quick selector dropdowns):
   - Track widths: `0.80 mm`, **`1.00 mm`**, `1.50 mm`, `2.00 mm`
   - Via sizes: **`1.80 / 0.80 mm`**, `2.00 / 1.00 mm`
4. **Board Setup → Physical Stackup / Solder Mask / Paste**:
   - **Solder mask expansion**: **`0.10 mm`** (`+0.10 mm` radial — **never leave at `0.00 mm`**).
   - **Solder mask minimum web**: `0.20 mm`.
5. **Fixing Existing Footprint Pads (e.g., 2.54 mm Headers / Resistors)**:
   - Double-click a pad (or select all footprint pads → `Properties` / `Edit Pad as Graphic Shapes` or `Tools → Update Footprint / Edit Pads`):
   - Avoid thin `1.05 mm` pads around `0.75 mm` holes (`project01`). Set through-hole pad size to **`>= Hole + 0.80 mm`** (`1.78–1.80 mm` circular or `1.70 × 2.20 mm` oblong/oval for 2.54 mm pitch).
   - Avoid `0402`/`0603` passives when milling; use **`0805` or `1206`** SMD footprints with enlarged pads.
6. **Copper Zones (`Add Filled Zone`)**:
   - Prefer **direct `1.0 mm` routing without full copper zones** (like `project02`).
   - If you must use a ground pour: set Zone **Clearance** = **`0.50 mm`**, **Minimum width** = **`0.50 mm`**, **Thermal relief gap** = `0.40 mm`, **Thermal spoke width** = **`0.60 mm`**.
7. **Gerber & Drill Export (`File → Fabrication Outputs → Gerbers / Drill Files`)**:
   - **Layers**: `F.Cu`, `B.Cu`, `F.Mask`, `B.Mask`, `F.Silkscreen`, `B.Silkscreen`, `Edge.Cuts`.
   - **Format**: Millimeters, `4.6` (`4 integer, 6 decimal`), **Use drill/place file origin** unchecked or consistent across all files, **Plot footprint values/references** on Silkscreen.
   - **Drill Files (Excellon)**: Millimeters, **Decimal format**, Separate PTH and NPTH files (`*-PTH.drl`, `*-NPTH.drl`), **Mirror Y axis OFF**, **Minimal header OFF**.

---

### B. EasyEDA (Standard & Pro)

EasyEDA's broader pad defaults (`1.524–1.778 mm` on through-hole pins) and `+0.101 mm` mask expansion are why `project02` milled cleanly. Lock in these DRC rules in EasyEDA so vias and clearances are equally safe:

1. **Design Rule Manager (`Design → Design Rule...` in Standard, or `Setup → Design Rule` in Pro)**:
   - **Track Width**: Default = **`1.00 mm`** (`39.4 mil`), Minimum = **`0.80 mm`** (`31.5 mil`)
   - **Clearance (Track to Track / Pad / Copper)**: Minimum = **`0.35 mm`** (`14 mil`; hard floor `0.302 mm`)
   - **Via Diameter / Drill Diameter**: **`1.80 mm` / `0.80 mm`** *(do NOT use EasyEDA's default `0.61 mm / 0.305 mm` via — change it in Design Rules before routing!)*
   - **Copper Area (Copper Pour)** *(if used)*: Clearance = **`0.50 mm`**, Spoke Width = **`0.60 mm`** (prefer **Spoke / Relief** connection, or omit copper pours altogether like `project02`).
   - **Board Outline Clearance**: `1.50 mm`
2. **Solder Mask Settings (`Global` or per-pad / `Tools → Set Solder Mask`)**:
   - **Solder Mask Expansion**: **`0.10 mm`** (`4 mil`, matching `project02`'s `+0.101 mm`).
3. **Through-Hole Component Pads**:
   - Verify any imported library footprint has `Pad Diameter >= Hole Diameter + 0.80 mm` (`>= 1.524 mm` for `0.90 mm` drills, `>= 1.778 mm` for `1.016 mm` drills).
4. **Gerber Export (`Fabrication → PCB Fabrication File (Gerber)`)**:
   - Export full layer set (`TopLayer.GTL`, `BottomLayer.GBL`, `TopSolderMaskLayer.GTS`, `BottomSolderMaskLayer.GBS`, `TopSilkscreenLayer.GTO`, `BoardOutlineLayer.GKO`, and Excellon `.DRL` drill files).
   - Keep **Millimeters (`4:5` or `4:6`)** and default EasyEDA coordinate origin consistent across copper, outline, and drill layers.

---

### C. Altium Designer, Eagle / Fusion 360 Electronics, & DipTrace

| EDA Tool | Where to Configure | Exact Settings to Enter |
| :--- | :--- | :--- |
| **Altium Designer** | `Design → Rules` | **Clearance**: `0.35 mm` (`14 mil`)<br>**Width**: Min `0.80 mm`, Pref **`1.00 mm`**, Max `3.00 mm`<br>**Routing Via Style**: Dia **`1.80 mm`**, Hole **`0.80 mm`**<br>**Minimum Annular Ring**: **`0.35 mm`**<br>**Solder Mask Expansion**: **`0.10 mm`** (`4 mil`)<br>**Polygon Connect Style**: Relief Connect, Conductor Width **`0.60 mm`**, Air Gap **`0.50 mm`** |
| **Eagle / Fusion 360** | `Tools → DRC` | **Clearance** tab (all wire/pad/via gaps): **`14 mil` (`0.35 mm`)**<br>**Distance → Copper/Dimension**: `60 mil` (`1.50 mm`)<br>**Sizes → Minimum Width**: `32 mil` (`0.80 mm`), Preferred `40 mil` (`1.00 mm`)<br>**Sizes → Minimum Drill**: **`32 mil` (`0.80 mm`)**<br>**Restring (Annular Ring)** tab: Min **`14 mil` (`0.35 mm`)** on Pads & Vias (Top/Bottom)<br>**Masks → Stop**: Min/Max **`4 mil` (`0.10 mm`)** |
| **DipTrace** | `Verification → Design Rules` | **Clearance**: All copper gaps **`0.35 mm`**; Board edge `1.50 mm`<br>**Sizes → Track Width**: Min `0.80 mm` (route at `1.00 mm`)<br>**Sizes → Via / Hole**: Min Hole **`0.80 mm`**, Min Via Dia **`1.80 mm`**, Min Ring **`0.35 mm`**<br>**Copper Pour**: Clearance **`0.50 mm`**, Spoke Width **`0.60 mm`**<br>**Export → Gerber**: Solder Mask Swell = **`+0.10 mm`** |

---

## 5. Pre-Milling Verification Checklist (Before Generating G-Code)

1. Run your EDA's **DRC (Design Rule Check)** with the rules above and ensure **0 violations**.
2. Inspect all **2.54 mm headers & IC pads**: confirm adjacent pads still have **`>= 0.30 mm`** gap (`1.78 mm` circular or `1.70 × 2.20 mm` oval gives `0.76–0.84 mm` gap at `2.54 mm` pitch).
3. Confirm **no `0.30–0.60 mm` micro-vias** remain anywhere on the board (upgrade all vias to **`0.80 mm` hole / `1.80 mm` pad**).
4. Confirm **no hairline `0.20–0.60 mm` tracks** remain (route signals at **`1.00 mm`** or `0.80 mm` minimum).
5. Confirm **Solder Mask layer (`F_Mask`/`GTS`, `B_Mask`/`GBS`)** openings are **`>= Pad + 0.20 mm` diameter** (`+0.10 mm` radial expansion).
6. Check `Edge.Cuts` / `BoardOutlineLayer`: verify a closed continuous contour (`0.1–0.2 mm` line width) with **`>= 1.5 mm`** clearance to any copper feature.
