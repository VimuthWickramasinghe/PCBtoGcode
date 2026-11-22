; === 4-Point Copper Board Edge Auto-Calibration Macro ===
; Probes 2 points on the Left edge (X) and 2 points on the Front edge (Y)
; to compute exact board offset (dX, dY) and rotation (Theta) before milling.
G21 ; Millimeters
G90 ; Absolute positioning
M5 ; Ensure spindle is off
G0 Z5.000 ; Safe Z
(MSG, Confirm spindle is off and probe is ready for 4-point edge alignment)
M0 (Confirm spindle is off and probe is ready for 4-point edge alignment)
; Point 1: Left Edge @ Y=30
G0 X-5.000 Y30.000
G0 Z-0.500
G38.2 X5.000 F60.0
G0 Z5.000
; Point 2: Left Edge @ Y=115
G0 X-5.000 Y115.000
G0 Z-0.500
G38.2 X5.000 F60.0
G0 Z5.000
; Point 3: Front Edge @ X=45
G0 X45.000 Y-5.000
G0 Z-0.500
G38.2 Y5.000 F60.0
G0 Z5.000
; Point 4: Front Edge @ X=165
G0 X165.000 Y-5.000
G0 Z-0.500
G38.2 Y5.000 F60.0
G0 Z5.000
G0 X0.000 Y0.000
(MSG, 4-Point Edge Alignment Probing Complete)
