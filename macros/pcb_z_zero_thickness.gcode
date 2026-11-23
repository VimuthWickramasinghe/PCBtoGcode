; === PCB Thickness Z-Zero Calibration Macro ===
; Raises Z by 1.600 mm from BED Z=0 and sets top copper surface as G54 Z=0
G21 ; Millimeters
G90 ; Absolute positioning
M5 ; Ensure spindle is off
(MSG, Confirm tool is at BED Z=0 and spindle is OFF before applying +1.600mm PCB thickness offset)
M0 (Confirm tool is at BED Z=0 and spindle is OFF)
G0 Z1.600 ; Move to top surface of 1.600mm copper board
G10 L20 P1 Z0.000 ; Set PCB top copper surface as Z=0
G0 Z5.000 ; Retract safely above copper board
(MSG, PCB Top Z=0 calibrated for 1.600mm board)
