; === PCB Thickness Z-Zero Calibration Macro (from Bed Z=0) ===
; Converts calibrated Bed G54 Z=0 to top copper surface G54 Z=0 for a 1.600mm PCB
G21 ; Millimeters
G90 ; Absolute positioning
M5 ; Ensure spindle is off
S0 ; Ensure spindle RPM is zero
G53 G0 Z10.000 ; Retract to machine top safe Z clear of limit switch
(MSG, Confirm Bed Z=0 is already calibrated and spindle is OFF before applying +1.600mm PCB thickness offset)
M0 (Confirm Bed Z=0 calibrated and spindle OFF)
G0 X20.000 Y20.000 ; Move clear of limit switch before lowering to PCB thickness Z
G0 Z1.600 ; Move to top surface of 1.600mm copper board
G10 L20 P1 Z0.000 ; Set PCB top copper surface as G54 Z=0
G53 G0 Z10.000 ; Retract safely to machine top safe Z
(MSG, PCB Top Z=0 calibrated for 1.600mm board)
