; === Change Tool Macro ===
; Ensures spindle is OFF (M5, S0) and moves to Machine X max, Z max, Y min
; for safe, easy manual tool changing.
G21 ; Millimeters
G90 ; Absolute positioning
M5 ; Ensure spindle is OFF
S0 ; Zero spindle RPM
G53 G0 Z10.000 ; Raise Z to maximum height first
G53 G0 X275.000 Y0.000 ; Move to X max, Y min
M5 ; Keep spindle OFF
(MSG, Spindle OFF at Tool Change position X275 Y0 Z10. Change tool by hand, then Continue.)
M0 (Spindle OFF - Change tool by hand, then Continue)
