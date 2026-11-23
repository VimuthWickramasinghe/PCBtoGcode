; === Bed Height Map Probe Macro (Front Half Copper Area) ===
; Probes Z surface across X=25..185mm, Y=25..130mm (clear of all M4 clamps)
; Run via the Height Map Scanner in the UI to record [PRB:...] Z offsets!
G21 ; Millimeters
G90 ; Absolute positioning
M5 ; Ensure spindle is off before probing
S0 ; Ensure spindle RPM is zero
G53 G0 Z10.000 ; Pull Z up to machine top safe Z (10mm) before moving
G0 X25.000 Y25.000
G38.2 Z-3.000 F60.0
G0 Z5.000
G0 X105.000 Y25.000
G38.2 Z-3.000 F60.0
G0 Z5.000
G0 X185.000 Y25.000
G38.2 Z-3.000 F60.0
G0 Z5.000
G0 X185.000 Y77.500
G38.2 Z-3.000 F60.0
G0 Z5.000
G0 X105.000 Y77.500
G38.2 Z-3.000 F60.0
G0 Z5.000
G0 X25.000 Y77.500
G38.2 Z-3.000 F60.0
G0 Z5.000
G0 X25.000 Y130.000
G38.2 Z-3.000 F60.0
G0 Z5.000
G0 X105.000 Y130.000
G38.2 Z-3.000 F60.0
G0 Z5.000
G0 X185.000 Y130.000
G38.2 Z-3.000 F60.0
G53 G0 Z10.000
G0 X20.000 Y20.000 ; Park safely clear of Front-Left M4 clamp
(MSG, Bed Height Map Scan Complete)
