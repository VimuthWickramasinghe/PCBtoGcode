; === Bed Height Map Probe Macro (Front Half Copper Area) ===
; Probes Z surface across X=25..185mm, Y=25..130mm (clear of all M4 clamps)
; Run via the Height Map Scanner in the UI to record [PRB:...] Z offsets!
G21 ; Millimeters
G90 ; Absolute positioning
G0 Z5.000 ; Safe Z travel height
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
G0 Z5.000
G0 X0.000 Y0.000
(MSG, Bed Height Map Scan Complete)
