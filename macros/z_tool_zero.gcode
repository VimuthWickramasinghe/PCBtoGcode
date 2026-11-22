; === Z-Tool Zero Calibration Macro ===
; Moves safely to the fixed Bed Z-Zero Limit Switch (X=272.001, Y=135.002)
; Probes down with G38.2 and sets G54 Z=0 (adjust offset for copper thickness if needed)
G21 ; Millimeters
G90 ; Absolute positioning
M5 ; Ensure spindle is off before moving to switch
G53 G0 Z0.000 ; Retract spindle to machine top safe Z
G53 G0 X272.001 Y135.002 ; Move over Z-Zero limit switch
(MSG, Confirm spindle is off and tool is above bed switch)
M0 (Confirm spindle is off and tool is above bed switch)
G38.2 Z-25.000 F80.0 ; Probe down until limit switch triggers
G91 ; Incremental
G0 Z2.000 ; Back off 2mm
G90 ; Absolute
G38.2 Z-5.000 F25.0 ; Slow precision re-probe
G10 L20 P1 Z0.000 ; Zero active G54 Z at switch datum
G53 G0 Z0.000 ; Retract to safe machine Z
(MSG, Z-Tool Zero complete at limit switch)

