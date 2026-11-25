; === Z-Tool Zero Calibration Macro (Bed Z=0 via Limit Switch) ===
; Fixed Bed Z-Zero Limit Switch at Machine G53 (X=272.001, Y=135.002)
; Limit switch trigger point is 7.270 mm ABOVE the CNC bed surface
G21 ; Millimeters
G90 ; Absolute positioning
M5 ; Ensure spindle is off before moving to switch
S0 ; Ensure spindle RPM is zero
G53 G0 Z10.000 ; Retract spindle to machine top safe Z
G53 G0 X272.001 Y135.002 ; Move over fixed Z-Zero limit switch
(MSG, Confirm spindle is off and tool is above bed switch at +7.270mm above bed)
M0 (Confirm spindle is off and tool is above bed switch at +7.270mm above bed)
G91 ; Incremental probing moves
G38.2 Z-25.000 F80.0 ; Probe down until limit switch triggers
G0 Z2.000 ; Back off 2mm
G38.2 Z-5.000 F25.0 ; Slow precision re-probe
G90 ; Restore absolute positioning
G10 L20 P1 Z7.270 ; Switch trigger is 7.270mm above bed -> Bed = G54 Z0.000
G53 G0 Z10.000 ; Retract to safe machine Z
(MSG, Bed Z-Tool Zero complete: switch datum = Z+7.270mm, Bed = Z0)
