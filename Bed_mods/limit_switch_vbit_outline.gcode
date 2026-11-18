( ========================================================== )
(  Limit Switch Pocket - V-Bit Perimeter Scoring             )
(  Generated: 2026-11-17 23:38:07 )
(  Bounding Box: X[269.000..274.900], Y[129.803..150.203] )
(  Pocket Size : 5.900 mm x 20.400 mm )
(  Target Depth: -0.40 mm (2 passes) )
(  Spindle     : 10000 RPM, Feed: 300.0 mm/min )
( ========================================================== )

G21          (Units in Millimeters)
G90          (Absolute Coordinates)
G94          (Feed Rate per Minute)
G17          (XY Plane Selection)
G54          (Work Coordinate System 1 / Active Work Offset)
G00 Z5.0000 (Rapid to Safe Z)
G00 X269.0000 Y129.8030 (Rapid directly to pocket start at Safe Z)
M03 S10000 (Spindle Start CW)
G04 P2.0     (Dwell 2s for spindle to reach speed)


( --- Pass 1/2: Depth -0.2000 mm --- )
G01 Z-0.2000 F80.0 (Plunge)
G01 X274.9000 Y129.8030 F300.0
G01 X274.9000 Y150.2030
G01 X269.0000 Y150.2030
G01 X269.0000 Y129.8030

( --- Pass 2/2: Depth -0.4000 mm --- )
G01 Z-0.4000 F80.0 (Plunge)
G01 X274.9000 Y129.8030 F300.0
G01 X274.9000 Y150.2030
G01 X269.0000 Y150.2030
G01 X269.0000 Y129.8030

G00 Z5.0000 (Retract to Safe Z)
M05          (Spindle Stop)
G00 Z20.0000 (Clearance Height)
M30          (Program End and Reset)