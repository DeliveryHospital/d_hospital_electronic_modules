(G-CODE GENERATED BY FLATCAM v8.994 - www.flatcam.org - Version Date: 2020/11/7)

(Name: Controle_Motor.TXT_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Wednesday, 11 August 2021 at 20:08)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)


(TOOLS DIAMETER: )
(Tool: 1 -> Dia: 0.7112)
(Tool: 2 -> Dia: 0.76)
(Tool: 3 -> Dia: 0.9)
(Tool: 4 -> Dia: 1.0)
(Tool: 5 -> Dia: 1.1)
(Tool: 6 -> Dia: 1.38)
(Tool: 7 -> Dia: 3.0)

(FEEDRATE Z: )
(Tool: 1 -> Feedrate: 300)
(Tool: 2 -> Feedrate: 300)
(Tool: 3 -> Feedrate: 300)
(Tool: 4 -> Feedrate: 300)
(Tool: 5 -> Feedrate: 300)
(Tool: 6 -> Feedrate: 300)
(Tool: 7 -> Feedrate: 300)

(FEEDRATE RAPIDS: )
(Tool: 1 -> Feedrate Rapids: 1500)
(Tool: 2 -> Feedrate Rapids: 1500)
(Tool: 3 -> Feedrate Rapids: 1500)
(Tool: 4 -> Feedrate Rapids: 1500)
(Tool: 5 -> Feedrate Rapids: 1500)
(Tool: 6 -> Feedrate Rapids: 1500)
(Tool: 7 -> Feedrate Rapids: 1500)

(Z_CUT: )
(Tool: 1 -> Z_Cut: -1.7)
(Tool: 2 -> Z_Cut: -1.7)
(Tool: 3 -> Z_Cut: -3.0)
(Tool: 4 -> Z_Cut: -1.7)
(Tool: 5 -> Z_Cut: -1.7)
(Tool: 6 -> Z_Cut: -1.7)
(Tool: 7 -> Z_Cut: -1.7)

(Tools Offset: )
(Tool: 3 -> Offset Z: 0.0)

(Z_MOVE: )
(Tool: 1 -> Z_Move: 2)
(Tool: 2 -> Z_Move: 2)
(Tool: 3 -> Z_Move: 2.0)
(Tool: 4 -> Z_Move: 2)
(Tool: 5 -> Z_Move: 2)
(Tool: 6 -> Z_Move: 2)
(Tool: 7 -> Z_Move: 2)

(Z Toolchange: 20.0 mm)
(X,Y Toolchange: 0.0000, 0.0000 mm)
(Z Start: None mm)
(Z End: 0.5 mm)
(X,Y End: None mm)
(Steps per circle: 64)
(Preprocessor Excellon: default)

(X range:    1.8022 ...   57.3802  mm)
(Y range:    2.1830 ...  118.0860  mm)

(Spindle Speed: 1000 RPM)
G21
G90
G94

G01 F300.00

M5
G00 Z20.0000
T3
G00 X0.0000 Y0.0000                
M6
(MSG, Change to Tool Dia = 0.9000 ||| Total drills for tool T3 = 22)
M0
G00 Z20.0000

G01 F300.00
M03 S1000
G00 X27.5592 Y46.8630
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X17.3992 Y46.8630
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X18.2882 Y60.5790
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X28.4482 Y60.5790
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X24.5112 Y75.9460
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X18.7962 Y76.0730
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X13.0812 Y76.2000
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X7.3662 Y76.2000
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X7.3662 Y86.3600
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X7.3662 Y90.8050
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X7.3662 Y100.9650
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X13.0812 Y86.3600
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X18.7962 Y86.2330
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X24.5112 Y86.1060
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X37.4652 Y85.9790
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X41.2752 Y85.8520
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X37.4652 Y75.8190
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X41.2752 Y75.6920
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X52.5782 Y55.3721
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X55.1182 Y55.3719
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X52.5782 Y46.9900
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X52.5782 Y36.8300
G01 Z-3.0000
G01 Z0
G00 Z2.0000
M05
G00 Z0.50

