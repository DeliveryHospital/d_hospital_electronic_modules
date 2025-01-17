(G-CODE GENERATED BY FLATCAM v8.994 - www.flatcam.org - Version Date: 2020/11/7)

(Name: Controle_Motor.TXT_cnc)
(Type: G-code from Geometry)
(Units: MM)

(Created on Saturday, 31 July 2021 at 12:48)

(This preprocessor is the default preprocessor used by FlatCAM.)
(It is made to work with MACH3 compatible motion controllers.)


(TOOLS DIAMETER: )
(Tool: 1 -> Dia: 0.9)
(Tool: 2 -> Dia: 0.91)
(Tool: 3 -> Dia: 1.0)
(Tool: 4 -> Dia: 1.1)
(Tool: 5 -> Dia: 1.38)
(Tool: 6 -> Dia: 3.0)

(FEEDRATE Z: )
(Tool: 1 -> Feedrate: 150.0)
(Tool: 2 -> Feedrate: 150.0)
(Tool: 3 -> Feedrate: 150.0)
(Tool: 4 -> Feedrate: 150.0)
(Tool: 5 -> Feedrate: 150.0)
(Tool: 6 -> Feedrate: 150.0)

(FEEDRATE RAPIDS: )
(Tool: 1 -> Feedrate Rapids: 1500)
(Tool: 2 -> Feedrate Rapids: 1500)
(Tool: 3 -> Feedrate Rapids: 1500)
(Tool: 4 -> Feedrate Rapids: 1500)
(Tool: 5 -> Feedrate Rapids: 1500)
(Tool: 6 -> Feedrate Rapids: 1500)

(Z_CUT: )
(Tool: 1 -> Z_Cut: -3.000000000000001)
(Tool: 2 -> Z_Cut: -3.000000000000001)
(Tool: 3 -> Z_Cut: -3.000000000000001)
(Tool: 4 -> Z_Cut: -3.000000000000001)
(Tool: 5 -> Z_Cut: -3.000000000000001)
(Tool: 6 -> Z_Cut: -3.000000000000001)

(Tools Offset: )
(Tool: 6 -> Offset Z: 0.0)

(Z_MOVE: )
(Tool: 1 -> Z_Move: 2)
(Tool: 2 -> Z_Move: 2)
(Tool: 3 -> Z_Move: 2)
(Tool: 4 -> Z_Move: 2)
(Tool: 5 -> Z_Move: 2)
(Tool: 6 -> Z_Move: 2)

(Z Toolchange: 15 mm)
(X,Y Toolchange: 0.0000, 0.0000 mm)
(Z Start: None mm)
(Z End: 20.0 mm)
(X,Y End: None mm)
(Steps per circle: 64)
(Preprocessor Excellon: default)

(X range:    2.0560 ...   57.6340  mm)
(Y range:    2.1830 ...  118.0860  mm)

(Spindle Speed: 1000 RPM)
G21
G90
G94

G01 F150.00

M5
G00 Z15.0000
T6
G00 X0.0000 Y0.0000                
M6
(MSG, Change to Tool Dia = 3.0000 ||| Total drills for tool T6 = 4)
M0
G00 Z15.0000

G01 F150.00
M03 S1000
G00 X56.1340 Y3.6830
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X3.5560 Y3.6830
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X3.5560 Y116.5860
G01 Z-3.0000
G01 Z0
G00 Z2.0000
G00 X56.1340 Y116.5860
G01 Z-3.0000
G01 Z0
G00 Z2.0000
M05
G00 Z20.00


