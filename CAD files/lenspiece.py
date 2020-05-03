'''
Freecad python code for creating lens with disks
'''

import FreeCAD
FreeCAD.open(u"/home/eric/omnirobot/CAD files/speakerCab20_8.fcstd")
App.setActiveDocument("speakerCab20_8")
App.ActiveDocument=App.getDocument("speakerCab20_8")
Gui.ActiveDocument=Gui.getDocument("speakerCab20_8")
Gui.activeDocument().activeView().viewTop()
Gui.activateWorkbench("DrawingWorkbench")
Gui.activateWorkbench("DraftWorkbench")


import Draft
pl=FreeCAD.Placement()
pl.Rotation.Q=(0.0,-0.0,-0.0,1.0)
pl.Base=FreeCAD.Vector(0.0,0.0,0.0)
Draft.makeCircle(radius=30.0,placement=pl,face=True,support=None)

N = 20
x0 = N/2*10
y0 = N/2*10
smallR = 2.5
xoffset = 1.1475        # 1.1475
yoffset = 1.224         # 1.224
for m in range(N):
    for n in range(N):
        pl=FreeCAD.Placement()
        pl.Rotation.Q=(0.0,-0.0,-0.0,1.0)
        pl.Base=FreeCAD.Vector(x0-2*m*(smallR+xoffset),y0-2*n*(smallR+yoffset),0.0)
        Draft.makeCircle(radius=smallR,placement=pl,face=True,support=None)
