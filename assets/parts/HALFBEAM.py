# -*- coding: utf-8 -*-
import sys

from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *

i2m = 25.4e-3

mdl = mdb.models['Model-1']

###################
# PART : HALFBEAM #
###################
# 1. Sketch and Extrude
mdl.ConstrainedSketch(name='__profile__', sheetSize=2.0)
sktch = mdl.sketches['__profile__']
sktch.Line(point1=(-36e-2, 1.27e-2), point2=(-6e-2, 1.27e-2))
sktch.Line(point1=(-6e-2, 1.27e-2), point2=(-6e-2, 0))
sktch.Line(point1=(-6e-2, 0), point2=(6e-2,0))
sktch.Line(point1=(6e-2,0), point2=(6e-2,-1.27e-2))
sktch.Line(point1=(6e-2,-1.27e-2), point2=(-36e-2,-1.27e-2))
sktch.Line(point1=(-36e-2,-1.27e-2), point2=(-36e-2,1.27e-2))

mdl.Part(dimensionality=THREE_D, name='HALFBEAM', type=DEFORMABLE_BODY)
hfbm = mdl.parts['HALFBEAM']
hfbm.BaseSolidExtrude(depth=25.4e-3, sketch=sktch)
del sktch

# 2. Cut out Holes
mdl.ConstrainedSketch(name='__profile__', sheetSize=2.0,
                      gridSpacing=30e-3, transform=
                      hfbm.MakeSketchTransform(
                          sketchPlane=hfbm.faces[2],
                          sketchPlaneSide=SIDE1,
                          sketchUpEdge=hfbm.edges[8],
                          sketchOrientation=RIGHT,
                          origin=(0.0, 0.0, 1.27e-2)))
sktch = mdl.sketches['__profile__']
cs = [-3e-2, 0.0, 3e-2];
for i in range(3):
    sktch.CircleByCenterPerimeter(center=(cs[i], 0),
                                  point1=(cs[i]+0.85e-2/2, 0))

hfbm.CutExtrude(sketchPlane=hfbm.faces[2], sketchPlaneSide=SIDE1,
                sketchUpEdge=hfbm.edges[8],
                sketchOrientation=RIGHT, sketch=sktch)
del sktch

# 3. Partition object
hfbm.PartitionCellByExtendFace(cells=hfbm.cells,
                               extendFace=hfbm.faces[6])
hfbm.PartitionCellByExtendFace(cells=hfbm.cells,
                               extendFace=hfbm.faces[7])

mdl.ConstrainedSketch(name='__profile__', sheetSize=2.0,
                      gridSpacing=30e-3, transform=
                      hfbm.MakeSketchTransform(
                          sketchPlane=hfbm.faces[11],
                          sketchPlaneSide=SIDE1,
                          sketchUpEdge=hfbm.edges[27],
                          origin=(0.0, 0.0, 1.27e-2)))
sktch = mdl.sketches['__profile__']
cs = [-3e-2, 0.0, 3e-2];
wor = i2m*0.34375
for i in range(3):
    sktch.CircleByCenterPerimeter(center=(cs[i], 0),
                                  point1=(cs[i]+wor, 0))
hfbm.PartitionCellBySketch(cells=hfbm.cells, sketch=sktch,
                           sketchUpEdge=hfbm.edges[27],
                           sketchPlane=hfbm.faces[11])
hfbm.PartitionCellByExtrudeEdge(cells=hfbm.cells, edges=hfbm.edges[0],
                                line=hfbm.edges[-1], sense=FORWARD)
hfbm.PartitionCellByExtrudeEdge(cells=hfbm.cells, edges=hfbm.edges[3],
                                line=hfbm.edges[-1], sense=FORWARD)
hfbm.PartitionCellByExtrudeEdge(cells=hfbm.cells, edges=hfbm.edges[6],
                                line=hfbm.edges[-1], sense=FORWARD)

pt = hfbm.InterestingPoint(edge=hfbm.edges[-2], rule=MIDDLE)
hfbm.PartitionCellByPlanePointNormal(cells=hfbm.cells, point=pt, normal=hfbm.edges[-2])

pt = hfbm.InterestingPoint(edge=hfbm.edges[75], rule=MIDDLE)
hfbm.PartitionCellByPlanePointNormal(cells=hfbm.cells, point=pt, normal=hfbm.edges[75])

pt = hfbm.InterestingPoint(edge=hfbm.edges[39], rule=MIDDLE)
hfbm.PartitionCellByPlanePointNormal(cells=hfbm.cells, point=pt, normal=hfbm.edges[39])

pt = hfbm.InterestingPoint(edge=hfbm.edges[85], rule=MIDDLE)
hfbm.PartitionCellByPlanePointNormal(cells=hfbm.cells, point=pt, normal=hfbm.edges[85])

pt = hfbm.InterestingPoint(edge=hfbm.edges[39], rule=MIDDLE)
hfbm.PartitionCellByPlanePointNormal(cells=hfbm.cells, point=pt, normal=hfbm.edges[39])

pt = hfbm.InterestingPoint(edge=hfbm.edges[41], rule=MIDDLE)
hfbm.PartitionCellByPlanePointNormal(cells=hfbm.cells, point=pt, normal=hfbm.edges[41])

pt = hfbm.InterestingPoint(edge=hfbm.edges[111], rule=MIDDLE)
hfbm.PartitionCellByPlanePointNormal(cells=hfbm.cells, point=pt, normal=hfbm.edges[111])

pt = hfbm.InterestingPoint(edge=hfbm.edges[135], rule=MIDDLE)
hfbm.PartitionCellByPlanePointNormal(cells=hfbm.cells, point=pt, normal=hfbm.edges[135])

# 4. Assign Material
regn = hfbm.Set(cells=hfbm.cells, name='Set-1')
hfbm.SectionAssignment(region=regn, sectionName='Section-1')

session.viewports['Viewport: 1'].setValues(displayedObject=hfbm)

