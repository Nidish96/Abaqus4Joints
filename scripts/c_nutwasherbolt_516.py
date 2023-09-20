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

mdl = mdb.models['Model-1']
# INCHES TO METERS CONVERSION CONSTANT
i2m = 0.0254;

###############
# PART : BOLT #
###############
mdl.ConstrainedSketch(name='__profile__', sheetSize=2.0)
sktch = mdl.sketches['__profile__']
sktch.ConstructionLine(angle=90.0, 
                       point1=(-i2m*0.25, i2m*0.0))
sktch.VerticalConstraint(addUndoState=
                         False, entity=sktch.geometry[2])
sktch.ConstructionLine(angle=90.0, 
                       point1=(i2m*0.25, i2m*0.0))
sktch.VerticalConstraint(addUndoState=
                         False, entity=sktch.geometry[3])
sktch.radialPattern(centerPoint=(i2m*0.0, 
                                 i2m*0.0), geomList=(sktch.geometry[2], 
                                                     sktch.geometry[3]), number=3, 
                    totalAngle=360.0, vertexList=())
sktch.Line(point1=(i2m*0.0, 
                   i2m*0.288675134594087), point2=(-i2m*0.25, i2m*0.144337567286129))
sktch.ParallelConstraint(addUndoState=
                         False, entity1=sktch.geometry[5], 
                         entity2=sktch.geometry[8])
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[0], entity2=
    sktch.geometry[5])
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[1], entity2=
    sktch.geometry[2])
sktch.Line(point1=(-i2m*0.25, 
                   i2m*0.144337567286129), point2=(-i2m*0.25, -i2m*0.144337567297043))
sktch.VerticalConstraint(addUndoState=
                         False, entity=sktch.geometry[9])
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[2], entity2=
    sktch.geometry[2])
sktch.Line(point1=(-i2m*0.25, 
                   -i2m*0.144337567297043), point2=(i2m*0.0, -i2m*0.288675134601363))
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[3], entity2=
    sktch.geometry[4])
sktch.Line(point1=(i2m*0.0, 
                   -i2m*0.288675134601363), point2=(i2m*0.25, -i2m*0.144337567304319))
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[4], entity2=
    sktch.geometry[3])
sktch.Line(point1=(i2m*0.25, 
                   -i2m*0.144337567304319), point2=(i2m*0.25, i2m*0.144337567297043))
sktch.VerticalConstraint(addUndoState=
                         False, entity=sktch.geometry[12])
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[5], entity2=
    sktch.geometry[3])
sktch.Line(point1=(i2m*0.25, 
                   i2m*0.144337567297043), point2=(i2m*0.0, i2m*0.288675134594087))
mdl.Part(dimensionality=THREE_D, name='BOLT', type=
         DEFORMABLE_BODY)

blt = mdl.parts['BOLT']
blt.BaseSolidExtrude(depth=i2m*0.203125, sketch=
                     sktch)
del sktch

mdl.ConstrainedSketch(gridSpacing=0.03, name='__profile__', 
                      sheetSize=1.58, transform=
                      blt.MakeSketchTransform(
                          sketchPlane=blt.faces[6], 
                          sketchPlaneSide=SIDE1, 
                          sketchUpEdge=blt.edges[0], 
                          sketchOrientation=RIGHT, origin=(i2m*0.0, i2m*0.0, i2m*0.203125)))
sktch = mdl.sketches['__profile__']
blt.projectReferencesOntoSketch(filter=
                                COPLANAR_EDGES, sketch=sktch)
sktch.CircleByCenterPerimeter(center=(
    i2m*0.0, i2m*0.0), point1=(i2m*0.15625, i2m*0.0))

blt.SolidExtrude(depth=i2m*1.5, 
                 flipExtrudeDirection=OFF, sketch=
                 sktch, sketchOrientation=RIGHT, 
                 sketchPlane=blt.faces[6], sketchPlaneSide=
                 SIDE1, sketchUpEdge=blt.edges[0])
del sktch

# Partition bolt shank at thread region (from calculations of other parts)
wtk = i2m*0.0625  # Washer thickness
btk = i2m  # Beam thickness
ntk = i2m*0.203125  # Nut thickness

blt.DatumPlaneByOffset(plane=blt.faces[8], flip=SIDE1, offset=2*wtk+btk)
dm1 = blt.datums.values()[0]
blt.DatumPlaneByOffset(plane=dm1, flip=SIDE1, offset=ntk)
dm2 = blt.datums.values()[1]

blt.PartitionCellByDatumPlane(cells=blt.cells, datumPlane=dm1)
blt.PartitionCellByDatumPlane(cells=blt.cells, datumPlane=dm2)

blt.PartitionCellByExtendFace(cells=blt.cells, extendFace=blt.faces[12])

# Assign Material
regn = blt.Set(cells=blt.cells, name='Set-1')
blt.SectionAssignment(region=regn, sectionName='Section-1')

#################
# PART : WASHER #
#################
mdl.ConstrainedSketch(name='__profile__', sheetSize=2.0)
sktch = mdl.sketches['__profile__']
sktch.CircleByCenterPerimeter(center=(
    i2m*0.0, i2m*0.0), point1=(i2m*0.171875, i2m*0.0))
sktch.CircleByCenterPerimeter(center=(
    i2m*0.0, i2m*0.0), point1=(i2m*0.34375, i2m*0.0))
mdl.Part(dimensionality=THREE_D, name='WASHER', type=
         DEFORMABLE_BODY)
wshr = mdl.parts['WASHER']
wshr.BaseSolidExtrude(depth=i2m*0.0625, sketch=
                      sktch)
del sktch

# Assign Material
regn = wshr.Set(cells=wshr.cells, name='Set-1')
wshr.SectionAssignment(region=regn, sectionName='Section-1')

##############
# PART : NUT #
##############
mdl.ConstrainedSketch(name='__profile__', sheetSize=2.0)
sktch = mdl.sketches['__profile__']
sktch.ConstructionLine(angle=90.0, 
                       point1=(-i2m*0.25, i2m*0.0))
sktch.VerticalConstraint(addUndoState=
                         False, entity=sktch.geometry[2])
sktch.ConstructionLine(angle=90.0, 
                       point1=(i2m*0.25, i2m*0.0))
sktch.VerticalConstraint(addUndoState=
                         False, entity=sktch.geometry[3])
sktch.radialPattern(centerPoint=(i2m*0.0, 
                                 i2m*0.0), geomList=(sktch.geometry[2], 
                                                     sktch.geometry[3]), number=3, 
                    totalAngle=360.0, vertexList=())
sktch.Line(point1=(i2m*0.0, 
                   i2m*0.288675134594087), point2=(-i2m*0.25, i2m*0.144337567286129))
sktch.ParallelConstraint(addUndoState=
                         False, entity1=sktch.geometry[5], 
                         entity2=sktch.geometry[8])
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[0], entity2=
    sktch.geometry[5])
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[1], entity2=
    sktch.geometry[2])
sktch.Line(point1=(-i2m*0.25, 
                   i2m*0.144337567286129), point2=(-i2m*0.25, -i2m*0.144337567297043))
sktch.VerticalConstraint(addUndoState=
                         False, entity=sktch.geometry[9])
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[2], entity2=
    sktch.geometry[2])
sktch.Line(point1=(-i2m*0.25, 
                   -i2m*0.144337567297043), point2=(i2m*0.0, -i2m*0.288675134601363))
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[3], entity2=
    sktch.geometry[4])
sktch.Line(point1=(i2m*0.0, 
                   -i2m*0.288675134601363), point2=(i2m*0.25, -i2m*0.144337567304319))
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[4], entity2=
    sktch.geometry[3])
sktch.Line(point1=(i2m*0.25, 
                   -i2m*0.144337567304319), point2=(i2m*0.25, i2m*0.144337567297043))
sktch.VerticalConstraint(addUndoState=
                         False, entity=sktch.geometry[12])
sktch.CoincidentConstraint(
    addUndoState=False, entity1=
    sktch.vertices[5], entity2=
    sktch.geometry[3])
sktch.Line(point1=(i2m*0.25, 
                   i2m*0.144337567297043), point2=(i2m*0.0, i2m*0.288675134594087))
mdl.Part(dimensionality=THREE_D, name='NUT', type=
         DEFORMABLE_BODY)
nut = mdl.parts['NUT']
nut.BaseSolidExtrude(depth=i2m*0.203125, sketch=
                     sktch)
del sktch

mdl.ConstrainedSketch(gridSpacing=0.03, name='__profile__', 
                      sheetSize=1.58, transform=
                      nut.MakeSketchTransform(
                          sketchPlane=nut.faces[6], 
                          sketchPlaneSide=SIDE1, 
                          sketchUpEdge=nut.edges[0], 
                          sketchOrientation=RIGHT, origin=(i2m*0.0, i2m*0.0, i2m*0.203125)))
sktch = mdl.sketches['__profile__']
nut.projectReferencesOntoSketch(filter=
                                COPLANAR_EDGES, sketch=sktch)
sktch.CircleByCenterPerimeter(center=(
    i2m*0.0, i2m*0.0), point1=(i2m*0.15625, i2m*0.0))
nut.CutExtrude(flipExtrudeDirection=OFF, sketch=
               sktch, sketchOrientation=RIGHT, 
               sketchPlane=nut.faces[6], sketchPlaneSide=
               SIDE1, sketchUpEdge=nut.edges[0])
del sktch

# Assign Material
regn = nut.Set(cells=nut.cells, name='Set-1')
nut.SectionAssignment(region=regn, sectionName='Section-1')
