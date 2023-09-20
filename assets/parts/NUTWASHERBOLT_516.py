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

mdl.parts['BOLT'].BaseSolidExtrude(depth=i2m*0.203125, sketch=
    sktch)
del sktch

mdl.ConstrainedSketch(gridSpacing=0.03, name='__profile__', 
    sheetSize=1.58, transform=
    mdl.parts['BOLT'].MakeSketchTransform(
    sketchPlane=mdl.parts['BOLT'].faces[6], 
    sketchPlaneSide=SIDE1, 
    sketchUpEdge=mdl.parts['BOLT'].edges[0], 
    sketchOrientation=RIGHT, origin=(i2m*0.0, i2m*0.0, i2m*0.203125)))
sktch = mdl.sketches['__profile__']
mdl.parts['BOLT'].projectReferencesOntoSketch(filter=
    COPLANAR_EDGES, sketch=sktch)
sktch.CircleByCenterPerimeter(center=(
    i2m*0.0, i2m*0.0), point1=(i2m*0.15625, i2m*0.0))

mdl.parts['BOLT'].SolidExtrude(depth=i2m*1.5, 
    flipExtrudeDirection=OFF, sketch=
    sktch, sketchOrientation=RIGHT, 
    sketchPlane=mdl.parts['BOLT'].faces[6], sketchPlaneSide=
    SIDE1, sketchUpEdge=mdl.parts['BOLT'].edges[0])
del sktch

# Assign Material

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
mdl.parts['WASHER'].BaseSolidExtrude(depth=i2m*0.0625, sketch=
    sktch)
del sktch

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
mdl.parts['NUT'].BaseSolidExtrude(depth=i2m*0.203125, sketch=
    sktch)
del sktch

mdl.ConstrainedSketch(gridSpacing=0.03, name='__profile__', 
    sheetSize=1.58, transform=
    mdl.parts['NUT'].MakeSketchTransform(
        sketchPlane=mdl.parts['NUT'].faces[6], 
    sketchPlaneSide=SIDE1, 
    sketchUpEdge=mdl.parts['NUT'].edges[0], 
    sketchOrientation=RIGHT, origin=(i2m*0.0, i2m*0.0, i2m*0.203125)))
sktch = mdl.sketches['__profile__']
mdl.parts['NUT'].projectReferencesOntoSketch(filter=
    COPLANAR_EDGES, sketch=sktch)
sktch.CircleByCenterPerimeter(center=(
    i2m*0.0, i2m*0.0), point1=(i2m*0.15625, i2m*0.0))
mdl.parts['NUT'].CutExtrude(flipExtrudeDirection=OFF, sketch=
    sktch, sketchOrientation=RIGHT, 
    sketchPlane=mdl.parts['NUT'].faces[6], sketchPlaneSide=
    SIDE1, sketchUpEdge=mdl.parts['NUT'].edges[0])
del sktch
