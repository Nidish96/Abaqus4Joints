#-*- coding: utf-8 -*-
import sys
import numpy as np

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
ras = mdl.rootAssembly


for i in range(1, 4):
    # Bolt-Washer Constraints
    mdl.Tie(name='BW-%d' %(i), main=ras.instances['TOPWASHER-%d' %(i)].surfaces['WSTOP'],
            secondary=ras.instances['BOLT-%d' %(i)].surfaces['BlW'],
            positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)

    # Nut-Washer Constraints
    mdl.Tie(name='NW-%d' %(i), main=ras.instances['BOTWASHER-%d' %(i)].surfaces['WSBOT'],
            secondary=ras.instances['NUT-%d' %(i)].surfaces['NW'],
            positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)

    # TopWasher-Beam Constraints
    mdl.Tie(name='BmTW-%d' %(i), main=ras.instances['TOPBEAM'].surfaces['BmW%d' %(i)],
            secondary=ras.instances['TOPWASHER-%d' %(i)].surfaces['WSBOT'],
            positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)
    
    # BotWasher-Beam Constraints
    mdl.Tie(name='BmBW-%d' %(i), main=ras.instances['BOTBEAM'].surfaces['BmW%d' %((3-i)%3+1)],
            secondary=ras.instances['BOTWASHER-%d' %(i)].surfaces['WSTOP'],
            positionToleranceMethod=COMPUTED, adjust=ON, tieRotations=ON, thickness=ON)
            

# Bolt & Nut Point Coupling
for i in range(1, 4):
    mdl.Coupling(name='BPC-%d' %(i), controlPoint=ras.instances['BPT-%d' %(i)].sets['Set-1'],
                 surface=ras.instances['BOLT-%d' %(i)].surfaces['BN'],
                 influenceRadius=WHOLE_SURFACE, couplingType=STRUCTURAL,
                 weightingMethod=UNIFORM)
    
    mdl.Coupling(name='NPC-%d' %(i), controlPoint=ras.instances['NPT-%d' %(i)].sets['Set-1'],
                 surface=ras.instances['NUT-%d' %(i)].surfaces['NB'],
                 influenceRadius=WHOLE_SURFACE, couplingType=STRUCTURAL,
                 weightingMethod=UNIFORM)

# Equation Constraints constraining bolt and nut ref-points to each other
for i in range(1, 4):
    for j in [1, 2, 4, 5, 6]:
        mdl.Equation(name='BNEQ%d-%d' %(i, j),
                     terms=((1.0, 'BPT-%d.Set-1' %(i), j),
                            (-1.0,'NPT-%d.Set-1' %(i), j)))

# Create static prestress step
mdl.StaticStep(name='PRESTRESS', previous='Initial', nlgeom=ON)

# Apply forces
bpmag = 12e3  # 12kN bolt-load
for i in range(1, 4):
    # Force in +z on bolt-points
    mdl.ConcentratedForce(name='BoltLoad-%d' %(i), createStepName='PRESTRESS',
                          region=ras.instances['BPT-%d' %(i)].sets['Set-1'],
                          cf3=bpmag, distributionType=UNIFORM)
    
    # Force in -z on bolt-points
    mdl.ConcentratedForce(name='NutLoad-%d' %(i), createStepName='PRESTRESS',
                          region=ras.instances['NPT-%d' %(i)].sets['Set-1'],
                          cf3=-bpmag, distributionType=UNIFORM)
