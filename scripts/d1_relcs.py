# -*- coding: utf-8 -*-
#### 1. Preamble
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

from abaqus import *
from abaqusConstants import *
from caeModules import * 
import regionToolset
import job
import step
import sets

from inpParser import *

mdbm = openMdb('./model_step1a.cae')
mdl = mdbm.models['Model-1']
ras = mdl.rootAssembly

mdl.setValues(noPartsInputFile=ON)

#### 2. Get top coordinates
topsurf = ras.instances['TOPBEAM'].surfaces['INSURF']
topnodes = topsurf.nodes
N = len(topnodes)  # Number of nodes
# Top Nodes & Coordinates
Topnd_dict = dict(zip([topnodes[i].label for i in range(N)], range(N)))
TopNdCds = np.array([topnodes[i].coordinates for i in range(N)])

#### 3. Import Reference point instances
rpt = mdl.parts['REFPT']
for i in range(N):
    ras.Instance(name='RELPT-%d' %(i+1), part=rpt, dependent=OFF)
    ras.translate(instanceList=('RELPT-%d' %(i+1), ), vector=topnodes[i].coordinates)

#### 4. Create set, Simplify model, and save as inp file
ras.Set(name='RELCSET',
        referencePoints=[ras.instances['RELPT-%d' %(i+1)].referencePoints[1] for i in range(N)] )

tmp = mdl.interactions
while len(tmp) > 0:
    del tmp[tmp.keys()[-1]]

tmp = mdl.interactionProperties
while len(tmp) > 0:
    del tmp[tmp.keys()[-1]]

# Remove all steps except initial
tmp = mdl.steps
while len(tmp) > 1:
    del tmp[tmp.keys()[-1]]

#### 5. Create a Job and write an inp file
mdbm.Job(name="MeshedModel", model='Model-1')
mdbm.jobs['MeshedModel'].writeInput()

mdbm.close()
