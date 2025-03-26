# -*- coding: utf-8 -*-
# 1. Preamble
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

# mdbm = openMdb('./model_step1a.cae')
# mdl = mdbm.models['Model-1']
# ras = mdl.rootAssembly

mdl = mdb.models['Model-1']
ras = mdl.rootAssembly

mdl.setValues(noPartsInputFile=ON)

# 2. Get top & bottom surfaces & nodes
topsurf = ras.instances['TOPBEAM'].surfaces['INSURF']
botsurf = ras.instances['BOTBEAM'].surfaces['INSURF']

topnodes = topsurf.nodes
botnodes = botsurf.nodes
N = len(topnodes)  # Number of nodes

# Top Nodes & Coordinates
Topnd_dict = dict(zip([topnodes[i].label for i in range(N)], range(N)))
# maps original node ID (in FE model) to node ID in interface node set
TopNdCds = np.array([topnodes[i].coordinates for i in range(N)])

# Top Elements
TopEls = np.array([topsurf.elements[i].connectivity for i in range(len(topsurf.elements))])
ELS = np.zeros((TopEls.shape[0], 5), dtype=int)
for ne in range(TopEls.shape[0]):
    elefac = topsurf.elements[ne].getElemFaces()
    # Gives you the list of faces on the interface (we only expect a single face)
    fe = np.argwhere([all([x in Topnd_dict.keys() for x in
                           [elefac[k].getNodes()[i].label for i in range(4)]])
                      for k in range(6)])[0,0]
    ELS[ne, 0] = ne
    # Searches for the face where all the nodes are in the interface and returns those nodes
    ELS[ne, 1:] = [Topnd_dict[x] for x in [elefac[fe].getNodes()[k].label
                                           for k in range(4)]]
    ELS[ne, :] += 1

# Save interfacial nodes & elements to txt files
np.savetxt('Nodes.dat', TopNdCds) # Save to dat file
np.savetxt('Elements.dat', ELS, fmt='%d')

# 3. Node Pairing. We assume len(botnodes)=len(topnodes).
botleft = list(range(N))
bts = []
tmi = 0
for i in range(N):
    # Calculates deviation of selected node coordinate on bottom to each
    # node coordinate on top and "assigns" the closest one to the index.
    bts.append(
        botleft.pop(
            np.argmin(
                np.linalg.norm(
                    topnodes[i].coordinates-np.array([botnodes[j].coordinates for j in botleft]),
                    axis=1)
            )
        )
    )

# 4. Adjust Nodes on Bottom Beam Interface to Match Top Beam Exactly
for i in range(N):
    ras.editNode(nodes=botnodes[bts[i]:bts[i]+1],
                 coordinates=(topnodes[i].coordinates,))

# 5. Create Node Sets
botpairednds = botnodes.sequenceFromLabels(tuple([botnodes[i].label for i in bts]))
# Reordering from the sorting above

ras.SetFromNodeLabels(name="TOPS_NDS",
                      nodeLabels=((topnodes[0].instanceName,
                                   tuple([topnodes[i].label for i in range(N)])),),
                      unsorted=True)
ras.SetFromNodeLabels(name="BOTS_NDS",
                      nodeLabels=((botpairednds[0].instanceName,
                                   [botpairednds[i].label for i in range(len(botpairednds))]),),
                      unsorted=True)

# 6. Simplify model (remove interactions, all steps, etc.)
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

# 7. Create a Frequency Step for fixed interface modal analysis
mdl.FrequencyStep(name="Fixed-Int-Modal", previous="Initial",
                  normalization=MASS, eigensolver=LANCZOS,
                  numEigen=60)
mdl.EncastreBC(name="TOPFIX", createStepName="Fixed-Int-Modal",
               region=ras.sets['TOPS_NDS'])
mdl.EncastreBC(name="BOTFIX", createStepName="Fixed-Int-Modal",
               region=ras.sets['BOTS_NDS'])

# 8. Create a substructuring step, specify the modes and retained DOFs
mdl.SubstructureGenerateStep(name="HCBCMS", previous="Fixed-Int-Modal",
                             substructureIdentifier=1,
                             retainedEigenmodesMethod=MODE_RANGE, modeRange=((1, 20, 1),),
                             recoveryMatrix=REGION, recoveryRegion=ras.sets['OutNodes'],
                             computeReducedMassMatrix=True)

# The name dictates ordering. A comes before B.
mdl.RetainedNodalDofsBC(name="A", createStepName="HCBCMS",
                        region=ras.sets['TOPS_NDS'],
                        u1=ON, u2=ON, u3=ON)
mdl.RetainedNodalDofsBC(name="B", createStepName="HCBCMS",
                        region=ras.sets['BOTS_NDS'],
                        u1=ON, u2=ON, u3=ON)

# Apply Bolt Loads (1N magnitude)
for i in range(1, 4):
    mdl.ConcentratedForce(name='BoltLoad-%d' %(i), createStepName="HCBCMS",
                          cf3=1.0, region=ras.instances['BPT-%d' %(i)].sets['Set-1'])
    mdl.ConcentratedForce(name='NutLoad-%d' %(i), createStepName="HCBCMS",
                          cf3=-1.0, region=ras.instances['NPT-%d' %(i)].sets['Set-1'])

sbs = mdl.steps['HCBCMS']
sbs.LoadCase(name="LCASE", loads=tuple(('BoltLoad-%d' %(i), 1.0) for i in range(1, 4)) +
             tuple(('NutLoad-%d' %(i), 1.0) for i in range(1, 4)))

# 9. Request substructure matrix outputs
# ABAQUS CAE doesn't support this yet (GUI or scripting),
# so the keywords need to be manually modified.
mdl.keywordBlock.synchVersions(storeNodesAndElements=False)
li = np.argwhere([mdl.keywordBlock.sieBlocks[i][0:20] == "*Retained Nodal Dofs" for i in range(len(mdl.keywordBlock.sieBlocks))])[0][0]
txi = mdl.keywordBlock.sieBlocks[li]
mdl.keywordBlock.replace(li, "*Retained Nodal Dofs, sorted=NO"+txi[20:])
mdl.keywordBlock.insert(len(mdl.keywordBlock.sieBlocks)-2,
                        "*Substructure Matrix Output, FILE NAME=Modelmats, MASS=YES, STIFFNESS=YES, SLOAD=YES, RECOVERY MATRIX=YES")

# 10. Create a job and write an inp file
mdb.Job(name="Job", model='Model-1')
mdb.jobs['Job'].writeInput()
