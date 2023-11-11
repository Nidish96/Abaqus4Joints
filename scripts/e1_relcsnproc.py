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

mdl = mdb.ModelFromInputFile(name='Model-1',
                             inputFileName='./MeshedModel.inp')
ras = mdl.rootAssembly

#### 2. Sort bottom node set
topsurf = ras.surfaces['TOPBEAM_INSURF']
botsurf = ras.surfaces['BOTBEAM_INSURF']

topnodes = topsurf.nodes
botnodes = botsurf.nodes
N = len(topnodes)  # Number of nodes
Topnd_dict = dict(zip([topnodes[i].label for i in range(N)], range(N)))
TopNdCds = np.array([topnodes[i].coordinates for i in range(N)])

# Top Elements
TopEls = np.array([topsurf.elements[i].connectivity for i in range(len(topsurf.elements))])
ELS = np.zeros((TopEls.shape[0], 5), dtype=int)
for ne in range(TopEls.shape[0]):
    elefac = topsurf.elements[ne].getElemFaces()
    
    # Gives you the list of faces on the interface (we only expect a single face)
    fe = np.argwhere([all([Topnd_dict.has_key(x) for x in 
                           [elefac[k].getNodes()[i].label for i in range(4)]])
                      for k in range(6)])[0,0]
    ELS[ne, 0] = ne
    # Searches for the face where all the nodes are in the interface and returns those nodes
    ELS[ne, 1:] = [Topnd_dict[x] for x in [elefac[fe].getNodes()[k].label
                                           for k in range(4)]]
    ELS[ne, :] += 1

#### 5. Node Pairing. We assume len(botnodes)=len(topnodes).
botleft = range(N)
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
    
#### 6. Adjust Nodes on Bottom Beam Interface to Match Top Beam Exactly
for i in range(N):
    ras.editNode(nodes=botnodes[bts[i]:bts[i]+1],
                 coordinates=(topnodes[i].coordinates,))

#### 7. Create Node Sets
botpairednds = botnodes.sequenceFromLabels(tuple([botnodes[i].label for i in bts]))
# Reordering from the sorting above

ras.SetFromNodeLabels(name="TOPS_NDS", 
                      nodeLabels=((topnodes[0].instanceName, 
                                   tuple([topnodes[i].label for i in range(N)])),),
                      unsorted=True)
ras.SetFromNodeLabels(name="BOTS_NDS", 
                      nodeLabels=((botpairednds[0].instanceName, 
                                   [botpairednds[i].label for i in range(N)]),),
                      unsorted=True)
rlcn = ras.sets['RELCSET'].nodes
ras.SetFromNodeLabels(name="RELCSET",
                      nodeLabels=((rlcn[0].instanceName,
                                   [rlcn[i].label for i in range(N)]),),
                      unsorted=True)

#### 8. Equation Constraints for relative coordinates
for i in range(3):
    mdl.Equation(name='RELCS-%d' %(i+1),
                 terms=((1.0, 'TOPS_NDS', i+1),
                        (-1.0, 'RELCSET', i+1),
                        (-1.0, 'BOTS_NDS', i+1)))


#### 9. Create a Frequency Step for fixed interface modal analysis
mdl.FrequencyStep(name="Fixed-Int-Modal", previous="Initial",
                  normalization=MASS, eigensolver=LANCZOS,
                  numEigen=60)
mdl.EncastreBC(name="RELFIX", createStepName="Fixed-Int-Modal",
               region=ras.sets['RELCSET'])
    
#### 10. Create a substructuring step, specify the modes and retained DOFs
mdl.SubstructureGenerateStep(name="HCBCMS", previous="Fixed-Int-Modal",
                             substructureIdentifier=1, 
                             retainedEigenmodesMethod=MODE_RANGE, modeRange=((1, 26, 1),),
                             recoveryMatrix=REGION, recoveryRegion=ras.sets['OUTNODES'],
                             computeReducedMassMatrix=True)

mdl.RetainedNodalDofsBC(name="A", createStepName="HCBCMS",
                        region=ras.sets['RELCSET'],
                        u1=ON, u2=ON, u3=ON)

# Apply Bolt Loads (1N magnitude)
for i in range(1, 4):
    mdl.ConcentratedForce(name='BoltLoad-%d' %(i), createStepName="HCBCMS",
                          cf3=1.0, region=ras.sets['BPT-%d_SET-1' %(i)])
    mdl.ConcentratedForce(name='NutLoad-%d' %(i), createStepName="HCBCMS",
                          cf3=-1.0, region=ras.sets['NPT-%d_SET-1' %(i)])

sbs = mdl.steps['HCBCMS']
sbs.LoadCase(name="LCASE", loads=tuple(('BoltLoad-%d' %(i), 1.0) for i in range(1, 4)) +
             tuple(('NutLoad-%d' %(i), 1.0) for i in range(1, 4)))

#### 11. Request substructure matrix outputs
# ABAQUS CAE doesn't support this yet (GUI or scripting),
# so the keywords need to be manually modified.
mdl.keywordBlock.synchVersions(storeNodesAndElements=False)
li = np.argwhere([mdl.keywordBlock.sieBlocks[i][0:20] == "*Retained Nodal Dofs" for i in range(len(mdl.keywordBlock.sieBlocks))])[0][0]
txi = mdl.keywordBlock.sieBlocks[li]
mdl.keywordBlock.replace(li, "*Retained Nodal Dofs, sorted=NO"+txi[20:])
mdl.keywordBlock.insert(len(mdl.keywordBlock.sieBlocks)-2, 
                        "*Substructure Matrix Output, FILE NAME=Modelmats, MASS=YES, STIFFNESS=YES, SLOAD=YES, RECOVERY MATRIX=YES")

#### 12. Create a job and write an inp file
mdb.Job(name="HCBCMSJob", model='Model-1')
mdb.jobs['HCBCMSJob'].writeInput()

# Save interfacial nodes & elements to txt files
np.savetxt('Nodes.dat', TopNdCds) # Save to dat file
np.savetxt('Elements.dat', ELS, fmt='%d')
