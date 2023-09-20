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

#######################
# MATERIAL PROPERTIES #
#######################
mdl.Material(name='STEEL')
mdl.materials['STEEL'].Density(table=((7800.0, ), ))
mdl.materials['STEEL'].Elastic(table=((200000000000.0, 0.29), 
    ))
mdl.HomogeneousSolidSection(material='STEEL', name=
    'Section-1', thickness=None)
