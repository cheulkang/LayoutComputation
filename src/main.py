import os
import sys

# load a project path to Blender
path = ""

dir = os.path.dirname(path + "src/")
if not dir in sys.path:
    sys.path.append(dir)

from read_data import Parser
from relationship import Relationship
from data import Data
from optimization import Optimization

# extraction
d = []
for i in range(1, 5):
    prior_data = Parser()
    prior_data._parserData(path + "/positive_examples/room_" + str(i) + ".txt")
    obj_relation = Relationship(prior_data.model, prior_data.pair)
    d.append(obj_relation)

# integrate prior data
data = Data()
data._integrateData(d)

# optimization
op = Optimization(data)

# Set unfeasible space for a robot
op._setUnfeasibleSpace()

# Find unfeasible objects
op._findUnfeasibleObjects()

# Set the parent for object.
op._setHierarchicalRelationship()

# d1, d2 fixed
op._initializedLocation()

# initialize -> cost
op._optimization()