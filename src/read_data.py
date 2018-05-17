import numpy as np
import common_function as cf

class Model:
    "Store the information of model"
    def __init__(self, parent, loc, rot, bb, mat):
        self.parent = parent
        self.loc = loc
        self.rot = rot
        self.bb = bb
        self.mat = mat
        self.bb_corners = cf.getBoundBoxCorners(self.mat, self.bb)
        self.fixed = True if (self.parent == 0 or self.parent == "None") else False
        self.spatial = []

    def _getSurface(self):
        surface = np.array([i+1 for i in range(6)])
        for x in range(self._getNumOfRotate(self.rot[0])):
            surface = self._rotateX(surface)
        for y in range(self._getNumOfRotate(self.rot[1])):
            surface = self._rotateY(surface)
        for z in range(self._getNumOfRotate(self.rot[2])):
            surface = self._rotateZ(surface)
        return surface

    def _rotateX(self, surface):
        temp = surface[4]
        surface[4] = surface[0]
        surface[0] = surface[5]
        surface[5] = surface[2]
        surface[2] = temp
        return surface

    def _rotateY(self, surface):
        temp = surface[4]
        surface[4] = surface[1]
        surface[1] = surface[5]
        surface[5] = surface[4]
        surface[4] = temp
        return surface

    def _rotateZ(self, surface):
        temp = surface[0]
        surface[0] = surface[1]
        surface[1] = surface[2]
        surface[2] = surface[3]
        surface[3] = temp
        return surface

    def _getNumOfRotate(self, rot):
        n = round(rot / (np.pi / 2))
        if(n < 0):
            return int(n + 4)
        else:
            return int(n)

class Parser:
    def __init__(self):
        self.model = {}
        self.pair = []

    def _parserData(self, filepath):
        with open(filepath, 'r') as f:
            for line in f:
                d = line.split()
                if(d[0] == "Model"):
                    name = d[1]
                elif(d[0] == "Parent"):
                    parent = d[1]
                elif(d[0] == "Location"):
                    loc = np.array([float(d[1]), float(d[2]), float(d[3])])
                elif(d[0] == "Rotation"):
                    rot = np.array([float(d[1]), float(d[2]), float(d[3])])
                elif(d[0] == "BoundBox"):
                    bb = [np.array([float(d[i*3+1].strip('(')), float(d[i*3+2]), float(d[i*3+3].strip(')'))]) for i in range(8)]
                elif(d[0] == "Matrix"):
                    mat = np.array([float(d[i + 1]) for i in range(16)]).reshape(4,4)
                    self.model[name] = Model(parent, loc, rot, bb, mat)
                elif(d[0] == "Pairwise"):
                    member = [d[1], d[2]]
                    self.pair.append(member)

        return self.model