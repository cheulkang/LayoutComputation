import numpy as np
import common_function as cf

class Spatial:
    def __init__(self, Z, rot, d1, d2, s1_o, s1_p, s2_p, theta1):
        self.Z = Z
        self.rot = rot
        self.d1 = d1
        self.d2 = d2
        self.s1_o = s1_o
        self.s1_p = s1_p
        self.s2_p = s2_p
        self.theta1 = theta1

class Pairwise:
    def __init__(self, d, theta, member):
        self.d = d
        self.theta = theta
        self.member = member

class Relationship:
    def __init__(self, model, pair):
        self.model = model
        self.surface = self._setSurfaceLabel()
        self._getSpatial()
        self.pairwise = self._getPairwise(pair)

    def _getSpatial(self):
        def _isNeighbor(a, b):
            for i in a:
                for j in b:
                    if(i == j):
                        return True

            return False

        for m in self.model:
            # model
            model = self.model[m]

            # skip when object is a room or furniture
            if(model.parent == "None" or model.parent == "room"):
                continue

            # parent of model
            model_parent = self.model[model.parent]

            # Z
            Z = np.min(model.bb_corners, axis=0)[2]
            rot = model.rot

            # find the top surface of model
            model_top = cf._getTopSurface(model.bb_corners)
            # find the top surface of model's parent
            parent_top = cf._getTopSurface(model_parent.bb_corners)

            # calculate c (center)
            c = cf._calculateCenterPoint(model.bb_corners, model_top)
            # calculate b (diagonal)
            b = cf._calculateL2Distance(c, model.bb_corners[0][0:2])

            # Distinguish diagonals
            diags = cf._seperateDiagonalsRect(model.bb_corners, model_top)
            diags_parent = cf._seperateDiagonalsRect(model_parent.bb_corners, parent_top)

            # Calculate the distances for each surfaces
            parent_point = []
            for i in diags_parent[0]:
                for j in diags_parent[1]:
                    distance = cf._calculateDistanceOfLineAndPoint(model_parent.bb_corners[i], model_parent.bb_corners[j], c)
                    parent_point.append(([i, j], distance))

            # Sort distance
            sorted_distance = sorted(parent_point, key=lambda x: x[1])
            nearest_parent_point = sorted_distance[0][0]

            ### calculate d1 (from center(c) to the nearest parent's surface)
            ### calculate d2 (from center(c) to the second nearest parent's surface and neighbor)
            d1 = sorted_distance[0][1]
            if(_isNeighbor(sorted_distance[0][0], sorted_distance[1][0])):
                nearest_parent_point2 = sorted_distance[1][0]
                d2 = sorted_distance[1][1]
            else:
                nearest_parent_point2 = sorted_distance[2][0]
                d2 = sorted_distance[2][1]

            # get Back_surface
            h = cf._calculateFootofPerpendicular(model_parent.bb_corners[nearest_parent_point[0]], model_parent.bb_corners[nearest_parent_point[1]], c)
            for i in diags[0]:
                for j in diags[1]:
                    if (cf._checkCross(c, h, model.bb_corners[i], model.bb_corners[j])):
                        s1_o = cf._getSurfaceLabel(cf._getSurfacePoint(model.bb_corners, i, j), self.surface)
                        backSurface_point = [i, j]

            # get Parent_surface
            s1_p = cf._getSurfaceLabel(cf._getSurfacePoint(model_parent.bb_corners, nearest_parent_point[0], nearest_parent_point[1]), self.surface)
            s2_p = cf._getSurfaceLabel(cf._getSurfacePoint(model_parent.bb_corners, nearest_parent_point2[0], nearest_parent_point2[1]), self.surface)

            # calculate theta1
            theta1 = cf._calculateTheta([model_parent.bb_corners[nearest_parent_point[0]][0:2], model_parent.bb_corners[nearest_parent_point[1]][0:2]], [model.bb_corners[backSurface_point[0]][0:2], model.bb_corners[backSurface_point[1]][0:2]])

            model.spatial = Spatial(Z, rot, d1, d2, s1_o, s1_p, s2_p, theta1)

    def _setSurfaceLabel(self):
        label = []

        label.append([2, 3, 6, 7]) # surface 1
        label.append([4, 5, 6, 7]) # surface 2
        label.append([0, 1, 4, 5]) # surface 3
        label.append([0, 1, 2, 3]) # surface 4
        label.append([1, 2, 5, 6]) # surface 5
        label.append([0, 3, 4, 7]) # surface 6

        return label

    def _getPairwise(self, pair):
        pairwise = []
        for g in pair:
            d = cf._calculateL2Distance(self.model[g[0]].loc, self.model[g[1]].loc )
            theta = cf._calculateTheta([self.model[g[0]].loc[0:2], self.model[g[1]].loc[0:2]], [self.model[g[0]].loc[0:2], [self.model[g[0]].loc[1], self.model[g[1]].loc[0]]])
            member = g
            pairwise.append(Pairwise(d, theta, member))

        return pairwise