import bpy
from mathutils import Vector
from random import randint
import copy

import numpy as np
import common_function as cf

class Cost:
    def __init__(self):
        self.visibility = 0
        self.accessibility = 0
        self.prior_d1 = 0
        self.prior_d2 = 0
        self.prior_theta1 = 0
        self.pair_d = 0
        self.pair_theta = 0
        self.total = 0

    def _getTotalCost(self):
        self.total = self.accessibility + self.visibility + self.prior_d1 + self.prior_d2 + self.prior_theta1 + self.pair_d + self.pair_theta
        return self.total

class Optimization:
    def __init__(self, data):
        self.data = data
        self.surface = self._setSurfaceLabel()
        self.cost = {}

        self.Wv = 20
        self.Wa = 0.1
        self.Wpr_d1 = 3
        self.Wpr_d2 = 2
        self.Wpr_theta1 = 3
        self.Wpair_d = 0.1
        self.Wpair_theta = 0.2

    def _setSurfaceLabel(self):
        label = []

        label.append([2, 3, 6, 7]) # surface 1
        label.append([4, 5, 6, 7]) # surface 2
        label.append([0, 1, 4, 5]) # surface 3
        label.append([0, 1, 2, 3]) # surface 4
        label.append([1, 2, 5, 6]) # surface 5
        label.append([0, 3, 4, 7]) # surface 6

        return label

    def _changeParent(self, c, p):
        bpy.ops.object.select_all(action='DESELECT')
        c.select = True
        p.select = True

        bpy.context.scene.objects.active = p
        bpy.ops.object.parent_set()

        c.location[0:2] = p.location[0:2]

    def _getSamePoint(self, top, surface):
        point = []
        for p in top:
            for s in surface:
                if (p == s):
                    point.append(s)

        return point

    def checkInclusion(self, model):
        m_loc = model.location

        for key, value in self.unfeasibleFurniture.items():
            if(key != model.parent.name):
                continue

            if(value[0][0] < m_loc[0] < value[1][0] and value[0][1] < m_loc[1] < value[1][1]):
                return True

        return False

    def _setUnfeasibleSpace(self):
        self.unfeasibleFurniture = {}

        self.unfeasibleFurniture['bed'] = [[-2.0, -2.0], [-1.15, -0.4]]
        self.unfeasibleFurniture['desk'] = [[-0.25, 1.6], [0.25, 2.0]]

    def _findUnfeasibleObjects(self):
        self.unfeasibleObjects = []

        for m in bpy.data.objects:
            model_name = m.name

            if (model_name == "room" or model_name.split('.')[0] == "viewing" or model_name == "Camera" or model_name == "Lamp" or m.parent.name == "room"):
                continue

            # check inclusion
            if(self.checkInclusion(m)):
                self.unfeasibleObjects.append(m.name)

    def _setHierarchicalRelationship(self):
        # check hierarchical relationship in learned_data
        for m in bpy.data.objects:
            model_name = m.name

            if (model_name == "room" or model_name.split('.')[0] == "viewing" or model_name == "Camera" or model_name == "Lamp" or m.parent.name == "room"):
                continue

            if model_name in self.unfeasibleObjects:
                continue

            # getParentName
            data_parent_name = self.data._getParent(model_name)
            #print(data_parent_name)
            data_model = self.data.objects[model_name+"_"+data_parent_name]

            # Parent setting
            if(m.parent.name != data_parent_name):
                # transfer object
                m_parent = bpy.data.objects[data_parent_name]
                self._changeParent(m, m_parent)
                # Search available space (X, Y)
                self._checkAvailableSpace(m, m_parent)
                bpy.context.scene.update()

            # rotation setting (todo: erase the rot)
            m.rotation_euler[0] = data_model.spatial[0].rot[0]
            m.rotation_euler[1] = data_model.spatial[0].rot[1]
            bpy.context.scene.update()

            # calculate Z index
            bbox_corners = [m.matrix_world * Vector(corner) for corner in m.bound_box]
            m.location[2] = data_model.spatial[0].Z + (m.location[2] - np.min(bbox_corners, axis=0)[2])
            bpy.context.scene.update()
            ### bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)

    def _checkAvailableSpace(self, model, parent):
        for i in range(100):
            collsion = False

            # save model's location and rotation
            loc = model.location
            rot = model.rotation_euler

            # Move object (x,y)
            self._moveObject(model)
            # Rotate object (z)
            self._rotateObject(model, 0)

            # collision check
            for c in model.parent.children:
                if(c == model):
                    continue
                if(cf._checkCollision(model, c) == True): # collision
                    model.location = loc
                    model.rotation_euler = rot
                    bpy.context.scene.update()
                    collsion = True
                    break

            # if not collision
            if(collsion == False):
                return

        if (parent.name.split('.')[0] == model.name.split('.')[0]):
            model.location[0] = parent.location[0]
            model.location[1] = parent.location[1]
            model.rotation_euler[0] = parent.rotation_euler[0]
            model.rotation_euler[1] = parent.rotation_euler[1]
            model.rotation_euler[2] = parent.rotation_euler[2]
            bpy.context.scene.update()
            ### bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)

    def _initializedLocation(self):
        def check_axis(points, bb_corner):
            if (np.abs( bb_corner[points[0]][0] - bb_corner[points[1]][0] ) <= 0.01):
                return 1
            else:
                return 0

        for m in bpy.data.objects:
            model_name = m.name

            if (model_name == "room" or model_name.split('.')[0] == "viewing" or model_name == "Camera" or model_name == "Lamp" or m.parent.name == "room"):
                continue
            if model_name in self.unfeasibleObjects:
                continue

            surface = self.surface
            m_bb_corners = cf._getWorldBoundingBox(m)
            m_parent_bb_corners = cf._getWorldBoundingBox(m.parent)

            model_top = cf._getTopSurface(m_bb_corners)
            parent_top = cf._getTopSurface(m_parent_bb_corners)

            name = m.name + "_" + m.parent.name

            s = self.data.objects[name].spatial[0]

            #### find the nearest surface
            parent_point = self._getSamePoint(parent_top, surface[s.s1_p])
            parent_point2 = self._getSamePoint(parent_top, surface[s.s2_p])
            model_point = self._getSamePoint(model_top, surface[s.s1_o])

            common_point = list(set(parent_point).intersection(parent_point2))[0]
            uncommon_point = list(set(list(set(parent_top).difference(set(parent_point)))).difference(set(parent_point2)))[0]

            if (check_axis(parent_point, m_parent_bb_corners) == 0):
                # parent_point y axis
                if (m_parent_bb_corners[common_point][1] > m_parent_bb_corners[uncommon_point][1]):
                    m.location[1] = m_parent_bb_corners[common_point][1] - s.d1
                else:
                    m.location[1] = m_parent_bb_corners[common_point][1] + s.d1
            else:
                # parent_point x axis
                if (m_parent_bb_corners[common_point][0] > m_parent_bb_corners[uncommon_point][0]):
                    m.location[0] = m_parent_bb_corners[common_point][0] - s.d1
                else:
                    m.location[0] = m_parent_bb_corners[common_point][0] + s.d1

            if (check_axis(parent_point2, m_parent_bb_corners) == 0):
                # parent_point y axis
                if (m_parent_bb_corners[common_point][1] > m_parent_bb_corners[uncommon_point][1]):
                    m.location[1] = m_parent_bb_corners[common_point][1] - s.d2
                else:
                    m.location[1] = m_parent_bb_corners[common_point][1] + s.d2
            else:
                # parent_point x axis
                if (m_parent_bb_corners[common_point][0] > m_parent_bb_corners[uncommon_point][0]):
                    m.location[0] = m_parent_bb_corners[common_point][0] - s.d2
                else:
                    m.location[0] = m_parent_bb_corners[common_point][0] + s.d2

            if (m.parent.parent.name != "room"):
                temp_parent = m.parent
                self._changeParent(m, bpy.data.objects['room'])
                m.location[0] = m.parent.location[0]
                m.location[1] = m.parent.location[1]
                self._changeParent(m, temp_parent)

        bpy.context.scene.update()
        # bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)

    def _moveObjectVF(self, model, T, xy_minmax, axis, min_parent, max_parent):
        def _calculateValue(ti):
            u = np.random.uniform(low=0.0, high=1.0, size=1)[0]
            return np.sign(u-0.5)*ti*(np.power(1+1/ti, np.abs(2*u-1))-1)

        if(axis == 0):
            # x
            cnt = _calculateValue(T[axis]) # [-1. 1]
            #g_value = 0.5 + np.sign(yi) / 2 * np.log(1+np.abs(yi) / T[axis]) / np.log(1 + 1 / T[axis])
            g_value_t = cnt * (xy_minmax[1]-xy_minmax[0])
            x_loc = model.location[0] + g_value_t
            y_loc = model.location[1]
        else:
            # y
            cnt = _calculateValue(T[axis]) # [-1. 1]
            #g_value = 0.5 + np.sign(yi) / 2 * np.log(1 + np.abs(yi) / T[axis]) / np.log(1 + 1 / T[axis])
            g_value_t = cnt * (xy_minmax[1]-xy_minmax[0])
            x_loc = model.location[0]
            y_loc = model.location[1] + g_value_t

        # check unfeasible space
        if model.parent.name in self.unfeasibleFurniture.keys():
            value = self.unfeasibleFurniture[model.parent.name]
            if (value[0][0] < x_loc < value[1][0] and value[0][1] < y_loc < value[1][1]):
                return

        # check inside parent object
        if(min_parent[0] < x_loc < max_parent[0] and min_parent[1] < y_loc < max_parent[1]):
            model.location[0] = x_loc
            model.location[1] = y_loc
            bpy.context.scene.update()

    def _rotateObjectVF(self, model, T):
        def _calculateValue(ti):
            u = np.random.uniform(low=0.0, high=1.0, size=1)[0]
            return np.sign(u-0.5)*ti*(np.power(1+1/ti, np.abs(2*u-1))-1)

        theta_i = _calculateValue(T[2])  # [-1. 1]
        # g_value = 0.5 + np.sign(yi) / 2 * np.log(1+np.abs(yi) / T[axis]) / np.log(1 + 1 / T[axis])
        g_value_t = theta_i * (np.pi)

        model.rotation_euler[2] = model.rotation_euler[2] - g_value_t
        bpy.context.scene.update()

    def _moveObject(self, model):
        ratio = 100000

        # parent's model -> bb_corners
        parent_model = model.parent
        p_bb_corners = cf._getWorldBoundingBox(parent_model)
        min_p_bb_corners = np.min(p_bb_corners, axis=0)
        max_p_bb_corners = np.max(p_bb_corners, axis=0)

        # model -> bb_corners
        m_bb_corners = cf._getWorldBoundingBox(model)
        min_m_bb_corners = np.min(m_bb_corners, axis=0)
        max_m_bb_corners = np.max(m_bb_corners, axis=0)

        length_x = np.abs(max_m_bb_corners[0] - min_m_bb_corners[0])/2
        length_y = np.abs(max_m_bb_corners[1] - min_m_bb_corners[1])/2

        range_min = min_p_bb_corners[0:2] + [length_x, length_y]
        range_max = max_p_bb_corners[0:2] - [length_x, length_y]

        try:
            value_x = randint(int(range_min[0] * ratio), int(range_max[0] * ratio))
            value_y = randint(int(range_min[1] * ratio), int(range_max[1] * ratio))
        except:
            model.location[0] = parent_model.location[0]
            model.location[1] = parent_model.location[1]
            model.rotation_euler[0] = parent_model.rotation_euler[0]
            model.rotation_euler[1] = parent_model.rotation_euler[1]
            bpy.context.scene.update()
            return

        x_loc = value_x/ratio
        y_loc = value_y/ratio

        # check unfeasible space
        if parent_model.name in self.unfeasibleFurniture.keys():
            value = self.unfeasibleFurniture[parent_model.name]
            if (value[0][0] < x_loc < value[1][0] and value[0][1] < y_loc < value[1][1]):
                return

        model.location[0] = x_loc
        model.location[1] = y_loc
        bpy.context.scene.update()

    def _rotateObject(self, model, i):
        degree = randint(0, 24) * 15

        model.rotation_euler[2] = np.deg2rad(degree)
        bpy.context.scene.update()

    def _optimization(self):
        # set initial cost
        self._initialization()

        self._postOrderSearch(bpy.data.objects['room'])

    def _postOrderSearch(self, model):
        if (len(model.children) == 0):
            return

        for c in model.children:
            self._postOrderSearch(c)
        if (model.name == "room"):
            return
        if (len(model.children) >= 3 and model.children[0].name.split('.')[0] == "viewing"):
            return

        #bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        self._veryFastSimulatedReannealing(model)

    def _veryFastSimulatedReannealing(self, parent):
        # find x_min, x_max, y_min, y_max
        m_bb_corners = cf._getWorldBoundingBox(parent)
        min_bbox = np.min(m_bb_corners, axis=0)
        max_bbox = np.max(m_bb_corners, axis=0)
        m_extend = (max_bbox - min_bbox) / 2
        xy_minmax = [-m_extend[0], m_extend[0], -m_extend[1], m_extend[1]]

        # Set initial temperature
        T = [1, 1, 1]
        D = 3
        C = [1.38064852e-23, 1.38064852e-23, 1.38064852e-23]
        iter = [0, 0, 0]

        # annealing-time
        k = 0

        # remove unfeasibleobjects
        num_of_children = len(parent.children)
        for model in parent.children:
            if model.name in self.unfeasibleObjects:
                num_of_children -= 1

        while (1):
            k = k + 1

            for model in parent.children:
                model_name = model.name
                if model_name in self.unfeasibleObjects:
                    continue

                loc = [l for l in model.location]
                rot = [r for r in model.rotation_euler]

                u = np.random.randint(3, size=1)

                if (k <= 25):
                    # Rotate object (z)
                    self._rotateObject(model, k - 1)

                    prior = self._costPrior(model)

                    alpha = np.min([np.exp((self.cost[model_name].prior_d2 - prior[2]) / k), 1])

                    # Total cost
                    if (alpha == 1):
                        self.cost[model_name].prior_d1 = prior[0]
                        self.cost[model_name].prior_d2 = prior[1]
                        self.cost[model_name].prior_theta1 = prior[2]
                        bpy.context.scene.update()
                        # bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
                    else:
                        model.location = loc
                        model.rotation_euler = rot
                        bpy.context.scene.update()
                        u = 2
                        iter[u] += 1
                        T[u] = T[u] * np.exp(-C[u] * np.power(iter[u], 1 / D))
                    continue
                elif(u == 2):
                    collsion = False

                    # Rotate object (z)
                    self._rotateObjectVF(model, T)

                    # collision check
                    for c in model.parent.children:
                        if (c == model):
                            continue
                        if (cf._checkCollision(model, c) == True):  # collision
                            collsion = True
                            break

                    if (collsion == True):
                        model.location = loc
                        model.rotation_euler = rot
                        bpy.context.scene.update()
                        continue
                else:
                    # Move object (x,y)
                    ## while loop -> until no collision
                    while (1):
                        collsion = False

                        self._moveObjectVF(model, T, xy_minmax, u, min_bbox, max_bbox)

                        # collision check
                        for c in model.parent.children:
                            if (c == model):
                                continue
                            if (cf._checkCollision(model, c) == True):  # collision
                                collsion = True
                                break

                        if (collsion == False):
                            break

                # cost check
                # prior
                cost = 0

                prior = self._costPrior(model)
                cost += prior[0] + prior[1] + prior[2]

                # pairwise
                pair = False
                for p in self.data.pairwise:
                    if (p.member[0] == model_name or p.member[1] == model_name):
                        p1 = p.member[0]
                        p2 = p.member[1]
                        pair_cost = self._costPairwise(p)
                        cost += pair_cost[0] + pair_cost[1]
                        pair = True

                # visibility
                cost_visibility = 0
                if (len(model.children) >= 3 and model.children[0].name.split('.')[0] == "viewing"):
                    cost_visibility = self._costVisibility(model)
                # accessibility
                cost_accessibility = self._costAccessibility(model)
                cost += cost_visibility + cost_accessibility

                alpha = np.min([np.exp((self.cost[model_name]._getTotalCost() - cost) / k), 1])

                # Total cost
                if (alpha == 1):
                    self.cost[model_name].prior_d1 = prior[0]
                    self.cost[model_name].prior_d2 = prior[1]
                    self.cost[model_name].prior_theta1 = prior[2]
                    self.cost[model_name].accessibility = cost_accessibility
                    self.cost[model_name].visibility = cost_visibility
                    if (pair == True):
                        self.cost[p1].pair_d = pair_cost[0]
                        self.cost[p1].pair_theta = pair_cost[1]
                        self.cost[p2].pair_d = pair_cost[0]
                        self.cost[p2].pair_theta = pair_cost[1]
                    bpy.context.scene.update()
                    # bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
                else:
                    model.location = loc
                    model.rotation_euler = rot
                    bpy.context.scene.update()

            # swap (Randomly select two objects)
            self._swapObject()

            # Break condition
            t_cost = 0.0

            for m in parent.children:
                if m.name in self.unfeasibleObjects:
                    continue
                t_cost += self.cost[m.name].total

            # Update temperature
            iter[u] += 1
            T[u] = T[u] * np.exp(-C[u] * np.power(iter[u], 1 / D))

            # Update k
            if (k > 30 and t_cost / k / num_of_children < 1e-3):
                break

    def _initialization(self):
        # initialize cost
        for model in bpy.data.objects:
            model_name = model.name

            if (model_name == "room" or model_name == "Camera" or model_name == "Lamp" or model_name.split('.')[0] == "viewing"):
                continue
            elif (model.parent.name == "room"):
                continue

            self.cost[model_name] = Cost()
            model_cost = self.cost[model_name]

            # prior d, theta
            prior = self._costPrior(model)
            model_cost.prior_d1 = prior[0]
            model_cost.prior_d2 = prior[1]
            model_cost.prior_theta1 = prior[2]

            # visibility
            if(len(model.children) >= 3 and model.children[0].name.split('.')[0] == "viewing"):
                model_cost.visibility = self._costVisibility(model)

            # accessibility
            model_cost.accessibility = self._costAccessibility(model)

        # pairwise
        pairwise_check = []
        for p in self.data.pairwise:
            p1 = p.member[0]
            p2 = p.member[1]
            if p1 in self.cost and p2 in self.cost:
                pairwise_check.append(p)
        self.data.pairwise = pairwise_check

        for p in self.data.pairwise:
            pair = self._costPairwise(p)
            self.cost[p.member[0]].pair_d = pair[0]
            self.cost[p.member[0]].pair_theta = pair[1]
            self.cost[p.member[1]].pair_d = pair[0]
            self.cost[p.member[1]].pair_theta = pair[1]

    def _swapObject(self):
        def _swap(m1, m2):
            temp_l = copy.deepcopy(m1.location[0:2])
            m1.location[0:2] = copy.deepcopy(m2.location[0:2])
            m2.location[0:2] = temp_l
            bpy.context.scene.update()

        # randomly select two objects
        room_child_len = len(bpy.data.objects['room'].children)
        furn = randint(0, room_child_len-1)

        furniture = bpy.data.objects['room'].children[furn]
        furn_child_len = len(furniture.children)
        if(len(furniture.children) < 2):
            return

        obj_1 = randint(0, furn_child_len-1)
        while(1):
            obj_2 = randint(0, furn_child_len-1)
            if(obj_1 != obj_2):
                break

        m1 = furniture.children[obj_1]
        m2 = furniture.children[obj_2]

        # pre-caculated cost
        pre_cost_m1 = self.cost[m1.name]._getTotalCost()
        pre_cost_m2 = self.cost[m2.name]._getTotalCost()

        # swap object
        _swap(m1, m2)

        # caculated cost
        cost_m1 = self._calculateOverallCost(m1)
        cost_m2 = self._calculateOverallCost(m2)

        # compare cost
        if (pre_cost_m1 > cost_m1 and pre_cost_m2 > cost_m2):
            ### bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
            pass
        else:
            _swap(m1, m2)

    # cost function
    def _costPrior(self, m):
        surface = self.surface
        m_bb_corners = cf._getWorldBoundingBox(m)
        m_parent_bb_corners = cf._getWorldBoundingBox(m.parent)

        model_top = cf._getTopSurface(m_bb_corners)
        parent_top = cf._getTopSurface(m_parent_bb_corners)

        name = m.name+"_"+m.parent.name
        # calculate c
        c = cf._calculateCenterPoint(m_bb_corners, model_top)

        total_cost = np.inf
        for s in self.data.objects[name].spatial:
            #### find the nearest surface
            parent_point = self._getSamePoint(parent_top, surface[s.s1_p])
            parent_point2 = self._getSamePoint(parent_top, surface[s.s2_p])
            model_point = self._getSamePoint(model_top, surface[s.s1_o])

            # calculate d1
            d1 = cf._calculateDistanceOfLineAndPoint(m_parent_bb_corners[parent_point[0]], m_parent_bb_corners[parent_point[1]], c)

            # calculate d2
            d2 = cf._calculateDistanceOfLineAndPoint(m_parent_bb_corners[parent_point2[0]], m_parent_bb_corners[parent_point2[1]], c)

            # calculate theta1
            theta1 = cf._calculateTheta([m_parent_bb_corners[parent_point[0]][0:2], m_parent_bb_corners[parent_point[1]][0:2]], [m_bb_corners[model_point[0]][0:2], m_bb_corners[model_point[1]][0:2]])

            temp_d1 = np.abs(d1 - s.d1) * self.Wpr_d1
            temp_d2 = np.abs(d2 - s.d2) * self.Wpr_d2
            temp_theta1 = np.abs(theta1 - s.theta1) * self.Wpr_theta1
            temp_cost = temp_d1+temp_d2+temp_theta1

            if(total_cost > temp_cost):
                total_cost = temp_cost
                cost_d1 = temp_d1
                cost_d2 = temp_d2
                cost_theta1 = temp_theta1

        return (cost_d1, cost_d2, cost_theta1)

    def _costPairwise(self, g):
        g1_loc = np.array(bpy.data.objects[g.member[0]].location)
        g2_loc = np.array(bpy.data.objects[g.member[1]].location)

        d = cf._calculateL2Distance(g1_loc, g2_loc)
        theta = cf._calculateTheta([g1_loc[0:2], g2_loc[0:2]], [g1_loc[0:2], [g1_loc[1], g2_loc[0]]])

        cost_d = np.abs(g.d - d) * self.Wpair_d
        cost_theta = np.abs(g.theta - theta) * self.Wpair_theta

        return (cost_d, cost_theta)

    def _costVisibility(self, m):
        cost = 0
        for obj in m.parent.children:
            if(obj.name == m.name):
                continue
            for vf in m.children:
                if (cf._checkCollision(vf, obj)):
                    m_bb_corners = cf._getWorldBoundingBox(obj)
                    m_top = cf._getTopSurface(m_bb_corners)
                    v = cf._calculateCenterPoint(m_bb_corners, m_top)
                    vd = cf._calculateL2Distance(v, m_bb_corners[m_top[0]])
                    # calculate c (center)
                    data_c = cf._calculateCenterPoint(m_bb_corners, m_top)
                    # calculate b (diagonal)
                    data_b = cf._calculateL2Distance(data_c, m_bb_corners[0][0:2])
                    cost += np.max([0, 1 - cf._calculateL2Distance(data_c, v) / (data_b + vd)])

        return cost * self.Wv

    def _costAccessibility(self, m):
        # accessibility
        cost = 0
        for f in m.parent.children:
            if (f.name == m.name):
                continue
            if (cf._checkAccessCollision(f, m)):
                m_bb_corners = cf._getWorldBoundingBox(m)
                m_top = cf._getTopSurface(m_bb_corners)
                data_p = cf._calculateCenterPoint(m_bb_corners, m_top)
                data_b = cf._calculateL2Distance(data_p, m_bb_corners[0][0:2])

                box_min = np.min(m_bb_corners, axis=0)
                box_max = np.max(m_bb_corners, axis=0)
                if (np.abs(box_max[0] - box_min[0]) > np.abs(box_max[1] - box_min[1])):
                    # y
                    data_a = [data_p[0], data_p[1] - box_min[1]]
                    data_pa = np.abs(data_p[1] - box_min[1]) / 2
                    data_ad = cf._calculateL2Distance(data_a, [box_min[0], box_min[1] - 0.05])
                else:
                    # x
                    data_a = [data_p[0] - box_min[0], data_p[1]]
                    data_pa = np.abs(data_p[0] - box_min[0]) / 2
                    data_ad = cf._calculateL2Distance([box_min[0] - 0.05, box_min[1]], data_a)
                cost += (1 - (data_pa / (data_b + data_ad)))

        return cost * self.Wa

    def _calculateOverallCost(self, model):
        model_name = model.name

        cost = 0

        # prior
        prior = self._costPrior(model)
        cost += prior[0] + prior[1] + prior[2]

        # pairwise
        for p in self.data.pairwise:
            if (p.member[0] == model_name or p.member[1] == model_name):
                pair_cost = self._costPairwise(p)
                cost += pair_cost[0] + pair_cost[1]

        # visibility
        if (len(model.children) >= 3 and model.children[0].name.split('.')[0] == "viewing"):
            cost += self._costVisibility(model)
        # accessibility
        cost += self._costAccessibility(model)

        return cost
