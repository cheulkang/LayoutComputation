import numpy as np
from collections import Counter

class Object:
    def __init__(self):
        # todo : erase the info (rotate x, y)
        self.count = 0
        self.spatial = []

class Data:
    def __init__(self):
        self.objects = {}
        self.pairwise = []

    def _integrateData(self, data):
        for d in data:
            self._setObjects(d.model)
            self._setPairwise(d.pairwise)

    def _setObjects(self, data):
        for model, info in data.items():
            if(model == "room" or info.parent == "room"):
                name = model + "_" + str(info.parent)
                self.objects[name] = info
                continue

            name = model + "_" + info.parent
            if(self.objects.get(name) == None):
                self.objects[name] = Object()
                self.objects[name].count += 1
                self.objects[name].spatial.append(info.spatial)
            else:
                add = False
                self.objects[name].count += 1
                for s in self.objects[name].spatial:
                    if(s.s1_o == info.spatial.s1_o and s.s1_p == info.spatial.s1_p and s.s2_p == info.spatial.s2_p):
                        add = True
                        s.d1 = (s.d1 + info.spatial.d1) / 2
                        s.d2 = (s.d2 + info.spatial.d2) / 2
                        s.theta1 = (s.theta1 + info.spatial.theta1) / 2
                        break

                if(add == False):
                    self.objects[name].spatial.append(info.spatial)

    def _setPairwise(self, data):
        for p in data:
            add = False
            for sp in self.pairwise:
                if((p.member[0] == sp.member[0] and p.member[1] == sp.member[1]) or (p.member[0] == sp.member[1] and p.member[0] == sp.member[1])):
                    add = True
                    sp.d = (sp.d + p.d) / 2
                    sp.theta = (sp.theta + p.theta) / 2
                    break

            if(add == False):
                self.pairwise.append(p)

    def _getParent(self, model_name):
        max_count = -1
        max_parent = ""

        for obj in self.objects:
            if(model_name != obj.split('_')[0]):
                continue

            if(max_count < self.objects[obj].count):
                max_count = self.objects[obj].count
                max_parent = obj.split('_')[1]

        return max_parent