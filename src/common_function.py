import numpy as np
import copy
from mathutils import Vector

def getBoundBoxCorners(mat, bb):
    """
    calculte the corners of bounding box
    """
    return [np.dot(mat, np.append(corner, 1))[0:3] for corner in bb]

def _getWorldBoundingBox(model):
    return [model.matrix_world * Vector(corner) for corner in model.bound_box]

def _calculateL2Distance(a, b):
    """
    :return: the distance between a and b (x, y) 2D
    """

    return np.sqrt(np.power(a[0] - b[0], 2) + np.power(a[1] - b[1], 2))

def _calculateCenterPoint(bb_corners, model_top):
    """
    :param bb_corners: the corners of bounding box
    :param model_top: the corners of top surfaces
    :return: the center point of model
    """
    position_p = [bb_corners[i] for i in model_top]
    min_p = np.min(position_p, axis=0)
    max_p = np.max(position_p, axis=0)

    return [(min_p[0] + max_p[0]) / 2, (min_p[1] + max_p[1]) / 2]

def _getTopSurface(bb_corners):
    """
    :param bb_corners: the corners of bounding box
    :return: the corners of top surfaces
    """
    top = []

    # find the middle of value in Z value.
    min_bb_corners = np.sort([a[2] for a in bb_corners])[3]

    # select the 4 corners of bounding box
    for i in range(8):
        if (bb_corners[i][2] > min_bb_corners):
            top.append(i)

    return top

def _seperateDiagonalsRect(bb_corners, model_top):
    """
    :param bb_corners:
    :param list:
    :return:
    """
    list = copy.deepcopy(model_top)
    max_d = -1
    i = 0
    for j in range(i + 1, 4):
        d = _calculateL2Distance(bb_corners[list[i]], bb_corners[list[j]])
        if (max_d < d):
            max_d = d
            seg1 = i
            seg2 = j

    a = []
    a.append(list.pop(seg2))
    a.insert(0, list.pop(seg1))

    return [a, list]

def _calculateDistanceOfLineAndPoint(a, b, c):
    """
    y = (y2-y1)/(x2-x1)*(x-x1) + y1
    (x2-x1) * y = (y2-y1) * x - (y2-y1) * x1 + (x2-x1) * y1
    (y2-y1) * x - (x2-x1) * y - y2*x1 + x2*y1 = 0
    :param a: x1, y1
    :param b: x2, y2
    :param c: the center of model top surface
    :return: distance = |ax + by + c| / sqrt(a^2+b^2)
    """
    A = float(b[1] - a[1])
    B = -float(b[0] - a[0])
    C = float(-b[1] * a[0] + b[0] * a[1])

    d = np.abs(A * c[0] + B * c[1] + C) / np.sqrt(A * A + B * B)

    return d

def _calculateFootofPerpendicular(a, b, c):
    A = float(a[1] - b[1])
    B = -float((a[0] - b[0]))
    C = float(a[0] * b[1] - a[1] * b[0])

    k = -(A * c[0] + B * c[1] + C) / (A * A + B * B)
    h = [A * k + c[0], B * k + c[1]]

    return h

def _checkCross(a, b, c, d):
    return _ccw(a, c, d) != _ccw(b, c, d) and _ccw(a, b, c) != _ccw(a, b, d)

def _ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])

def _getSurfaceLabel(data, surface):
    for i in range(6):
        if (data == surface[i]):
            return i

    return None

def _calculateTheta(a, b):
    def _unitVector(vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)
    a = np.array(a)
    b = np.array(b)
    v1 = a[1] - a[0]
    v2 = b[1] - b[0]
    v1_u = _unitVector(v1)
    v2_u = _unitVector(v2)

    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def _getSurfacePoint(bb_corners, a, b):
    n = []
    for i in range(8):
        if (bb_corners[a][0] == bb_corners[i][0] and bb_corners[a][1] == bb_corners[i][1]):
            n.append(i)
        elif (bb_corners[b][0] == bb_corners[i][0] and bb_corners[b][1] == bb_corners[i][1]):
            n.append(i)

    return n

def _checkCollision(box1, box2):
    """
    :return 0: not collision 1: collision
    """
    box1_corners = _getWorldBoundingBox(box1)
    box2_corners = _getWorldBoundingBox(box2)

    box1_min = np.min(box1_corners, axis=0)
    box1_max = np.max(box1_corners, axis=0)
    box2_min = np.min(box2_corners, axis=0)
    box2_max = np.max(box2_corners, axis=0)

    if(box1_max[0] < box2_min[0] or box1_min[0] > box2_max[0]): return False
    if(box1_max[1] < box2_min[1] or box1_min[1] > box2_max[1]): return False
    if(box1_max[2] < box2_min[2] or box1_min[2] > box2_max[2]): return False

    return True

def _checkAccessCollision(box1, box2, gripper_size=0.05):
    """
    :return 0: not collision 1: collision
    """
    box1_corners = _getWorldBoundingBox(box1)
    box2_corners = _getWorldBoundingBox(box2)

    box1_min = np.min(box1_corners, axis=0)
    box1_max = np.max(box1_corners, axis=0)
    box2_min = np.min(box2_corners, axis=0)
    box2_max = np.max(box2_corners, axis=0)

    box2_min[0] -= gripper_size
    box2_min[1] -= gripper_size
    box2_max[0] += gripper_size
    box2_max[1] += gripper_size

    if(box1_max[0] < box2_min[0] or box1_min[0] > box2_max[0]): return False
    if(box1_max[1] < box2_min[1] or box1_min[1] > box2_max[1]): return False
    if(box1_max[2] < box2_min[2] or box1_min[2] > box2_max[2]): return False

    return True