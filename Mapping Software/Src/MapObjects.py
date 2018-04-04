import math

class Wall:
    def __init__(self, startPoint, endPoint):
        self.startPoint = startPoint
        self.endPoint = endPoint
        if endPoint.x - startPoint.x != 0:
            self.a = 1
            self.k = (endPoint.y - startPoint.y) / (endPoint.x - startPoint.x)
            self.b = -self.k * startPoint.x + startPoint.y
        else:
            self.a = 0
            self.k = 1
            self.b = -startPoint.x

    def __str__(self):
        return "s.x = %f, s.y = %f ,e.x = %f, e.y = %f" %(self.startPoint.x, self.startPoint.y, self.endPoint.x, self.endPoint.y)

    def __eq__(self, other):
        if self.a == other.a and self.k == other.k and self.b == other.b:
            return True
        return False

    def greater_x(self):
        if self.startPoint.x > self.endPoint.x:
            return self.startPoint.x
        else:
            return self.endPoint.x

    def greater_y(self):
        if self.startPoint.y > self.endPoint.y:
            return self.startPoint.y
        else:
            return self.endPoint.y

    def smaller_x(self):
        if self.startPoint.x < self.endPoint.x:
            return self.startPoint.x
        else:
            return self.endPoint.x

    def smaller_y(self):
        if self.startPoint.y < self.endPoint.y:
            return self.startPoint.y
        else:
            return self.endPoint.y

    def parallel_to_x(self):
        if self.startPoint.y == self.endPoint.y:
            return True
        return False

    def parallel_to_y(self):
        if self.startPoint.x == self.endPoint.x:
            return True
        return False

    def rotate(self, base_point, angle=90):
        if self.parallel_to_x():
            if angle == 90:
                return Wall(base_point, Point(base_point.x, float('inf')))
            if angle == 180:
                return Wall(base_point, Point(float('inf'), base_point.y))
        elif self.parallel_to_y():
            if angle == 90:
                return Wall(base_point, Point(float('inf'), base_point.y))
            if angle == 180:
                return Wall(base_point, Point(base_point.x, float('inf')))

    def set_endPoint(self, intersections):
        index = 0
        length = float('inf')
        for i in range(len(intersections)):
            if self.parallel_to_x():
                temp = abs(intersections[i].x - self.startPoint.x)
            else:
                temp = abs(intersections[i].y - self.startPoint.y)
            if temp < length:
                length = temp
                index = i
        self.endPoint = intersections[index]
        return intersections[i]
    def length(self):
        x1 = self.startPoint.x
        x2 = self.endPoint.x
        y1 = self.startPoint.x
        y2 = self.startPoitn.y
        z1 = (x1 - x2)*(x1 - x2)
        z2 = (y1 - y2)*(y1 -y2)
        return math.sqrt(z1 + z2)


class Point:
    def __init__(self, x, y, angle=0):
        self.x = x
        self.y = y
        self.angle = angle

    def __str__(self):
        return "x=%f, y=%f, angle=%f" % (self.x, self.y, self.angle)

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        return False

    def on_wall(self, wall):
        if wall.a*self.y != wall.k*self.x + wall.b:
            return False
        if wall.greater_x() >= self.x >= wall.smaller_x() and wall.greater_y() >= self.y >= wall.smaller_y():
            return True
        return False


def rotate_by_alpha(V, a):  # rotate a vector by a in the XY-plane counter clockwise

    return((V[0] * math.cos(a) - V[1] * math.sin(a)), (V[0] * math.sin(a) + V[1] * math.cos(a)))

"""
A, B, C are Points
"""
def calculate_angle(A, B, C):   # function

    # vector from B to A - direction is important
    BA = (A.x - B.x, A.y - B.y)
    # vector from B to C - direction is important
    BC = (C.x - B.x, C.y - B.y)

    pheta = math.acos((BA[0] * BC[0] + BA[1] * BC[1]) /
                      (math.sqrt(BA[0]**2 + BA[1]**2) * math.sqrt(BC[0]**2 + BC[1]**2)))

    # check the sign:  is the angle Pi/2 or -Pi/2 ? This is the difference inner vertex (+) and outher vertex(-)
    # get a - angle between BA and X-axis
    X = (1, 0)
    a = math.acos((BA[0] * X[0]) / math.sqrt(BA[0]**2 + BA[1]**2))

    if (BA[1] > 0):
        # note: we need to rotate in clockwise direction
        BC_rotate = rotate_by_alpha(BC, -a)
    else:
        # note: we need to rotate in anti-clockwise direction
        BC_rotate = rotate_by_alpha(BC, a)

    # now simply check the Y-coordinate of BC - on which side of the unit vector (1,0) does BC lie

    if (BC_rotate[1] < 0):  # we have an outher angle
        pheta = -pheta

    return math.degrees(pheta)
