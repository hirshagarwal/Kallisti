import math
import turtle


class Map:

    def __init__(self, points):
        self.points = points
        if self.check_points():
            self.walls = [Wall(points[i], points[(i + 1) % len(points)])
                          for i in range(len(points) - 1)]
        else:
            print("init failed, illegal angle in point list")

    def addWall(self, newWall):
        self.walls.append(newWall)

    def numWalls(self):
        return len(self.walls)

    def getWalls(self):
        return self.walls

    # check if there is incorrect angle in provided points
    # @return: true if all the angles are correct
    def check_points(self):
        n = len(self.points)
        answer = -1  # not yet checked
        for i in range(0, n):
            if i == 0:
                if(self.points[0].angle != calculate_angle(self.points[n - 1], self.points[0], self.points[1])):
                    answer = 0  # problem, alert somehow
                    print()
                    print(i)
                    return False
            elif i == n - 1:
                if(self.points[n - 1].angle != calculate_angle(self.points[n - 2], self.points[n - 1], self.points[0])):
                    answer = 0  # problem, alert somehow
                    print()
                    print(i)
                    return False
                else:
                    answer = 1  # everything seems to be great
            else:
                if(self.points[i].angle != calculate_angle(self.points[i - 1], self.points[i], self.points[i + 1])):
                    answer = 0  # problem, alert somehow
                    print()
                    print(i)
                    return False
        return True

    def draw_map(self):
        """ draw map using a list of coordinates """
        turtle.pencolor("black")

        # draw coordinate axis
        origin = Point(0, 0)
        turtle.goto((1000, 0))
        turtle.goto((-1000, 0))
        turtle.goto((0, 0))
        turtle.goto((0, 1000))
        turtle.goto((0, -1000))
        turtle.goto((0, 0))

        # make the turtle go to the origin
        turtle.penup()
        turtle.goto((origin.x, origin.y))
        turtle.pendown()
        turtle.pencolor("green")

        for point in self.points:  # go through a list of (relative) points
            dx, dy = point.x, point.y
            turtle.goto(origin.x + dx, origin.y + dy)

        # connect them to start point to form a closed shape
        turtle.goto((origin.x, origin.y))
        turtle.penup()
        turtle.done()


class Wall:
    def __init__(self, startPoint, endPoint):
        self.startPoint = startPoint
        self.endPoint = endPoint


class Point:
    def __init__(self, x, y, angle=0):
        self.x = x
        self.y = y
        self.angle = angle

    def __str__(self):
        return "x=%f, y=%f, angle=%f" % (self.x, self.y, self.angle)



def rotate_by_alpha(V, a):  # rotate a vector by a in the XY-plane counter clockwise

    return((V[0] * math.cos(a) - V[1] * math.sin(a)), (V[0] * math.sin(a) + V[1] * math.cos(a)))


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
