import math
import turtle


class Map:

    def __init__(self, points=[]):
        self.points = points
        self.walls = []
        self.auxiliary_walls = []
        self.auxiliary_points = []
        if self.check_points():
            self.walls = [Wall(points[i], points[(i + 1) % len(points)])
                          for i in range(len(points))]
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
                if not math.isclose(self.points[0].angle,calculate_angle(self.points[n - 1], self.points[0], self.points[1]), abs_tol=0.001):
                    answer = 0  # problem, alert somehow
                    print(0)
                    print(i)
                    return False
            elif i == n - 1:
                if not math.isclose(self.points[n - 1].angle, calculate_angle(self.points[n - 2], self.points[n - 1], self.points[0]), abs_tol=0.001):
                    answer = 0  # problem, alert somehow
                    print(1)
                    print(i)
                    return False
                else:
                    answer = 1  # everything seems to be great
            else:
                if not math.isclose(self.points[i].angle, calculate_angle(self.points[i - 1], self.points[i], self.points[i + 1]), abs_tol=0.001):
                    answer = 0  # problem, alert somehow
                    print(2)
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
        turtle.pensize(3)

        for point in self.points:  # go through a list of (relative) points
            dx, dy = point.x, point.y
            turtle.goto(origin.x + dx, origin.y + dy)

        # connect them to start point to form a closed shape
        turtle.goto((origin.x, origin.y))
        turtle.pencolor("red")
        turtle.pensize(1)
        for line in self.auxiliary_walls:
            print(line)
            turtle.penup()
            turtle.goto((line.startPoint.x, line.startPoint.y))
            turtle.pendown()
            turtle.goto((line.endPoint.x, line.endPoint.y))
        turtle.done()

    # @param wall_m: new wall
    def get_intersection(self, wall_m, wall_n):
        x = y = 0
        # when two lines concide
        if wall_m == wall_n and wall_m.a == 0:
            if abs(wall_m.startPoint.y - wall_n.startPoint.y) < abs(wall_m.startPoint.y - wall_n.endPoint.y):
                return Point(wall_m.startPoint.x, wall_n.startPoint.y)
            return Point(wall_m.startPoint.x, wall_n.endPoint.y)
        if wall_m == wall_n and wall_m.k == 0:
            if abs(wall_m.startPoint.x - wall_n.startPoint.x) < abs(wall_m.startPoint.x - wall_n.endPoint.x):
                return Point(wall_n.startPoint.x, wall_m.startPoint.y)
            return Point(wall_n.endPoint.x, wall_m.startPoint.y)

        # when two walls do not concide
        if wall_m.a != 0 and wall_n.a != 0:
            if wall_m.k == wall_n.k:
                return Point(float('inf'), float('inf'))
            else:
                x = (wall_m.b - wall_n.b) / (wall_n.k - wall_m.k)
                y = wall_m.k * x + wall_m.b
        else:
            if wall_m.a == wall_n.a == 0 :
                return Point(float('inf'), float('inf'))
            else:
                if wall_n.a == 0:
                    x = -wall_n.b
                    y = wall_m.k*x+wall_m.b
                else:
                    x = -wall_m.b
                    y = wall_n.k * x + wall_n.b
        if Point(x, y).on_wall(wall_n):
            return Point(x, y)
        else:
            return Point(float('inf'), float('inf'))

    def segment(self):

        def is_valid_intersection(point, index):
            if point.x !=float('inf') and point != self.walls[index].endPoint and point!=self.walls[index-1].startPoint and point!=self.walls[index-1].endPoint:
                return True
            return False

        # def is_good_intersection(point):
        #     for wall in walls:
        #         if point.on_wall(wall) and point != wall.startPoint and point != wall.endPoint and wall.k != 0 or wall.a !=0:
        #             return False
        #     return True

        for i in range(len(self.points)):
            if self.points[i].angle != 90:
                add_line_num = (self.points[i].angle+360) % 360 // 90
                if (self.points[i].angle+360) % 360 == 270:
                    add_line_num -= 1
                
                for num in range(add_line_num):
                    if add_line_num > 0 and self.walls[i - 1].parallel_to_x() or self.walls[i - 1].parallel_to_y():
                        new_wall = self.walls[i - 1].rotate(self.walls[i - 1].endPoint, (num+1)*90)
                    elif add_line_num > 0 and self.walls[i].parallel_to_x() or self.walls[i].parallel_to_y():
                        new_wall = self.walls[i].rotate(self.walls[i].startPoint, (num+1)*90)

                    if new_wall in self.auxiliary_walls:
                        continue

                    intersections = [self.get_intersection(new_wall, self.walls[j]) for j in range(len(self.walls)) if j != i and j != ((i-1)+len(self.walls))%len(self.walls)]
                    intersections += [self.get_intersection(new_wall, self.auxiliary_walls[j]) for j in range(len(self.auxiliary_walls))]
                    intersections = list(filter(lambda point: is_valid_intersection(point, i), intersections))
                    intersection = new_wall.set_endPoint(intersections)
                    self.auxiliary_walls.append(new_wall)



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
        if self.a == other.a and self.k ==other.k and self.b == other.b:
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
