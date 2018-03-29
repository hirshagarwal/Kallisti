from PointPlotter import *
from MapObjects import *
from turtle import *
import numpy as np

wall_lines = []
turn_left = "left"
turn_right = "right"


def get_wall_lines(walls):
    for wall in walls:
        point1 = wall.startPoint
        point2 = wall.endPoint
        line = Line(point1.x, point1.y, point2.x, point2.y)
        wall_lines.append(line)


class Robot:
    def __init__(self, x, y, angle=0):
        self.x = x
        self.y = y
        self.angle = angle
        self.currentWall = 0

    def __str__(self):
        return "Robot located at x=%f, y%f, angle=%f" % (self.x, self.y, self.angle)

    def move(self, movement):
        theta = math.radians(self.angle)
        x_change = math.sin(theta)*movement
        y_change = math.cos(theta)*movement
        self.x = self.x + x_change
        self.y = self.y + y_change

    def rotate(self, angle):
        self.angle = (self.angle + angle)
        # Converts negative/large angles to range of [0, 360]
        if self.angle > 360 or self.angle < -360:
            self.angle = self.angle % 360
        if self.angle < 0:
            self.angle = 360 + self.angle

    def get_reading(self, angle):
        adjusted_angle = self.angle + angle
        robot_line = Line(self.x, self.y, adjusted_angle)

        min_distance = 999999

        for line in wall_lines:
            checking_lines = [line] + [robot_line]
            # if the line from the robot intersects the wall within the wall's endpoints
            if check_lines_intersect_specific_one(checking_lines):
                poi = point_of_intersection(checking_lines)
                points = [Point(self.x, self.y), poi]
                # if the robot is in the right direction to the wall
                if check_direction_of_line(points, adjusted_angle):
                    # get the distance from the robot to that wall
                    distance = abs(line.a * self.x + line.b * self.y - line.c) / math.sqrt(
                        line.a * line.a + line.b * line.b)
                    if distance < min_distance:
                        min_distance = distance

        return min_distance

    @staticmethod
    def very_different(distance_before, distance_after, tolerance):
        return abs(distance_after - distance_before) >= tolerance

    def turn_left(self):
        self.rotate(-90)

    def turn_right(self):
        self.rotate(90)

    def turn_around(self):
        self.rotate(180)

    def path_loop(self, initial_location):
        current_location = initial_location
        unfinished = True
        while unfinished:
            left_init = self.get_reading(270)
            info = self.wall_loop(left_init)

            theta = math.radians(self.angle)
            x_change = math.sin(theta) * info[1]
            y_change = math.cos(theta) * info[1]

            current_location = Point(current_location.x + x_change, current_location.y + y_change)

            turtle.goto((current_location.x, current_location.y))

            if math.isclose(current_location.x, initial_location.x) and math.isclose(current_location.y, initial_location.y):
                return "Finished"

            if info[0] == turn_right:
                self.rotate(90)
                continue

            if info[0] == turn_left:
                self.rotate(-90)
                continue

    def wall_loop(self, left_init):
        total_distance = 0
        while True:
            left_distance = self.get_reading(270)
            front_distance = self.get_reading(0)
            # if the distance to the left increases suddenly, turn left
            if self.very_different(left_init, left_distance, 1):
                return turn_left, total_distance
            # if the front distance is less than 15, turn right
            if front_distance <= 10:
                return turn_right, total_distance
            self.move(10)
            total_distance += 10

    def get_reading_unreliable(self, angle, absolute_error):
        """
        Gets a distance reading, but adds a degree of error to it
        :param angle:
        :return:
        """
        true_distance = self.get_reading(angle)
        # If the true distance is between 0-15 cm, the error is larger
        if 0 < true_distance < 15:
            uncertain_distances = np.random.normal(true_distance, absolute_error)
        # Reduce error for closer distances
        elif 15 <= true_distance < 30:
            return np.random.normal(true_distance, 0.5*absolute_error)
        # Normal error amount for longer distances
        elif 30 <= true_distance < 200:
            return np.random.normal(true_distance, absolute_error)
        # Greater error for larger distances
        elif 200 <= true_distance < 250:
            return np.random.normal(true_distance, 1.5*absolute_error)
        else:
            return np.random.normal(true_distance, 15*absolute_error)






def main():
    points_4 = [Point(0, 0, 90), Point(0, 100, 90), Point(100, 100, -90), Point(100, 200, 90), Point(200, 200, 90),
                Point(200, 0, 90)]
    points_simple = [Point(0, 0, 90), Point(0, 100, 90), Point(100, 100, 90), Point(100, 0, 90)]

    map_test = Map(points_4)
    # print(map.get_intersection(map.walls[0], map.walls[4]))
    get_wall_lines(map_test.getWalls())
    map_test.draw_map()

    robot = Robot(10, 10, 0)
    initial_location = Point(10, 10)

    turtle.penup()
    turtle.goto((10, 10))
    turtle.pendown()

    robot.path_loop(initial_location)

    turtle.done()

if __name__ == "__main__":
    main()
