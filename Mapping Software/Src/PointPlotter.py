from MapObjects import *
import math
from statistics import mean
from vectors import Vector
from sklearn.linear_model import LinearRegression


"""
Finds the wall vector based on distance readings and the starting location of the robot
wall_distances - a list of distances from the robot to the left wall
start_location - a list with two elements (x and y) showing where the robot is located at the start of its movement
"""
def find_wall_vector(start_angle):
    # Convert starting angle from radians to degrees
    angle_rad = math.radians(start_angle)
    # Find the vector direction
    x1 = math.sin(angle_rad)
    y1 = math.cos(angle_rad)
    return Vector(x1, y1)

"""
Finds a point on the wall closest to the robot
left_wall_distances - list of distances from robot to left wall
start_position - list with two elements (x and y) for where the robot is
start_angle - angle that the robot is facing. (North = 0)
"""
def find_wall_point(left_wall_distances, start_position, start_angle):
    avg_wall_distance = mean(left_wall_distances)
    start_x = start_position[0]
    start_y = start_position[1]

    # Mod by 360 if an angle greater than that is put in.
    if start_angle >= 360:
        start_angle = start_angle % 360

    if start_angle == 0:
        return [start_x - avg_wall_distance, start_y]

    elif 0 < start_angle < 90:
        theta = math.radians(start_angle)
        x = math.cos(theta)*avg_wall_distance
        y = math.sin(theta)*avg_wall_distance
        return [start_x - x, start_y + y]

    elif start_angle == 90:
        return [start_x, start_y + avg_wall_distance]

    elif 90 < start_angle < 180:
        theta = math.radians(start_angle - 90)
        x = math.sin(theta)*avg_wall_distance
        y = math.cos(theta)*avg_wall_distance
        return [start_x + x, start_y + y]

    elif start_angle == 180:
        return [start_x + avg_wall_distance, start_y]

    elif 180 < start_angle < 270:
        theta = math.radians(start_angle - 180)
        x = math.cos(theta)*avg_wall_distance
        y = math.sin(theta)*avg_wall_distance
        return [start_x + x, start_y - y]

    elif start_angle == 270:
        return [start_x, start_y - avg_wall_distance]

    else:
        theta = math.radians(360 - start_angle)
        x = math.sin(theta)*avg_wall_distance
        y = math.cos(theta)*avg_wall_distance
        return[start_x - x, start_y - y]



"""
Checks if two lines intersect or not
"""
def check_points_intersect(p1, p2, p3, p4):
    return ((p4.y - p3.y)*(p2.x - p1.x) - (p4.x - p3.x)*(p2.y - p1.y)) != 0


"""
Finds intersection between two lines:
p1-p4 - in form of Point
p1, p2 = points on line A
p3, p4 = points on line B
"""
def point_of_intersection(points):
    p1 = points[0]
    p2 = points[1]
    p3 = points[2]
    p4 = points[3]

    u = ((p4.x - p3.x)*(p1.y - p3.y) - (p4.y - p3.y)*(p1.x - p3.x))/((p4.y - p3.y)*(p2.x - p1.x) - (p4.x - p3.x)*(p2.y - p1.y))
    x = p1.x + u*(p2.x - p1.x)
    y = p1.y + u*(p2.y - p1.y)
    return Point(x, y)


def main():
    points = [Point(0, 0), Point(0,2), Point(2, 2), Point(2,0)]

    point = point_of_intersection(points)
    print(point)


if __name__ == "__main__":
    main()