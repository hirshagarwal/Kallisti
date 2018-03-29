from MapObjects import *
import math
import numpy as np
from statistics import mean
from vectors import Vector


class Line:
    def __init__(self, x1, y1, *args):
        self.point1 = Point(x1, y1)
        # if there is only one extra argument, it is the angle
        if len(args) == 1:
            self.angle = args[0]
            # Converts negative/large angles to range of [0, 360]
            if self.angle >= 360 or self.angle <= -360:
                self.angle = self.angle % 360
            if self.angle < 0:
                self.angle = 360 + self.angle
            self.point2 = get_second_point(self.point1, self.angle)
            line_constants = get_line(self.point1, self.point2)
            self.a = line_constants[0]
            self.b = line_constants[1]
            self.c = line_constants[2]
            self.m = -self.a
        # if there are two arguments, they are the coordinates of the points
        elif len(args) == 2:
            self.point2 = Point(args[0], args[1])
            line_constants = get_line(self.point1, self.point2)
            self.a = line_constants[0]
            self.b = line_constants[1]
            self.c = line_constants[2]
            if self.b == 0:
                self.m = math.inf
            else:
                self.m = -self.a / self.b
            self.angle = 90 - math.degrees(math.atan(self.m))
            # Converts negative/large angles to range of [0, 360]
            if self.angle > 360 or self.angle < -360:
                self.angle = self.angle % 360
            if self.angle < 0:
                self.angle = 360 + self.angle
        else:
            raise Exception('The input arguments must be of length 2 at most')

    def __str__(self):
        return "Point 1 = " + str(self.point1) + " Point 2 = " + str(self.point2)


def get_second_point(point, angle):
    # Convert starting angle from radians to degrees
    angle_rad = math.radians(angle)
    # Find the vector direction
    x1 = math.sin(angle_rad)
    y1 = math.cos(angle_rad)

    point2 = Point(point.x + x1, point.y + y1)
    return point2


def get_line(point1, point2):
    """
    Finds a, b, c in ax + by = c from two points and returns as a list
    """
    x1 = point1.x
    x2 = point2.x
    y1 = point1.y
    y2 = point2.y

    if x1 == x2:
        return [1, 0, x1]

    m = (y1 - y2)/(x1 - x2)
    c = y1 - m*x1
    # a = -m, b = 1, c = c (rearrange y = mx + c)
    return [-m, 1, c]


def find_wall_vector(start_angle):
    """
    Finds the wall vector based on distance readings and the starting location of the robot
    wall_distances - a list of distances from the robot to the left wall
    start_location - a list with two elements (x and y) showing where the robot is located at the start of its movement
    """
    # Convert starting angle from radians to degrees
    angle_rad = math.radians(start_angle)
    # Find the vector direction
    x1 = math.sin(angle_rad)
    y1 = math.cos(angle_rad)
    return Vector(x1, y1)


def find_wall_point(left_wall_distances, start_position, start_angle):
    """
    Finds a point on the wall closest to the robot
    left_wall_distances - list of distances from robot to left wall
    start_position - list with two elements (x and y) for where the robot is
    start_angle - angle that the robot is facing. (North = 0)
    """
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


def check_points_intersect(lines):
    """
    Checks if two lines intersect or not
    """
    p1 = lines[0].point1
    p2 = lines[0].point2
    p3 = lines[1].point1
    p4 = lines[1].point2
    return ((p4.y - p3.y)*(p2.x - p1.x) - (p4.x - p3.x)*(p2.y - p1.y)) != 0


def check_lines_intersect_specific_both(lines):
    """
    Checks if two lines intersect within the point range in the points given for both points
    :param lines:
    :return true if there is an intersection within both the points:
    """
    if check_points_intersect(lines):
        poi = point_of_intersection(lines)
    else:
        return False
    p1 = lines[0].point1
    p2 = lines[0].point2
    p3 = lines[1].point1
    p4 = lines[1].point2

    x_max_1 = max(p1.x, p2.x)
    x_min_1 = min(p1.x, p2.x)

    x_max_2 = max(p3.x, p4.x)
    x_min_2 = min(p3.x, p4.x)

    return (x_min_1 <= poi.x <= x_max_1) and (x_min_2 <= poi.x <= x_max_2)


def check_lines_intersect_specific_one(lines):
    """
    Checks that two lines a) intersect and
    b) that the intersection point is within the lines
    :param lines:
    :return:
    """
    if check_points_intersect(lines):
        poi = point_of_intersection(lines)
    else:
        return False
    p1 = lines[0].point1
    p2 = lines[0].point2

    x_max_1 = max(p1.x, p2.x)
    x_min_1 = min(p1.x, p2.x)

    y_max_1 = max(p1.y, p2.y)
    y_min_1 = min(p1.y, p2.y)

    return x_min_1 <= poi.x <= x_max_1 and y_min_1 <= poi.y <= y_max_1


def point_of_intersection(lines):
    """
    Finds intersection between two lines
    """
    p1 = lines[0].point1
    p2 = lines[0].point2
    p3 = lines[1].point1
    p4 = lines[1].point2

    u = ((p4.x - p3.x)*(p1.y - p3.y) - (p4.y - p3.y)*(p1.x - p3.x))\
    /((p4.y - p3.y)*(p2.x - p1.x) - (p4.x - p3.x)*(p2.y - p1.y))
    x = p1.x + u*(p2.x - p1.x)
    y = p1.y + u*(p2.y - p1.y)
    return Point(x, y)


def check_direction_of_line(points, angle):
    x1 = points[0].x
    y1 = points[0].y
    x2 = points[1].x
    y2 = points[1].y

    # Convert starting angle from radians to degrees
    angle_rad = math.radians(angle)
    # Find the vector direction
    x_vec = math.sin(angle_rad)
    y_vec = math.cos(angle_rad)

    x_new = x1 + x_vec
    y_new = y1 + y_vec

    dist_orig = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2)
    dist_new = (x_new - x2)*(x_new - x2) + (y_new - y2)*(y_new - y2)

    if dist_new <= dist_orig:
        return True
    else:
        return False


def find_line_params(points, num_iterations, thresh_dist, inlier_ratio):
    """
    Uses the RANSAC algorithm to find the parameters of a line from a set of points with possible outliers
    Based from the MATLAB algorithm in https://en.wikipedia.org/wiki/Random_sample_consensus
    :param points: an Nx2 matrix containing all the points
    :param num_iterations: #iterations
    :param thresh_dist: threshold of distances between points and the fitting line to be counted as close
    :param inlier_ratio: threshold of the number of inliers required e.g. 0.4 = 40%
    :return: (a, b, c) from the equation ax + by + c = 0
    IMPORTANT: If a, b, c are all returned as 0 then no line was found. You may need to run the loop again
    """
    number = points.shape[0] # Total number of points
    best_in_num = 0 # Best fitting line with largest number of inliers
    best_a = 0
    best_b = 0
    best_c = 0 # parameters for best fitting

    for i in range(0, num_iterations):
        # Randomly select 2 points
        np.random.shuffle(points)
        sample = points[:2, :]

        k_line = sample[0, :] - sample[1, :] # two points relative distance
        k_line_norm = k_line/np.linalg.norm(k_line)

        norm_vector = [-k_line_norm[1], k_line_norm[0]] # Ax + By + c = 0 A = -k_line_norm[1], B = k_line_norm[0]

        distance = np.dot(points - np.tile(sample[0, :], (number, 1)), norm_vector)

        # Compute the inliers with distances smaller than the threshold
        inlier_idx = np.nonzero(abs(distance) <= thresh_dist)[0]

        inlier_num = len(inlier_idx)
        # Update the number of inliers and fitting model if better model is found

        if inlier_num >= round(inlier_ratio*number) and inlier_num > best_in_num:
            best_in_num = inlier_num
            x_diff = (sample[1, 0] - sample[0, 0])
            if x_diff != 0:
                # line is not vertical
                # y = mx + c  --> rearrange to mx - y + c = 0 (Ax + By + C = 0)
                best_a = (sample[1, 1] - sample[0, 1])/x_diff
                best_c = sample[0, 1] - best_a * sample[0, 0]
                best_b = -1
            else:
                # line is vertical, i.e. of the form x = C
                best_a = 1
                best_b = 0
                best_c = sample[1, 0]

    return best_a, best_b, best_c


def convert_points_to_matrix(points):
    """
    Converts a list of points to an Nx2 matrix
    :param points: list of all points
    :return: points in a matrix form (for multiplying etc.)
    """
    n = len(points)
    matrix = np.zeros((n, 2))
    for idx, point in enumerate(points):
        matrix[(idx, 0)] = point.x
        matrix[(idx, 1)] = point.y

    return matrix


def main():
    print("Goodbye cruel world...")


if __name__ == "__main__":
    main()