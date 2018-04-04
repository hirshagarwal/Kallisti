import ev3dev.ev3 as ev3
from statistics import mean, pvariance
from collections import Counter
from MapObjects import Wall, Point
import time
import math
# from client import *

# Test the new branch
prev_wall = 'concave'
prev_back_distance = 0
m1 = ev3.LargeMotor('outA')   # front
m2 = ev3.LargeMotor('outB')   # right
m3 = ev3.LargeMotor('outC')   # back
m4 = ev3.LargeMotor('outD')   # left


ult1 = ev3.UltrasonicSensor('in1')  # Front-sensor
ult2 = ev3.UltrasonicSensor('in2')  # Right-sensor
ult3 = ev3.UltrasonicSensor('in3')  # Back-sensor
ult4 = ev3.UltrasonicSensor('in4')  # Left-sensor

# For button press:
butt = ev3.Button()

# Initialise motors and sensors to defaults - will change depending on orientation.

m = [m1, m2, m3, m4]
frontMotor = m[0]
rightMotor = m[1]
backMotor = m[2]
leftMotor = m[3]

ult = [ult1, ult2, ult3, ult4]
frontSensor = ult[0]
rightSensor = ult[1]
backSensor = ult[2]
leftSensor = ult[3]

# String values for messages passed to the path Loop
turn_right = "Right"
turn_left = "Left"

# Threshold for how close the robot needs to be from the front wall before it changes direction
front_threshold = 22

initial_time_step = 500

left_corner_threshold = 5

wheel_circumference = 15


# remember to initialise
self_location = (14.5, 17.5)  # (x, y)

start_point = Point(0, 0)
end_point = start_point

orientations = ["up", "right", "down", "left"]


def moveFORWARD(speed, t):
    frontMotor.run_timed(speed_sp=speed, time_sp=t)
    backMotor.run_timed(speed_sp=-speed, time_sp=t)


def moveBACKWARD(speed, t):
    moveFORWARD(-speed, t)


# Don't call the following two functions, they won't update robot's self location.
def moveRIGHT(speed, t):
    rightMotor.run_timed(speed_sp=speed, time_sp=t)
    leftMotor.run_timed(speed_sp=-speed, time_sp=t)


def moveLEFT(speed, t):
    rightMotor.run_timed(speed_sp=-speed, time_sp=t)
    leftMotor.run_timed(speed_sp=speed, time_sp=t)


def rotateLEFT(speed, t):
    frontMotor.run_timed(speed_sp=speed, time_sp=t)
    rightMotor.run_timed(speed_sp=speed, time_sp=t)
    backMotor.run_timed(speed_sp=speed, time_sp=t)
    leftMotor.run_timed(speed_sp=speed, time_sp=t)


def rotateRIGHT(speed, t):
    frontMotor.run_timed(speed_sp=-speed, time_sp=t)
    rightMotor.run_timed(speed_sp=-speed, time_sp=t)
    backMotor.run_timed(speed_sp=-speed, time_sp=t)
    leftMotor.run_timed(speed_sp=-speed, time_sp=t)


def move_left_approx_dist(distance=10):
    num_rotations = distance / wheel_circumference
    degrees = 360 * num_rotations
    rightMotor.run_to_rel_pos(position_sp=-degrees, speed_sp=500)
    leftMotor.run_to_rel_pos(position_sp=degrees, speed_sp=500)


def move_right_approx_dist(distance=10):
    num_rotations = distance / wheel_circumference
    degrees = 360 * num_rotations
    rightMotor.run_to_rel_pos(position_sp=degrees, speed_sp=500)
    leftMotor.run_to_rel_pos(position_sp=-degrees, speed_sp=500)


def move_forward_approx_dist(distance=10):
    num_rotations = distance / wheel_circumference
    degrees = 360 * num_rotations
    backMotor.run_to_rel_pos(position_sp=degrees, speed_sp=500)
    frontMotor.run_to_rel_pos(position_sp=-degrees, speed_sp=500)


def rotate_right_90_approx():
    frontMotor.run_to_rel_pos(position_sp=-320, speed_sp=400)
    rightMotor.run_to_rel_pos(position_sp=-320, speed_sp=400)
    backMotor.run_to_rel_pos(position_sp=-320, speed_sp=400)
    leftMotor.run_to_rel_pos(position_sp=-320, speed_sp=400)


def rotate_left_90_approx():
    frontMotor.run_to_rel_pos(position_sp=-300, speed_sp=500)
    rightMotor.run_to_rel_pos(position_sp=-300, speed_sp=500)
    backMotor.run_to_rel_pos(position_sp=-300, speed_sp=500)
    leftMotor.run_to_rel_pos(position_sp=-300, speed_sp=500)


def orientateLeft():
    global orientations
    temp_orient = orientations.pop()
    orientations.insert(0, temp_orient)


def orientateRight():
    global orientations
    temp_orient = orientations.pop(0)
    orientations.append(temp_orient)


def orientateBackwards():
    orientateLeft()
    orientateLeft()


def getFrontDistance():
    return getSingleDistanceHelper(frontSensor)


def getRightDistance():
    return getSingleDistanceHelper(rightSensor)


def getBackDistance():
    # print("Get Back Distance: ", backSensor == ult4)
    return getSingleDistanceHelper(backSensor)


def getLeftDistance():
    return getSingleDistanceHelper(leftSensor)


def getSingleDistanceHelper(sensor):

    # distances = [sensor.distance_centimeters for _ in range(10)]
    # while True:
    #     # The deviation of Ultrasonic Sensor is within 1 cm. If the standard variance of
    #     # a set of readings is greater than 1 (supposed to be 1), it must contains invalid data.
    #     if pvariance(distances) > 1:
    #         # Always trust smaller readings and remove larger ones.
    #         num_to_remove = max(distances)
    #         distances = [i for i in distances if i != num_to_remove]
    #     else:
    #         data_map = Counter(distances)
    #         most_common_value = max(set(distances), key=distances.count)
    #         # If the occurance of the most common value is greater than 13, return most_common_value.
    #         # Otherwise the mean of readings is more reliable.
    #         if data_map[most_common_value] >= 6:
    #             return most_common_value
    #         else:
    #             return mean(distances)
    if sensor == frontSensor or sensor == backSensor:
        readings = []
        for i in range(3):
            readings.append(sensor.distance_centimeters + 6)
            time.sleep(0.5)
        return mean(readings)
    else:
        return sensor.distance_centimeters + 6


def move_forward_small():
    moveFORWARD(100, 500)


def move_back_small():
    moveBACKWARD(100, 500)


def rotate_left_small():
    rotateLEFT(100, 500)


def rotate_right_small():
    rotateRIGHT(100, 500)


# This method can only be called after wallLoop()
def same_location(location1, location2):
    if (location1[0] - location2[0])**2 + (location1[1] - location2[1])**2 < 5**2:
        return True
    return False


def move_back_to_corner(time_amount, direction, approx_wall_dist):

    if time_amount <= 25:
        return

    if direction == "forward":
        while True:
            moveFORWARD(150, time_amount)
            left_dist = getLeftDistance()
            if is_close(left_dist, approx_wall_dist, 10):
                break
        move_back_to_corner(time_amount / 2, "backward", approx_wall_dist)

    else:
        while True:
            moveBACKWARD(150, time_amount)
            left_dist = getLeftDistance()
            if not is_close(left_dist, approx_wall_dist, 10):
                break
        move_back_to_corner(time_amount / 2, "forward", approx_wall_dist)


def move_forward_to_corner(time_amount, direction, approx_wall_dist):
    if time_amount <= 25:
        return

    if direction == "forward":
        while True:
            moveFORWARD(150, time_amount)
            time.sleep(time_amount / 1000)
            left_dist = getLeftDistance()
            if not is_close(left_dist, approx_wall_dist, 10):
                break
        move_forward_to_corner(time_amount / 2, "backward", approx_wall_dist)

    elif direction == "backward":
        while True:
            moveBACKWARD(150, time_amount)
            time.sleep(time_amount / 1000)
            left_dist = getLeftDistance()
            if is_close(left_dist, approx_wall_dist, 10):
                break
        move_back_to_corner(time_amount / 2, "forward", approx_wall_dist)


def turn_left_convex():
    # move_right_approx_dist(10)
    # time.sleep(2)
    # move_forward_approx_dist(25)
    # time.sleep(2)
    # rotate_left_90_approx()
    # time.sleep(1)
    # move_forward_approx_dist(25)
    # crash_into_wall("towards")
    # move_back_to_corner(800, "backwards", 12)
    moveFORWARD(300, 2700)
    moveLEFT(300, 2700)


def crash_into_wall(direction):
    """
    Lets the robot gracefully crash into a wall
    :param direction: has values towards or away depending on if the robot is moving towards or away from the wall
    :return:
    """
    distances = [999]
    num_distance_errors = 0
    while True:
        dist_left = getLeftDistance()
        print("dist_left = {}".format(dist_left))
        distances.append(dist_left)
        print("prev_dist = {}\n".format(distances[-2]))
        if distances[-2] > dist_left > 12.1:
            moveLEFT(200, 1000)
            time.sleep(1)
        elif distances[-2] == dist_left and dist_left > 12.2:
            if direction == "towards":
                rotate_right_small()
                time.sleep(0.5)
            else:  # direction == "away":
                rotate_left_small()
                time.sleep(0.5)
            moveLEFT(100, 500)
            time.sleep(0.5)
        elif num_distance_errors > 10:
            print("Large number of errors seen!")
            # Move back a little if there are a lot of erroneous readings
            moveRIGHT(100, 500)
            time.sleep(0.5)
            rotate_left_small()
            time.sleep(0.5)
            num_distance_errors = 0
        elif dist_left <= 12 or dist_left >= 255:
            # Ignore readings less than 12 or 255 as they are wrong
            distances.pop()
            num_distance_errors += 1
            continue
        else:
            check_left = getLeftDistance()
            if check_left <= 12.1:
                moveLEFT(100, 2000)
                time.sleep(2)
                break


def turn_right_concave():
    move_right_approx_dist(3)
    time.sleep(2)
    rotate_right_90_approx()
    time.sleep(2)
    move_left_approx_dist(5)
    time.sleep(2)
    moveBACKWARD(300, 1000)
    moveLEFT(300, 1000)
    time.sleep(2)
    moveBACKWARD(300, 500)

    # crash_into_wall("towards")


def move_to_start_convex_corner(time_step):
    """
    Makes the robot move to the start of a convex corner
    :param: time_step - time to move forwards/back for
    :return:
    """
    # If this is the first time the robot has moved, rotate to 90 degrees.
    pass


def find_new_wall(distance):
    global end_point
    temp_point = end_point
    print("\nStart point = {}".format(temp_point))
    x = end_point.x
    y = end_point.y
    cur_orientation = current_orientation()
    print("Orientation = {}".format(cur_orientation))
    if cur_orientation == "up":
        end_point = Point(x, y + distance)
        print("End point = {}\n".format(end_point))
        return Wall(temp_point, end_point)
    elif cur_orientation == "down":
        end_point = Point(x, y - distance)
        print("End point = {}\n".format(end_point))
        return Wall(temp_point, end_point)
    elif cur_orientation == "left":
        end_point = Point(x - distance, y)
        print("End point = {}\n".format(end_point))
        return Wall(temp_point, end_point)
    else:
        end_point = Point(x + distance, y)
        print("End point = {}\n".format(end_point))
        return Wall(temp_point, end_point)


def find_location_from_corner(wall, front_dist, corner_type):
    wall_end_point = wall.endPoint
    x = wall_end_point.x
    y = wall_end_point.y
    cur_orientation = current_orientation()
    if corner_type == "convex":
        if cur_orientation == "up":
            return Point(x + 12, y)
        elif cur_orientation == "down":
            return Point(x - 12, y)
        elif cur_orientation == "left":
            return Point(x, y + 12)
        else:
            return Point(x, y - 12)
    elif corner_type == "concave":
        if cur_orientation == "up":
            return Point(x + 12, y - front_dist)
        elif cur_orientation == "down":
            return Point(x - 12, y + front_dist)
        elif cur_orientation == "left":
            return Point(x + front_dist, y + 12)
        else:
            return Point(x - front_dist, y - 12)


def current_orientation():
    return orientations[0]


def path_loop_2():
    global prev_wall
    global prev_back_distance
    left_init = getLeftDistance()
    back_init = getBackDistance()
    prev_corner_type = "concave"
    current_location = Point(0, 0)
    walls = []
    # should start each loop pressed against the wall
    while True:
        move_right_approx_dist(8)
        next_instruction = wall_loop_2(left_init)
        if prev_corner_type == "concave" and next_instruction == "Right":
            front_end = getFrontDistance()
            back_end = getBackDistance()
            total_length = front_end + back_end
            # Find new wall based on orientation
            new_wall = find_new_wall(total_length)
            current_location = find_location_from_corner(new_wall, front_end, "concave")
            # Change orientation
            orientateRight()
            # prev_corner type doesn't need to be changed

        elif prev_corner_type == "concave" and next_instruction == "Left":
            front_end = getFrontDistance()
            back_end = getBackDistance()
            total_length = back_end
            new_wall = find_new_wall(total_length)
            current_location = find_location_from_corner(new_wall, front_end, "convex")
            orientateLeft()
            prev_corner_type = "convex"

        elif prev_corner_type == "convex" and next_instruction == "Right":
            front_end = getFrontDistance()
            back_end = getBackDistance()
            total_length = front_end + back_end - back_init
            new_wall = find_new_wall(total_length)
            current_location = find_location_from_corner(new_wall, front_end, "concave")
            orientateRight()
            prev_corner_type = "concave"

        else:  # prev_corner_type = "convex" and next_instruction = "Left"
            front_end = getFrontDistance()
            back_end = getBackDistance()
            total_length = back_end - back_init
            new_wall = find_new_wall(total_length)
            current_location = find_location_from_corner(new_wall, front_end, "convex")
            orientateLeft()

        walls.append(new_wall)
        to_next_wall_2(next_instruction)
        # Reset wall measurements after moving to next wall
        left_init = getLeftDistance()
        back_init = getBackDistance()


def wall_loop_2(left_init_dist):
    while True:
        moveFORWARD(300, 1000)
        time.sleep(1)
        new_left = getLeftDistance()
        new_front = getFrontDistance()

        # If a new wall is found, return the total length of the wall
        print("left_init_dist: {}, new_left: {}, new_front: {}".format(
            left_init_dist, new_left, new_front))
        if check_new_wall(left_init_dist, new_left):
            crash_into_wall("away")
            move_forward_to_corner(800, "backwards", left_init_dist)
            return turn_left

        elif new_front <= front_threshold:
            return turn_right

        elif err_check_too_far(left_init_dist, new_left):
            crash_into_wall("away")
            move_right_approx_dist(8)

        elif err_check_too_close(left_init_dist, new_left):
            crash_into_wall("towards")
            move_right_approx_dist(8)


def to_next_wall_2(instruction):
    if instruction == 'Left':
        print("turn left")
        turn_left_convex()
    else:
        print("turn right")
        turn_right_concave()


def err_check_too_close(left_init, left_current):
    return left_init - left_current >= 2


def err_check_too_far(left_init, left_current):
    return left_current - left_init >= 2


def check_new_wall(left_init, left_current):
    # Check if the current reading is a new wall, and repeats to be sure
    return left_current - left_init >= left_corner_threshold and getLeftDistance() - left_init >= left_corner_threshold


def is_close(num_1, num_2, threshold):
    return abs(num_1 - num_2) <= threshold


def is_very_different(num_1, num_2):
    return abs(num_1 - num_2) >= 10


def path_loop_demo():
    global prev_wall
    global prev_back_distance
    left_init = getLeftDistance()
    back_init = getBackDistance()
    prev_corner_type = "concave"
    walls = []

    while True:
        input_1 = input("Enter the next direction")
        if input_1 == "l" or input_1 == "r":
            next_instruction = input_1
        else:
            print("Not a valid input. Should be \"l\" or \"r\"")
            continue
        print("Current orientation: {}\nInitial back distance: {}\nPrevious corner type: {}\n".format(
            orientations[0], back_init, prev_corner_type))

        if prev_corner_type == "concave" and next_instruction == "r":
            front_end = getFrontDistance()
            back_end = getBackDistance()
            total_length = front_end + back_end
            print("Front end length = {}\nBack end length = {}\nTotal length = {}".format(
                front_end, back_end, total_length))
            # Find new wall based on orientation
            new_wall = find_new_wall(total_length)
            # Change orientation
            orientateRight()
            # prev_corner type doesn't need to be changed

        elif prev_corner_type == "concave" and next_instruction == "l":
            back_end = getBackDistance()
            total_length = back_end
            print("Back end length = {}\nTotal length = {}".format(
                back_end, total_length))
            new_wall = find_new_wall(total_length)
            orientateLeft()
            prev_corner_type = "convex"

        elif prev_corner_type == "convex" and next_instruction == "r":
            front_end = getFrontDistance()
            back_end = getBackDistance()
            total_length = front_end + back_end - back_init
            print("Front end length = {}\nBack end length = {}\nTotal length = {}".format(front_end, back_end,
                                                                                          total_length))
            new_wall = find_new_wall(total_length)
            orientateRight()
            prev_corner_type = "concave"

        elif prev_corner_type == "convex" and next_instruction == "Left":
            back_end = getBackDistance()
            total_length = back_end - back_init
            print("Back end length = {}\nTotal length = {}".format(
                back_end, total_length))
            new_wall = find_new_wall(total_length)
            orientateLeft()
        print("New orientation = {}".format(orientations[0]))
        print("New wall start point: {}\nEnd point: {}".format(
            new_wall.startPoint, new_wall.endPoint))
        walls.append(new_wall)
        input_continue = input("Press any key to continue")
        # Reset wall measurements after moving to next wall
        back_init = getBackDistance()


if __name__ == "__main__":
    print("Starting")
    # path_loop_demo()
    path_loop_2()
