import ev3dev.ev3 as ev3
from statistics import mean, pvariance
from collections import Counter
from MapObjects import Wall, Point
import time
import math
#from client import *

# Test the new branch

m1=ev3.LargeMotor('outA')   # front
m2=ev3.LargeMotor('outB')   # right
m3=ev3.LargeMotor('outC')   # back
m4=ev3.LargeMotor('outD')   # left


ult1=ev3.UltrasonicSensor('in1') #Front-sensor
ult2=ev3.UltrasonicSensor('in2') #Right-sensor
ult3=ev3.UltrasonicSensor('in3') #Back-sensor
ult4=ev3.UltrasonicSensor('in4') #Left-sensor

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
# TODO test threshold and find appropriate value - chosen arbitrarily currently
front_threshold = 18.5

# TODO test initial_time_step and find appropriate value
initial_time_step = 100

left_corner_threshold = 10

# String values for different orientations - easier to pass as arguments
orient_front = "Front" # 0
orient_right = "Right" # 1
orient_back = "Back" # 2
orient_left = "Left" # 3


# remember to initialise
self_location = (14.5, 17.5) # (x, y)
orientation = 0


start_point = Point(0, 0)
end_point = start_point

orientations = ["up", "right", "down", "left"]


def moveFORWARD(speed, t):
    frontMotor.run_timed(speed_sp = 2 * speed, time_sp = t)
    backMotor.run_timed(speed_sp = -2 * speed, time_sp = t)


def moveBACKWARD(speed, time):
    moveFORWARD(-speed, time)


# Don't call the following two functions, they won't update robot's self location.
def moveRIGHT(speed, time):
    rightMotor.run_timed(speed_sp = speed, time_sp = time)
    leftMotor.run_timed(speed_sp = -speed, time_sp = time)


def moveLEFT(speed, time):
    rightMotor.run_timed(speed_sp = -speed, time_sp = time)
    leftMotor.run_timed(speed_sp = speed, time_sp = time)


def rotateLEFT(speed, time):
    frontMotor.run_timed(speed_sp = speed, time_sp = time)
    rightMotor.run_timed(speed_sp = speed, time_sp = time)
    backMotor.run_timed(speed_sp = speed, time_sp = time)
    leftMotor.run_timed(speed_sp = speed, time_sp = time)


def rotateRIGHT(speed, time):
    frontMotor.run_timed(speed_sp = -speed, time_sp = time)
    rightMotor.run_timed(speed_sp = -speed, time_sp = time)
    backMotor.run_timed(speed_sp = -speed, time_sp = time)
    leftMotor.run_timed(speed_sp = -speed, time_sp = time)


def orientateLeft():
    print("left")
    global orientation
    global frontMotor
    global rightMotor
    global leftMotor
    global backMotor
    global frontSensor
    global rightSensor
    global leftSensor
    global backSensor
    orientation = (orientation - 1 + 4) % 4

    temp_motor = m.pop()
    m.insert(0, temp_motor)
    frontMotor = m[0]
    rightMotor = m[1]
    backMotor = m[2]
    leftMotor = m[3]

    temp_sensor = ult.pop()
    ult.insert(0, temp_sensor)
    frontSensor = ult[0]
    rightSensor = ult[1]
    backSensor = ult[2]
    leftSensor = ult[3]

    temp_orient = orientations.pop()
    orientations.insert(0, temp_orient)


def orientateForwards():
    pass


def orientateRight():
    print("right")
    global orientation
    global orientation
    global frontMotor
    global rightMotor
    global leftMotor
    global backMotor
    global frontSensor
    global rightSensor
    global leftSensor
    global backSensor

    orientation= (orientation + 1 + 4) % 4

    temp_motor = m.pop(0)
    m.append(temp_motor)
    frontMotor = m[0]
    rightMotor = m[1]
    backMotor = m[2]
    leftMotor = m[3]

    temp_sensor = ult.pop(0)
    ult.append(temp_sensor)
    frontSensor = ult[0]
    rightSensor = ult[1]
    backSensor = ult[2]
    leftSensor = ult[3]

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
    distances = [sensor.distance_centimeters for _ in range(10)]
    while True:
        # The deviation of Ultrasonic Sensor is within 1 cm. If the standard variance of
        # a set of readings is greater than 1 (supposed to be 1), it must contains invalid data.
        #TODO find a appropriate value for standard variance
        if pvariance(distances) > 1:
            # Always trust smaller readings and remove larger ones.
            num_to_remove = max(distances)
            distances = [i for i in distances if i != num_to_remove]
        else:
            data_map = Counter(distances)
            most_common_value = max(set(distances), key=distances.count)
            # If the occurance of the most common value is greater than 13, return most_common_value.
            # Otherwise the mean of readings is more reliable.
            if data_map[most_common_value] >= 6:
                return most_common_value
            else:
                return mean(distances)


def getDistances():
    return [getFrontDistance(), getRightDistance(), getBackDistance(), getLeftDistance()]


def moveForwardSmall():
    moveFORWARD(100, 500)


def moveBackSmall():
    moveBACKWARD(100, 500)


def rotateLeftSmall():
    rotateLEFT(100, 500)


def rotateRightSmall():
    rotateRIGHT(100, 500)


# This method can only be called after wallLoop()
def same_location(location1, location2):
    if (location1[0]-location2[0])**2 + (location1[1]-location2[1])**2 < 5**2:
        return True
    return False


def toNextWall(direction, distance_to_prev_wall):
    half_width_of_robot = 15
    ideal_distance_to_wall = 11
    if direction == turn_left:
        moveForward(half_width_of_robot + ideal_distance_to_wall)
        orientateLeft()
        moveForward(distance_to_prev_wall + 5)
    elif direction == turn_right:
        orientateRight()
    next_left_init = getLeftDistance()
    next_invariant_init = getFrontDistance() + getBackDistance()
    return next_left_init, next_invariant_init


def moveForward(distance):
    front1 = front2 = getFrontDistance()
    back1 = back2 = getBackDistance()
    while ((front1 - front2) + (back2 - back1)) / 2 < distance:
        moveForwardSmall()
        front2 = getFrontDistance()
        back2 = getBackDistance()
        if front2 < front_threshold:
            print("too close!")
            exit()


def find_90_rotate(power_rotation, time_rotation, direction, kind):
    """
    Test function to get the robot to rotate in place so that it finds a minimum
    Assuming robot is moving away
    """
    print("rotating {} with power: {} and time: {}".format(direction, power_rotation, time_rotation))
    min_left= getLeftDistance()


    if kind == "power":
        if power_rotation <= 100:
            # Stop recursing and finish
            return
    else:
        if time_rotation < 100:
            if direction == "left":
                rotateLEFT(power_rotation, time_rotation*2)
            else:
                rotateRIGHT(power_rotation, time_rotation*2)
            return

    while True:
        if direction == "left":
            rotateLEFT(power_rotation, time_rotation)
        else:
            rotateRIGHT(power_rotation, time_rotation)
        new_left = getLeftDistance()
        print("new left = {}".format(new_left))
        # If the value is greater the robot must have rotated past the minimum
        if new_left < min_left:
            check_left = getLeftDistance()
            if check_left < min_left:
                min_left = (new_left + check_left) / 2
        else:
            break

    if direction == "left":
        if kind == "power":
            find_90_rotate(power_rotation / 2, time_rotation, "right", "power")
        else:
            find_90_rotate(power_rotation, time_rotation / 2, "right", "time")
    else:
        if kind == "power":
            find_90_rotate(power_rotation / 2, time_rotation, "left", "power")
        else:
            find_90_rotate(power_rotation, time_rotation / 2, "left", "time")



def should_rotate_left():
    """
    Finds if the robot needs to rotate left or right to obtain the minimum distance
    :return:
    """
    dist_2 = getLeftDistance()
    rotateLeftSmall()
    dist_1 = getLeftDistance()
    rotateRightSmall()
    rotateRightSmall()
    dist_3 = getLeftDistance()

    # If the min distance was the rightmost one, need to go further right to find min
    if dist_3 <= dist_2 and dist_3 <= dist_1:
        return False
    # If the min distance was in the middle, or on the left, then you need to rotate left
    else:
        return True


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





def move_to_start_convex_corner(time_step):
    """
    Makes the robot move to the start of a convex corner
    :param: time_step - time to move forwards/back for
    :return:
    """
    # If this is the first time the robot has moved, rotate to 90 degrees.
    if time_step == initial_time_step:
        if should_rotate_left():
            find_90_rotate(time_step, "left")
        else:
            find_90_rotate(time_step, "right")




def find_new_wall(distance):
    global end_point
    temp_point = end_point
    x = end_point.x
    y = end_point.y
    cur_orientation = current_orientation()
    if cur_orientation == "up":
        end_point = Point(x, y + distance)
        return Wall(temp_point, end_point)
    elif orientations == "down":
        end_point = Point(x, y - distance)
        return Wall(temp_point, end_point)
    elif cur_orientation == "left":
        end_point = Point(x - distance, y)
        return Wall(temp_point, end_point)
    else:
        end_point = Point(x + distance, y)
        return Wall(temp_point, end_point)


def current_orientation():
    return orientations[0]


def path_loop_2():
    left_init = getLeftDistance()
    front_init = getFrontDistance()
    back_dist = getBackDistance()
    walls = []

    while True:
        next_instruction, wall_length = wall_loop_2()
        walls.append(find_new_wall(wall_length))


def wall_loop_2(left_init_dist, front_init_dist, wall_init_length):
    left_distances = [left_init_dist, 0]
    front_distances = [0]
    front_distance_travelled = 0
    wall_length = wall_init_length

    front_temp = front_init_dist

    while True:
        moveFORWARD(100, 500)
        new_left = getLeftDistance()
        new_front = getFrontDistance()
        front_change = front_temp - new_front
        front_temp = new_front
        front_distance_travelled += front_change
        left_distances[1] = new_left

        # If a new wall is found, return the total length of the wall
        if check_new_wall(left_init_dist, new_left):
            wall_length = calculate_wall_length(wall_length)
            return turn_left, wall_length

        elif new_front <= front_threshold:
            return turn_right, wall_length

        elif err_check_too_far(left_init_dist, new_left):
            correct_error_far(left_init_dist, new_left, front_distance_travelled)

        elif err_check_too_close(left_init_dist, new_left):
            correct_error_near(left_init_dist, new_left, front_distance_travelled)


def err_check_too_close(left_init, left_current):
    return left_init - left_current >= 2


def err_check_too_far(left_init, left_current):
    return left_current - left_init >= 2


def check_new_wall(left_init, left_current):
    # Check if the current reading is a new wall, and repeats to be sure
    return left_current - left_init >= left_corner_threshold and getLeftDistance() - left_init >= left_corner_threshold


# Calculates length of wall, allowing for the robot to have moved away somewhat.
def calculate_wall_length(prev_length, left_init, left_current, front_travelled):
    x_diff = abs(left_init - left_current)
    length_1 = math.sqrt(x_diff*x_diff + front_travelled*front_travelled)
    if left_init <= left_current:
        theta = math.atan(x_diff/front_travelled)
        length_2 = math.sin(theta)*left_init

    else:
        if x_diff != 0:
            theta = math.atan(front_travelled/x_diff)
            length_2 = math.cos(theta)*left_current
        else:
            length_2 = 0

    return prev_length + length_1 + length_2


def correct_error_far(left_init, left_current, front_travelled, prev_wall_length):
    x_diff = left_current - left_init
    theta = math.atan(front_travelled/x_diff)
    expected_left = math.sin(theta)*left_current
    rotate_left_to_distance(expected_left, left_init, initial_time_step)

    wall_removed = math.cos(theta)*left_current
    new_wall_length = prev_wall_length - wall_removed


def rotate_left_to_distance(dist_target, dist_init, time):
    prev_left = dist_init
    while True:
        rotateLeftSmall()
        new_left = getLeftDistance()
        if is_close(dist_target, new_left, 2):
            break
        else:
            #TODO finish
            continue


def correct_error_near(left_init, left_current, front_travelled, prev_wall_length):
    #TODO finish
    pass


def is_close(num_1, num_2, threshold):
    return abs(num_1 - num_2) <= threshold


def is_very_different(num_1, num_2):
    return abs(num_1 - num_2) >= 10


if __name__ == "__main__":
    orientateLeft()
    init_left = getLeftDistance()
    move_back_to_corner(400, "backward", init_left)
