#print("Please wait...") # Encourage patience :-)



import ev3dev.ev3 as ev3
import getch
from statistics import mean, pvariance, mode
from collections import Counter
import time
#from client import *

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


# String values for error messages - keep consistent
# Shows if the wall is too near/far away/new wall found
err_left_near = "errLeftNear"
err_left_far_moving = "errLeftFarMoving"
err_new_wall = "errLeftFarNewWall"
# Shows if the invariant (vertical distance) is different (now too small/large)
err_invariant_small = "errInvariantSmall"
err_invariant_large = "errInvariantLarge"


err_both = "errBoth"
no_err = "noErr"

# String values for messages passed to the path Loop
turn_right = "Right"
turn_left = "Left"

# Threshold for how close the robot needs to be from the front wall before it changes direction
# TODO test threshold and find appropriate value - chosen arbitrarily currently
front_threshold = 18.5


# String values for different orientations - easier to pass as arguments
orient_front = "Front" # 0
orient_right = "Right" # 1
orient_back = "Back" # 2
orient_left = "Left" # 3


# remember to initialise
self_location = (14.5, 17.5) # (x, y)
orientation = 0



def moveFORWARD(speed, t):
    front1 = getFrontDistance()
    back1 = getBackDistance()
    print("front1", front1)
    print("back1", back1)

    frontMotor.run_timed(speed_sp = 2 * speed, time_sp = t)
    backMotor.run_timed(speed_sp = -2 * speed, time_sp = t)

    time.sleep(1)

    front2 = getFrontDistance()
    back2 = getBackDistance()

    print("front2", front2)
    print("back2", back2)

    # avg_value = ((front1 - front2) + (back2 - back1)) / 2
    avg_value = (front1 - front2)
    print("avg: ", avg_value)
    updateLocation(avg_value)
    print(self_location)

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

def orientateBackwards():
    orientateLeft()
    orientateLeft()

def updateLocation(distance):
    global self_location
    if orientation == 0: # Front
        self_location = (self_location[0],self_location[1] + distance)
    elif orientation == 1: # Right
        self_location = (self_location[0] + distance, self_location[1])
    elif orientation == 2: # Back
        self_location = (self_location[0],self_location[1] -  distance)
    elif orientation == 3: # Left
        self_location = (self_location[0] - distance, self_location[1])
    #toSend(self_location[0], self_location[1], "self_location")


def toPoint(distance):
    if orientation == 0: # Front
        #print(distance)
        return (self_location[0] - distance, self_location[1])
    elif orientation == 1: # Right
        return (self_location[0], self_location[1] + distance)
    elif orientation == 2: # Back
        return (self_location[0] + distance, self_location[1])
    elif orientation == 3: # Left
        return (self_location[0], self_location[1] - distance)



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
    moveFORWARD(100, 100)

def moveBackSmall():
    moveBACKWARD(100, 100)


def rotateLeftSmall():
    rotateLEFT(100, 100)

def rotateRightSmall():
    rotateRIGHT(100, 100)


# Check that the readings are close enough to the initial values for the left wall
# distances are a list of [F, R, B, L]
def checkLeftWall(left_init, distances):
    variance = distances[3] - left_init
    # The measurement is within +/- 1 of the initial value.
    if -1 <= variance <= 1:
        return no_err
    # The measurement is 1 less than it was (implying it is too near the wall)
    elif variance < -1:
        return err_left_near
    # The measurement is 1 more than it was before (implying it has gone too far away)
    # This may be due to natural error in the instrument, or could be because of a new wall
    elif variance > 1 and variance <= 10:
        #TODO check threshold of 10 is appropriate here.
        return err_left_far_moving
    else:
        return err_new_wall

# Check that the readings are close enough to the initial values for the invariant (vertical distance)
# distances are a list of [F, R, B, L]
def checkInvariant(invariant_init, distances):
    variance = (distances[0] + distances[2]) - invariant_init
    # The measurements are within +/- 2 of the inital value
    if -2 <= variance <= 2:
        return no_err
    # The measurements are at least 2 less than before (implying the invariant is too small now)
    elif variance < -2:
        return err_invariant_small
    # The measurements are at least 2 more than before (implying the invariant is too large now)
    else:
        return err_invariant_large


# Loop to follow each wall until the robot comes back to its initial location
def pathLoop(location, init_orientation=0):
    global self_location
    init_location = location
    self_location = location
    global orientation
    orientation = init_orientation
    not_start = True
    #print("right: ", getRightDistance())
    #print("back: ", getBackDistance())
    #exit()
    left_init = getLeftDistance()
    invariant_init = getFrontDistance() + getBackDistance()
    while not_start or not same_location(self_location, init_location):
        not_start = False
        direction, distance = wallLoop(left_init,invariant_init)
        left_init, invariant_init = toNextWall(direction, distance)
       # print(self_location)
       # print(init_location)






# Loop to follow a single wall
# left_init is the intial distance from the wall on the left.
# invariant_init is the intial value of distance of the front and back sensors
# added together.
# orientation is the current orientation relative to the left wall and the map

# returns "go left/right" to show which direction the robot needs to move next.


# TODO estimate current position based on readings vs old ones.
# Keep track and store as (position, distance readings) tuple perhaps?
# Main problem: orientation changes
def wallLoop(left_init, invariant_init):
    # Stores disatnce readings (all in a list, could change)
    distance_readings = []
    while True:

        # Take in sensor readings
        distances = getDistances()

        # Check that the readings are relatively error-free (can't be 100% sure but close)
        check_left_msg = checkLeftWall(left_init, distances)
        check_invariant_msg = checkInvariant(invariant_init, distances)

        # There was no significant error detected

        if True:
        #if check_left_msg == no_err and check_invariant_msg == no_err:
            print("front", distances[0], "right", distances[1], "back", distances[2],  "left", distances[3])
            # add the distances to the overall readings, (orientated correctly)
            new_point = toPoint(distances[3])
            distance_readings.append(new_point)
            # Send new readings back to self-managed machine
            #	toSend(new_point[0], new_point[1], "point")
            # If the distance from the front is greater than the threshold, keep going
            if distances[0] > front_threshold:
                # Move forward by some amount (can change)
                moveFORWARD(100, 500)
                continue
            # The distance is less, break from the loop and tell it to turn right next.
            else:
                return (turn_right, distances[0])
        # TODO cover case of invariant error - wasn't sure how best to respond
        # The left wall is too close (may be heading towards it)
        if check_left_msg == err_left_near:
            # Rotate right and move back slightly to try to fix the error
            rotateRightSmall()
            moveBackSmall()
            # Loop again (will check if it's fixed)
            continue

        # The left wall is too far away - may be heading away from it as error is small
        if check_left_msg == err_left_far_moving:
            # Rotate left and move back slightly to try to fix the error
            rotateLeftSmall()
            moveBackSmall()
            continue

        # A new wall has potentially been found.
        if check_left_msg == err_new_wall:
            # Move forward and back slightly and check the distances are the same
            # Done in case the error is simply due to an anomaly
            moveForwardSmall()
            checking_distances = getDistances()
            moveBackSmall()
            checking_distances2 = getDistances()

            check_msg_1 = checkLeftWall(left_init, checking_distances)
            check_msg_2 = checkLeftWall(left_init, checking_distances2)

            # If both messages indicate a new wall, then it probably is
            if check_msg_1 == err_new_wall and check_msg_2 == err_new_wall:
                # break out of the loop
                return (turn_left, distances[3])
            else:
                # Rotate left and move back slightly to try to fix the error
                rotateLeftSmall()
                moveBackSmall()
                continue
        # Some invariant error
        #TODO fix this - shouldn't happen but might
        else:
            # Rotate left and move back slightly to try to fix the error
            rotateLeftSmall()
            moveBackSmall()
            continue

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
    return (next_left_init, next_invariant_init)

def moveForward(distance):
    front1 = front2 = getFrontDistance()
    back1 = back2 = getBackDistance()
    while ((front1 - front2) + (back2 - back1)) / 2 < distance:
        moveForwardSmall()
        front2 = getFrontDistance()
        back2 = getBackDistance()
        if(front2 < front_threshold):
            print("too close!")
            exit()


def find_90_rotate_left(time_rotation):
    """
    Test function to get the robot to rotate in place so that it finds a minimum
    Assuming robot is moving away
    """
    min_left = getLeftDistance()
    found_min = False

    # TODO test which time of rotation as a minimum works best
    if time_rotation < 25:
        # Stop recursing and finish
        return

    while not found_min:
        # TODO test which values work best for very small movements
        rotateLEFT(100, time_rotation)
        new_left = getLeftDistance()
        # If the value is greater the robot must have rotated past the minimum
        if new_left > min_left:
            found_min = True

    # Go back to the point just before the previous minimum
    rotateRIGHT(100, time_rotation*2)
    find_90_rotate_left(time_rotation / 2)


def find_90_rotate_right(time_rotation):
    """
    Test function to get the robot to rotate in place so that it finds a minimum.
    Assuming robot is moving towards the wall
    """
    min_left = getLeftDistance()
    found_min = False

    # TODO test which time of rotation as a minimum works best
    if time_rotation < 25:
        # Stop recursing and finish
        return

    while not found_min:
        # TODO test which values work best for very small movements
        rotateRIGHT(100, time_rotation)
        new_left = getLeftDistance()
        # If the value is greater the robot must have rotated past the minimum
        if new_left > min_left:
            found_min = True

    # Go back twice and start rotating again more slowly
    rotateLEFT(100, time_rotation*2)
    find_90_rotate_right(time_rotation / 2)



if __name__ == "__main__":
    while(not butt.up):
        pass
    pathLoop((14.5,17.5))
