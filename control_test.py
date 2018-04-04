#!/usr/bin/python3
print("Please wait...") # Encourage patience :-)

# http://www.inf.ed.ac.uk/teaching/courses/sdp/SDP2018/sdp_ev3.pdf

import ev3dev.ev3 as ev3
import getch

m1=ev3.LargeMotor('outA')   # front
m2=ev3.LargeMotor('outB')   # right
m3=ev3.LargeMotor('outC')   # back
m4=ev3.LargeMotor('outD')   # left
#s1=ev3.TouchSensor('in1')  # left sensor
#s2=ev3.TouchSensor('in2')  # right sensor

ult1=ev3.UltrasonicSensor('in1') #Front-sensor
ult2=ev3.UltrasonicSensor('in2') #Right-sensor
ult3=ev3.UltrasonicSensor('in3') #Back-sensor
ult4=ev3.UltrasonicSensor('in4') #Left-sensor




quit_moving = "Stop" # String to return if user decides to stop
obj_detected = "Obstacle" # String to return if there is an object detected

def moveBACKWARD(speed, time):
    m1.run_timed(speed_sp = -speed, time_sp = time)
    m3.run_timed(speed_sp = speed, time_sp = time)

def moveFORWARD(speed, time):
    m1.run_timed(speed_sp = speed, time_sp = time)
    m3.run_timed(speed_sp = -speed, time_sp = time)

def moveRIGHT(speed, time):
    m2.run_timed(speed_sp = speed, time_sp = time)
    m4.run_timed(speed_sp = -speed, time_sp = time)

def moveLEFT(speed, time):
    m2.run_timed(speed_sp = -speed, time_sp = time)
    m4.run_timed(speed_sp = speed, time_sp = time)

def rotateLEFT(speed, time):
    m1.run_timed(speed_sp = speed, time_sp = time)
    m2.run_timed(speed_sp = speed, time_sp = time)
    m3.run_timed(speed_sp = -speed, time_sp = time)
    m4.run_timed(speed_sp = -speed, time_sp = time)

def rotateRIGHT(speed, time):
    m1.run_timed(speed_sp = -speed, time_sp = time)
    m2.run_timed(speed_sp = -speed, time_sp = time)
    m3.run_timed(speed_sp = -speed, time_sp = time)
    m4.run_timed(speed_sp = -speed, time_sp = time)

def getFrontDistance():
    return ult1.distance_centimeters

def getRightDistance():
    return ult2.distance_centimeters

def getBackDistance():
    return ult3.distance_centimeters

def getLeftDistance():
    return ult4.distance_centimeters


def movementLoop():
    print("Ready to accept keyboard input!")
    while True:
        # If the bumper is touched
        if(False):        
        #if (s1.value() == 1) or (s2.value() ==1) :
            moveBACKWARD(100, 500)
            return obj_detected
        else:
            key = ord(getch.getch())
            if key == 27: #ESC
                input_val = input("Are you sure you want to stop moving? [y/n]")
                if (input_val == "y"):
                    return quit_moving
                else:
                    print("Continuing...")
                    continue
    # code for moving in a particular direction
            elif key == 56: # up arrow (numpad 8)
                moveFORWARD(100, 50)
            elif key == 50: # down arrow (numpad 2)
                moveBACKWARD(100, 50)
            elif key == 52: # left arrow (numpad 4)
                moveLEFT(100, 50)
            elif key == 54: # right arrow (numpad 6)
                moveRIGHT(100, 50)
             #   print('Box 1 dis: ' + str(getRightDistance()))
              #  print('Box 2 dis: ' + str(getBackDistance()))
              #  print('Invariant distance = ' + str(getRightDistance() + getBackDistance()))
            elif key == 55: # turn left (numpad 7)
               rotateLEFT(100, 100)
            elif key == 57: # turn right (numpad 9)
               rotateRIGHT(100, 100)

if __name__ == "__main__":

    while True:
        cause_of_stopping = movementLoop()
        if (cause_of_stopping == obj_detected):

            input_val = input("Object detected? Continue moving? [y/n]\n")
            if (input_val[-1] == "y"):
                continue
            else:
                print("System stopping...")
                break
        elif(cause_of_stopping == quit_moving):
            print("System stopping...")
            break
        else:
            print("Unexpected string returned: " + cause_of_stopping + "\nSystem stopping...")
            break
