import re
import ev3dev.ev3 as ev3
import sys
#from client import *                         
                                              
m1=ev3.LargeMotor('outA')   # front           
m2=ev3.LargeMotor('outB')   # right           
m3=ev3.LargeMotor('outC')   # back            
m4=ev3.LargeMotor('outD')   # left            
                                              
                                              
ult1=ev3.UltrasonicSensor('in1') #Front-sensor
ult2=ev3.UltrasonicSensor('in2') #Right-sensor
ult3=ev3.UltrasonicSensor('in3') #Back-sensor 
ult4=ev3.UltrasonicSensor('in4') #Left-sensor

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


orientations = ["up", "right", "down", "left"]

def set_motors():
    global m
    frontMotor = m[0]
    rightMotor = m[1]
    backMotor = m[2]
    leftMotor = m[3]


def set_sensors():
    global ult
    frontSensor = ult[0]
    rightSensor = ult[1]
    backSensor = ult[2]
    leftSensor = ult[3]


def orientate_left():
    global orientations
    global m
    global ult

    temp_m = m.pop()
    m.insert(0, temp_m)
    set_motors()

    temp_ult = ult.pop()
    ult.insert(0, temp_ult)
    set_sensors()

    temp_orient = orientations.pop()
    orientations.insert(0, temp_orient)


def orientate_right():
    global orientations
    global m
    global ult

    temp_m = m.pop(0)
    m.append(temp_m)
    set_motors()

    temp_ult = ult.pop(0)
    ult.append(temp_ult)
    set_sensors()

    temp_orient = orientations.pop(0)
    orientations.append(temp_orient)


def run_motor(motor, speed, time):
    motor.run_timed(speed_sp=speed, time_sp=time)


def test_motor(name, speed, time_amount):
    speed = int(speed)
    time_amount = int(time_amount)*1000
    if time_amount < 0:
        print("Value for time should be positive")
        return
    elif speed > 1000:
        speed = 1000
    elif speed < -1000:
        speed = -1000

    if name == "A":
        run_motor(m1, speed, time_amount)
    elif name == "B":
        run_motor(m2, speed, time_amount)
    elif name == "C":
        run_motor(m3, speed, time_amount)
    elif name == "D":
        run_motor(m4, speed, time_amount)
    elif name == "front" or name == "f":
        run_motor(frontMotor, speed, time_amount)
    elif name == "back" or name == "b":
        run_motor(backMotor, speed, time_amount)
    elif name == "left" or name == "l":
        run_motor(leftMotor, speed, time_amount)
    elif name == "right" or name == "r":
        run_motor(rightMotor, speed, time_amount)
    else:
        print("Invalid settings given")


def test_sensor(name):
    if name == "1":
        print(ult1.distance_centimeters)
    elif name == "2":
        print(ult2.distance_centimeters)
    elif name == "3":
        print(ult3.distance_centimeters)
    elif name == "4":
        print(ult4.distance_centimeters)
    elif name == "front" or name == "f":
        print(frontSensor.distance_centimeters)
    elif name == "back" or name == "b":
        print(backSensor.distance_centimeters)
    elif name == "left" or name == "l":
        print(leftSensor.distance_centimeters)
    elif name == "right" or name == "r":
        print(rightSensor.distance_centimeters)
    else:
        print("Invalid input given!")


def test_rotation(direction, speed, time_amount):
    speed = int(speed)
    time_amount = int(time_amount) * 1000
    if time_amount < 0:
        print("Value for time should be positive")
        return
    elif speed > 1000:
        speed = 1000
    elif speed < -1000:
        speed = -1000

    if direction == "left":
        run_motor(m1, speed, time_amount)
        run_motor(m2, speed, time_amount)
        run_motor(m3, speed, time_amount)
        run_motor(m4, speed, time_amount)
    elif direction == "right":
        run_motor(m1, -speed, time_amount)
        run_motor(m2, -speed, time_amount)
        run_motor(m3, -speed, time_amount)
        run_motor(m4, -speed, time_amount)


def test_stop():
    m1.stop()
    m2.stop()
    m3.stop()
    m4.stop()


def test_turn(direction):
    if direction == "left":
        orientate_left()
    elif direction == "right":
        orientate_right()


def test_orientation():
    print("Current orientation is: {}".format(orientations[0]))


def main():
    print("Ready to test!\n-----------------")
    while True:
        comm = input("Enter what you want to test: ")
        if re.match('motor ([a-zA-Z]*) ([0-9]*) ([0-9]*)', comm):
            strings = comm.split(" ")
            test_motor(strings[1], strings[2], strings[3])
        elif re.match('sensor ([a-zA-Z]*)', comm):
            strings = comm.split(" ")
            test_sensor(strings[1])
        elif re.match('rotate_left ([0-9]*) ([0-9]*)', comm):
            strings = comm.split(" ")
            test_rotation("left", strings[1], strings[2])
        elif re.match('rotate_right ([0-9]*) ([0-9]*)', comm):
            strings = comm.split(" ")
            test_rotation("right", strings[1], strings[2])
        elif comm == "turn_left":
            test_turn("left")
        elif comm == "turn_right":
            test_turn("right")
        elif comm == "stop" or comm == "s":
            test_stop()
        elif comm == "orientation":
            test_orientation()
        else:
            print("Unexpected input. Please consult the user guide for a list of accepted inputs")


if __name__ == "__main__":
    main()

