import turtle
import os

self_location = (10, 10)

# Call this method before the first time of drawing points. Only need to call it once.
def start_drawing():
    turtle.pencolor("black")

    # draw coordinate axis
    turtle.goto((500, 0))
    turtle.goto((-500, 0))
    turtle.goto((0, 0))
    turtle.goto((0, 500))
    turtle.goto((0, -500))
    turtle.goto((0, 0))
    turtle.penup()

    if os.path.exists("./blueprint.txt"):
        with open("./blueprint.txt", 'r') as f:
            lines = f.readlines()
        first_location = format_string(lines[0])
        turtle.goto((first_location[0], first_location[1]))
        turtle.pendown()
        for line in lines:
            location = format_string(line)
            turtle.goto(location[0], location[1])
        turtle.goto((first_location[0], first_location[1]))
        turtle.penup()



def format_string(s):
    s = s.strip()
    s = s.replace('(', '')
    s = s.replace(')', '')
    data = [int(i) for i in s.split(',')]
    return data


# @param location: tuple (x, y)
# @param flag: When flag is 1, the location is robot's self-location; When
# flag is 0, the location is a measured point of the wall.
def draw(location, flag = 0):
    global self_location
    if flag == 0:
        turtle.pencolor("blue")
        turtle.penup()
        turtle.goto(location)
        turtle.pendown()
        turtle.dot()
    elif flag == 1:
        turtle.pencolor("red")
        turtle.penup()
        turtle.goto(self_location)
        turtle.pendown()
        # turtle will draw a continuous line from last self_location to the new self location, 
        # representing the path of the robot.
        turtle.goto(location)
        self_location = location

# Call this method when drawing is finshed
def end_drawing():
    turtle.done()


