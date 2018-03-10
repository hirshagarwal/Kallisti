import turtle

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

if __name__ == "__main__":
  start_drawing()

  draw((0.2, 10))
  draw((-0.3, 10))
  draw((10, 12), 1)
  draw((0.33, 12))
  draw((10, 50), 1)
  draw((-0.6, 50))

  end_drawing()
