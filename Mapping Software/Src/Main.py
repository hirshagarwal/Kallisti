from MapObjects import *

def main():
    # currently, the first point should be at (x=0, y=0)
    points = [Point(0, 0, -90), Point(0, 100, 90), Point(100, 100, -90), Point(100, 200, 90), Point(200, 200, 135),
          Point(300, 100, 135), Point(300, -50, 90), Point(-50, -50, 90), Point(-50, 0, 90)]
    map = Map(points)
    map.draw_map()


if __name__ == "__main__":

    main()