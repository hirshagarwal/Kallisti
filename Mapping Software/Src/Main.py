from MapObjects import *
import math


def main():
    # currently, the first point should be at (x=0, y=0)
    points = [Point(0, 0, -90), Point(0, 100, 90), Point(100, 100, -90), Point(100, 200, 90), Point(200, 200, 135),
              Point(300, 100, 135), Point(300, -50, 90), Point(-50, -50, 90), Point(-50, 0, 90)]
    points_2 = [Point(0, 0, 90), Point(0, 250, 90), Point(50 * math.sqrt(3), 250, 135), Point(100 + 50 * math.sqrt(3), 150, 135),
                Point(100 + 50 * math.sqrt(3), 50, 120), Point(100, 0, 150)]
    points_3 = [Point(0, 0, 90), Point(0, 100, 90),
                Point(100, 100, 135), Point(200, 0, 45)]
    map = Map(points)
    #print(map.get_intersection(map.walls[0], map.walls[4]))
    map.segment()
    map.draw_map()


if __name__ == "__main__":

    main()
