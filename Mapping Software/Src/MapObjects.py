class Map:
    # List of walls in the current map
    walls = list()

    def __init__(self):
       pass

    def addWall(self, newWall):
       self.walls.append(newWall)

    def numWalls(self):
        return len(self.walls)

    def getWalls(self):
       return self.walls

class Wall:
    def __init__(self, startPoint, endPoint):
       self.startPoint =  startPoint
       self.endPoint = endPoint

class Point:
    def __init__(self, x, y):
       self.x = x
       self.y = y