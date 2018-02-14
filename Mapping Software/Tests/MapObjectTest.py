import unittest
import sys
sys.path.append("..")
from Src.MapObjects import *

class TestMapObjects(unittest.TestCase):
    
    def testCreateWall(self):
        startPoint = Point(0 ,0)
        endPoint = Point(0, 1)
        wall = Wall(startPoint, endPoint)
        mainMap = Map()
        self.assertEqual(0, mainMap.numWalls())
        mainMap.addWall(wall)
        self.assertEqual(1, mainMap.numWalls())

if __name__ == '__main__':
    unittest.main()