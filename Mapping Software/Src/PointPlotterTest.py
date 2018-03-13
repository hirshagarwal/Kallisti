import unittest

from PointPlotter import *


"""
Sanity check for the point plotter methods.
"""
class TestPointPlotterMethods(unittest.TestCase):
    def setUp(self):
        self.test_wall_distances = [1]
        self.test_position = [1, 1]

    def test_zero_degrees(self):
        test_point = find_wall_point(self.test_wall_distances, self.test_position, 0)
        self.assertEqual(test_point, [0, 1])

    def test_45_degrees(self):
        test_point = find_wall_point(self.test_wall_distances, self.test_position, 45)
        self.assertEqual(test_point, [0.2928932188134524, 1.7071067811865475])

    def test_90_degrees(self):
        test_point = find_wall_point(self.test_wall_distances, self.test_position, 90)
        self.assertEqual(test_point, [1, 2])

    def test_135_degrees(self):
        test_point = find_wall_point(self.test_wall_distances, self.test_position, 135)
        self.assertEqual(test_point, [1.7071067811865475, 1.7071067811865475])

    def test_180_degrees(self):
        test_point = find_wall_point(self.test_wall_distances, self.test_position, 180)
        self.assertEqual(test_point, [2, 1])

    def test_225_degrees(self):
        test_point = find_wall_point(self.test_wall_distances, self.test_position, 225)
        self.assertEqual(test_point, [1.7071067811865475, 0.2928932188134524])

    def test_270_degrees(self):
        test_point = find_wall_point(self.test_wall_distances, self.test_position, 270)
        self.assertEqual(test_point, [1, 0])

    def test_315_degrees(self):
        test_point = find_wall_point(self.test_wall_distances, self.test_position, 315)
        self.assertEqual(test_point, [0.2928932188134524, 0.2928932188134524])

    def test_360_degrees(self):
        test_point = find_wall_point(self.test_wall_distances, self.test_position, 360)
        self.assertEqual(test_point, [0, 1])

    def test_405_degrees(self):
        test_point = find_wall_point(self.test_wall_distances, self.test_position, 405)
        self.assertEqual(test_point, [0.2928932188134524, 1.7071067811865475])


if __name__ == "__main__":
    unittest.main()