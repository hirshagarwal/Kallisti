last_point
points = [last_point]

last_point = (0 - getLeftDistance(), 0 - getBackDistance())
next_point = (robot_location[0] - getLeftDistance(), robot_location[1] + getFrontDistance())
points.append(next_point)

