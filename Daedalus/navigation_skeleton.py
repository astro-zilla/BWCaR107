from dataclasses import dataclass
from daedalus.aruco import analyse
from daedalus.navigation import angle_finder, find_starting_square
import cv2
import numpy as np


def navigation_white_square(frame, frame_copy):

    # finds centroid of the starting square
    centroid = find_starting_square()

    # checks current position of the robot
    position_heading = {}
    analyse(frame, position_heading)
    array = position_heading.get(7)

    # checks whether the robot is inside the white square
    position_heading1 = {}
    analyse(frame_copy, position_heading1)
    array0 = position_heading1.get(7)

    # while the robot is not in the square
    while array0 == None:

        robot_position = array0[0]
        heading0 = array0[1]
        # angle_white = get_angle(robot_position, heading0, centroid)
        # if 10 < angle_white <= 180:
        # print("Turn Right.")
        # elif -180 < angle_white < -10:
        # print("Turn Left.")
        # elif angle_white <= 10 and angle_white >= -10:
        # print("Go straight")'''

    # when the robot reaches the square
    print("Robot stop")
    return



