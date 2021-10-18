import cv2
import numpy as np


# stores all the contours that have an area bigger/smaller than size
def get_main_contours(img, lower_size=0, upper_size=1000000 ):
    if len(img.shape) > 2:
        cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    list_contour = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if upper_size > area > lower_size:
            list_contour.append(contour)
    return list_contour


# gets the centroid and draws it on the image provided
def get_centroid(img, contours):
    for contour in contours:
        cv2.drawContours(img, contour, -1, (255, 255, 255), 3)

    ret, thresh = cv2.threshold(img, 127, 255, 0)
    # calculate moments of binary image
    M = cv2.moments(thresh)

    # calculate x,y coordinate of center
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    cv2.circle(img, (cx, cy), 5, (255, 255, 255), -1)
    position = (cx, cy)

    return position


# finds angles
def angle_finder(img, robot_position, destination, heading):
    path_vector = destination - robot_position

    path_magnitude = np.linalg.norm(path_vector)
    heading_magnitude = np.linalg.norm(heading)
    angle_radians = np.arccos((np.dot(path_vector, heading)) / (path_magnitude * heading_magnitude))
    angle_degrees = (angle_radians * 180) / np.pi
    cv2.line(img, robot_position, destination, (255, 0, 0), 9)
    cv2.putText(img, f'angle: {angle_degrees}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                cv2.LINE_AA)

    return angle_degrees


def just_angle(img, position, heading, target):
    to_target = target - position
    rads = np.arctan2(to_target[1], to_target[0]) - np.arctan2(heading[1], heading[0])
    if rads > np.pi:
        rads -= 2 * np.pi
    elif rads <= -np.pi:
        rads += 2 * np.pi
    cv2.putText(img, f'angle: {(rads*180)/np.pi}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                cv2.LINE_AA)
    return (rads * 180) / np.pi
