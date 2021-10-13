import cv2
import numpy as np

def angle_finder(img, robot_position, destination, pointing_position):
    path_vector = np.zeros(2)
    pointing_vector = np.zeros(2)
    for i in range(2):
        path_vector[i] = destination[i] - robot_position[i]
        pointing_vector[i] = pointing_position[i] - robot_position[i]
    path_magnitude = np.linalg.norm(path_vector)
    pointing_magnitude = np.linalg.norm(pointing_vector)
    angle_radians = np.arccos((np.dot(path_vector, pointing_vector)) / (path_magnitude * pointing_magnitude))
    angle_degrees = (angle_radians * 180) / np.pi
    cv2.line(img, robot_position, destination, (255, 0, 0), 9)
    cv2.line(img, robot_position, pointing_position, (255, 0, 0), 9)
    cv2.putText(img, "angle: {}".format(angle_degrees), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                cv2.LINE_AA)
    return angle_degrees