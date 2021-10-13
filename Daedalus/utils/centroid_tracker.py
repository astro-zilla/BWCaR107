import cv2
import numpy as np


# stores all the contours that have an area bigger than size
def get_main_contours(img, size):

    if len(img.shape) > 2:
        cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    list_contour = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > size:
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
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
    position = (cX, cY)

    return position
