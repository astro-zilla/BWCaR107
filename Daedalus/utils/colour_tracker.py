import cv2
import numpy as np


def customizable_mask(img):

    def empty(x):
        pass

    # create window for trackbars
    cv2.namedWindow("TrackBars")
    cv2.resizeWindow("TrackBars", 640, 1000)

    # create trackbars to adjust different colour parameters
    cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)
    cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, empty)
    cv2.createTrackbar("Sat Min", "TrackBars", 0, 255, empty)
    cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, empty)
    cv2.createTrackbar("Val Min", "TrackBars", 0, 255, empty)
    cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)

    # use HSV colour system for detection
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # get the different values from the trackbars
    h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")

    # values for customizable mask
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(imgHSV, lower, upper)

    return mask


def thinning_algorithm(img):

    # Create a kernel to perform erosion and dilation
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    # Create an empty skeleton where to store values progressively
    thin = np.zeros(img.shape, dtype='uint8')
    img_copy = img.copy()

    # while loop until all white pixels are eroded
    while cv2.countNonZero(img_copy) != 0:
        # Erosion
        eroded_img = cv2.erode(img_copy, kernel)
        # Open (erosion+dilation) eroded image
        opening = cv2.morphologyEx(eroded_img, cv2.MORPH_OPEN, kernel)
        # Subtract these two
        subtraction = eroded_img - opening
        # Add the results of the subtraction to the skeleton
        thin = cv2.bitwise_or(subtraction, thin)
        # Change the original image to the eroded one
        img_copy = eroded_img.copy()

    return thin