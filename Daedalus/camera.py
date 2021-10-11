import cv2
from StreamHandlers import VideoStreamHandler
import os
import numpy as np


def main():
    #video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    #video_stream.start()

        def empty(x):
            pass

        # path to access one single frame of the video (change on a different laptop)
        path = "/Users/massimozambernardi/PycharmProjects/BWCaR107/Daedalus/single-frame.jpg"

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

        # read the image
        img = cv2.imread(path)

        # crop it to include just the bottom left corner (collection point)
        cropped_img = img[580:720, 160:300]

        while True:

            # use HSV colour system for detection
            imgHSV = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

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

            # values for white line mask
            lower_line = np.array([0, 0, 213])
            upper_line = np.array([178, 81, 255])
            mask_line = cv2.inRange(imgHSV, lower_line, upper_line)

            # values for blocks mask (whether red top or blue top)
            lower_block = np.array([90, 107, 123])
            upper_block = np.array([179, 255, 255])
            mask_block = cv2.inRange(imgHSV, lower_block, upper_block)

            # display results to see just the effect of the masks
            cv2.imshow("hsv", imgHSV)
            #cv2.imshow("mask_customizable", mask)
            #cv2.imshow("mask_line", mask_line)
            #cv2.imshow("mask_block", mask_block)

            # display results with original colours
            img_results_line = cv2.bitwise_and(cropped_img, cropped_img, mask=mask_line)
            img_results_blocks = cv2.bitwise_and(cropped_img, cropped_img, mask=mask_block)
            cv2.imshow("results_line", img_results_line)
            cv2.imshow("results_block", img_results_blocks)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    #video_stream.terminate()


if __name__ == "__main__":
    main()


