import cv2
import numpy as np
from Daedalus.utils.streaming import VideoStreamHandler
from Daedalus.utils.Image import undistort, square
from Daedalus.utils.centroid_tracker import get_main_contours, get_centroid

def main():

    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

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

    while True:

        frame = video_stream.frame
        frame = undistort(frame, balance=0.5)
        frame = square(frame)

        # use HSV colour system for detection
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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
        lower_line = np.array([0, 0, 220])
        upper_line = np.array([178, 27, 255])
        mask_line = cv2.inRange(imgHSV, lower_line, upper_line)
        mask_line_copy = mask_line.copy()

        # values for blocks mask (whether red top or blue top)
        lower_block = np.array([90, 107, 123])
        upper_block = np.array([179, 255, 255])
        mask_block = cv2.inRange(imgHSV, lower_block, upper_block)

        # values for big red square
        lower_rsquare = np.array([157, 114, 122])
        upper_rsquare = np.array([179, 255, 255])
        mask_rquare = cv2.inRange(imgHSV, lower_rsquare, upper_rsquare)

        # values for big blue square
        lower_bsquare = np.array([89, 96, 123])
        upper_bsquare = np.array([143, 255, 255])
        mask_bsquare = cv2.inRange(imgHSV, lower_bsquare, upper_bsquare)

        # display results to see just the effect of the masks
        #cv2.imshow("hsv", imgHSV)
        #cv2.imshow("mask_customizable", mask)
        #cv2.imshow("mask_line", mask_line)
        #cv2.imshow("mask_block", mask_block)
        #cv2.imshow("mask_rsquare", mask_rsquare)
        #cv2.imshow("mask_bsquare", mask_bsquare)

        # display results with original colours
        img_results_line = cv2.bitwise_and(frame, frame, mask=mask_line)
        img_results_blocks = cv2.bitwise_and(frame, frame, mask=mask_block)
        img_results_rsquare = cv2.bitwise_and(frame, frame, mask=mask_rquare)
        img_results_bsquare = cv2.bitwise_and(frame, frame, mask=mask_bsquare)

        stacked_results = np.concatenate((frame, img_results_blocks), axis=1)
        stacked_results1 = np.concatenate((img_results_rsquare, img_results_bsquare), axis=1)
        cv2.imshow("frame and blocks", stacked_results)
        cv2.imshow("squares", stacked_results1)

        # Create a kernel to perform erosion and dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        # Create an empty skeleton where to store values progressively
        thin = np.zeros(mask_line_copy.shape, dtype='uint8')

        # while loop until all white pixels are eroded
        while cv2.countNonZero(mask_line_copy) != 0:

            # Erosion
            eroded_img = cv2.erode(mask_line_copy, kernel)
            # Open (erosion+dilation) eroded image
            opening = cv2.morphologyEx(eroded_img, cv2.MORPH_OPEN, kernel)
            # Subtract these two
            subtraction = eroded_img - opening
            # Add the results of the subtraction to the skeleton
            thin = cv2.bitwise_or(subtraction, thin)
            # Change the original image to the eroded one
            mask_line_copy = eroded_img.copy()

        stacked_results2 = np.concatenate((mask_line, thin), axis=1)
        cv2.imshow("lines", stacked_results2)

        # find centroid of big square for direction
        im = np.zeros(mask_bsquare.shape, "uint8")

        contours = get_main_contours(mask_bsquare, 300)
        position = get_centroid(im, contours)

        cv2.imshow("im", im)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_stream.terminate()


if __name__ == "__main__":
    main()


