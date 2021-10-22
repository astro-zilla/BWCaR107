import cv2
import numpy as np
from daedalus.streaming import VideoStreamHandler
from daedalus.Image import undistort, square
from daedalus.colour_tracker import thinning_algorithm
from daedalus.navigation import angle_finder, get_main_contours, get_centroid, just_angle
from daedalus.aruco import analyse


def main():

    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

    while True:

        frame = video_stream.frame
        frame = undistort(frame, balance=0.5)
        frame = square(frame)
        frame_copy = frame.copy()
        frame_copy = frame_copy[170:610,:]
        cv2.imshow("cropped", frame_copy)

        # use HSV colour system for detection
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        imgHSV1 = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)

        # values for white line mask
        lower_line = np.array([0, 0, 220])
        upper_line = np.array([178, 27, 255])
        mask_line = cv2.inRange(imgHSV, lower_line, upper_line)
        mask_line_copy = mask_line.copy()
        mask_line1 = cv2.inRange(imgHSV1, lower_line, upper_line)

        # values for blocks mask (whether red top or blue top)
        lower_block = np.array([90, 107, 123])
        upper_block = np.array([179, 255, 255])
        mask_block = cv2.inRange(imgHSV, lower_block, upper_block)

        # values for big red square
        lower_rsquare = np.array([157, 114, 122])
        upper_rsquare = np.array([179, 255, 255])
        mask_rsquare = cv2.inRange(imgHSV, lower_rsquare, upper_rsquare)

        # values for big blue square
        lower_bsquare = np.array([89, 93, 123])
        upper_bsquare = np.array([143, 255, 255])
        mask_bsquare = cv2.inRange(imgHSV, lower_bsquare, upper_bsquare)

        # display results to see just the effect of the masks
        #cv2.imshow("hsv", imgHSV)
        cv2.imshow("mask_line", mask_line)
        #cv2.imshow("mask_block", mask_block)
        #cv2.imshow("mask_rsquare", mask_rsquare)
        #cv2.imshow("mask_bsquare", mask_bsquare)
        cv2.imshow("frame", frame)

        # display results with original colours
        img_results_blocks = cv2.bitwise_and(frame, frame, mask=mask_block)
        img_results_rsquare = cv2.bitwise_and(frame, frame, mask=mask_rsquare)
        img_results_bsquare = cv2.bitwise_and(frame, frame, mask=mask_bsquare)

        stacked_results = np.concatenate((frame, img_results_blocks), axis=1)
        stacked_results1 = np.concatenate((img_results_rsquare, img_results_bsquare), axis=1)
        #cv2.imshow("frame and blocks", stacked_results)
        #cv2.imshow("squares", stacked_results1)

        # get a thinner version to create a path or reduce image noise
        thin = thinning_algorithm(mask_line_copy)

        stacked_results2 = np.concatenate((mask_line, thin), axis=1)
        #cv2.imshow("lines", stacked_results2)

        # find centroid of big square for direction
        im = np.zeros(mask_bsquare.shape, "uint8")
        # get contours of blue mask
        contours = get_main_contours(mask_bsquare, 500)
        # get position of the centroid
        if len(contours) == 0:
            pass
        else:
            position_blue = np.asarray(get_centroid(im, contours))
            # get direction
            position_heading = {}
            analyse(frame, position_heading)
            array_1 = position_heading.get(7)
            if array_1 != None:
                robot_position = array_1[0]
                header = array_1[1]
                angle_blue = just_angle(robot_position, header, position_blue)
                cv2.putText(im, f'angle: {angle_blue}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                            cv2.LINE_AA)
                if 10 < angle_blue <= 180:
                    print("Turn Right.")
                elif -180 < angle_blue < -10:
                    print("Turn Left.")
                elif angle_blue <= 10 and angle_blue >= -10:
                    print("Go straight")

        # find centroid of the red square for direction
        im1 = np.zeros(mask_rsquare.shape, "uint8")
        # get contours of red mask
        contours1 = get_main_contours(mask_rsquare, 500)
        # get position of the centroid
        if len(contours1) == 0:
            pass
        else:
            position_red = np.asarray(get_centroid(im1, contours1))
            # get direction
            position_heading1 = {}
            analyse(frame, position_heading1)
            array1 = position_heading1.get(7)
            if array1 != None:
                robot_position = array1[0]
                header1 = array1[1]
                angle_red = just_angle(robot_position, header1, position_red)
                #cv2.putText(im1, f'angle: {angle_red}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                        #cv2.LINE_AA)
                #if 10 < angle_red <= 180:
                    #print("Turn Right.")
                #elif -180 < angle_red < -10:
                    #print("Turn Left.")
                #elif angle_red <= 10 and angle_red >= -10:
                    #print("Go straight")
            pass

        im3 = np.zeros(mask_block.shape, "uint8")
        contours3 = get_main_contours(mask_block, 100, 300)
        if len(contours3) == 0:
            pass
        else:
            position_blocks = get_centroid(im3, contours3)




        cv2.imshow("im", im)
        #cv2.imshow("im1", im1)
        #cv2.imshow("IM3", im3)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    video_stream.terminate()


if __name__ == "__main__":
    main()

