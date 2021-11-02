"""A utility program for squaring images.

square_picker.py can be used to generate some extra calibration
parameters for squaring an image. It does not undistors from a camera
matrix, it merely performs an affine transform of an undistorted image
to show a specified quadrilateral mapped to a square."""

import cv2
import numpy as np

from daedalus.Image import square, undistort
from daedalus.streaming import VideoStreamHandler

points = np.int32([[156, 707], [224, 63], [861, 102], [830, 765]])


def draw_circle(event, x, y, flags, param):
    """Change one of the vertices of the quadrilateral.
    move mouse and press:
        left click for pt0
        middle click for pt1
        right click for pt2
        mousewheel for pt3
    """

    if event == cv2.EVENT_LBUTTONDOWN:
        points[0] = (x, y)
    elif event == cv2.EVENT_MBUTTONDOWN:
        points[1] = (x, y)
    elif event == cv2.EVENT_RBUTTONDOWN:
        points[2] = (x, y)
    elif event == cv2.EVENT_MOUSEWHEEL:
        points[3] = (x, y)


def main():
    # setup cv video
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

    # setup mouse callback event
    cv2.namedWindow('frame_ext')
    cv2.setMouseCallback('frame_ext', draw_circle)

    while True:
        # grab and undistort frame
        frame = video_stream.frame
        undistorted = undistort(frame, balance=0.5)
        frame_ext = cv2.copyMakeBorder(undistorted, 0, 100, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        frame_sq = square(undistorted, np.float32(points))

        # draw circles on vertices
        for point in list(points):
            cv2.circle(frame_ext, point, 3, (255, 255, 255), -1)

        # connect vertices with lines
        for i in range(len(points) - 1):
            cv2.line(frame_ext, points[i][:], points[i + 1][:], (255, 255, 255), 1)

        # show modified and unmodified frames
        cv2.imshow('frame_ext', frame_ext)
        cv2.imshow('frame', frame_sq)

        k = cv2.waitKey(20)
        if k == ord('q'):
            # exit when q is pressed
            break
        elif k == ord('p'):
            # print points when p is pressed (can be used in daedalus.Image.square())
            print(
                f'[[{points[0, 0]},{points[0, 1]}],[{points[1, 0]},{points[1, 1]}],[{points[2, 0]},{points[2, 1]}],[{points[3, 0]},{points[3, 1]}]]')

    # graceful exit
    video_stream.terminate()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
