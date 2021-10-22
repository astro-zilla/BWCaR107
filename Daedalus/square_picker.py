import cv2
import numpy as np

from daedalus.Image import square, undistort
from daedalus.streaming import VideoStreamHandler

points = np.int32([[156, 707], [224, 63], [861, 102], [830, 765]])


def draw_circle(event, x, y, flags, param):
    # move mouse and press:
    #   left click for pt0
    #   middle click for pt1
    #   right click for pt2
    #   mousewheel for pt3
    if event == cv2.EVENT_LBUTTONDOWN:
        points[0] = (x, y)
    elif event == cv2.EVENT_MBUTTONDOWN:
        points[1] = (x, y)
    elif event == cv2.EVENT_RBUTTONDOWN:
        points[2] = (x, y)
    elif event == cv2.EVENT_MOUSEWHEEL:
        points[3] = (x, y)


def main():
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

    cv2.namedWindow('frame_ext')
    cv2.setMouseCallback('frame_ext', draw_circle)

    while True:
        frame = video_stream.frame
        frame = undistort(frame, balance=0.5)
        frame_ext = cv2.copyMakeBorder(frame, 0, 100, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
        frame = square(frame, np.float32(points))

        for point in points:
            cv2.circle(frame_ext, point, 3, (255, 255, 255), -1)

        for i in range(len(points) - 1):
            cv2.line(frame_ext, points[i][:], points[i + 1][:], (255, 255, 255), 1)

        cv2.imshow('frame_ext', frame_ext)
        cv2.imshow('frame', frame)

        k = cv2.waitKey(20)
        if k == ord('q'):
            break
        elif k == ord('p'):
            print(
                f'[[{points[0, 0]},{points[0, 1]}],[{points[1, 0]},{points[1, 1]}],[{points[2, 0]},{points[2, 1]}],[{points[3, 0]},{points[3, 1]}]]')

    video_stream.terminate()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
