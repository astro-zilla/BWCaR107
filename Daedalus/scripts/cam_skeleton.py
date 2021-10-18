import cv2
import numpy as np

from Daedalus.utils.Image import square, undistort
from Daedalus.utils.aruco import analyse
from Daedalus.utils.navigation import just_angle
from Daedalus.utils.streaming import VideoStreamHandler


def main(robot_aruco_id=2):
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

    while True:
        frame = video_stream.frame
        """
        frame = undistort(frame, balance=0.5)
        frame = square(frame)
        dictionary = {}
        frame = analyse(frame, dictionary)
        a = 0
        if robot_aruco_id in dictionary.keys():
            position, heading = dictionary[robot_aruco_id]
            a = just_angle(position, heading, np.int32([mX, mY]))
            v = [mX, mY] - position
            h = position + np.int32(50 * v / np.linalg.norm(v))
            cv2.line(frame, position, h, (255, 0, 0), 2)

        for id in data:
            print(f'{id}:{data[id]}')
        """
        cv2.imshow('frame', frame)

        k = cv2.waitKey(20)
        if k == ord('q'):
            break

    video_stream.terminate()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
