import time

import cv2
import numpy as np

from Daedalus.utils.streaming import VideoStreamHandler

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()


def main():
    video_stream = VideoStreamHandler(0)
    video_stream.start()

    while True:
        frame = video_stream.frame

        corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            ids = ids.flatten()

            for markerCorner, markerID in zip(corners, ids):
                corner = np.int32(markerCorner.reshape((4, 2)))
                # draw the bounding box of the ArUCo detection
                for i in range(4):
                    cv2.line(frame, corner[i], corner[(i + 1) % 4], (0, 255, 0), 2)

                cX = int(np.mean(corner[:, 0]))
                cY = int(np.mean(corner[:, 1]))
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the frame
                cv2.putText(frame, str(markerID), (corner[3, 0], corner[3, 1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('frame', frame)
        k = cv2.waitKey(20)
        if k == ord('q'):
            break

    video_stream.terminate()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
