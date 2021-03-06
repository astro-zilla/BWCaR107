import cv2
from numpy import array, int32, mean, zeros
from numpy.linalg import norm

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()


def generate(marker_id):
    tag = zeros((300, 300, 1), dtype='uint8')
    cv2.aruco.drawMarker(arucoDict, marker_id, 300, tag, 1)


def analyse(frame, data=[], visualise=True):
    frame = frame.copy()
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    if len(corners) > 0:
        ids = ids.flatten()

        for markerCorner, markerID in zip(corners, ids):
            corner = int32(markerCorner.reshape((4, 2)))

            position = int32(mean(corner, axis=0))
            heading = array((corner[0] - corner[3] + corner[1] - corner[2]) / 2)
            heading = heading / norm(heading)
            data[markerID] = [position, heading]

            if visualise:
                # draw the bounding box of the ArUCo detection
                for i in range(4):
                    cv2.line(frame, corner[i], corner[(i + 1) % 4], (0, 255, 0), 2)
                cv2.line(frame, position, position + int32(50 * heading), (255, 0, 0), 2)
                cv2.circle(frame, position, 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the frame
                cv2.putText(frame, str(markerID), int32((5 * corner[3] - position) / 4 + array([-5, 5])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    if visualise:
        return frame
    else:
        return None
