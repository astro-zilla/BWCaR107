import cv2
import numpy as np

DIM = (1016, 760)
K = np.array([[644.0995890009748, 0.0, 514.3136068698651], [0.0, 645.7478380069424, 401.9020082118956], [0.0, 0.0, 1.0]])
D = 1.1 * np.array([[-0.16047886876042616], [0.5186348922845948], [-1.0904314778194455], [0.8084994142726131]])
from_pts_default = np.float32([[154, 713], [225, 68], [862, 104], [832, 783]])


def undistort(img, balance=0.0, dim2=None, dim3=None):
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
    if dim1[0] / dim1[1] != DIM[0] / DIM[1]:
        cv2.resize(img, DIM)

    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K
    # used to un-distort image. OpenCV document failed to make this clear!

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        scaled_K, D, dim1, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim1, cv2.CV_16SC2)
    return cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def square(img, pts=None):
    if pts is None:
        from_pts = from_pts_default
    else:
        from_pts = pts
    to_pts = np.float32([[0, DIM[1]], [0, 0], [DIM[1], 0], [DIM[1], DIM[1]]])
    squaring_M = cv2.getPerspectiveTransform(from_pts, to_pts)
    return cv2.warpPerspective(img, squaring_M, (DIM[1], DIM[1]))
