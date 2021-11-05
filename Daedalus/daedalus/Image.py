from cv2 import BORDER_CONSTANT, CV_16SC2, INTER_LINEAR, fisheye, getPerspectiveTransform, remap, resize, warpPerspective
from numpy import array, eye, float32

DIM = (1016, 760)
K = array([[644.0995890009748, 0.0, 514.3136068698651], [0.0, 645.7478380069424, 401.9020082118956], [0.0, 0.0, 1.0]])
D = 1.1 * array([[-0.16047886876042616], [0.5186348922845948], [-1.0904314778194455], [0.8084994142726131]])
from_pts_default = [float32([[226, 711], [202, 93], [808, 50], [865, 675]]),
                    float32([[154, 713], [225, 68], [862, 104], [832, 783]])]


def undistort(img, balance=0.0):
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
    if dim1[0] / dim1[1] != DIM[0] / DIM[1]:
        resize(img, DIM)

    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K
    # used to un-distort image. OpenCV document failed to make this clear!

    new_K = fisheye.estimateNewCameraMatrixForUndistortRectify(
        scaled_K, D, dim1, eye(3), balance=balance)
    map1, map2 = fisheye.initUndistortRectifyMap(scaled_K, D, eye(3), new_K, dim1, CV_16SC2)
    return remap(img, map1, map2, interpolation=INTER_LINEAR, borderMode=BORDER_CONSTANT)


def square(img, pts=None, cam=2):
    if pts is None:
        from_pts = from_pts_default[cam - 1]
    else:
        from_pts = pts
    to_pts = float32([[0, DIM[1]], [0, 0], [DIM[1], 0], [DIM[1], DIM[1]]])
    squaring_M = getPerspectiveTransform(from_pts, to_pts)
    return warpPerspective(img, squaring_M, (DIM[1], DIM[1]))
