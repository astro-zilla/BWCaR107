import cv2
import numpy as np

source = 0
if source == 0:
    DIM = (1016, 760)
    K = np.array([[644.0995890009748, 0.0, 514.3136068698651], [0.0, 645.7478380069424, 401.9020082118956], [0.0, 0.0, 1.0]])
    D = np.array([[-0.16047886876042616], [0.5186348922845948], [-1.0904314778194455], [0.8084994142726131]])
else:
    DIM = (1016, 760)
    K = np.array(
        [[1755.3785742532216, 0.0, 514.8025594784805], [0.0, 1776.2509154908232, 386.5369111505285], [0.0, 0.0, 1.0]])
    D = np.array([[-3.648490408032536], [36.0872172086949], [-348.7599801137575], [1495.0899545935063]])


def undistort(img, balance=0.0, dim2=None, dim3=None):
    # img = cv2.resize(img, DIM)
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort

    assert dim1[0] / dim1[1] == DIM[0] / DIM[1], ("Image to undistort needs to"
                                                  " have same aspect ratio as the"
                                                  " ones used in calibration")
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1

    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K
    # used to un-distort image. OpenCV document failed to make this clear!

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3),
                                                     new_K, dim3, cv2.CV_16SC2)
    return cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def square(img):
    dim1 = img.shape[:2][::-1]
    from_pts = np.float32([[227, 66], [863, 102], [159, 712]])
    to_pts = np.float32([[0, 0], [dim1[1], 0], [0, dim1[1]]])
    squaring_M = cv2.getAffineTransform(from_pts, to_pts)

    return cv2.warpAffine(img, squaring_M, (dim1[1],dim1[1]))
