import warnings

import cv2
import numpy as np
"""    if source == 0:
        DIM = (1016, 760)
        K = np.array(
            [[644.0995890009748, 0.0, 514.3136068698651], [0.0, 645.7478380069424, 401.9020082118956], [0.0, 0.0, 1.0]])
        D = np.array([[-0.16047886876042616], [0.5186348922845948], [-1.0904314778194455], [0.8084994142726131]])
    else:
        DIM = (1016, 760)
        K = np.array(
            [[1058.1023742892644, 0.0, 512.955162129911], [0.0, 1061.107506095904, 391.3875769097182], [0.0, 0.0, 1.0]])
        D = np.array([[-1.0091055968032048], [2.686215769977072], [-9.543542386366333], [18.128853807009133]])"""



def undistort(img, balance=0.0, source=0, dim2=None, dim3=None):
    if source == 0:
        DIM = (1016, 760)
        K = np.array(
            [[644.0995890009748, 0.0, 514.3136068698651], [0.0, 645.7478380069424, 401.9020082118956], [0.0, 0.0, 1.0]])
        D = 1.3*np.array([[-0.16047886876042616], [0.5186348922845948], [-1.0904314778194455], [0.8084994142726131]])
    else:
        DIM = (1016, 760)
        K = np.array(
            [[1058.1023742892644, 0.0, 512.955162129911], [0.0, 1061.107506095904, 391.3875769097182], [0.0, 0.0, 1.0]])
        D = np.array([[-1.0091055968032048], [2.686215769977072], [-9.543542386366333], [18.128853807009133]])

    img = cv2.resize(img, DIM)

    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
    try:
        assert dim1[0] / dim1[1] == DIM[0] / DIM[1], ("Image to undistort needs to"
                                                      " have same aspect ratio as the"
                                                      " ones used in calibration")
    except ZeroDivisionError or AssertionError:
        warnings.warn("can't undidstort")
        return img

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


def square(img, from_pts_in=None):
    dim1 = img.shape[:2][::-1]
    if from_pts_in is not None:
        from_pts = from_pts_in
    else:
        from_pts = np.float32([[131, 729], [215, 54], [873, 90], [850, 789]])
    to_pts = np.float32([[0, dim1[1]], [0, 0], [dim1[1], 0], [dim1[1], dim1[1]]])

    squaring_M = cv2.getPerspectiveTransform(from_pts, to_pts)

    return cv2.warpPerspective(img, squaring_M, (dim1[1], dim1[1]))
