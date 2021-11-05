from cv2 import CHAIN_APPROX_NONE, COLOR_BGR2GRAY, COLOR_BGR2HSV, RETR_EXTERNAL, circle, contourArea, cvtColor, \
    drawContours, findContours, inRange, moments, threshold
from numpy import arctan2, array, asarray, ndarray, pi, zeros


def get_main_contours(img, lower_size=0, upper_size=1000000):
    if len(img.shape) > 2:
        cvtColor(img, COLOR_BGR2GRAY)

    contours, hierarchy = findContours(img, RETR_EXTERNAL, CHAIN_APPROX_NONE)
    list_contour = []
    for contour in contours:
        area = contourArea(contour)
        if upper_size > area > lower_size:
            list_contour.append(contour)
    return list_contour


# gets the centroid and draws it on the image provided
def get_centroid(contours, img):
    for contour in contours:
        drawContours(img, contour, -1, (255, 255, 255), 3)

    ret, thresh = threshold(img, 127, 255, 0)
    # calculate moments of binary image
    M = moments(thresh)

    # calculate x,y coordinate of center
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    circle(img, (cx, cy), 5, (255, 255, 0), -1)
    position = (cx, cy)

    return position


def get_angle(position: ndarray, heading: ndarray, target: ndarray) -> float:
    to_target = target - position
    rads = arctan2(to_target[1], to_target[0]) - arctan2(heading[1], heading[0])
    if rads > pi:
        rads -= 2 * pi
    elif rads <= -pi:
        rads += 2 * pi

    return (rads * 180) / pi


def find_block(img):
    # turn the image into HSV for colour detection
    imgHSV = cvtColor(img, COLOR_BGR2HSV)

    imgHSV[:500, :, :] = 0
    imgHSV[:, 200:, :] = 0

    # values for blocks mask (whether red top or blue top)
    lower_block = array([90, 107, 123])
    upper_block = array([179, 255, 255])
    mask_block = inRange(imgHSV, lower_block, upper_block)

    # create matrix to store contours and centroid of the block
    im = zeros(mask_block.shape, "uint8")
    contours = get_main_contours(mask_block, 50, 250)
    if len(contours) == 0:
        return False
    else:
        position_block = asarray(get_centroid(contours, im))

    # imshow("block", mask_block)
    return position_block
