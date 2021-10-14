import json
import socket
import time

import cv2
import numpy as np

from Daedalus.utils.Image import square, undistort
from Daedalus.utils.aruco import visualise
from Daedalus.utils.navigation import just_angle
from Daedalus.utils.streaming import ArduinoStreamHandler, VideoStreamHandler

mX, mY = 0, 0


def nothing(_): pass


def mouse(event, x, y, flags, params):
    global mX,mY
    mX, mY = x, y


def main():
    # broadcast locally on 53282
    host = socket.gethostname()
    port = 53282

    # listen on streaming socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen()

    # init asynchronous "threading" stream handlers
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    arduino_stream = ArduinoStreamHandler(server, json.dumps(data))

    # start streams
    video_stream.start()
    arduino_stream.start()

    start_time = int(time.time_ns() / 1000000)
    img_count = 0
    robot_aruco_id = 2

    cv2.namedWindow('sliders', cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar('motor 1', 'sliders', 0, 255, nothing)
    cv2.createTrackbar('motor 2', 'sliders', 0, 255, nothing)

    cv2.namedWindow('frame', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('frame',mouse)

    while True:
        # input run-time to data array
        time_since_start = int(time.time_ns() / 1000000) - start_time

        data["motors"][0] = cv2.getTrackbarPos('motor 1', 'sliders')
        data["motors"][1] = cv2.getTrackbarPos('motor 2', 'sliders')

        # get/send arduino data from
        arduino_stream.set_data(json.dumps(data))
        arduinodata = arduino_stream.get_data()

        # get video data from stream
        frame = video_stream.frame
        frame = undistort(frame, balance=0.5)
        frame = square(frame)
        dictionary = {}
        frame = visualise(frame, dictionary)
        a = 0
        if robot_aruco_id in dictionary.keys():
            position, heading = dictionary[robot_aruco_id]
            a = just_angle(position, heading, np.int32([mX,mY]))
            v = [mX, mY] - position
            h = position + np.int32(50 * v / np.linalg.norm(v))
            cv2.line(frame, position, h, (255, 0, 0), 2)

        overlay = frame.copy()
        output = frame.copy()

        # draw info bar and fps
        cv2.rectangle(overlay, (0, 0), (220, 130), (50, 50, 50), -1)
        cv2.putText(img=overlay, text=f'RATES:',
                    org=(20, 30),
                    fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(255, 255, 255), thickness=1)
        cv2.putText(img=overlay, text=f'    VIDEO: {video_stream.get_rate():.1f} fps',
                    org=(20, 50),
                    fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(255, 255, 255), thickness=1)
        cv2.putText(img=overlay, text=f'    ARDUINO: {arduino_stream.get_rate():.1f} Hz',
                    org=(20, 70),
                    fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(255, 255, 255), thickness=1)
        cv2.putText(img=overlay, text=f'DATA:',
                    org=(20, 90),
                    fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(255, 255, 255), thickness=1)
        cv2.putText(img=overlay, text=f'    angle: {a:.1f} deg',
                    org=(20, 110),
                    fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(255, 255, 255), thickness=1)
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, output)

        # output to frame
        cv2.imshow('frame', output)

        # press q key to exit
        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            break
        # spacebar to capture a screenshot
        elif k & 0xFF == ord(' '):
            cv2.imwrite(f'screenshots/frame{str(img_count).zfill(5)}.png', output)
            img_count += 1
            print(f'image saved to: screenshots/frame{str(img_count).zfill(5)}.png')

        if video_stream.terminated or arduino_stream.terminated:
            break

    # graceful exit
    cv2.destroyAllWindows()
    video_stream.terminate()
    arduino_stream.terminate()

    print('# main thread exit')


if __name__ == "__main__":
    data = {
        "time": 0,
        "motors": [100, 100, 0, 0],
        "servos": [0, 0],
        "LEDs": [0, 0, 0, 0]
    }
    main()
