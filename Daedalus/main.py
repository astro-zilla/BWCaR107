import json
import socket
import time

import cv2
import numpy as np
from pynput.keyboard import Key, KeyCode, Listener

from Daedalus.utils.Image import square, undistort
from Daedalus.utils.aruco import analyse
from Daedalus.utils.navigation import just_angle
from Daedalus.utils.streaming import ArduinoStreamHandler, VideoStreamHandler

mX, mY = 0, 0
keys = set()

key_q = KeyCode.from_char('q')

key_w = KeyCode.from_char('w')
key_a = KeyCode.from_char('a')
key_s = KeyCode.from_char('s')
key_d = KeyCode.from_char('d')

key_f = KeyCode.from_char('f')
key_t = KeyCode.from_char('t')
key_p = KeyCode.from_char('p')


def nothing(_): pass


def on_press(key):
    keys.add(key)


def on_release(key):
    keys.discard(key)
    if key == key_q:
        # Stop listener
        return False


def main(robot_aruco_id=7):
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
    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # start streams
    video_stream.start()
    arduino_stream.start()

    start_time = int(time.time_ns() / 1000000)
    img_count = 0

    times = [0.] * 10

    while True:
        times.append(time.time())
        times = times[-10:]
        tickrate = 1 / np.mean(np.diff(times))

        # data["motors"][0] = cv2.getTrackbarPos('motor1', 'sliders')
        # data["motors"][1] = cv2.getTrackbarPos('motor2', 'sliders')

        # get/send arduino data from
        arduino_stream.data = json.dumps(data)
        arduinodata = arduino_stream.data
        # get video data from stream
        # todo figure out how to multiprocess the resource-intensive bits
        frame1 = video_stream.frame

        frame = undistort(frame1, balance=0.5)
        frame = square(frame)

        dictionary = {}
        frame = analyse(frame, dictionary, visualise=True)
        print(dictionary)
        a = 0
        if robot_aruco_id in dictionary.keys():
            position, heading = dictionary[robot_aruco_id]
            a = just_angle(position, heading, np.int32([mX, mY]))
            v = [mX, mY] - position
            h = position + np.int32(50 * v / np.linalg.norm(v))
            cv2.line(frame, position, h, (255, 0, 0), 2)

        # press q key to exit
        if key_q in keys:
            listener.join()
            break
        motorspeed = np.zeros(2)

        if key_w in keys:
            motorspeed += np.int32([255, 255])
        if key_a in keys:
            motorspeed += np.int32([-127, 127])
        if key_s in keys:
            motorspeed += np.int32([-255, -255])
        if key_d in keys:
            motorspeed += np.int32([127, -127])

        if Key.space in keys:
            cv2.imwrite(f'screenshots/frame{str(img_count).zfill(5)}.png', frame)
            img_count += 1
        if key_f in keys:
            print(f'fps: {video_stream.get_rate():.1f}')
        if key_t in keys:
            print(f'tps: {tickrate:.1f}')
        if key_p in keys:
            print(f'ping: {arduino_stream.get_rate():.1f}')

        motorspeed = list(np.clip(motorspeed,-255,255))
        data["motors"] = motorspeed

        # output to frame
        cv2.imshow('frame', frame)
        cv2.waitKey(1)


    # graceful exit
    cv2.destroyAllWindows()
    video_stream.terminate()
    arduino_stream.terminate()


if __name__ == "__main__":
    data = {
        "time": 0,
        "motors": [0, 0],
        "servos": [0, 0],
        "LEDs": [0, 0, 0, 0]
    }
    main()
