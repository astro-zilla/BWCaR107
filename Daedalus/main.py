import curses
import json
import socket
import sys
import time

import cv2
import numpy as np
from pynput.keyboard import KeyCode, Listener

from utils.Image import square, undistort
from utils.aruco import analyse
from utils.navigation import PID_consts, just_angle, offset, sigmoid
from utils.streaming import ArduinoStreamHandler, VideoStreamHandler

mouse_pos = np.int32([0, 0])
keys = set()

key_q = KeyCode.from_char('q')

key_w = KeyCode.from_char('w')
key_a = KeyCode.from_char('a')
key_s = KeyCode.from_char('s')
key_d = KeyCode.from_char('d')

key_r = KeyCode.from_char('r')
key_f = KeyCode.from_char('f')


def mouse(event, x, y, flags, params):
    global mouse_pos
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_pos = np.int32([x, y])


def nothing(_): pass


def on_press(key):
    keys.add(key)


def on_release(key):
    keys.discard(key)
    if key == key_q:
        # Stop listener
        return False


def main(screen=curses.initscr(), robot_aruco_id=7):
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
    arduino_stream = ArduinoStreamHandler(server)
    arduino_stream.write(bytes(json.dumps(data), 'utf-8'))

    # start streams
    video_stream.start()
    arduino_stream.start()

    # curses window
    # screen = curses.initscr()

    curses.curs_set(False)
    curses.start_color()
    curses.use_default_colors()
    for i in range(0, 255):
        curses.init_pair(i + 1, i, -1)

    screen.clear()

    # keypress handler
    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # times for mainloop tickrate
    times = [0.] * 10

    # mouse callback in main window
    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', mouse)

    # control params
    K_rot = PID_consts(1.5, 0, 0)

    while True:
        times.append(time.time())
        times = times[-10:]
        tickrate = 1 / np.mean(np.diff(times))

        # get/send arduino data from
        arduino_stream.write(data)
        arduinodata = arduino_stream.read()
        # get video data from stream
        # todo figure out how to multiprocess the resource-intensive bits
        frame1 = video_stream.frame

        frame = undistort(frame1, balance=0.5)
        frame = square(frame)

        dictionary = {}
        frame = analyse(frame, dictionary, visualise=True)
        ang = 0
        dist = 0
        if robot_aruco_id in dictionary.keys():
            position, heading = dictionary[robot_aruco_id]

            ang = just_angle(position, heading, mouse_pos)
            dist = np.linalg.norm(mouse_pos - position)

            cv2.line(frame, position, mouse_pos, (255, 0, 0), 1)
            cv2.circle(frame, mouse_pos, 3, (0, 0, 255))

        # press q key to exit
        if key_q in keys:
            listener.join()
            data["motors"] = [0, 0]
            arduino_stream.write(data)
            break
        motorspeed = np.zeros(2)

        if key_r in keys:
            data["servos"][0] += 1
        if key_f in keys:
            data["servos"][0] -= 1

        # if key_w in keys:
        #     motorspeed += np.int32([255, 255])
        # if key_a in keys:
        #     motorspeed += np.int32([-255, 255])
        # if key_s in keys:
        #     motorspeed += np.int32([-255, -255])
        # if key_d in keys:
        #     motorspeed += np.int32([255, -255])

        err = ang
        s = K_rot.p * err
        d = np.clip(dist / 100, 0.1, 1)
        if abs(ang) < 90:
            motorspeed = [-d * 150 * np.cos(ang * np.pi / 180) + s, -d * 150.0 * np.cos(ang * np.pi / 180) - s]
        elif dist < 50:
            motorspeed = [0.0, 0.0]
        else:
            s = offset(s, 50)
            motorspeed = [s, -s]

        fps, ping = video_stream.get_rate(), arduino_stream.get_rate()
        screen.clear()
        # arduino data
        screen.addstr('\nArduino: ', curses.color_pair(16))
        screen.addstr(f'{arduino_stream.status}\n', curses.color_pair(arduino_stream.status_color))
        if arduino_stream.status == 'connected':

            screen.addstr(f'\u251c\u2500\u2500')
            screen.addstr(f'ping: ', curses.color_pair(16))
            screen.addstr(f'{ping:.1f}\n', curses.color_pair(10))

            screen.addstr(f'\u251c\u2500\u2500')
            screen.addstr(f'data\n', curses.color_pair(16))

            for i, (k, v) in enumerate(arduinodata.items()):
                if i < len(arduinodata) - 1:
                    screen.addstr(f'\u2502  \u251c\u2500\u2500{k}: {v}\n')
                else:
                    screen.addstr(f'\u2502  \u2514\u2500\u2500{k}: {v}\n')

            screen.addstr(f'\u2514\u2500\u2500')
            screen.addstr(f'commands\n', curses.color_pair(16))

            for i, (k, v) in enumerate(data.items()):
                if i < len(data) - 1:
                    screen.addstr(f'   \u251c\u2500\u2500{k}: {v}\n')
                else:
                    screen.addstr(f'   \u2514\u2500\u2500{k}: {v}\n')

        # video data
        screen.addstr('Camera:  ', curses.color_pair(16))
        screen.addstr(f'{video_stream.status}\n', curses.color_pair(video_stream.status_color))
        if video_stream.status == 'connected':
            screen.addstr(f'\u2514\u2500fps: ')
            screen.addstr(f'{fps:.1f}\n', curses.color_pair(10))

        # program data
        # screen.addstr(f'tps: {tickrate:.1f}\tmotors: {data["motors"]}')
        # screen.addstr(f'ange: {ang:.1f}\tdistance: {dist:.1f}\tmotor parameter: {s:.1f}')
        screen.refresh()

        motorspeed = list(np.clip(motorspeed, -255, 255))
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

    curses.wrapper(main)
