import curses
import socket
import time

import cv2
import numpy as np
from numpy import array
from pynput.keyboard import KeyCode, Listener

from Daedalus.daedalus.peripherals import Buffer, flash
from daedalus.Image import square, undistort
from daedalus.aruco import analyse
from daedalus.navigation import find_block, get_angle
from daedalus.streaming import ArduinoStreamHandler, VideoStreamHandler

# global vars for callback
mouse_pos = np.int32([678, 86])

out = [array([680, 80]), array([547, 218]), array([500, 266]), array([274, 492]), array([140, 624])]
back = [array([178, 587]), array([274, 494]), array([500, 266])]
blue = [array([612, 283]), array([551, 222]), array([495, 169])]
red = [array([477, 149]), array([548, 218]), array([597, 267])]

waypoints = out

keys = set()

# pynput keycodes
key_q = KeyCode.from_char('q')

key_r = KeyCode.from_char('r')
key_f = KeyCode.from_char('f')

key_i = KeyCode.from_char('i')
key_w = KeyCode.from_char('w')
key_p = KeyCode.from_char('p')
key_d = KeyCode.from_char('d')

# curses colors
BLUE = 10
GREEN = 11
CYAN = 12
RED = 13
MAGENTA = 14
YELLOW = 15
WHITE = 16

# mainloop status codes
TRACKING = 'tracking'
MARKER_NOT_FOUND = 'marker not found'
NO_IMAGE = 'no image'

# control state codes
IDLE = 'idle'
WAYPOINTS = 'waypoints'
POSITIONING = 'positioning'
DETECT = 'detecting'


def mouse(event: int, x: float, y: float, flags, params):
    global waypoints
    if event == cv2.EVENT_RBUTTONDOWN:
        waypoints.append(np.int32([x, y]))
    elif event == cv2.EVENT_LBUTTONDOWN:
        waypoints = [np.int32([x, y])]


def nothing(_): pass


def on_press(key):
    keys.add(key)


def arr_output(arr):
    out = ''
    for item in arr:
        out += f'{item}, '
    return out


def on_release(key):
    keys.discard(key)
    if key == key_q:
        # Stop listener
        return False


def draw_ui(screen_: curses.window, arduino_stream_: ArduinoStreamHandler, ping_: float, arduinodata_: dict,
            video_stream_: VideoStreamHandler, fps_: float, status_: str, status_color_: int, tickrate_: float,
            ctrl_state_: str, distance_: float, speed_: float) -> None:
    screen_.clear()
    # arduino data
    screen_.addstr('\nArduino: ', curses.color_pair(WHITE))
    screen_.addstr(f'{arduino_stream_.status}\n', curses.color_pair(arduino_stream_.status_color))

    if arduino_stream_.status == 'connected':

        screen_.addstr(f'\u251c\u2500\u2500')
        screen_.addstr(f'ping: ', curses.color_pair(WHITE))
        screen_.addstr(f'{ping_:.1f} ms\n', curses.color_pair(BLUE))

        screen_.addstr(f'\u251c\u2500\u2500')
        screen_.addstr(f'data\n', curses.color_pair(WHITE))

        for i, (k, v) in enumerate(arduinodata_.items()):
            if i < len(arduinodata_) - 1:
                screen_.addstr(f'\u2502  \u251c\u2500\u2500{k}: {v}\n')
            else:
                screen_.addstr(f'\u2502  \u2514\u2500\u2500{k}: {v}\n')

        screen_.addstr(f'\u2514\u2500\u2500')
        screen_.addstr(f'commands\n', curses.color_pair(WHITE))

        for i, (k, v) in enumerate(data.items()):
            if i < len(data) - 1:
                screen_.addstr(f'   \u251c\u2500\u2500{k}: {v}\n')
            else:
                screen_.addstr(f'   \u2514\u2500\u2500{k}: {v}\n')

    # video data
    screen_.addstr('Camera: ', curses.color_pair(WHITE))
    screen_.addstr(f'{video_stream_.status}\n', curses.color_pair(video_stream_.status_color))

    if video_stream_.status == 'connected':
        screen_.addstr(f'\u2514\u2500\u2500')
        screen_.addstr(f'fps: ', curses.color_pair(WHITE))
        screen_.addstr(f'{fps_:.1f}\n', curses.color_pair(BLUE))

    screen_.addstr('Daedalus: ', curses.color_pair(WHITE))
    screen_.addstr(f'{status_}\n', curses.color_pair(status_color_))

    screen_.addstr(f'\u251c\u2500\u2500')
    screen_.addstr(f'tickrate: ', curses.color_pair(WHITE))
    screen_.addstr(f'{tickrate_:.1f} Hz\n', curses.color_pair(BLUE))

    screen_.addstr(f'\u251c\u2500\u2500')
    screen_.addstr(f'control state: ', curses.color_pair(WHITE))
    screen_.addstr(f'{ctrl_state_}\n', curses.color_pair(MAGENTA))

    if status_ == TRACKING:
        screen_.addstr(f'\u251c\u2500\u2500')
        screen_.addstr(f'distance: ', curses.color_pair(WHITE))
        screen_.addstr(f'{distance_:.1f} px\n', curses.color_pair(BLUE))

        screen_.addstr(f'\u251c\u2500\u2500')
        screen_.addstr(f'speed: ', curses.color_pair(WHITE))
        screen_.addstr(f'{speed_:.1f} px/s\n', curses.color_pair(BLUE))

    screen_.addstr(f'\u2514\u2500\u2500')
    screen_.addstr(f'waypoints\n', curses.color_pair(WHITE))

    for i, (x, y) in enumerate(waypoints):
        if i < len(waypoints) - 1:
            screen_.addstr(f'   \u251c\u2500\u2500 [{x}, {y}]\n')
        else:
            screen_.addstr(f'   \u2514\u2500\u2500 [{x}, {y}]\n')

    screen_.refresh()


def main(screen: curses.window = curses.initscr(), robot_aruco_id: int = 7):
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
    arduino_stream.write(data)

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
    motorspeed = [0.0, 0.0]
    magnetometer = Buffer(40)

    # status vars
    status = NO_IMAGE
    status_color = RED
    ctrl_state = IDLE
    t_detect = 0

    # frame ids
    f_num = 1.
    detect_num = 0.

    # init stream vals
    frame = video_stream.frame
    arduinodata = arduino_stream.read()

    positions = [([0, 0], [0, 0], 0.), ([0, 0], [0, 0], 1.)]
    speed = 0
    block = [0, 0]

    while True:
        # calculate main loop tickrate
        times.append(time.time())
        times = times[-10:]
        tickrate = 1 / np.mean(np.diff(times))

        # get/send arduino data from
        arduino_stream.write(data)
        if arduino_stream.available:
            arduinodata = arduino_stream.read()
            magnetometer.push(arduinodata["magnetometer"])

        # get video data from stream
        if video_stream.available:
            frame1 = video_stream.frame

            frame = undistort(frame1, balance=0.5)
            frame = square(frame)

            b = find_block(frame) # consider inputting just collection area part of the frame
            if b:
                block = b

            f_num = time.time()

        # detect robot position and heading
        if detect_num != f_num and frame is not None:
            dictionary = {}
            frame = analyse(frame, dictionary, visualise=True)
            if robot_aruco_id in dictionary.keys():
                status = TRACKING
                status_color = GREEN

                position, heading = dictionary[robot_aruco_id]
                positions.append((position, heading, time.time()))
                positions = positions[-50:]
                speed = np.linalg.norm(positions[-1][0] - positions[-2][0]) / (positions[-1][2] - positions[-2][2])

                detect_num = f_num
            else:
                status = MARKER_NOT_FOUND
                status_color = YELLOW

        # press q key to exit
        if key_q in keys:
            listener.join()
            data["motors"] = [0, 0]
            arduino_stream.write(data)
            break

        # servo debug inputs r and f
        if key_r in keys:
            data["servos"][0] += 1
        if key_f in keys:
            data["servos"][0] -= 1

        if key_i in keys:
            ctrl_state = IDLE
        elif key_w in keys:
            ctrl_state = WAYPOINTS
        elif key_p in keys:
            ctrl_state = POSITIONING
        elif key_d in keys:
            ctrl_state = DETECT
            magnetometer.flush()

        # control AI

        dist = 0
        if ctrl_state == IDLE:
            motorspeed = [0, 0]
        elif ctrl_state == WAYPOINTS:
            if status == TRACKING:

                target = waypoints[0]
                pos, head, t = positions[-1]

                ang = get_angle(pos, head, target)
                dist = np.linalg.norm(target - pos)
                cv2.line(frame, pos, target, (255, 0, 0), 1)
                cv2.circle(frame, target, 3, (0, 0, 255))

                # calculate absolute speed

                s = np.clip(1.5 * ang, -180, 180)
                d = np.clip(dist / 100, 0.5, 1)

                if dist < 30:
                    if len(waypoints) > 1:
                        waypoints.pop(0)
                        positions = positions[-2:]
                    else:
                        motorspeed = [0.0, 0.0]
                        ctrl_state = IDLE
                elif abs(ang) < 90:
                    e = 2 * (30. - speed)
                    m = 150 + e
                    motorspeed = [d * m * np.cos(ang * np.pi / 180) + s, d * m * np.cos(ang * np.pi / 180) - s]

                else:
                    motorspeed = [s, -s]
            else:
                motorspeed = [motorspeed[0] * 0.9, motorspeed[1] * 0.9]
                dist = 0
                speed = 0
        elif ctrl_state == POSITIONING:
            pos, head, t = positions[-1]
            pos0, head0, t0 = positions[-2]
            ang = get_angle(pos, head, block)
            ang0 = get_angle(pos0, head0, block)
            motorspeed = np.clip([ang, -ang], -100, 100)
            if abs(ang) < 5 and abs(ang - ang0) / (t - t0) < 1:
                ctrl_state = DETECT
                magnetometer.flush()
        # move forwards and
        elif ctrl_state == DETECT:
            if magnetometer.is_full():
                before = magnetometer.get_mean()
                block_persistent = block

            pos, head, t = positions[-1]

        # draw waypoints
        for i in range(len(waypoints) - 1):
            cv2.line(frame, waypoints[i], waypoints[i + 1], (255, 100, 100), 1)
            cv2.circle(frame, waypoints[i + 1], 3, (100, 100, 255))

        # flash warning light when moving
        if motorspeed != [0, 0]:
            data["LEDs"][0] = flash(freq=2)
        # write motor data
        data["motors"] = list(np.clip(motorspeed, -255, 255))

        # draw info to screen
        fps, ping = video_stream.get_rate(), arduino_stream.get_rate()
        draw_ui(screen_=screen,
                arduino_stream_=arduino_stream,
                ping_=ping,
                arduinodata_=arduinodata,
                video_stream_=video_stream,
                fps_=fps,
                status_=status,
                status_color_=status_color,
                tickrate_=tickrate,
                ctrl_state_=ctrl_state,
                distance_=dist,
                speed_=speed)

        # output image to frame
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

    # graceful exit
    time.sleep(0.2)
    cv2.destroyAllWindows()
    video_stream.terminate()
    arduino_stream.terminate()
    arduino_stream.join()
    server.close()


if __name__ == "__main__":
    data = {
        "time": 0,
        "motors": [0, 0],
        "servos": [0, 0],
        "LEDs": [0, 0, 0, 0]
    }
    curses.wrapper(main)
