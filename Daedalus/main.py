import curses
import socket
import time

import cv2
import numpy as np
from numpy import array
from pynput.keyboard import KeyCode, Listener

from daedalus.Image import square, undistort
from daedalus.aruco import analyse
from daedalus.navigation import PID_consts, get_angle
from daedalus.streaming import ArduinoStreamHandler, VideoStreamHandler

# global vars for callback
mouse_pos = np.int32([678, 86])
# waypoints = [[678, 86], [581, 187], [486, 285], [334, 441], [254, 517], [154, 617], [69, 696], [163, 609], [276, 499],
#              [433, 344], [494, 278], [605, 284], [543, 227], [512, 193], [573, 195], [684, 86]]
waypoints = [array([684, 83]), array([600, 170]), array([508, 263]), array([335, 435]), array([265, 510]), array([181, 482]),
             array([180, 549]), array([220, 561]), array([264, 511]), array([432, 340]), array([553, 219]), array([694, 75])]

keys = set()

# pynput keycodes
key_q = KeyCode.from_char('q')

key_w = KeyCode.from_char('w')
key_a = KeyCode.from_char('a')
key_s = KeyCode.from_char('s')
key_d = KeyCode.from_char('d')

key_r = KeyCode.from_char('r')
key_f = KeyCode.from_char('f')

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
            distance_: float, speed_: float) -> None:
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

    screen_.addstr('OpenCV: ', curses.color_pair(WHITE))
    screen_.addstr(f'{status_}\n', curses.color_pair(status_color_))
    screen_.addstr(f'\u251c\u2500\u2500')
    screen_.addstr(f'tickrate: ', curses.color_pair(WHITE))
    screen_.addstr(f'{tickrate_:.1f} Hz\n', curses.color_pair(BLUE))
    screen_.addstr(f'\u251c\u2500\u2500')
    screen_.addstr(f'distance: ', curses.color_pair(WHITE))
    screen_.addstr(f'{distance_:.1f} px\n', curses.color_pair(BLUE))
    screen_.addstr(f'\u251c\u2500\u2500')
    screen_.addstr(f'speed: ', curses.color_pair(WHITE))
    screen_.addstr(f'{speed_:.1f} px/s\n', curses.color_pair(BLUE))
    screen_.addstr(f'\u2514\u2500\u2500')
    screen_.addstr(f'waypoints: ', curses.color_pair(WHITE))
    screen_.addstr(f'{waypoints}\n')

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
    K_rot = PID_consts(1.5, 0, 0)
    motorspeed = [0.0, 0.0]

    # status vars
    status = NO_IMAGE
    status_color = RED

    # frame ids
    f_num = 1.
    detect_num = 0.
    frame = video_stream.frame

    positions = [([0, 0], [0, 0], 0.), ([0, 0], [0, 0], 1.)]

    while True:
        # calculate main loop tickrate
        times.append(time.time())
        times = times[-10:]
        tickrate = 1 / np.mean(np.diff(times))

        # get/send arduino data from
        arduino_stream.write(data)
        arduinodata = arduino_stream.read()

        # get video data from stream
        if video_stream.available:
            frame1 = video_stream.frame

            frame = undistort(frame1, balance=0.5)
            frame = square(frame)
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

        # control AI
        if status == TRACKING:

            target = waypoints[0]
            pos, head, t = positions[-1]

            ang = get_angle(pos, head, target)
            dist = np.linalg.norm(target - pos)
            cv2.line(frame, pos, target, (255, 0, 0), 1)
            cv2.circle(frame, target, 3, (0, 0, 255))

            # calculate motor speeds
            speed = np.linalg.norm(positions[-1][0] - positions[-2][0]) / (positions[-1][2] - positions[-2][2])

            s = np.clip(K_rot.p * ang, -180, 180)
            d = np.clip(dist / 100, 0.5, 1)

            if dist < 30:
                if len(waypoints) > 1:
                    waypoints.pop(0)
                    positions = positions[-2:]
                else:
                    motorspeed = [0.0, 0.0]
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

        for i in range(len(waypoints) - 1):
            cv2.line(frame, waypoints[i], waypoints[i + 1], (255, 100, 100), 1)
            cv2.circle(frame, waypoints[i + 1], 3, (100, 100, 255))

        data["motors"] = list(np.clip(motorspeed, -255, 255))

        # draw info to screen
        fps, ping = video_stream.get_rate(), arduino_stream.get_rate()
        draw_ui(screen, arduino_stream, ping, arduinodata, video_stream, fps, status, status_color, tickrate, dist, speed)

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
