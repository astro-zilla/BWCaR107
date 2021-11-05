"""Main daedalus server program: IDP M1 2021

This program uses daedalus to track an ArUco marker mounted on the back
of a robot, and drive it towards a target object. It commands the robot
to pick up the object, test for metal contents and then proceeds to
deposit it in the corresponding collection area.

May also be used as a framework to develop further usage of daedalus in
robot navigation and computer vision.

Written by Ellis Collins and Massimo Zambernadi."""

# standard library modules
import curses
import datetime
import socket
import time

# installed library modules (see /requirements.txt)
import cv2
import numpy as np
from numpy import array
from pynput.keyboard import Key, KeyCode, Listener

from daedalus.Image import square, undistort
from daedalus.aruco import analyse
from daedalus.navigation import find_block, get_angle
# daedalus modules
from daedalus.peripherals import Buffer
from daedalus.streaming import ArduinoStreamHandler, VideoStreamHandler

# global vars for callback
mouse_pos = np.int32([678, 86])

# waypoint lists
out = [array([575, 182]), array([495, 263]), array([261, 498]), array([160, 602])]
back_red = [array([161, 607]), array([215, 553]), array([266, 503]), array([495, 272]), array([488, 215]), array([576, 254])]
back_blu = [array([159, 602]), array([215, 550]), array([262, 499]), array([493, 268]), array([567, 264]), array([504, 178])]
home = [array([555, 203]), array([606, 155]), array([698, 68])]

# must use a copy to preservve original list
waypoints = out.copy()

# set to contain keypresses
keys = set()

# pynput keycodes
key_q = KeyCode.from_char('q')

key_c = KeyCode.from_char('c')
key_o = KeyCode.from_char('o')

key_t = KeyCode.from_char('t')
key_g = KeyCode.from_char('g')

key_i = KeyCode.from_char('i')
key_w = KeyCode.from_char('w')
key_p = KeyCode.from_char('p')
key_d = KeyCode.from_char('d')
key_f = KeyCode.from_char('f')

key_n = KeyCode.from_char('n')
key_r = KeyCode.from_char('r')
key_s = KeyCode.from_char('s')

key_1 = KeyCode.from_char('1')
key_2 = KeyCode.from_char('2')
key_3 = KeyCode.from_char('3')
key_4 = KeyCode.from_char('4')

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
TERMINATED = 'terminated'

# control state codes
IDLE = 'idle'
WAYPOINTS = 'waypoints'
POSITIONING = 'positioning'
DETECT = 'detecting'
DROP = 'dropping'
RESTART = 'checking for restart'

# thresholds
MAGNETOMETER_THRESHOLD = 130


def mouse(event: int, x: float, y: float, flags, params):
    # mouse callback
    global waypoints
    # on rightclick, append to waypoints
    if event == cv2.EVENT_RBUTTONDOWN:
        waypoints.append(np.int32([x, y]))
        # on leftclick, replace waypoints
    elif event == cv2.EVENT_LBUTTONDOWN:
        waypoints = [np.int32([x, y])]


# keypress callback: add key to key set
def on_press(key):
    keys.add(key)


# key release callback: discard key from key set
def on_release(key):
    # discard does not throw an error if the key is (for some reason) not present
    keys.discard(key)
    if key == key_q:
        # Stop listener
        return False


# draws UI using curses to show important info at runtime
def draw_ui(screen_: curses.window, arduino_stream_: ArduinoStreamHandler, ping_: float, arduinodata_: dict,
            magnetometer_: Buffer, video_stream_: VideoStreamHandler, cam_: int, fps_: float, status_: str,
            status_color_: int,
            tickrate_: float, ctrl_state_: str, distance_: float, speed_: float) -> None:
    screen_.clear()
    # arduino thread info
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

        screen_.addstr(f'\u251c\u2500\u2500')
        screen_.addstr(f'magnetometer: ', curses.color_pair(WHITE))
        screen_.addstr(f'{magnetometer_.get_mean():.1f}\n', curses.color_pair(BLUE))
        if 'before' in magnetometer_.means:
            screen_.addstr(f'\u251c\u2500\u2500')
            screen_.addstr(f'magnetometer \u0394: ', curses.color_pair(WHITE))
            delta = magnetometer_.get_mean() - magnetometer_.retrieve_mean("before")
            screen_.addstr(f'{delta:.1f}, {100 * delta / max(magnetometer_.get_mean(), 1):.0f}%\n', curses.color_pair(BLUE))

        screen_.addstr(f'\u2514\u2500\u2500')
        screen_.addstr(f'commands\n', curses.color_pair(WHITE))

        for i, (k, v) in enumerate(data.items()):
            if i < len(data) - 1:
                screen_.addstr(f'   \u251c\u2500\u2500{k}: {v}\n')
            else:
                screen_.addstr(f'   \u2514\u2500\u2500{k}: {v}\n')

    # video thread info
    screen_.addstr('Camera: ', curses.color_pair(WHITE))
    screen_.addstr(f'idpcam{cam_} {video_stream_.status}\n', curses.color_pair(video_stream_.status_color))

    if video_stream_.status == 'connected':
        screen_.addstr(f'\u2514\u2500\u2500')
        screen_.addstr(f'fps: ', curses.color_pair(WHITE))
        screen_.addstr(f'{fps_:.1f}\n', curses.color_pair(BLUE))

    # main thread info
    screen_.addstr('Daedalus: ', curses.color_pair(WHITE))
    screen_.addstr(f'{status_}\n', curses.color_pair(status_color_))

    if status_ != TERMINATED:
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


def main(screen: curses.window = curses.initscr(), robot_aruco_id: int = 7, cam=1):
    # waypoints needs to be accessible from mouse callbacks
    global waypoints
    # broadcast locally on 53282
    host = socket.gethostname()
    port = 53282

    # listen on streaming socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen()

    # clears logfile of previous entries
    with open('daedalus/daedalus.log', 'w+') as f:
        f.write(f'{datetime.time()}: START')
        pass

    # init asynchronous threading stream handlers
    video_stream = VideoStreamHandler(f"http://localhost:808{cam}/stream/video.mjpeg")
    # video_stream = VideoStreamHandler(f"http://idpcam{cam}.eng.cam.ac.uk:8080/stream/video.mjpeg")
    arduino_stream = ArduinoStreamHandler(server)
    # begin with write to poll arduino
    arduino_stream.write(data)

    # start streams
    video_stream.start()
    arduino_stream.start()

    # curses terminal: invisible cursor, init colours
    curses.curs_set(False)
    curses.start_color()
    curses.use_default_colors()
    for i in range(0, 255):
        curses.init_pair(i + 1, i, -1)

    # clear terminal
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
    next_state = IDLE

    # frame ids
    f_num = 1.
    detect_num = 0.
    tstart = 0

    # init stream vals
    frame = video_stream.frame
    arduinodata = arduino_stream.read()

    # init positional data
    positions = [([0, 0], [0, 0], 0.), ([0, 0], [0, 0], 1.)]
    speed = 0
    block = np.array([0, 0])
    blocks_collected = 0

    # init timer vars
    time_start = 0
    LED_timer = 0

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
            frame = video_stream.frame

            frame = undistort(frame, balance=0.5)
            frame = square(frame, cam=cam)

            # find block in corner
            b = find_block(frame)
            if b is not False:
                block = b

            # set variable representing unique frame id
            f_num = time.time()

        # check if we have detected the marker this frame
        if detect_num != f_num and frame is not None:
            dictionary = {}

            # find ArUco marker and annotate
            frame = analyse(frame, dictionary, visualise=True)

            # look for specific marker
            if robot_aruco_id in dictionary.keys():
                status = TRACKING
                status_color = GREEN

                # get state vectors of marker
                position, heading = dictionary[robot_aruco_id]
                positions.append((position, heading, time.time()))

                # clip position list to 50
                positions = positions[-50:]
                # calc speed in px/s
                speed = np.linalg.norm(positions[-1][0] - positions[-2][0]) / (positions[-1][2] - positions[-2][2])

                # show that we have detected the marker for this fra
                detect_num = f_num
            else:
                # 200ms grace period for losing the marker
                if time.time() - detect_num > 0.2:
                    status = MARKER_NOT_FOUND
                    status_color = YELLOW
                else:
                    status = TRACKING
                    status_color = GREEN

        # press q key to exit
        if key_q in keys:
            listener.join()
            data["motors"] = [0, 0]
            arduino_stream.write(data)
            break

        # close and open pincers
        if key_c in keys:
            data["servos"] -= 5
        if key_o in keys:
            data["servos"] += 5

        # debug control state
        if key_i in keys:
            data["LEDs"] = [0, 0, 0]
            ctrl_state = IDLE
        elif key_w in keys:
            ctrl_state = WAYPOINTS
        elif key_p in keys:
            data["LEDs"] = [0, 0, 0]
            tstart = time.time()
            ctrl_state = POSITIONING
        elif key_d in keys:
            data["LEDs"] = [0, 0, 0]
            ctrl_state = DETECT
            magnetometer.means = {}
            magnetometer.flush()

        # manually flush magnetometer buffer
        if key_f in keys:
            magnetometer.flush()

        # next waypoint
        if key_n in keys:
            ctrl_state = next_state
        # start run
        elif key_s in keys:
            blocks_collected = 0
            time_start = time.time()
            ctrl_state = RESTART
        # restart run (keep collected blocks and start time same)
        elif key_r in keys:
            ctrl_state = RESTART

        # debug waypoint presets - change next_state value if you
        # want to use this section to debug parts of the run-through
        if key_1 in keys:
            waypoints = out.copy()
            data["LEDs"] = [0, 0, 0]
            next_state = IDLE
            # next_state = POSITIONING
        elif key_2 in keys:
            waypoints = back_red.copy()
            data["LEDs"] = [0, 1, 0]
            next_state = IDLE
            # next_state = DROP
        elif key_3 in keys:
            waypoints = back_blu.copy()
            data["LEDs"] = [0, 0, 1]
            next_state = IDLE
            # next_state = DROP
        elif key_4 in keys:
            waypoints = home.copy()
            data["LEDs"] = [0, 0, 0]
            next_state = IDLE
            # next_state = IDLE

        # change camera stream
        if Key.right in keys:
            cam = 2
            video_stream.cap = None
            video_stream.source = f"http://localhost:808{2}/stream/video.mjpeg"
        elif Key.left in keys:
            cam = 1
            video_stream.cap = None
            video_stream.source = f"http://localhost:808{1}/stream/video.mjpeg"

        # control AI
        dist = 0
        # IDLE - do nothing
        if ctrl_state == IDLE:
            motorspeed = [0, 0]

        # WAYPOINTS - follow wayoints
        elif ctrl_state == WAYPOINTS:
            if status == TRACKING:

                # state vectors
                target = waypoints[0]
                pos, head, t = positions[-1]

                # get and draw error angle, distance
                ang = get_angle(pos, head, target)
                dist = np.linalg.norm(target - pos)
                cv2.line(frame, pos, target, (255, 0, 0), 1)
                cv2.circle(frame, target, 3, (0, 0, 255))

                # calculate absolute speed in px/s
                s = np.clip(5 * ang, -200, 200)
                # tuning parameter to slow down when approaching a point (left off for IDP M1 2021)
                d = 1  # np.clip(dist / 20, 0.5, 1)

                # condition for reaching a waypoint
                if dist < 30:
                    if len(waypoints) > 1:
                        waypoints.pop(0)
                        positions = positions[-2:]
                    else:
                        motorspeed = [0.0, 0.0]
                        tstart = time.time()
                        ctrl_state = next_state
                # condition for correcting course en route to waypoint
                elif abs(ang) < 90:
                    e = (50. - speed)
                    m = 200 + 1.5 * e

                    motorspeed = [d * m * np.cos(ang * np.pi / 180) + s, d * m * np.cos(ang * np.pi / 180) - s]
                # condition for stationary turn to waypoints
                else:
                    motorspeed = [s, -s]

                # kick up ramp
                if data["LEDs"] == [1,0,0]:
                    if speed<10:
                        data["servos"] = 25
                    else:
                        data["servos"] = 0

            # if not tracking, decay motor speeds (not instant stop)
            else:
                motorspeed = [motorspeed[0] * 0.99, motorspeed[1] * 0.99]
                dist = 0
                speed = 0

        # POSITIONING - turn towards block and approach slowly
        elif ctrl_state == POSITIONING:
            # open pincers
            data["servos"] = 180

            # state vectors and errors
            pos, head, t = positions[-1]
            pos0, head0, t0 = positions[-2]
            ang = get_angle(pos, head, block)
            ang0 = get_angle(pos0, head0, block)

            # turning to block
            motorspeed = np.clip([10 * ang, -10 * ang], -100, 100)
            if (abs(ang) < 5 and abs(ang - ang0) / (t - t0) < 5) or (time.time() - tstart) > 45:
                if np.linalg.norm(block - positions[-1][0]) > 35 and (time.time() - tstart) < 46:
                    # moving forwards to block
                    motorspeed[:] += 100
                else:
                    # reached collection point
                    ctrl_state = DETECT
                    # flush magnetometer buffer
                    magnetometer.flush()

        # move forwards andddd
        elif ctrl_state == DETECT:
            motorspeed = [0, 0]

            # check if control data has been collected
            if 'before' in magnetometer.means.keys():
                # close pincer arms if they are not closed
                if arduinodata["servos"] > 15:
                    data["servos"] = 0
                    magnetometer.flush()
                # if pincer arms are closed and the buffer has been filled, check for metal and continue
                elif not magnetometer.is_full():
                    # timer for 5s LED pulse
                    LED_timer = time.time()
                else:
                    # if discrepancy between control and test data > threshold assume metallic block
                    # if abs(magnetometer.get_mean() - magnetometer.retrieve_mean('before')) > MAGNETOMETER_THRESHOLD:
                    if abs(magnetometer.get_mean() - magnetometer.retrieve_mean('before')) / magnetometer.get_mean() > 0.065:
                        # metal detected
                        data["LEDs"] = [0, 1, 0]
                        waypoints = back_red.copy()
                    else:
                        # no metal detected
                        data["LEDs"] = [0, 0, 1]
                        waypoints = back_blu.copy()
                    # hold LED high for 5s
                    #if time.time() - LED_timer > 5:
                    ctrl_state = WAYPOINTS
                    next_state = DROP

            elif arduinodata["servos"] < 179:
                data["servos"] = 180
                magnetometer.flush()
            # collect control data
            elif magnetometer.is_full():
                magnetometer.save_mean('before')

        # DROP - drop block in deposition area and retreat
        elif ctrl_state == DROP:
            # halt
            motorspeed = [0, 0]
            # open pincers
            if arduinodata["servos"] < 120:
                data["servos"] = 180
            # retreat back
            elif np.linalg.norm(waypoints[0] - positions[-1][0]) < 100:
                motorspeed = [-150, -150]
            # complete: restart cycle
            else:
                data["servos"] = 10
                blocks_collected += 1
                ctrl_state = RESTART

        # RESTART - check if there is enough time to restart
        elif ctrl_state == RESTART:
            magnetometer.means = {}
            data["LEDs"] = [0, 0, 0]

            # enough time: go again
            if 300 - (time.time() - time_start) > 90:  # (allowed time) - (runtime) > (lap time) req. for a full run
                waypoints = out.copy()
                ctrl_state = WAYPOINTS
                next_state = POSITIONING

            # no time remaining: return to initial square
            else:
                waypoints = home.copy()
                ctrl_state = WAYPOINTS
                next_state = IDLE

        # draw waypoints
        for i in range(len(waypoints) - 1):
            cv2.line(frame, waypoints[i], waypoints[i + 1], (255, 100, 100), 1)
            cv2.circle(frame, waypoints[i + 1], 3, (100, 100, 255))

        # draw block
        cv2.circle(frame, block, 3, (50, 255, 100), -1)

        # write motor data
        data["motors"] = list(np.clip(motorspeed, -255, 255))

        # flashing LED (0) to display when moving
        if data["motors"] != [0, 0]:
            data["LEDs"][0] = 1
        else:
            data["LEDs"][0] = 0

        # draw info to screen
        fps, ping = video_stream.get_rate(), arduino_stream.get_rate()
        draw_ui(screen_=screen,
                arduino_stream_=arduino_stream,
                ping_=ping,
                arduinodata_=arduinodata,
                magnetometer_=magnetometer,
                video_stream_=video_stream,
                cam_=cam,
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
    status = TERMINATED
    status_color = RED
    draw_ui(screen_=screen,
            arduino_stream_=arduino_stream,
            ping_=0,
            arduinodata_=arduinodata,
            magnetometer_=magnetometer,
            video_stream_=video_stream,
            cam_=0,
            fps_=0,
            status_=status,
            status_color_=status_color,
            tickrate_=tickrate,
            ctrl_state_=ctrl_state,
            distance_=0,
            speed_=speed)
    arduino_stream.join()
    server.close()


if __name__ == "__main__":
    # data template
    data = {
        "time": 0,
        "motors": [0, 0],
        "servos": 0,
        "LEDs": [0, 0, 0]
    }
    curses.wrapper(main)
