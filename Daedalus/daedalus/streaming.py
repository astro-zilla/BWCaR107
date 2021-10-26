import json
import socket
import time
from socket import error as socketError
from threading import Event, Thread

import cv2
import numpy as np

BLUE = 10
GREEN = 11
CYAN = 12
RED = 13
MAGENTA = 14
YELLOW = 15
WHITE = 16

INITIALIZED = 'initialized'
CONNECTING = 'connecting'
CONNECTED = 'connected'
TERMINATED = 'terminated'
ERROR = 'error'


class ArduinoStreamHandler(Thread):
    def __init__(self, server: socket.socket):
        super().__init__()
        self.server = server
        self.client = None
        self.file = None

        self.out_data = {}
        self.in_data = {}

        self.terminated = Event()
        self.out = Event()

        self._status = INITIALIZED
        self.status_color = BLUE

        self.t0 = time.time()
        self.times = {}

    def connect(self) -> bool:
        try:
            self._status = CONNECTING
            self.status_color = YELLOW
            self.client, addr = self.server.accept()
            self.file = self.client.makefile()
            self._status = CONNECTED
            self.status_color = GREEN
            return True
        except socket.timeout or socketError:
            return False

    def run(self) -> None:
        while not self.connect():
            if self.terminated.is_set():
                return

        while not self.terminated.is_set():

            self.out_data["time"] = int((time.time() - self.t0) * 100)
            # try loop is new: if client is reset to None, expected behaiviour is a fail by ConnectionResetError
            # todo check is the old setup was running this loop really fast and just hitting the JSONDecodeError
            try:
                self.client.send(bytes(json.dumps(self.out_data), 'utf-8'))
            except socket.timeout or ConnectionError:
                print('conn error')
            # it's ok for this to block because the arduino can't handle more writes than reads to it
            try:
                r = json.loads(self.file.readline())
                self.in_data = r
                self._status = CONNECTED
                self.status_color = GREEN
            except json.decoder.JSONDecodeError:
                self._status = ERROR
                self.status_color = RED
                print('json error')

            self.times[time.time()] = time.time() - (self.in_data["time"] / 100 + self.t0)

    # this should be called regularly as it not only clears the times buffer but checks that the socket is not hanging
    def get_rate(self) -> float:

        self.times = {t: p for (t, p) in self.times.items() if (time.time() - t) < 5}

        # we have recieved a packet in the last 5 seconds
        if self.times:
            ping = 1000 * np.mean(list(self.times.values()))
            if ping < 0 or ping > 5000:
                ping = np.nan
            return ping
        # we have not recieved anything in the last 5 seconds: reconnect
        else:
            return 0

    @property
    def status(self) -> str:
        # pretty print connecting with a spinner
        if self._status == CONNECTING:
            return f'{CONNECTING} {spinner()}'
        else:
            return self._status

    def reconnect(self) -> None:
        # not used: could be used to implement a non-blocking manual reconnect if the mainloop error catch doesnt work
        Thread(target=self.connect).start()

    def read(self) -> dict:
        # return last data read
        return self.in_data

    def write(self, data: dict) -> None:
        # set data to be output
        self.out_data = data

    def terminate(self) -> None:
        # graceful termination of all objects, and the thread (may block for a while)
        self.terminated.set()
        if self.client:
            self.client.close()
            self.file.close()

        self._status = TERMINATED
        self.status_color = BLUE


class VideoStreamHandler(Thread):
    def __init__(self, source: str):
        super().__init__()
        self.cap = None
        self.width = 300
        self.height = 150
        self.terminated = False
        self._frame = np.zeros((self.height, self.width, 3), dtype='uint8')
        cv2.putText(self._frame, 'NO SIGNAL', (self.width // 2 - 80, self.height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 255, 255), 2)
        self.times = []
        self.source = source
        self._status = INITIALIZED
        self.status_color = BLUE
        self.available = False

    def connect(self) -> None:
        self._status = CONNECTING
        self.status_color = YELLOW
        self.cap = cv2.VideoCapture(self.source)
        # self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        # self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._status = CONNECTED
        self.status_color = GREEN

    def run(self) -> None:
        while not self.terminated:

            if self.cap is None:
                self.connect()
            ret, frame = self.cap.read()  # read frame from stream (blocking)
            if frame is not None and frame.shape != (0, 0, 3):
                self._frame = frame
                self.times.append(time.time())
                self.available = True
            else:
                self.cap.release()
                self._frame = np.zeros((150, 300, 3), dtype='uint8')
                cv2.putText(self._frame, 'NO SIGNAL', (300 // 2 - 80, 150 // 2), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2)
                self.connect()

        self.cap.release()
        self._status = TERMINATED
        self.status_color = BLUE

    @property
    def frame(self):
        self.available = False
        return self._frame

    def get_rate(self) -> float:
        self.times = [t for t in self.times if (time.time() - t) < 5]
        if len(self.times)>1:
            return 1 / np.mean(np.diff(self.times))  # return calc fps
        else:
            return 0

    @property
    def status(self) -> str:
        if self._status == CONNECTING:
            return f'{CONNECTING} {spinner()}'
        else:
            return self._status

    def terminate(self):
        self.terminated = True


def spinner() -> str:
    t = time.time() % 1

    if t < 0.25:
        return '\u2502'
    elif t < 0.5:
        return '\u2571'
    elif t < 0.75:
        return '\u2500'
    else:
        return '\u2572'
