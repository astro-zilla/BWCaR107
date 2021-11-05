"""Streaming handlers for daedalus datastreams.

daedalus requires a connection to a WiFi-enables Arduino and a video
source, both of which operate at different rates. This package provides
asynchronous handlers for these data streams and allows for
availability polling and blind grabbing of data."""
from json import decoder, loads
from socket import error as socketError, timeout as socketTimeout, socket
from threading import Event, Thread
from time import time

from cv2 import FONT_HERSHEY_SIMPLEX, VideoCapture, putText
from numpy import diff, mean, nan, zeros

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
    def __init__(self, server: socket):
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
        self.available = False

        self.t0 = time()
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
        except socketTimeout or socketError:
            return False

    def run(self) -> None:
        while not self.connect():
            if self.terminated.is_set():
                return

        while not self.terminated.is_set():

            self.out_data["time"] = int((time() - self.t0) * 100)
            # try loop is new: if client is reset to None, expected behaiviour is a fail by ConnectionError

            try:
                self.client.send(bytes(str(self.out_data), 'utf-8'))
            except socketTimeout or ConnectionError as e:
                pass
            # it's ok for this to block because the arduino can't handle more writes than reads to it
            try:
                r = loads(self.file.readline())
                self.in_data = r
                self.available = True
            except decoder.JSONDecodeError as e:
                pass

            self.times[time()] = time() - (self.in_data["time"] / 100 + self.t0)

    # this should be called regularly as it not only clears the times buffer but checks that the socket is not hanging
    def get_rate(self) -> float:

        self.times = {t: p for (t, p) in self.times.items() if (time() - t) < 5}

        # we have recieved a packet in the last 5 seconds
        if self.times:
            ping = 1000 * mean(list(self.times.values()))
            if ping < 0 or ping > 5000:
                ping = nan
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
        self.available = False
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
        self.status_color = RED


class VideoStreamHandler(Thread):
    def __init__(self, source: str):
        super().__init__()
        self.cap = None
        self.width = 300
        self.height = 150
        self.terminated = Event()
        self._frame = zeros((self.height, self.width, 3), dtype='uint8')
        putText(self._frame, 'NO SIGNAL', (self.width // 2 - 80, self.height // 2), FONT_HERSHEY_SIMPLEX, 1,
                    (255, 255, 255), 2)
        self.times = []
        self.source = source
        self._status = INITIALIZED
        self.status_color = BLUE
        self.available = False

    def connect(self) -> None:
        self._status = CONNECTING
        self.status_color = YELLOW
        self.cap = VideoCapture(self.source)

        self._status = CONNECTED
        self.status_color = GREEN

    def run(self) -> None:
        while not self.terminated.is_set():

            if self.cap is None:
                self.connect()
            ret, frame = self.cap.read()  # read frame from stream (blocking)
            if frame is not None and frame.shape != (0, 0, 3):
                self._frame = frame
                self.times.append(time())
                self.available = True
            else:
                self.cap.release()
                self._frame = zeros((150, 300, 3), dtype='uint8')
                putText(self._frame, 'NO SIGNAL', (300 // 2 - 80, 150 // 2), FONT_HERSHEY_SIMPLEX, 1,
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
        self.times = [t for t in self.times if (time() - t) < 5]
        if len(self.times) > 1:
            return 1 / mean(diff(self.times))  # return calc fps
        else:
            return 0

    @property
    def status(self) -> str:
        # pretty print connecting with a spinner
        if self._status == CONNECTING:
            return f'{CONNECTING} {spinner()}'
        else:
            return self._status

    def terminate(self):
        self._status = TERMINATED
        self.status_color = RED
        self.terminated.set()


def spinner() -> str:
    t = time() % 1

    if t < 0.25:
        return '\u2502'
    elif t < 0.5:
        return '\u2571'
    elif t < 0.75:
        return '\u2500'
    else:
        return '\u2572'
