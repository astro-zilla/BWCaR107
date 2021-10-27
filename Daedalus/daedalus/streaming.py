import json
import socket
import time
from socket import error as socketError
from threading import Event, Thread
from typing import Dict

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
        except socketError:
            return False

    def run(self) -> None:
        while not self.client:
            self.connect()
            if self.terminated.is_set():
                return

        while not self.terminated.is_set():
            self.out_data["time"] = int((time.time() - self.t0) * 100)
            self.client.send(bytes(json.dumps(self.out_data), 'utf-8'))
            # it's ok for this to block because the arduino can't handle more writes than reads to it
            try:
                r = json.loads(self.file.readline())
                self.in_data = r
            except json.decoder.JSONDecodeError:
                pass

            self.times[time.time()] = time.time() - (self.in_data["time"] / 100 + self.t0)

    def get_rate(self) -> float:

        self.times = {t: p for (t, p) in self.times.items() if (time.time() - t) < 5}

        if self.times:
            ping = 1000 * np.mean(list(self.times.values()))
            if ping < 0 or ping > 5000:
                ping = np.nan
            return ping
        else:
            self.reconnect()
            return 0

    @property
    def status(self) -> str:
        if self._status == CONNECTING:
            return f'{CONNECTING} {spinner()}'
        else:
            return self._status

    def reconnect(self) -> None:
        pass

    def read(self) -> dict:
        return self.in_data

    def write(self, data: dict) -> None:
        self.out_data = data

    def terminate(self) -> None:
        self.terminated.set()
        if self.client:
            self.client.close()

        self.server.close()
        self._status = TERMINATED
        self.status_color = BLUE


class StreamReader(Thread):
    def __init__(self, client):
        super().__init__()
        self.data = b''
        self.file = client.makefile()

        self.ready = Event()
        self.terminated = Event()

    def run(self) -> None:
        while not self.terminated.is_set():
            self.ready.wait(5)
            self.data = self.file.readline()
            self.ready.clear()


class StreamWriter(Thread):
    def __init__(self, client, data):
        super().__init__()
        self.client = client
        self.data = data

        self.ready = Event()
        self.terminated = Event()

    def run(self):
        while not self.terminated.is_set():
            self.ready.wait(5)
            self.client.send(self.data)
            self.ready.clear()


class VideoStreamHandler(Thread):
    def __init__(self, source:str):
        super().__init__()
        self.cap = None
        self.width = 300
        self.height = 150
        self.terminated = False
        self.frame = np.zeros((self.height, self.width, 3), dtype='uint8')
        cv2.putText(self.frame, 'NO SIGNAL', (self.width // 2 - 80, self.height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 255, 255), 2)
        self.times = []
        self.source = source
        self._status = INITIALIZED
        self.status_color = BLUE

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
                self.frame = frame
                self.times.append(time.time())
            else:
                self.cap.release()
                self.frame = np.zeros((150, 300, 3), dtype='uint8')
                cv2.putText(self.frame, 'NO SIGNAL', (300 // 2 - 80, 150 // 2), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2)
                self.connect()

        self.cap.release()
        self._status = TERMINATED
        self.status_color = BLUE

    def get_rate(self) -> float:
        self.times = [t for t in self.times if (time.time() - t) < 5]
        if self.times:
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


def spinner() -> int:
    t = time.time() % 1

    if t < 0.25:
        return 1
    elif t < 0.5:
        return 0
    elif t < 0.75:
        return 1
    else:
        return 0
