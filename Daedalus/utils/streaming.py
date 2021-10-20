import json
import socket
import time
from socket import error as socketError
from threading import Event, Thread

import cv2
import numpy as np


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

        self.times = [0.] * 50

    def connect(self):
        try:
            print(f'# listening for connection from Arduino')
            self.client, addr = self.server.accept()
            self.file = self.client.makefile()
            print(f'# recieved connection from arduino: {addr[0]}:{addr[1]}')
            return True
        except socketError:
            return False

    def run(self):
        while not self.client:
            self.connect()
            if self.terminated.is_set():
                return

        while not self.terminated.is_set():
            # if self.out.is_set():
            self.out_data["time"] = time.time()
            self.client.send(bytes(json.dumps(self.out_data),'utf-8'))
            # self.out.clear()

            # it's ok for this to block because the arduino can't handle more writes than reads to it
            self.in_data = json.loads(self.file.readline())
            print(time.time(),self.in_data["time"])
            self.times.append(time.time()-self.in_data["time"])
            self.times = self.times[-50:]

    def get_rate(self):
        return np.mean(self.times)

    def read(self):
        return self.in_data

    def write(self, data):
        self.out_data = data
        self.out.set()

    def terminate(self):
        self.terminated.set()
        if self.client:
            self.client.close()

        self.server.close()
        print('# arduino connection successfully terminated')


class StreamReader(Thread):
    def __init__(self, client):
        super().__init__()
        self.data = b''
        self.file = client.makefile()

        self.ready = Event()
        self.terminated = Event()

    def run(self):
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
    def __init__(self, source):
        super().__init__()
        self.cap = None
        self.width = 300
        self.height = 150
        self.terminated = False
        self.frame = np.zeros((self.height, self.width, 3), dtype='uint8')
        cv2.putText(self.frame, 'NO SIGNAL', (self.width // 2 - 80, self.height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 255, 255), 2)
        self.times = [0] * 10  # init list of frame arrival times to calculate fps
        self.source = source

    def connect(self):
        print('# connecting to camera')
        self.cap = cv2.VideoCapture(self.source)
        # self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        # self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print('# camera datastream initiated')

    def run(self):
        while not self.terminated:

            if self.cap is None:
                self.connect()
            ret, frame = self.cap.read()  # read frame from stream (blocking)
            if frame is not None and frame.shape != (0, 0, 3):
                self.frame = frame
                self.times.append(time.time())
                self.times = self.times[-10:]  # keep 10-buffer of frame times to calc avg
            else:
                self.cap.release()
                self.frame = np.zeros((150, 300, 3), dtype='uint8')
                cv2.putText(self.frame, 'NO SIGNAL', (300 // 2 - 80, 150 // 2), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2)
                self.connect()

        self.cap.release()
        print('# camera connection successfully terminated')

    def get_rate(self) -> float:
        if self.times[-1] < (time.time() - 10):
            return 0  # fps < 0.1 = 0
        else:
            return 1 / np.mean(np.diff(self.times))  # return calc fps

    def terminate(self):
        self.terminated = True
