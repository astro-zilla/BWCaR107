import socket
import time
from socket import error as socketError
from threading import Event, Thread

import cv2
import numpy as np


class ArduinoStreamHandler(Thread):
    def __init__(self, server: socket.socket, data: str):
        super().__init__()
        self.server = server
        self.client = None

        self.out_data = '{"NULL":"NULL"}'
        self.in_data = ''

        self.writer = None
        self.reader = None

        self.terminated = Event()
        self.times = [0.] * 20

    def connect(self):
        try:
            print(f'# listening for connection from Arduino')
            self.client, addr = self.server.accept()
            print(f'# recieved connection from arduino: {addr[0]}:{addr[1]}')
            return True
        except socketError:
            return False

    def run(self):
        while not self.client:
            self.connect()
            if self.terminated.is_set():
                return
        self.writer = StreamWriter(self.client, bytes(self.out_data, 'utf-8'))
        self.reader = StreamReader(self.client)

        self.writer.start()
        self.reader.start()
        while not self.terminated.is_set():
            self.writer.ready.set()
            self.reader.ready.set()
            self.reader.ready.wait(1)
            self.times.append(time.time())
            self.times = self.times[-20:]

    def get_rate(self):
        return np.mean(np.diff(self.times))

    @property
    def data(self):
        if self.reader:
            return self.reader.data
        else:
            return self.in_data

    @data.setter
    def data(self, data):
        if self.writer:
            self.writer.data = bytes(data, 'utf-8')
        else:
            self.out_data = data

    def terminate(self):
        self.terminated.set()
        if self.client:
            self.client.close()
            if self.writer:
                self.writer.ready.set()
                self.writer.terminated.set()
            if self.reader:
                self.reader.ready.set()
                self.reader.terminated.set()

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
        self.connect()

    def connect(self):
        self.cap = cv2.VideoCapture(self.source)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print('# camera datastream initiated')

    def run(self):
        while not self.terminated:
            ret, frame = self.cap.read()  # read frame from stream (blocking)

            if self.cap is None:
                self.connect()
            if frame is not None and frame.shape != (0, 0, 3):
                self.frame = frame
                self.times.append(time.time())
                self.times = self.times[-10:]  # keep 10-buffer of frame times to calc avg
            else:
                self.cap.release()
                self.frame = np.zeros((150, 300, 3), dtype='uint8')
                cv2.putText(self.frame, 'NO SIGNAL', (300 // 2 - 80, 150 // 2), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2)
                print('# camera connection lost... retrying')
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
