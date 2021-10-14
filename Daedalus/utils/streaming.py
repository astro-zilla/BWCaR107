import time
from socket import error as socketError
from threading import Thread

import cv2
import numpy as np


class ArduinoStreamHandler(Thread):
    def __init__(self, server, data):
        super().__init__()

        self.server = server
        self.client = None
        self.file = None
        self.terminated = False
        self.times = [0] * 10  # init list of data arrival times to calculate fps
        self.out_data = data
        self.in_data = ''

    def connect(self):
        try:
            print(f'# listening for connection from Arduino')
            self.client, addr = self.server.accept()
            print(f'# recieved connection from arduino: {addr[0]}:{addr[1]}')
            self.file = self.client.makefile()  # file access for reads ensures full reads
            self.client.send(bytes(self.out_data, 'utf8'))
            self.in_data = self.file.readline()  # readline to ensure full read of JSON
            return True
        except socketError:
            return False

    def run(self):
        while not self.terminated:
            if self.client is None:
                if self.connect():
                    print('# arduino datastream initiated')
                continue
            try:
                self.client.send(bytes(self.out_data, 'utf8'))
                self.in_data = self.file.readline()
                self.times.append(time.time())
                self.times = self.times[-10:]  # keep 10-buffer of frame times to calc avg
            except ConnectionAbortedError:
                print(f'connection to arduino lost, reconnecting...')
                self.connect()
                print('# arduino datastream recovered')

        # exit gracefully
        if self.file:
            self.file.close()
        if self.client:
            self.client.close()
        print('# arduino connection successfully terminated')

    def get_rate(self) -> float:
        if self.times[-1] < (time.time() - 10):
            return 0  # fps < 0.1 = 0
        else:
            return 1 / np.mean(np.diff(self.times))  # return calc fps

    def set_data(self, data):
        self.out_data = data

    def get_data(self):
        return self.in_data

    def terminate(self):
        self.terminated = True


class VideoStreamHandler(Thread):
    def __init__(self, source):
        super().__init__()
        self.cap = None
        self.width = 300
        self.height = 150
        self.terminated = False
        self.frame = np.zeros((self.height, self.width, 3),dtype='uint8')
        cv2.putText(self.frame, 'NO SIGNAL', (self.width//2-80, self.height//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        self.times = [0] * 10  # init list of frame arrival times to calculate fps
        self.source = source
        self.connect(self.source)

    def connect(self, source):
        self.cap = cv2.VideoCapture(source)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print('# camera datastream initiated')

    def run(self):
        while not self.terminated:
            ret, frame = self.cap.read()  # read frame from stream (blocking)

            if self.cap is None:
                self.connect(self.source)
            if frame is not None:
                self.frame = frame
                self.times.append(time.time())
                self.times = self.times[-10:]  # keep 10-buffer of frame times to calc avg
            else:
                self.cap.release()
                self.frame = np.zeros((self.height, self.width, 3), dtype='uint8')
                cv2.putText(self.frame, 'NO SIGNAL', (self.width // 2 - 80, self.height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2)
                print('# camera connection lost... retrying')
                self.connect(self.source)

        self.cap.release()
        print('# camera connection successfully terminated')

    def get_rate(self) -> float:
        if self.times[-1] < (time.time() - 10):
            return 0  # fps < 0.1 = 0
        else:
            return 1 / np.mean(np.diff(self.times))  # return calc fps

    def terminate(self):
        self.terminated = True
