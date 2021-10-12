import math
import threading
import time

import cv2
import numpy
import numpy as np


class VideoStreamHandler(threading.Thread):
    def __init__(self, source):
        super().__init__()
        self.cap = None
        self.width = 1016
        self.height = 760
        self.terminated = False
        self.frame = np.zeros((760, 1016, 3))
        self.times = [0] * 10  # init list of frame arrival times to calculate fps
        self.source = source
        self.connect(self.source)

    def connect(self, source):
        self.cap = cv2.VideoCapture(source)
        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
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
                print('# camera connection lost... retrying')
                self.connect(self.source)

        self.cap.release()
        print('# camera connection successfully terminated')

    def get_fps(self) -> float:

        if self.times[-1] < (time.time() - 10):
            return 0  # fps < 0.1 = 0
        else:
            return 1 / np.mean(np.diff(self.times))  # return calc fps

    def terminate(self):
        self.terminated = True


class ArduinoStreamHandler(threading.Thread):
    def __init__(self, server, data):
        super().__init__()

        self.server = server
        self.client = None
        self.file = None
        self.terminated = False
        self.out_data = data
        self.in_data = ''

    def connect(self):
        self.client, addr = self.server.accept()
        print(f'# recieved connection from arduino: {addr[0]}:{addr[1]}')
        self.file = self.client.makefile()  # file access for reads ensures full reads
        self.client.send(bytes(self.out_data, 'utf8'))  # TODO CASE IN WHICH DATA IS AN EMPTY STRING
        self.in_data = self.file.readline()  # readline to ensure full read of JSON
        print('# arduino datastream initiated')

    def run(self):
        while not self.terminated:
            if self.client is None:
                self.connect()
            try:
                self.client.send(bytes(self.out_data, 'utf8'))
                self.in_data = self.file.readline()
            except ConnectionAbortedError:
                print(f'connection to arduino lost, reconnecting...')
                self.connect()
                print('# arduino datastream recovered')

        # exit gracefully
        self.file.close()
        self.client.close()
        print('# arduino connection successfully terminated')

    def set_data(self, data):
        self.out_data = data

    def get_data(self):
        return self.in_data

    def terminate(self):
        self.terminated = True
