import threading
import time

import cv2
import numpy


class VideoStreamHandler(threading.Thread):
    def __init__(self, source):
        super().__init__()
        self.cap = None
        self.width = 0
        self.height = 0
        self.terminated = False
        self.frame = None
        self.times = list(numpy.linspace(0, 1, 30))  # init list of frame arrival times to calculate fps
        self.source = source
        self.connect(self.source)

    def connect(self, source):
        self.cap = cv2.VideoCapture(source)
        self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        ret, self.frame = self.cap.read()  # read first frame to init variable
        print('# camera datastream initiated')

    def run(self):
        while not self.terminated:
            ret, frame = self.cap.read()  # read frame from stream (blocking)
            if frame is not None:
                self.frame = frame
            else:
                self.cap.release()
                print('# camera connection lost... retrying')
                self.connect(self.source)
            self.times.append(time.time())
            self.times = self.times[-30:]  # keep 30-buffer of frame times to calc avg
        self.cap.release()
        print('# camera connection successfully terminated')

    def get_frame(self):
        return self.frame

    def get_fps(self) -> float:
        # calculate fps
        return 1 / numpy.mean(numpy.diff(self.times))

    def terminate(self):
        self.terminated = True


class ArduinoStreamHandler(threading.Thread):
    def __init__(self, client):
        self.client = client
        self.file = client.makefile()  # file access for reads ensures full reads
        self.terminated = False

        super().__init__()

        self.out_data = '{"time": 0}'
        self.client.send(bytes(self.out_data, 'utf-8'))
        self.in_data = self.file.readline()  # readline to ensure full read of JSON

        print('# arduino datastream initiated')

    def run(self):
        while not self.terminated:
            self.client.send(bytes(self.out_data, 'utf8'))
            self.in_data = self.file.readline()

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
