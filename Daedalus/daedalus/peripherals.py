import time


class Buffer:
    def __init__(self, windowlen):
        self.windowlen = windowlen
        self.data = []
        self.sum = 0

    def push(self, newvalue):
        self.data.append(newvalue)
        self.sum += newvalue
        if len(self.data) > self.windowlen:
            self.sum -= self.data[0]
            self.data.pop(0)

    def flush(self):
        self.data = []
        self.sum = 0

    def is_full(self):
        if len(self.data) == self.windowlen:
            return True
        else:
            return False

    def get_buffer(self):
        return self.data

    def get_mean(self):
        if self.sum == 0:
            return 0
        else:
            return self.sum / len(self.data)


def flash(freq):
    t = (time.time() * freq) % 1
    return round(t)
