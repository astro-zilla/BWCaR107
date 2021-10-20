
class Smoother:
    def __init__(self, windowlen):
        self.windowlen = windowlen
        self.data = []
        self.sum = 0

    def process(self, newvalue):
        self.data.append(newvalue)
        self.sum += newvalue
        if len(self.data) > self.windowlen:
            self.sum -= self.data[0]
            self.data.pop(0)
        return float(self.sum) / len(self.data)