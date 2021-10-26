import time
from random import random

from daedalus.smoother import Smoother

def main():

    window_length = 10  #length of rolling average window
    threshold = 100  # average at which metal is present

    smoother = Smoother(window_length)

    t0 = time.time()
    while True:
        # import new data from A0 pin arduino as new_data
        data = random.randint(0,255)
        smoother.process(data)
        rolling_average = smoother.process(data)
        if 15>time.time()-t0>10:
            pass
            # move to block
        elif time.time()-t0>15:
            if rolling_average < threshold:
                print("metal")
                #set arduino red LED pin to high (D9)
            else:
                print("not metal")
                #set arduino green LED pin to high (D10)

if __name__ == "__main__":
    main()
