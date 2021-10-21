import matplotlib.pyplot as plt
import numpy as np

from Daedalus.utils.navigation import sigmoid


def main():
    x = np.linspace(-50,50,1000)
    y = sigmoid(x,max=255,offset=50)

    plt.plot(x,y)
    plt.show()


if __name__ == "__main__":
    main()
