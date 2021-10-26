import matplotlib.pyplot as plt
import numpy as np


def main():
    x = np.linspace(0,500,1000)
    y = np.clip(x / 100, 0.1, 1)

    plt.plot(x,y)
    plt.show()

# import curses
#
# def main(stdscr=curses.initscr()):
#
#     curses.start_color()
#     curses.use_default_colors()
#     for i in range(0, 255):
#         curses.init_pair(i+1, i, -1)
#     stdscr.addstr(0, 0, '{0} colors available'.format(curses.COLORS))
#     maxy, maxx = stdscr.getmaxyx()
#     maxx = maxx - maxx % 5
#     x = 0
#     y = 1
#     try:
#         for i in range(0, curses.COLORS):
#             stdscr.addstr(y, x, '{0:5}'.format(i), curses.color_pair(i))
#             x = (x + 5) % maxx
#
#             if x == 0:
#                 y += 1
#     except Exception as e:
#         pass
#     stdscr.getch()




if __name__ == "__main__":
    main()
