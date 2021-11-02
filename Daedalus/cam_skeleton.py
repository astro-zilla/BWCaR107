"""Template for files using PyOpenCV image manipulation."""

import cv2
from daedalus.streaming import VideoStreamHandler


def main():
    # init stream handler
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    # start stream handler
    video_stream.start()

    while True:
        # grab frame
        frame = video_stream.frame
        # show frame
        cv2.imshow('frame', frame)

        # wait 20ms for keypress
        k = cv2.waitKey(20)
        if k == ord('q'):
            # exit if keypress was a 'q'
            break

    # gracefully terminate streamhandler
    video_stream.terminate()
    # close cv2 window
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
