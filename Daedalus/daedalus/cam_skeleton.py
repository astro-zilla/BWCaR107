"""Template for files using PyOpenCV image manipulation."""
from cv2 import destroyAllWindows, imshow, waitKey

from .streaming import VideoStreamHandler


def main():
    # init stream handler
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    # start stream handler
    video_stream.start()

    while True:
        # grab frame
        frame = video_stream.frame
        # show frame
        imshow('frame', frame)

        # wait 20ms for keypress
        k = waitKey(20)
        if k == ord('q'):
            # exit if keypress was a 'q'
            break

    # gracefully terminate streamhandler
    video_stream.terminate()
    # close cv2 window
    destroyAllWindows()


if __name__ == "__main__":
    main()
