import cv2

from Daedalus.utils.Image import undistort
from Daedalus.utils.StreamHandlers import VideoStreamHandler


def main():
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

    while True:
        frame = video_stream.frame

        frame0 = undistort(frame, source = 0, balance=0.5)
        frame1 = undistort(frame,source = 1, balance=0.5)

        cv2.imshow('frame0', frame0)
        cv2.imshow('frame1', frame1)

        k = cv2.waitKey(20)
        if k == ord('q'):
            break

    video_stream.terminate()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
