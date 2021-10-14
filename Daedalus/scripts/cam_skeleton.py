import cv2

from Daedalus.utils.aruco import visualise
from Daedalus.utils.streaming import VideoStreamHandler


def main():
    video_stream = VideoStreamHandler(0)  # ("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()
    data = {}

    while True:
        frame = video_stream.frame

        # undistort(frame, balance=0.5)
        # square(frame)

        frame = visualise(frame,data)

        for id in data:
            print(f'{id}:{data[id]}')

        cv2.imshow('frame', frame)

        k = cv2.waitKey(20)
        if k == ord('q'):
            break

    video_stream.terminate()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
