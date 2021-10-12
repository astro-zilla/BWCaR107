import cv2

from Daedalus.utils.StreamHandlers import VideoStreamHandler


def main():
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

    while True:
        frame = video_stream.frame
        cv2.imshow('frame', frame)

        k = cv2.waitKey(20)
        if k == ord('q'):
            break

    video_stream.terminate()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
