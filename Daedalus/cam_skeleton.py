import cv2

from Daedalus.StreamHandlers import VideoStreamHandler


def main():
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

    while True:
        frame = video_stream.frame
        cv2.imshow('frame', frame)


if __name__ == "__main__":
    main()
