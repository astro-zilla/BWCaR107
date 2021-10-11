import cv2
from StreamHandlers import VideoStreamHandler


def main():
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

    while True:
        frame = video_stream.get_frame()
        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    video_stream.terminate()


if __name__ == "__main__":
    main()
