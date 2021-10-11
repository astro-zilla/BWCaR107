import cv2
from StreamHandlers import VideoStreamHandler


def main():
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

    while True:
        frame = video_stream.get_frame()
        overlay = frame.copy()
        output = frame.copy()
        # draw info bar and fps
        cv2.rectangle(overlay, (0, 0), (150, 60), (50, 50, 50), -1)
        cv2.putText(img=overlay, text=f'FPS: {video_stream.get_fps():.1f}', org=(20, 30), fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                    fontScale=0.5, color=(255, 255, 255), thickness=1)
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, output)
        cv2.imshow("frame", output)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    video_stream.terminate()


if __name__ == "__main__":
    main()
