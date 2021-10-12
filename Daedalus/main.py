import cv2
import json
import socket
import time

from StreamHandlers import ArduinoStreamHandler, VideoStreamHandler
from Image import undistort,square


def nothing(_): pass


def main():
    # broadcast locally on 53282
    host = socket.gethostname()
    port = 53282

    # listen on streaming socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen()
    print(f'# listening for connection from Arduino on {host}:{port}')

    # init asynchronous "threading" stream handlers
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    arduino_stream = ArduinoStreamHandler(server,json.dumps(data))

    # start streams
    video_stream.start()
    print(f'# video stream started')
    arduino_stream.start()
    print(f'# arduino stream started')

    start_time = int(time.time_ns() / 1000000)

    cv2.namedWindow('frame')
    cv2.createTrackbar('motor 1', 'frame', 0, 255, nothing)
    cv2.createTrackbar('motor 2', 'frame', 0, 255, nothing)

    while True:
        # input run-time to data array
        time_since_start = int(time.time_ns() / 1000000) - start_time
        data["time"] = time_since_start
        data["motors"][0] = cv2.getTrackbarPos('motor 1', 'frame')
        data["motors"][1] = cv2.getTrackbarPos('motor 2', 'frame')

        # get/send arduino data from
        arduino_stream.set_data(json.dumps(data))
        arduinodata = arduino_stream.get_data()

        # get video data from stream
        frame = video_stream.frame
        frame = undistort(frame, balance=0.5)
        frame = square(frame)
        overlay = frame.copy()
        output = frame.copy()

        # draw info bar and fps
        cv2.rectangle(overlay, (0, 0), (150, 60), (50, 50, 50), -1)
        cv2.putText(img=overlay, text=f'FPS: {video_stream.get_fps():.1f}', org=(20, 30), fontFace=cv2.FONT_HERSHEY_TRIPLEX,
                    fontScale=0.5, color=(255, 255, 255), thickness=1)
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, output)

        # output to frame
        cv2.imshow('frame', output)

        # press q key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # graceful exit
    cv2.destroyAllWindows()
    video_stream.terminate()
    arduino_stream.terminate()


if __name__ == "__main__":
    data = {
        "time": 0,
        "motors": [100, 100, 0, 0],
        "servos": [0, 0],
        "LEDs": [0, 0, 0, 0]
    }
    main()
