import time
import cv2

from Daedalus.Image import undistort, square
from Daedalus.StreamHandlers import VideoStreamHandler


def draw_circle(event,x,y,flags,param):
    global mouseX,mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(frame,(x,y),100,(255,0,0),-1)
        mouseX,mouseY = x,y

def main():
    video_stream = VideoStreamHandler("http://localhost:8081/stream/video.mjpeg")
    video_stream.start()

    cv2.namedWindow('frame')
    cv2.setMouseCallback('frame', draw_circle)

    while True:
        frame = video_stream.frame
        frame = undistort(frame, balance=0.5)

        cv2.imshow('frame', frame)

        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break
        elif k == ord('a'):
            print(mouseX, mouseY)




if __name__ == "__main__":
    frame = None
    main()
