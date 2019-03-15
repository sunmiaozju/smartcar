#!/usr/bin/python
# -*- coding:utf-8 -*-

import cv2
import subprocess
import signal

class RTMPClient:
    def __init__(self):
        self.rtmp_url = "rtmp://10.0.0.15:1935/live"

    def init_rmtp(self, sizeStr, fps):
        print("init rmtp")
        command = ['ffmpeg', '-re', '-y',
                   '-f', 'rawvideo',
                   '-pix_fmt', 'bgr24',
                   '-s', sizeStr,
                   '-r', str(fps),
                   '-i', '-',
                   '-pix_fmt', 'yuv420p',
                   '-preset', 'ultrafast',
                   '-f', 'flv', self.rtmp_url]
        self.pipe = subprocess.Popen(command, stdin=subprocess.PIPE, shell=False)

    def start(self):
        camera = cv2.VideoCapture(0)
        if camera.isOpened():
            print("Camera 0 is opened.")
        else:
            print("Cannot open Camera 0.")
            exit(1)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # camera.set(cv2.CAP_PROP_FPS, 1)
        size = (int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)), int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        size_str = str(size[0]) + "x" + str(size[1])
        fps = int(camera.get(cv2.CAP_PROP_FPS))
        print("size: " + size_str + " fps: " + str(fps))

        self.init_rmtp(size_str, fps)

        while True:
            ret, frame = camera.read()
            if not ret:
                print("camera read error!")
                break
            self.push_frame(frame)
        camera.release()

    def push_frame(self, frame):
        if not self.pipe:
            print("Pipe not opened!")
            exit(1)
        # TODO: Supress the frame before transport, and test the change of transport_fps

        frame_str = frame.tostring()
        print("sending size: " + str(len(frame_str)))
        self.pipe.stdin.write(frame_str)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close_push()

    def close_push(self):
        self.pipe.send_signal(signal.SIGINT)


if __name__ == '__main__':
    with RTMPClient() as client:
        client.start()
