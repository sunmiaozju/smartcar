#!/usr/bin/python
# -*- coding:utf-8 -*-

import sys
import cv2
import numpy as np
import socket
import time
import SimpleHTTPServer
import SocketServer

LISTEN_HOST = "127.0.0.1"
LISTEN_PORT = 9998


class Server:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((LISTEN_HOST, LISTEN_PORT))
        self.sock.listen(True)
        print("Server is ready...")

    def recv_size(self, sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf:
                return None
            buf += newbuf
            count -= len(newbuf)
        return buf  # len(buf必等于16,因为设置的是ljust(16))

    def recv_all(self, sock, count):
        buf = b''
        while count:
            # For python
            newbuf = sock.recv(count)
            if not newbuf:
                return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def video_cb(self, sock, addr):
        print("Reveived connection from %s:%s" % addr)
        while True:
            start = time.time()
            length = self.recv_size(sock, 16)
            if length is not None and length != 'exit':
                try:
                    stringData = self.recv_all(sock, int(length))
                    # data = np.fromstring(stringData, dtype='uint8')
                    data = np.frombuffer(stringData,
                                         np.uint8)  # 将获取到的字符流数据转换成1维数组
                    decimg = cv2.imdecode(data, cv2.IMREAD_COLOR)
                    cv2.imshow('Server', decimg)
                    if cv2.waitKey(25) == ord('q'):
                        break
                    end = time.time()
                    fps = 1.0 / (end - start)
                    print("received length: {:<5} fps: {:.2f}".format(
                        int(length), fps))
                    sock.send("FPS: {:.2f}".format(fps).encode('utf-8'))
                except:
                    break
        print("Current connection closed (from %s:%s)" % addr)
        sock.close()
        cv2.destroyAllWindows()

    def run(self):
        while True:
            sock, addr = self.sock.accept()
            self.video_cb(sock, addr)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.sock.close()
        cv2.destroyAllWindows()
        return self


if __name__ == '__main__':
    with Server() as server:
        server.run()