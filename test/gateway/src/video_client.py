#!/usr/bin/python
#-*- coding:utf-8 -*-

import sys
import cv2
import numpy as np
import socket
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

HOST = '127.0.0.1'
PORT = 9998

# class Client_TCP:
#     def __init__(self):
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         try:
#             self.sock.connect((HOST, PORT))
#         except:
#             print("Cannot connct server.")
#             exit(1)
#         print("Client is ready...")

#     def send_pic(self, frame):
#         # cv2.imencode将图片格式转换(编码)成流数据，赋值到内存缓存中;主要用于图像数据格式的压缩，方便网络传输
#         # cv2.imdecode()函数从指定的内存缓存中读取数据，并把数据转换(解码)成图像格式;主要用于从网络传输数据中恢复出图像。
#         result, imgencode = cv2.imencode('.jpg', frame)
#         data = np.array(imgencode)
#         stringData = data.tostring()
#         """首先发送图片的大小信息"""
#         self.sock.send(str(len(stringData)).ljust(16))
#         # 返回一个原字符串左对齐,并使用空格填充至指定长度的新字符串。
#         # 如果指定的长度小于原字符串的长度则返回原字符串

#         self.sock.sendall(stringData)
#         # 如果server端是Python的socket,那么可以一次性发送
#         # 但是如果server端是C++写的,那么需要一个字节一个字节的发送 --因为编码里有字符串结束标识位C++会进行截断处理
#         # e.g:
#         # for i in range(0, len(stringData)):
#         #   sock.send(stringData[i])

#         data_rec = self.sock.recv(50)
#         print(data_rec)

#     def run(self):
#         self.cap = cv2.VideoCapture(0)
#         if not self.cap.isOpened():
#             print("Cannot open camera, exit.")
#             exit(1)
#         while True:
#             ret, frame = self.cap.read()
#             if ret:
#                 print "send frame"
#                 self.send_pic(frame)
#             else:
#                 break
#             if cv2.waitKey(25) == ord('q'):
#                 break
#             # cv2.imshow("client", frame)
#         self.cap.release()
#         # cv2.destroyAllWindows()

#     def __enter__(self):
#         return self

#     def __exit__(self, exc_type, exc_val, exc_tb):
#         if self.cap.isOpened():
#             self.cap.release()
#         cv2.destroyAllWindows()
#         self.sock.sendall("exit".ljust(16))
#         self.sock.close()
#         print("Current connection closed")


class Client_TCP(object):
    def __init__(self):
        rospy.init_node("monitor_camera_node")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((HOST, PORT))
        except Exception as e:
            print e
            exit(1)
        self.bridge = CvBridge()
        print("Camear client is ready")

    def _add_sub(self):
        rospy.Subscriber("/cv_camera/image_raw", Image, self._image0_cb)

    def _image0_cb(self, msg):
        assert isinstance(msg, Image)
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        imgencode = cv2.imencode('.jpg', cv_img)[1]
        data = np.array(imgencode)
        stringData = data.tostring()
        self.sock.send(str(len(stringData)).ljust(16))
        self.sock.send(stringData)

        data_rec = self.sock.recv(50)
        print(data_rec)

    def _image_other_cb(self, msg):
        pass

    def run(self):
        self._add_sub()
        rospy.spin()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # TODO: 判断当前socket状态,以确认是否需要关闭操作
        try:
            self.sock.sendall('exit')
            self.sock.close()
            print("close socket")
        except Exception as e:
            print e
        return self


class DefaultCamear_UDP(object):
    def __init__(self, sock, target):
        self.sock = sock
        self.target = target
        self.bridge = CvBridge()
        print("UDP socket is ready to get camear default...")

    def handle(self, msg):
        assert isinstance(msg, Image)

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        imgencode = cv2.imencode('.jpg', img)[1]
        data = np.array(imgencode)
        stringData = data.tostring()
        self.sock.sendall(stringData)
        data_rec = self.sock.recv(50)
        print(data_rec)


class Client_UDP:
    def __init__(self):
        rospy.init_node("monitor_camera_node")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_server = self._init_socket()
        self._add_monitor()

    def _add_monitor(self):
        self.camear_monitor = DefaultCamear_UDP(self.sock, self.udp_server)

    def _init_socket(self):
        server_ip = rospy.get_param("server_ip", '127.0.0.1')
        server_port = rospy.get_param("server_port", 9998)
        return server_ip, server_port

    def _add_sub(self):
        rospy.Subscriber("/cv_camera/image_raw", Image, self._camera_cb)

    def _camera_cb(self, msg):
        self.camear_monitor.handle(msg)

    def run(self):
        self._add_sub()
        rospy.spin()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return self


if __name__ == '__main__':
    with Client_TCP() as client:
        client.run()
