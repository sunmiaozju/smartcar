#!/usr/bin/python
# -*- coding:utf-8 -*-

import time
from time import ctime
from twisted.internet import reactor
from twisted.internet.protocol import Protocol, Factory
from twisted.internet.protocol import DatagramProtocol
from twisted.protocols.basic import NetstringReceiver

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

PORT = 9998

# pub_image = rospy.Publisher("/cv_camear/image_raw",10)


class CamearHandler_Protocol(NetstringReceiver):
    def connectionMade(self):
        self.pub_image = rospy.Publisher(
            "/cv_camear/image_raw_from_server", Image, queue_size=10)
        peer = self.clnt = self.transport.getPeer().host
        print("...connected from : ", peer)

    def dataReceived(self, data):
        self._camera0_cb(data)

    def _camera0_cb(self, data):
        try:
            start = time.time()
            data_1 = np.fromstring(data, np.uint8)  # 将获取到的字符流数据转换成1维数组
            print "type of data_1 = ", type(data_1)
            print data_1.shape
            decimg = cv2.imdecode(data, cv2.IMREAD_COLOR)
            print type(decimg)
            print decimg.shape
            end = time.time()
            fps = 1.0 / (end - start)
            print fps
            # self.transport.write("FPS: {:.2f}".format(fps).encode('utf-8'))
        except:
            print "Error at _camear0_cb"
            reactor.stop()
            return

    def connectionLost(self, reason):
        print("connection lost")


if __name__ == "__main__":
    rospy.init_node("video_server")
    factory = Factory()
    factory.protocol = CamearHandler_Protocol
    print "waiting for connection..."
    reactor.listenTCP(PORT, factory)
    reactor.run()
