#!/usr/bin/python
# -*- coding:utf-8 -*-

from time import ctime
from twisted.internet import reactor
from twisted.internet.protocol import Protocol, Factory, ClientFactory
from twisted.internet.protocol import DatagramProtocol
from twisted.protocols.basic import NetstringReceiver

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

HOST = 'localhost'
PORT = 9998


class TSClntProtocol(NetstringReceiver):
    # def sendData(self):
    #     data = raw_input('> ')
    #     if data:
    #         print '...sending %s...' % data
    #         self.transport.write(data)
    #     else:
    #         self.transport.loseConnection()

    def connectionMade(self):
        self.bridge = CvBridge()
        self._add_sub()
        print("client connected to %s:%s" % (HOST, PORT))

    def _add_sub(self):
        rospy.Subscriber("/cv_camera/image_raw", Image, self._image0_cb)

    def _image0_cb(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        imgencode = cv2.imencode('.jpg', cv_img)[1]
        data = np.array(imgencode)
        print data.shape
        stringData = data.tostring()
        self.transport.write(stringData)

    def dataReceived(self, data):
        print data

    def connectionLost(self, reason):
        print reason
        cv2.destroyAllWindows()


class TSClntFactory(ClientFactory):
    protocol = TSClntProtocol
    clientConnectionLost = clientConnectionFailed = \
        lambda self, connector, reason: reactor.stop()


if __name__ == "__main__":
    rospy.init_node("camera_gateway")
    reactor.connectTCP(HOST, PORT, TSClntFactory())
    reactor.run()
