#!/usr/bin/python
#-*- coding:utf-8 -*-

import sys
import os
proto_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, proto_path + '/proto')

try:
    from proto import protoc_msg_pb2
except:
    import protoc_msg_pb2
import rospy
from geometry_msgs.msg import PoseStamped
from utils import utils, utils_pb2ros

from twisted.internet.protocol import Protocol, Factory, ClientFactory
from twisted.internet.protocol import DatagramProtocol
from twisted.protocols.basic import NetstringReceiver

# HOST = '10.0.0.15'
HOST = "127.0.0.1"
PORT = 9998


class MyClientProtocol(Protocol):
    def connectionMade(self):
        self._add_sub()
        print("client connected to %s:%s" % (HOST, PORT))

    def _add_sub(self):
        rospy.Subscriber("/ndt/current_pose", PoseStamped,
                         self._current_pose_cb)

    def _current_pose_cb(self, msg):
        CurrentPose = protoc_msg_pb2.CurrentPose()
        CurrentPose.msg_type = protoc_msg_pb2.current_pose
        utils.make_PoseStamped(CurrentPose.msg, msg)

        utils.make_vehicle_info(CurrentPose.info, "10000000", True)

        binary_msg = CurrentPose.SerializeToString()
        self.transport.write(binary_msg)

    def dataReceived(self, data):
        print data

    def connectionLost(self, reason):
        print reason
        # reactor.stop()


class MyFactory(ClientFactory):
    protocol = MyClientProtocol
    clientConnectionLost = clientConnectionFailed = \
        lambda self, connector, reason: reactor.stop()


if __name__ == "__main__":
    rospy.init_node("gateway_client")
    from twisted.internet import reactor
    reactor.connectTCP(HOST, PORT, MyFactory())
    reactor.run()