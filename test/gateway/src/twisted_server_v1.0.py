#!/usr/bin/python
# -*- coding:utf-8 -*-

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

from twisted.internet import reactor
from twisted.internet.protocol import Protocol, Factory, ClientFactory
from twisted.internet.protocol import DatagramProtocol
from twisted.protocols.basic import NetstringReceiver

HOST = 'localhost'
PORT = 9998


class MyServerProtocol(Protocol):
    def connectionMade(self):
        self._init_publisher()
        # self.pub_car_pose = rospy.Publisher("/car0/current_pose", PoseStamped, queue_size=10)
        peer = self.clnt = self.transport.getPeer().host
        print("...connected from : ", peer)

    # TODO: 不同的车发布不同的消息,根据car*进行区分
    def _init_publisher(self):
        self.pub_car_pose = []
        for i in range(10):
            topic = "/car" + str(i) + "/current_pose"
            pub = rospy.Publisher(topic, PoseStamped,queue_size=10)
            self.pub_car_pose.append(pub)

    def dataReceived(self, data):
        self._data_cb(data)

    def _data_cb(self, data):
        protoc_current_pose = protoc_msg_pb2.CurrentPose()
        protoc_current_pose.ParseFromString(data)
        self._car_pose_cb(protoc_current_pose)

    def _car_pose_cb(self, protoc_current_pose):
        msg_current_pose = PoseStamped()
        utils_pb2ros.PoseStamped2Msg(protoc_current_pose.msg, msg_current_pose)

        vehicle_info = protoc_current_pose.info
        car_vin = int(vehicle_info.vin[1:])
        car_type = vehicle_info.vin[0]
        car_station = vehicle_info.state

        topic = "/car" + str(car_vin) + "/current_pose"
        self.pub_car_pose[car_vin].publish(msg_current_pose)

    def connectionLost(self, reason):
        print("connection lost")


if __name__ == "__main__":
    rospy.init_node("gataway_server")
    factory = Factory()
    factory.protocol = MyServerProtocol
    print("waiting for connection...")
    reactor.listenTCP(PORT, factory)
    reactor.run()

