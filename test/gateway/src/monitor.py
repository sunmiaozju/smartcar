#!/usr/bin/python
#-*- coding:utf-8 -*-

import os  # python引入别处目录中的文件
import sys
proto_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, proto_path + '/proto')

from proto import protoc_msg_pb2
# import protoc_msg_pb2
import rospy
from geometry_msgs.msg import PoseStamped
import socket
from src.utils import utils


# class CurrentPoseMonitor_TCP(object):
#     def __init__(self, sock):
#         assert isinstance(sock, socket.socket)
#         self.sock = sock
#
#     def handle(self, msg):
#         assert isinstance(msg, PoseStamped)
#         CurrentPose = protoc_msg_pb2.CurrentPose()
#         CurrentPose.msg_type = protoc_msg_pb2.current_pose
#         utils.make_PoseStamped(CurrentPose, msg)
#         binary_msg = CurrentPose.SerializeToString()
#         print("send pose start")
#         self.sock.sendall(binary_msg)
#         print("send pose end")
#
#
# class Monitor_TCP(object):
#     def __init__(self):
#         rospy.init_node("monitor_node_TCP", log_level=rospy.INFO)
#         self.sock = None
#         self._init_socket()
#
#         self.current_pose_monitor = CurrentPoseMonitor_TCP(self.sock)
#
#     def _init_socket(self):
#         server_ip = rospy.get_param("server_ip", '127.0.0.1')
#         server_port = rospy.get_param("server_port", 9999)
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         try:
#             self.sock.connect((server_ip, server_port))
#         except Exception as e:
#             rospy.logerr("Cannot establish socket, exit.")
#             print(e)
#             self.sock.close()
#             exit(1)
#         print("Socket connected, waitting for transmitting...")
#         # sock.sendall("Hello, server.")
#         # return sock
#
#     def _add_sub(self):
#         rospy.Subscriber("/ndt/current_pose", PoseStamped, self._current_pose_cb)
#
#     def _current_pose_cb(self, msg):
#         self.current_pose_monitor.handle(msg)
#
#     def run(self):
#         self._add_sub()
#         rospy.spin()
#
#     def __enter__(self):
#         return self
#
#     def __exit__(self, exc_type, exc_val, exc_tb):
#         # TODO: 判断当前socket状态,以确认是否需要关闭操作
#         return self


class CurrentPoseMonitor_UDP(object):
    def __init__(self, sock, target):
        self.sock = sock
        self.target = target
        print("UDP socket is ready monitor current pose...")

    def handle(self, msg):
        assert isinstance(msg, PoseStamped)

        CurrentPose = protoc_msg_pb2.CurrentPose()
        CurrentPose.msg_type = protoc_msg_pb2.current_pose
        utils.make_PoseStamped(CurrentPose.msg, msg)
        binary_msg = CurrentPose.SerializeToString()
        self.sock.sendto(binary_msg, self.target)


class Monitor_UDP(object):
    def __init__(self):
        rospy.init_node("monitor_node_UDP", log_level=rospy.INFO)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_server = self._init_socket()

        self.current_pose_monitor = CurrentPoseMonitor_UDP(
            self.sock, self.udp_server)

    def _init_socket(self):
        server_ip = rospy.get_param("server_ip", '127.0.0.1')
        server_port = rospy.get_param("server_port", 9999)
        return server_ip, server_port

    def _add_sub(self):
        rospy.Subscriber("/ndt/current_pose", PoseStamped,
                         self._current_pose_cb)

    def _current_pose_cb(self, msg):
        self.current_pose_monitor.handle(msg)

    def run(self):
        self._add_sub()
        rospy.spin()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # TODO: 判断当前socket状态,以确认是否需要关闭操作
        try:
            self.sock.close()
        except Exception as e:
            print e
        return self


if __name__ == "__main__":
    with Monitor_UDP() as monitor:
        monitor.run()
