# !/usr/bin/python
# -*- coding:utf-8 -*-

import sys
sys.path.append("../protoc/")
import socket
import protoc_msg_pb2
import threading
from src.utils import utils_pb2ros
from geometry_msgs.msg import PoseStamped


class TCPServer:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('127.0.0.1', 9999))
        self.sock.listen(5)

    def current_pose_cb(self, bytes_msg):
        assert isinstance(bytes_msg, basestring)
        protoc_current_pose = protoc_msg_pb2.CurrentPose()
        protoc_current_pose.ParseFromString(bytes_msg)
        print("frame_id : {}".format(protoc_current_pose.pose.header.frame_id))

    def switch_msgs(self, sock, addr):
        assert isinstance(sock, socket.socket)
        assert isinstance(addr, tuple)

        print("Received msg from %s:%s" % addr)
        buffer = []
        while True:
            data = sock.recv(1024)
            if not data:
                break
            buffer.append(data)
        bytes_msg = b''.join(buffer)
        msg_type = protoc_msg_pb2.BaseMsg()
        msg_type.ParseFromString(bytes_msg)
        if msg_type == protoc_msg_pb2.current_pose:
            print "received current pose"
            t = threading.Thread(target=self.current_pose_cb, args=(bytes_msg))
            t.start()
        else:
            print "error"

    def run(self):
        print("TCP_Server is ready...")
        while True:
            sock, addr = self.sock.accept()
            t = threading.Thread(target=self.switch_msgs, args=(sock, addr))
            t.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.sock.close()
        return self


class UDPServer:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 9999))

    def current_pose_cb(self, bytes_msg):
        print("Handle current pose")
        protoc_current_pose = protoc_msg_pb2.CurrentPose()
        protoc_current_pose.ParseFromString(bytes_msg)
        print("frame_id : {}".format(protoc_current_pose.pose.header.frame_id))

    def run(self):
        print("UDP_Server is ready...")
        while True:
            bytes_msg, addr = self.sock.recvfrom(1024)
            base_msg = protoc_msg_pb2.BaseMsg()
            base_msg.ParseFromString(bytes_msg)
            if base_msg.msg_type == protoc_msg_pb2.current_pose:
                print "received current pose"
                # self.current_pose_cb(bytes_msg)
                proto_cur = protoc_msg_pb2.CurrentPose()
                proto_cur.ParseFromString(bytes_msg)
                print proto_cur.msg.header.stamp.nsecs

                current_pose_msg = PoseStamped()
                utils_pb2ros.PoseStamped2Msg(proto_cur.msg, current_pose_msg)
                print "---------------"
                print "msg.header: "
                print current_pose_msg.header.seq
                print current_pose_msg.header.stamp.secs, current_pose_msg.header.stamp.nsecs
                print current_pose_msg.header.frame_id

                print "Pose: "
                p = current_pose_msg.pose.position
                o = current_pose_msg.pose.orientation
                print p.x, p.y, p.z
                print o.x, o.y, o.z, o.w
                print "---------------"
            else:
                print("Not current pose, please check")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.sock.close()
        return self


if __name__ == '__main__':
    with UDPServer() as server:
        server.run()
