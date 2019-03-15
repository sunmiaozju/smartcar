# !/usr/bin/python
# -*- coding:utf-8 -*-

# import protoc_msg_pb2
from proto import protoc_msg_pb2
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header


def PoseStamped2Msg(proto_ps, msg):
    assert isinstance(proto_ps, protoc_msg_pb2.PoseStamped)
    assert isinstance(msg, PoseStamped)

    Header2Msg(proto_ps.header, msg.header)
    Pose2Msg(proto_ps.pose, msg.pose)


def Header2Msg(proto_header, header):
    assert isinstance(proto_header, protoc_msg_pb2.Header)
    assert isinstance(header, Header)

    header.seq = proto_header.seq
    header.stamp.secs = proto_header.stamp.secs
    header.stamp.nsecs = proto_header.stamp.nsecs
    header.frame_id = proto_header.frame_id


def Pose2Msg(proto_pose, pose):
    assert isinstance(proto_pose, protoc_msg_pb2.Pose)
    assert isinstance(pose, Pose)

    Point2Msg(proto_pose.position, pose.position)
    Orientation2Msg(proto_pose.orientation, pose.orientation)


def Point2Msg(proto_point, position):
    assert isinstance(proto_point, protoc_msg_pb2.Point)
    assert isinstance(position, Point)
    position.x = proto_point.x
    position.y = proto_point.y
    position.z = proto_point.z


def Orientation2Msg(proto_quaternion, quaternion):
    assert isinstance(proto_quaternion, protoc_msg_pb2.Quaternion)
    assert isinstance(quaternion, Quaternion)

    quaternion.x = proto_quaternion.x
    quaternion.y = proto_quaternion.y
    quaternion.z = proto_quaternion.z
    quaternion.w = proto_quaternion.w