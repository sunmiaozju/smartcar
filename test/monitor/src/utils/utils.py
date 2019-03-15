# !/usr/bin/python
# -*- coding:utf-8 -*-

# import protoc_msg_pb2
from proto import protoc_msg_pb2
import geometry_msgs.msg
import std_msgs.msg

def make_header(protoc_header, ros_header):
    """
    :param ros_header: std_msgs/Header
    :return: protoc_msg/Header
    """
    assert isinstance(ros_header, std_msgs.msg.Header)
    assert isinstance(protoc_header, protoc_msg_pb2.Header)
    protoc_header.seq = ros_header.seq
    protoc_header.stamp.secs = ros_header.stamp.secs
    protoc_header.stamp.nsecs = ros_header.stamp.nsecs
    protoc_header.frame_id = ros_header.frame_id
    # return protoc_header

def make_PoseStamped(proto_PoseStamped, pose_stamped):
    """

    :param pose_stamped: geometry_msgs/PoseStamped
    :return: protoc_msg/PoseStamped
    """
    assert isinstance(pose_stamped, geometry_msgs.msg.PoseStamped)
    assert isinstance(proto_PoseStamped, protoc_msg_pb2.PoseStamped)
    make_header(proto_PoseStamped.header, pose_stamped.header)
    make_pose(proto_PoseStamped.pose, pose_stamped.pose)
    # return proto_PoseStamped

def make_pose(protoc_pose, pose):
    """

    :param pose: geometry_msgs/Pose
    :return: protoc_msg/Pose
    """
    assert isinstance(pose,geometry_msgs.msg.Pose)
    assert isinstance(protoc_pose, protoc_msg_pb2.Pose)
    make_point(protoc_pose.position, pose.position)
    make_quaternion(protoc_pose.orientation, pose.orientation)
    # return protoc_pose

def make_point(protoc_point, point):
    """

    :param point: geometry_msgs/Point
    :return: protoc_msg/Point
    """
    assert isinstance(point, geometry_msgs.msg.Point)
    assert isinstance(protoc_point, protoc_msg_pb2.Point)
    protoc_point.x = point.x
    protoc_point.y = point.y
    protoc_point.z = point.z
    # return protoc_point

def make_quaternion(protoc_orientation, orientation):
    """

    :param orientation: geometry_msgs/Quaternion
    :return: protoc_msg/Quaternion
    """
    assert isinstance(orientation, geometry_msgs.msg.Quaternion)
    assert isinstance(protoc_orientation, protoc_msg_pb2.Quaternion)
    protoc_orientation.x = orientation.x
    protoc_orientation.y = orientation.y
    protoc_orientation.z = orientation.z
    protoc_orientation.w = orientation.w
    # return protoc_orientation