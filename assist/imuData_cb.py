# coding: utf-8
# !/usr/bin/env python

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import csv
import math

# --- 解析bag源数据,并整理成csv文件 ---
def convert_bag2csv():
    with open('imu.csv', 'w') as csvfile:
        writer = csv.writer(csvfile)
        bag = rosbag.Bag('imu.bag')
        # for topic, msg, t in bag.read_messages(topics=["Feedback"]):
        #     # 0:sec,1:nsec,2:cur_speed,3:cur_steer,4:cur_direction(L0/R1-direction),5:shift_level,6:ignition_status
        #     writer.writerow(
        #         [float(msg.header.stamp.secs), float(msg.header.stamp.nsecs), float(msg.cur_speed), float(msg.cur_steer),
        #          int(msg.cur_direction), int(msg.shift_level), float(msg.ignition_status)])
        print("start convert...")
        for topic,msg,t in bag.read_messages(topics=["Imu"]):
            # 0:secs,1:nsecs,
            # 2:ori.x,3:ori.y,4:ori.z,5:ori.w,6,ori.covariance,
            # 7:ang.x,8:ang.y,9:ang.z,10:ang.covariance,
            # 11:acc.x,12:acc.y,13:acc.z,14:acc.covariance

            # writer.writerow([float(msg.header.stamp.secs),float(msg.header.stamp.nsecs),
            #                  float(msg.orientation.x),float(msg.orientation.y),float(msg.orientation.z),list(msg.orientation_covariance),
            #                  float(msg.angular_velocity.x),float(msg.angular_velocity.y),float(msg.angular_velocity.z),list(msg.angular_velocity_covariance),
            #                  float(msg.linear_acceleration.x),float(msg.linear_acceleration.y),float(msg.linear_acceleration.z),list(msg.linear_acceleration_covariance)])

            # collect without covariance
            writer.writerow([float(msg.header.stamp.secs), float(msg.header.stamp.nsecs),
                             float(msg.orientation.x), float(msg.orientation.y), float(msg.orientation.z),
                             float(0),
                             float(msg.angular_velocity.x), float(msg.angular_velocity.y), float(msg.angular_velocity.z),
                             float(0),
                             float(msg.linear_acceleration.x), float(msg.linear_acceleration.y),
                             float(msg.linear_acceleration.z), float(0)])

        print("Done")
        bag.close()




def imudata_cb():
    wheelBase = 1.9   # 轴距
    ns = 1000000000.0
    x = 0.0
    y = 0.0
    yaw = 0.0
    dataSet = []
    pos_x = [0.]
    pos_y = [0.]
    vx_pred = [0.]
    vy_pred = [0.]
    ang_pred = [0]

    time_piece = [0]

    fr = open('imu.csv')
    for line in fr.readlines():
        dataSet.append(map(float, line.strip().split(',')))
    length = len(dataSet)
    print("length of dataSet = ",length)
    fr.close()

    for i in range(length - 1):
        seconds = (dataSet[i+1][0]+dataSet[i+1][1]/ns) - (dataSet[i][0]+dataSet[i][1]/ns)
        time_piece.append(seconds)
        vx_pred.append(vx_pred[-1] + dataSet[i][11]*seconds)
        vy_pred.append(vy_pred[-1] + dataSet[i][12]*seconds)
        print(dataSet[i][11]*seconds)

        pos_x.append(pos_x[-1] + vx_pred[-1]*seconds)
        pos_y.append(pos_y[-1] + vy_pred[-1]*seconds)

        # if dataSet[i][4] == 0:
        #     angle = dataSet[i][3]
        # else:
        #     angle = -dataSet[i][3]
        # ang_pred =ang_pred[i] + dataSet[] * math.pi / 180.0
        # x = x + v * seconds * math.cos(yaw)
        # y = y + v * seconds * math.sin(yaw)
        # yaw = yaw + v * seconds * math.tan(steer) / wheelBase
        # pos_x.append(x)
        # pos_y.append(y)
    print("Total time ={} s".format(dataSet[-1][0]-dataSet[0][0]))
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    # ax.plot(pos_x, pos_y)
    ax.grid()
    plt.plot(vx_pred,label="vx_predict")
    plt.legend()
    plt.show()

import time
if __name__ == "__main__":
    convert_bag2csv()
    print("Now pos_xy")
    # time.sleep(3)
    imudata_cb()


# %0:time,1:field.header.seq,2:field.header.stamp,||3:field.header.frame_id,4:field.cur_steer,
# 5:field.cur_speed,6:field.cur_direction,7:field.shift_level,8:field.ignition_status
# 0:time, 1:speed 2:steer 3:direction
# wheelBase = 1.9
# ns = 1e9
# x = 0.0
# y = 0.0
# yaw = 0.0
# pos_x = [0]
# pos_y = [0]
# dataSet = []
# with open('Feedback.txt', 'r') as f:
#     for line in f.readlines():
#         data = line.strip().split(',')
#         data.remove('')
#         dataSet.append(map(float, data))
#     length = len(dataSet)
#     time = []
#     for i in range(length - 1):
#         seconds = (dataSet[i + 1][0] - dataSet[i][0]) / ns
#         time.append(seconds)
#         v = dataSet[i][4]
#         if dataSet[i][5] == 0:
#             angle = dataSet[i][3]
#         else:
#             angle = -dataSet[i][3]
#         steer = angle * math.pi / 180.0
#         x = x + v * seconds * math.cos(yaw)
#         y = y + v * seconds * math.sin(yaw)
#         yaw = yaw + v * seconds * math.tan(steer) / wheelBase
#         pos_x.append(x)
#         pos_y.append(y)
#
# time = sorted(time)
# print(time[-1])
# print(time[-1] - time[0])
#
# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.plot(pos_x, pos_y)
# ax.grid()
# plt.show()
