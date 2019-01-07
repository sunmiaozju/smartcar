#!/usr/bin/python
# -*- coding:utf-8 -*-

import csv
import numpy as np
import matplotlib.pyplot as plt

dataSet1 = []
dataSet2 = []

##########  // 提取csv
fr1 = open("no_odom.csv",'r')
for line in fr1.readlines():
    dataSet1.append(map(float,line.strip().split(',')))
fr1.close()

fr2 = open("with_odom.csv",'r')
for line in fr2.readlines():
    dataSet2.append(map(float,line.strip().split(',')))
fr2.close()

print("file1: {} points".format(len(dataSet1)))
print("file2: {} points".format(len(dataSet2)))
########## // 处理csv
dataSet1 = np.array(dataSet1)

x_datas1 = dataSet1[:,0]
y_datas1 = dataSet1[:,1]
z_datas1 = dataSet1[:,2]

dataSet2 = np.array(dataSet2)
x_datas2 = dataSet2[:,0]
y_datas2 = dataSet2[:,1]
z_datas2 = dataSet2[:,2]

##########  // 二维图
plt.plot(x_datas1,y_datas1,label="no_odom")
plt.plot(x_datas2,y_datas2,label="with_odom")
plt.legend()
plt.show()

##########  // 三维轨迹图
from mpl_toolkits.mplot3d import axes3d

fig = plt.figure()
ax = fig.gca(projection="3d")

ax.set_title("3D tractory")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

figure = ax.plot(x_datas1,y_datas1,z_datas1,c="r")
print figure
plt.show()

























