#!/usr/bin/python2
# -*- coding:utf-8 -*-

import rosbag
import csv

# --- 解析bag文件,并转换成csv文件
def convert_bag2csv():
    with open("result.csv", 'w') as csvfile:
        writer = csv.writer(csvfile)
        bag = rosbag.Bag("my_test.bag")
        print("start convert ...")
        for topic, msg, t in bag.read_messages(topics=["/NDT_attr"]):
            writer.writerow([msg.UsedTime])  # writer写入行时,传入的是一个[list]
        print("convert finished.")
        bag.close()


if __name__ == '__main__':
    convert_bag2csv()
