#!/usr/bin/env python
import rospy
import serial
import string
import math
import sys

#from time import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from razor_imu_9dof.cfg import imuConfig
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

degrees2rad = math.pi/180.0
imu_yaw_calibration = 0.0

# Callback for dynamic reconfigure requests
def reconfig_callback(config, level):
    global imu_yaw_calibration
    rospy.loginfo("""Reconfigure request for yaw_calibration: %d""" %(config['yaw_calibration']))
    #if imu_yaw_calibration != config('yaw_calibration'):
    imu_yaw_calibration = config['yaw_calibration']
    rospy.loginfo("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
    return config


rospy.init_node("razor_node")
#We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('imu', Imu, queue_size=1)
rpy_pub = rospy.Publisher('/imu/rpy/filtered',Vector3Stamped,queue_size=1)
srv = Server(imuConfig, reconfig_callback)  # define dynamic_reconfigure callback
diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
diag_pub_time = rospy.get_time();

imuMsg = Imu()
rpyMsg = Vector3Stamped()
port='/dev/ttyACM0'
# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=5)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

roll=pitch=yaw=seq=0

accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
rospy.loginfo("Giving the razor IMU board 3 seconds to boot...")
rospy.sleep(3) # Sleep for 5 seconds to wait for the board to boot

#start datastream
#ser.write('e' + chr(13))

#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working - it breaks the serial connection
rospy.loginfo("Flushing first 200 IMU entries...")
for i in range(200):
    line = ser.readline()
rospy.loginfo("Publishing IMU data...")
#f = open("raw_imu_data.log", 'w')

while not rospy.is_shutdown():
    line = ser.readline()
#    line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
    #f.write(line)                     # Write to the output log file
    words = string.split(line,",")    # Fields split
    if len(words) > 3:
        #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
        #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
        
        try:
            yaw = -float(words[3])#*degrees2rad
            pitch = float(words[1])#*degrees2rad
            roll = float(words[2])#*degrees2rad
        except:
            continue
        
        rpyMsg.vector.x=roll
        rpyMsg.vector.y=pitch
        rpyMsg.vector.z=yaw 
        rpy_pub.publish(rpyMsg)
        # Publish message
        # AHRS firmware accelerations are negated
        # This means y and z are correct for ROS, but x needs reversing
#        try:
#            imuMsg.linear_acceleration.x = -float(words[3]) * accel_factor
#            imuMsg.linear_acceleration.y = float(words[4]) * accel_factor
#            imuMsg.linear_acceleration.z = float(words[5]) * accel_factor
#    
#            imuMsg.angular_velocity.x = float(words[6])
#            #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
#            imuMsg.angular_velocity.y = -float(words[7])
#            #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
#            imuMsg.angular_velocity.z = -float(words[8])
#        except:
#            continue
    q = quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'base_imu_link'
    imuMsg.header.seq = seq
    seq = seq + 1
    pub.publish(imuMsg)

    if (diag_pub_time < rospy.get_time()) :
        diag_pub_time += 1
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = rospy.get_rostime()
        diag_arr.header.frame_id = '1'
        diag_msg = DiagnosticStatus()
        diag_msg.name = 'Razor_Imu'
        diag_msg.level = DiagnosticStatus.OK
        diag_msg.message = 'Received AHRS measurement'
        diag_msg.values.append(KeyValue('roll (deg)',
                                str(roll*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('pitch (deg)',
                                str(pitch*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('yaw (deg)',
                                str(yaw*(180.0/math.pi))))
        diag_msg.values.append(KeyValue('sequence number', str(seq)))
        diag_arr.status.append(diag_msg)
        diag_pub.publish(diag_arr)
        
ser.close
#f.close
