#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from visual import *
import math
import wx

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

rad2degrees = 180.0/math.pi
precision = 2 #round to this number of digits
yaw_offset = 0 #used to align animation upon key press


#Create shutdown hook to kill visual displays
def shutdown_hook():
    #print "Killing displays"
    wx.Exit()

#register shutdown hook
rospy.on_shutdown(shutdown_hook)


# Main scene
scene=display(title="9DOF Razor IMU Main Screen")
scene.range=(1.3,1.3,1.3)
scene.forward = (1,0,-0.25)
# Change reference axis (x,y,z) - z is up
scene.up=(0,0,1)
scene.width=500
scene.height=500

# Second scene (Roll, Pitch, Yaw)
scene2 = display(title='9DOF Razor IMU Roll, Pitch, Yaw',x=550, y=0, width=500, height=500,center=(0,0,0), background=(0,0,0))
scene2.range=(1,1,1)
scene2.select()
#Roll, Pitch, Yaw
#Default reference, i.e. x runs right, y runs up, z runs backward (out of screen)
cil_roll = cylinder(pos=(-0.5,0.3,0),axis=(0.2,0,0),radius=0.01,color=color.red)
cil_roll2 = cylinder(pos=(-0.5,0.3,0),axis=(-0.2,0,0),radius=0.01,color=color.red)
cil_pitch = arrow(pos=(0.5,0.3,0),axis=(0,0,-0.4),shaftwidth=0.02,color=color.green)
arrow_course = arrow(pos=(0.0,-0.4,0),color=color.cyan,axis=(0,0.2,0), shaftwidth=0.02, fixedwidth=1)

#Roll,Pitch,Yaw labels
label(pos=(-0.5,0.6,0),text="Roll (degrees / radians)",box=0,opacity=0)
label(pos=(0.5,0.6,0),text="Pitch (degrees / radians)",box=0,opacity=0)
label(pos=(0.0,0.02,0),text="Yaw (degrees / radians)",box=0,opacity=0)
label(pos=(0.0,-0.16,0),text="N",box=0,opacity=0,color=color.yellow)
label(pos=(0.0,-0.64,0),text="S",box=0,opacity=0,color=color.yellow)
label(pos=(-0.24,-0.4,0),text="W",box=0,opacity=0,color=color.yellow)
label(pos=(0.24,-0.4,0),text="E",box=0,opacity=0,color=color.yellow)
label(pos=(0.18,-0.22,0),height=7,text="NE",box=0,color=color.yellow)
label(pos=(-0.18,-0.22,0),height=7,text="NW",box=0,color=color.yellow)
label(pos=(0.18,-0.58,0),height=7,text="SE",box=0,color=color.yellow)
label(pos=(-0.18,-0.58,0),height=7,text="SW",box=0,color=color.yellow)

rollLabel = label(pos=(-0.5,0.52,0),text="-",box=0,opacity=0,height=12)
pitchLabel = label(pos=(0.5,0.52,0),text="-",box=0,opacity=0,height=12)
yawLabel = label(pos=(0,-0.06,0),text="-",box=0,opacity=0,height=12)

#acceleration labels
label(pos=(0,0.9,0),text="Linear Acceleration x / y / z (m/s^2)",box=0,opacity=0)
label(pos=(0,-0.8,0),text="Angular Velocity x / y / z (rad/s)",box=0,opacity=0)
linAccLabel = label(pos=(0,0.82,0),text="-",box=0,opacity=0,height=12)
angVelLabel = label(pos=(0,-0.88,0),text="-",box=0,opacity=0,height=12)

# Main scene objects
scene.select()
# Reference axis (x,y,z) - using ROS conventions (REP 103) - z is up, y left (west, 90 deg), x is forward (north, 0 deg)
# In visual, z runs up, x runs forward, y runs left (see scene.up command earlier) 
# So everything is good
arrow(color=color.green,axis=(1,0,0), shaftwidth=0.04, fixedwidth=1)
arrow(color=color.green,axis=(0,1,0), shaftwidth=0.04 , fixedwidth=1)
arrow(color=color.green,axis=(0,0,1), shaftwidth=0.04, fixedwidth=1)

# labels
label(pos=(0,0,-1.2),text="Press 'a' to align",box=0,opacity=0)
label(pos=(1,0.1,0),text="X",box=0,opacity=0)
label(pos=(0,1,-0.1),text="Y",box=0,opacity=0)
label(pos=(0,-0.1,1),text="Z",box=0,opacity=0)
# IMU object
platform = box(length=1.0, height=0.05, width=0.65, color=color.red,up=(0,0,1),axis=(-1,0,0))
p_line = box(length=1.1,height=0.08,width=0.1,color=color.yellow,up=(0,0,1),axis=(-1,0,0))
plat_arrow = arrow(length=-0.8,color=color.cyan,up=(0,0,1),axis=(-1,0,0), shaftwidth=0.06, fixedwidth=1)
plat_arrow_up = arrow(length=0.4,color=color.cyan,up=(-1,0,0),axis=(0,0,1), shaftwidth=0.06, fixedwidth=1)
rospy.init_node("display_3D_visualization_node")

def processIMU_message(imuMsg):
    global yaw_offset

    roll=0
    pitch=0
    yaw=0

    quaternion = (
      imuMsg.orientation.x,
      imuMsg.orientation.y,
      imuMsg.orientation.z,
      imuMsg.orientation.w)
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)

    #add align offset to yaw
    yaw += yaw_offset

    axis=(-cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
    up=(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw),-sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw),cos(roll)*cos(pitch))
    platform.axis=axis
    platform.up=up
    platform.length=1.0
    plat_arrow_up.axis=up
    plat_arrow_up.up=axis
    plat_arrow_up.length=0.4
    plat_arrow.axis=axis
    plat_arrow.up=up
    plat_arrow.length=-0.8
    p_line.axis=axis
    p_line.up=up
    p_line.length=1.1
    cil_roll.axis=(-0.2*cos(roll),0.2*sin(roll),0)
    cil_roll2.axis=(0.2*cos(roll),-0.2*sin(roll),0)
    cil_pitch.axis=(0,-0.4*sin(pitch),-0.4*cos(pitch))
    #remove yaw_offset from yaw display
    arrow_course.axis=(-0.2*sin(yaw-yaw_offset),0.2*cos(yaw-yaw_offset),0)
    
    #display in degrees / radians
    rollLabel.text = str(round(roll*rad2degrees, precision)) + " / " + str(round(roll,precision))
    pitchLabel.text = str(round(pitch*rad2degrees, precision)) + " / " + str(round(pitch, precision))
    #remove yaw_offset from yaw display
    yawLabel.text = str(round((yaw-yaw_offset)*rad2degrees, precision)) + " / " + str(round((yaw-yaw_offset), precision))

    linAccLabel.text = str(round(imuMsg.linear_acceleration.x, precision)) + " / " + str(round(imuMsg.linear_acceleration.y, precision)) + " / " + str(round(imuMsg.linear_acceleration.z, precision))
    angVelLabel.text = str(round(imuMsg.angular_velocity.x, precision)) + " / " + str(round(imuMsg.angular_velocity.y, precision)) + " / " + str(round(imuMsg.angular_velocity.z, precision))

    #check for align key press - if pressed, next refresh will be aligned
    if scene.kb.keys: # event waiting to be processed?
        s = scene.kb.getkey() # get keyboard info
        if s == 'a':
            #align key pressed - align
            yaw_offset += -yaw


sub = rospy.Subscriber('imu', Imu, processIMU_message)
rospy.spin()
