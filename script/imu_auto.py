#!/usr/bin/env python
# latest editor:
# HaoChih, LIN. 2017, Hypha ROS Workshop
#
# Original Author:
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
import serial
import string
import math
import sys

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

### ---------------- ###
### Global variables ###
### ---------------- ### 
degrees2rad = math.pi/180.0
rad2deg = 180/math.pi#/180.0
imu_yaw_calibration = 0.0
yaw_deg = 0.0
current = 0.0
last = 0.0
dt = 0.0
roll=0
pitch=0
yaw=0
seq=0
accel_factor = 9.806 / 256.0    # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.

rospy.init_node("imu_auto_cali_node")
pub = rospy.Publisher('/imu_data', Imu, queue_size=1)
imuMsg = Imu()

# Orientation covariance estimation:
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.000025
]

# Angular velocity covariance estimation:
imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

# linear acceleration covariance estimation:
imuMsg.linear_acceleration_covariance = [
-1 , 0 , 0,
0 , 0, 0,
0 , 0 , 0
]

### -------------- ###
### Get parameters ###
### -------------- ### 
# IMU frame
imu_frame = rospy.get_param('~imu_frame', 'IMU_link')

# device port
port = rospy.get_param('~port', '/dev/gy85')

# baudrate
baudrate = rospy.get_param('~baudrate', 57600)

# yam compensation
imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+port + ". Did you specify the correct port in the launch file?")
    #exit
    sys.exit(0)

rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5) # Sleep for 5 seconds to wait for the board to boot

### configure board ###
#stop datastream
ser.write('#o0' + chr(13))

#discard old input
#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working
discard = ser.readlines() 

#start datastream
ser.write('#ox' + chr(13)) # To start display angle and sensor reading in text
ser.flushInput()
ser.write('#o1' + chr(13))

#automatic flush - NOT WORKING
#ser.flushInput()  #discard old input, still in invalid format
#flush manually, as above command is not working - it breaks the serial connection
rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = ser.readline()


### -------------------- ###
### Auto yaw-calibration ###
### -------------------- ### 
rospy.loginfo("Auto yaw calibration...")
count = 0.0
vyaw_sum = 0.0
for x in range(0, 300):
    try:
        line = ser.readline()
        line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
        words = string.split(line,",")    # Fields split
        vyaw_sum = vyaw_sum + float(words[8])
        count = count + 1.0
    except:
	    rospy.logwarn("Error in Sensor values")
	    pass

vyaw_bias = float(vyaw_sum/count)
rospy.loginfo("Bias of Vyaw is: %f", vyaw_bias)

### --------------- ###
### Publishing data ###
### --------------- ### 
rospy.loginfo("Publishing IMU data...")
while not rospy.is_shutdown():
    try:
        line = ser.readline()
        line = line.replace("#YPRAG=","")   # Delete "#YPRAG="
        words = string.split(line,",")    # Fields split
        #print(words) # For debug
        if len(words) > 2:
            #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
            yaw_deg = -float(words[0])
            yaw_deg = yaw_deg + imu_yaw_calibration
            if yaw_deg > 180.0:
                yaw_deg = yaw_deg - 360.0
            if yaw_deg < -180.0:
                yaw_deg = yaw_deg + 360.0
            yaw = yaw_deg*degrees2rad
            #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
            pitch = -float(words[1])*degrees2rad
            roll = float(words[2])*degrees2rad

            # Publish message
            # AHRS firmware accelerations are negated
            # This means y and z are correct for ROS, but x needs reversing
            imuMsg.linear_acceleration.x = -float(words[3]) * accel_factor
            imuMsg.linear_acceleration.y = float(words[4]) * accel_factor
            imuMsg.linear_acceleration.z = float(words[5]) * accel_factor

            imuMsg.angular_velocity.x = float(words[6])
            #in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
            imuMsg.angular_velocity.y = -float(words[7])
            #in AHRS firmware z axis points down, in ROS z axis points up (see REP 103) 
            imuMsg.angular_velocity.z = -( float(words[8]) - vyaw_bias )
        yaw_deg = -( float(words[8]) - vyaw_bias )*rad2deg*dt + yaw_deg
        if (yaw_deg <= -180):
	    yaw_deg += 360
        if (yaw_deg > 180):
	    yaw_deg -=360
        yaw_rad = yaw_deg * degrees2rad
        current = rospy.get_time()
        dt = current - last;
        last = current
        #q = quaternion_from_euler(roll,pitch,yaw)
        q = quaternion_from_euler(0,0,yaw_rad)
        imuMsg.orientation.x = q[0]
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3]
        imuMsg.header.stamp= rospy.Time.now()
        imuMsg.header.frame_id = imu_frame
        imuMsg.header.seq = seq
        seq = seq + 1
        pub.publish(imuMsg)
  
    except:
	    rospy.logwarn("Error in Sensor values")
	    pass
ser.close
#f.close

