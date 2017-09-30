#!/usr/bin/env python



import rospy
import time
import sys
import math
import serial
import string
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class BaseControl:
    def __init__(self, baseId, device_port, baudrate):
        # Serial Communication
        try:
            self.serial = serial.Serial(device_port, baudrate, timeout=10)
	    rospy.loginfo("Communication success !")
	    # ROS handler        
            self.sub = rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, self.cmdCB, queue_size=10) 
	    self.sub1 = rospy.Subscriber('/car/cmd_vel', Twist, self.cmdCB1, queue_size=10) 
            self.timer_cmd = rospy.Timer(rospy.Duration(0.1), self.timerCmdCB) # 10Hz
            self.baseId = baseId
            # variable        
            self.trans_x = 0.0
            self.rotat_z = 0.0
            self.trans_x1 = 0.0
            self.rotat_z1 = 0.0
            self.flag=0
        except serial.serialutil.SerialException:
            rospy.logerr("Can not receive data from the port: "+device_port + 
            ". Did you specify the correct port in the launch file?")
            self.serial.close
            sys.exit(0) 

    def cmdCB(self, data):
        self.trans_x = data.drive.speed
	self.rotat_z = data.drive.steering_angle
    def cmdCB1(self, data):
        self.trans_x1 = data.linear.x
	self.rotat_z1 = data.angular.z
	self.flag=data.linear.z
    def timerCmdCB(self, event):
	if self.flag==1:
		self.trans_x=self.trans_x1
		self.rotat_z=self.rotat_z1
        # send cmd to motor
        if self.trans_x >0 and self.trans_x <=0.15 :
		self.throttle = 1455
	elif self.trans_x >0.15 and self.trans_x <=0.4 :
		self.throttle = 1450
	elif self.trans_x <0 and self.trans_x >=-0.15 :
		self.throttle = 1605
	elif self.trans_x <-0.15 and self.trans_x >=-0.4 :
		self.throttle = 1615
	else:
		self.throttle = 1500

	self.turn = self.rotat_z*180/3.1415926+90
        
        values = [str(self.turn), str(self.throttle), '0']
        values[2] = str(len(values[0]) + len(values[1]))
	#values1=[str(self.rotat_z1), str(self.trans_x1)]
        cmd = ",".join(values).encode()
	#cmd1 = ",".join(values1).encode()
	rospy.logerr(cmd)
	#rospy.logerr(cmd1)        
        self.serial.flushInput()
	self.serial.write(cmd)
if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('base_control', anonymous=True)

        # Get params
        baseId = rospy.get_param('~base_id', 'base_link') # base link
        device_port = rospy.get_param('~port', '/dev/uno') # device port
        baudrate = float( rospy.get_param('~baudrate', '57600') ) 
        # Constract BaseControl Obj
        rospy.loginfo("Start base control node ...")
        bc = BaseControl(baseId, device_port, baudrate)
        rospy.spin()
    except KeyboardInterrupt:
        bc.serial.close
    print("shutting down")
