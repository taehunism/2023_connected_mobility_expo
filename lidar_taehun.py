#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

from math import pi
import numpy as np

class Lidar():
    def __init__(self):
        rospy.init_node("lidar_labacorn")
        self.drive_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.flag_sub = rospy.Subscriber("flag", Int8, self.flag_callback)
        self.rate = rospy.Rate(60)
        self.wheelbase = 0.21

    def find_gap(self,ranges):
        gap_index = np.argmax(ranges)
        gap = ranges[gap_index]

        return gap, gap_index

    '''def aeb(self,ranges):
	aeb_range = ranges[210:220] # front angle 13 deg 
	aeb_distance = ranges[np.argmax(aeb_range)]
	print("aeb distance : ", aeb_distance)
	if aeb_distance < 0.5:
	    aeb_flag = 1
	else :
	    aeb_flag = 0

	return aeb_flag'''
        
    # def other_gap(self,ranges):
    #     other_ranges = data.ranges[74:366]
    #     other_gap = other_ranges[]

    def scan_callback(self, data):
    # lidar angle 180 deg, right -> left index sequence , index 440 
    # 440/180 -> one index per deg 
        gap_threshold = 215 # center line of valid range area 
        drive = Twist()
		
	front = data.ranges[210:220]
	left = data.ranges[250:430]
	right = data.ranges[0:180]

        gap, gap_index = self.find_gap(data.ranges) # called gap find function
	#aeb_flag = self.aeb(data.ranges)

        # gap_index > gap_treshold : left steer      < explain 
        # gap_index < gap_treshold : right steer     < explain   
	print("===================================== ")        
	print("lidar_total_index : ", len(data.ranges))
	print("follow_gap_index : ", gap_index)
	print("gap_distance : ", gap)
	#print("steer_threshold : ", gap_threshold)

	#print("left:", left)
	#print("left:", right)

	gap_deg = (gap_index - gap_threshold) / 2.39 # deg convert
        steer_angle = (gap_deg * pi / 180) #* self.wheelbase  # deg -> rad convert process
	
	print("steer_angle [rad] :", steer_angle)

        #drive.linear.x = 0.5 # default speed 
        #drive.angular.z = 0.0

	#if aeb_flag == 0:
	if gap > 1.5 and front > 0.7 and left > 0.5 and right > 0.5:
	    drive.linear.x = 0.5
	    drive.angular.z = steer_angle	
	else:
	    drive.linear.x = 0.2
	    drive.angular.z = steer_angle
	
	#else: 
	    #drive.linear.x = 0.0
 
        self.drive_pub.publish(drive)
	self.rate.sleep()


    def flag_callback(self, data):
        self.flag = data
        

def start():
    lidar = Lidar()
    rospy.spin()
    

if __name__ == "__main__":
    try:
        start()
    except KeyboardInterrupt:
        print("program down") 
