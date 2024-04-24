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
        
    def find_goal(self,ranges):
        '''range_gap = ranges[143:287]
        gap_index = np.argmax(range_gap)
        gap = range_gap[gap_index]
        '''
        right = ranges[0:143]
        right_min_index = np.argmin(right)
        right_min = right[right_min_index]
        
        front = ranges[144:287]
        front_min_index = np.argmin(front)
        front_min = front[front_min_index]

        left = ranges[287:430]
        left_min_index = np.argmin(left)
        left_min = left[left_min_index]
        
        if right_min > front_min:
            steer_angle = (right_min_index / 2.39) * pi / 180
        
        else :
            steer_angle = (71.5 - front_min_index) / 2.39 * pi / 180

        if left_min > front_min:
            steer_angle = (-left_min_index / 2.39) * pi / 180
        
        else :
            steer_angle = (71.5 - front_min_index) / 2.39 * pi / 180

            
        return steer_angle

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
    # 430/180=2.39 -> index 2.39 = 1 [deg] 
        #gap_threshold = 72 # center line of valid range area 
        drive = Twist()

        #gap, gap_index = self.find_gap(data.ranges) # called gap find function
        #right_min, front_min, left_min = self.find_goal(data.ranges)
        steer_angle = self.find_goal(data.ranges)
		#aeb_flag = self.aeb(data.ranges)
 
        print("===================================== ")        
        #print("lidar_total_index : ", len(data.ranges))
        #print("follow_gap_index : ", gap_index)
        #print("gap_distance : ", gap)
		#print("steer_threshold : ", gap_threshold)

	#print("left:", left)
	#print("left:", right)
        #gap_deg = (gap_index - gap_threshold) / 2.39 # deg convert
        #steer_angle = (gap_deg * pi / 180) #* self.wheelbase  # deg -> rad convert process
        drive.angular.z = steer_angle
        drive.linear.x = 0.2
        print("steer_angle [rad] :", steer_angle)
        

	#if aeb_flag == 0:
        '''if gap > 0.8 and front > 0.5 and left > 0.15 and right > 0.15:
            drive.linear.x = 0.2
            drive.angular.z = steer_angle

        else:
	        drive.linear.x = 0.3
	        drive.angular.z = steer_angle
        
        
        if left < 0.12:
            drive.angular.z = 0.0
            drive.angular.z = 0.2

        if right < 0.12:
            drive.angular.z = 0.0
            drive.angular.z = -0.2
	    	    
        if front < 0.2:
            drive.linear.x = 0.0
            drive.linear.x = -0.3
        '''
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
