#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String

import csv
import copy
import time
from math import pi
import numpy as np
import math

class Slam_Drive():
    def __init__(self):

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        #self.path_sub = rospy.Subscriber("_path", String , self.path_callback)
        self.odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)
        self.wheelbase = 0.21
        self.aeb = 0
        #v2x variable
        #self.path_topic = "x"
        #self.mode = "x"
        self.poses = [0.0, 0.0]
        # ref and path [x,y] 
        self.global_ref = np.genfromtxt("/home/agilex/taehun_limo/maps/way.csv", delimiter=',')
        #self.path_a = np.genfromtxt("/home/agilex/path_a.csv", delimiter=',', skip_header = 1)
        #self.path_b = np.genfromtxt("/home/agilex/path_b.csv", delimiter=',', skip_header = 1)
        #self.path_c = np.genfromtxt("/home/agilex/path_c.csv", delimiter=',', skip_header = 1)

        self.global_ref_x = self.global_ref[:,0]
        self.global_ref_y = self.global_ref[:,1]

        #self.now_path = self.global_ref
        self.drive_msg = PoseStamped()

    def find_lookahead_idx(self, pose_x, pose_y, global_ref):
        start = time.time()

        # find the lookahead index directly in front of the car
        self.lookahead_distance = 1.0
        distances = np.sqrt((self.global_ref[:, 0] - pose_x) ** 2 + (self.global_ref[:, 1] - pose_y) ** 2)
	#distances = np.sqrt((local_ref[:, 0]) ** 2 + (local_ref[:, 1]) ** 2)
        min_idx = np.argmin(distances)
        lookahead_idx = min_idx

        while True:
            lookahead_idx += 1

            if lookahead_idx >= len(self.global_ref):
                lookahead_idx = 0

            dist_to_lookahead = np.hypot(self.global_ref[lookahead_idx, 0] - pose_x, self.global_ref[lookahead_idx, 1] - pose_y)
            #dist_to_lookahead = np.hypot(local_ref[lookahead_idx, 0], local_ref[lookahead_idx, 1])

            if dist_to_lookahead >= self.lookahead_distance:
                break

        return lookahead_idx

    
    def odom_callback(self,data):
        self.poses = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        self.ori = data.pose.pose.orientation

    '''def path_callback(self, data):
        self.path_topic = data.data
        print("now path : ", self.path_topic)
                #v2x
        if self.path_topic == "A":
            self.now_path = self.path_a

        elif self.path_topic == "B":
            self.now_path = self.path_b

        elif self.path_topic == "C":
            self.now_path = self.path_c

        ## or 
        if self.path_topic == "A":
            self.now_path.append(self.path_a)

        elif self.path_topic == "B":
            self.now_path.append(self.path_b)

        elif self.path_topic == "C":
            self.now_path.append(self.path_c)
        
	'''
            
    def scan_callback(self,lidar):
        lidar_ranges = lidar.ranges
        front_scan = lidar_ranges[150:280]

        if front_scan < 0.3:
            self.aeb = 1
        else:
            self.aeb = 0

    def main(self):
        pose_x = self.poses[0]
        pose_y = self.poses[1]
        print('pose_x: ', pose_x)
        print('pose_y: ', pose_y)
        
	    ori_x = 0.0
	    ori_y = 0.0	
	    ori_z = 1.0
	    ori_w = 0.0        
     
        lookahead_idx = self.find_lookahead_idx(pose_x, pose_y, self.global_ref)

	    goal_x = self.global_ref_x
        goal_y = self.global_ref_y

	    self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.header.frame_id = "map"
        
        self.drive_msg.pose.position.x = goal_x[lookahead_idx]
        self.drive_msg.pose.position.y = goal_y[lookahead_idx]
        self.drive_msg.pose.orientation.x = ori_x
        self.drive_msg.pose.orientation.x = ori_y
        self.drive_msg.pose.orientation.x = ori_z
        self.drive_msg.pose.orientation.x = ori_w
	    '''
        if self.aeb == 1 :
            drive_msg.pose.position.x = goal_x[lookahead_idx - 1]
            drive_msg.pose.position.y = goal_y[lookahead_idx - 1]
	    else : 
	    pass
	    '''

        self.goal_pub.publish(self.drive_msg)
        
if __name__ == "__main__":
        rospy.init_node("slam_drive", anonymous=True)
        slam_drive = Slam_Drive()

        while not rospy.is_shutdown():
            slam_drive.main()
            rospy.Rate(60).sleep()

