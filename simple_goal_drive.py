#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class Simple_Goal():
    def __init__(self):
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 5)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        
        way_path = '/home/agilex/taehun_limo/maps/way.csv'
        way = np.genfromtxt(way_path, delimiter=",")
        self.ref = np.array(way)
        
        self.drive_msg = PoseStamped()
        
        
    def pose_callback(self, data):
        self.pose_x = np.array(data.pose.pose.position.x)
        self.pose_y = np.array(data.pose.pose.position.y)
        
    def follow_point(self, pose_x, pose_y, ref):
        dist_list = np.hypot((ref[:,0] - pose_x),(ref[:,1] - pose_y))
        near_index = np.argmin(dist_list) # nearest point index
        
        if dist_list[near_index] <= 0.2:
            near_index += 1
         
        if near_index >= len(dist_list):
            near_index = 0 # reset 
        else :
            pass
        
        return near_index
    
    def goal(self):
        near_index = self.follow_point(self.pose_x, self.pose_y, self.ref)

        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.header.frame_id = "map"
        
        self.drive_msg.pose.position.x = self.ref[near_index,0]
        self.drive_msg.pose.position.y = self.ref[near_index,1]
        
        self.drive_msg.pose.orientation.x = 0.0
        self.drive_msg.pose.orientation.y = 0.0
        self.drive_msg.pose.orientation.z = 1.0
        self.drive_msg.pose.orientation.w = 0.0
        
        self.goal_pub.publish(self.drive_msg)
        
if __name__ == "__main__":
    rospy.init_node("simpe_goal_taehun", anonymous = True)
    simple_goal = Simple_Goal()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        simple_goal.goal()
        rate.sleep()