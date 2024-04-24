#!/usr/bin/env python3

import rospy
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import csv
import copy
from drivers_ppc_kkh import PurePursuitDriver
drivers = PurePursuitDriver(lookahead_distance=0.3)

class Simple_Goal:
    def __init__(self):
        self.driver = drivers
        self.goal_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.point_pub = rospy.Publisher("/lookahead_point", Marker, queue_size=10)
        
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        #rospy.Subscriber('/path_', String, self.path_callback)

        # way_path = '/home/agilex/taehun_limo/maps/way.csv'
        # way_path_a = '/home/agilex/taehun_limo/maps/way_a.csv'
        # way_path_b = '/home/agilex/taehun_limo/maps/way_b.csv'
        # way_path_c = '/home/agilex/taehun_limo/maps/way_c.csv'

        way_path = '/home/agilex/target_path.txt'
        # way_path_a = '/home/agilex/taehun_limo/maps/way_a.csv'
        # way_path_b = '/home/agilex/taehun_limo/maps/way_b.csv'
        # way_path_c = '/home/agilex/
        
        self.way = np.genfromtxt(way_path, delimiter=",", skip_header=1)
        # self.way_a = np.genfromtxt(way_path_a, delimiter=",", skip_header=1)
        # self.way_b = np.genfromtxt(way_path_b, delimiter=",", skip_header=1)
        # self.way_c = np.genfromtxt(way_path_c, delimiter=",", skip_header=1)

        self.now_path = np.array(self.way)
        self.path = "A"
        self.drive_msg = Twist()

        self.poses = [0.0, 0.0]
        self.lookahead_distance = 0.1
        self.cnt = 0
        self.quat = Quaternion()
    
    def points_marker(self,points, frame_id="map", ns="points", id=0, color=(1,0,0)):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.ns = ns
        marker.id = id

        marker.scale.x = 0.1  # specifies the radius of the points
        marker.scale.y = 0.1
 
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        marker.points = []
        
        for i in points:
            p = Point()
            p.x, p.y = points
            marker.points.append(p)

        return marker

    def pose_callback(self, data):
        self.poses = np.array([data.pose.pose.position.x,data.pose.pose.position.y])
        self.quat = data.pose.pose.orientation
        
    # def path_callback(self, data):
    #     #v2x
    #     self.path = data.data
    #     if self.path == "A":
    #         self.now_path = self.way_c

    #     elif self.path == "B":
    #         self.now_path = self.way_b

    #     elif self.path == "C":
    #         self.now_path = self.way_c

    #     else :
    #         self.now_path = self.way

    def main(self):
        pose_x = self.poses[0]
        pose_y = self.poses[1]
        
        print("==========================================================")
        print("pose_x : {}, pose_y : {}".format(pose_x, pose_y))
        
        quat = (self.quat.x, self.quat.y, self.quat.z, self.quat.w)
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        pose_theta = yaw
        
        print('pose_theta: ',pose_theta)
        
        local_ref = copy.deepcopy(self.now_path)
        local_ref[:,0] = math.cos(pose_theta)*(self.now_path[:,0]- pose_x) + math.sin(pose_theta)*(self.now_path[:,1] - pose_y)
        local_ref[:,1] = -math.sin(pose_theta)*(self.now_path[:,0]- pose_x) + math.cos(pose_theta)*(self.now_path[:,1] - pose_y)

        print('len(ref): ',len(local_ref))
        speed, steering_angle, lookahead_idx = drivers.pure_pursuit_control(pose_x, pose_y, pose_theta, local_ref)
        
        #################### visualization marker ################
        lookahead_point2 = self.now_path[lookahead_idx]
        #print('lookahead_point: ',lookahead_point2)
        marker = self.points_marker(lookahead_point2)
        ################### publish ###################
        
        print("lookahead idx : {}".format(lookahead_idx))
        print("==========================================================")
        self.drive_msg.linear.x = speed
        self.drive_msg.angular.z = steering_angle
        
        if lookahead_idx <= 4:
                self.drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                self.drive_msg.drive.speed = 0.2

        elif lookahead_idx >= 5 and lookahead_idx <= 9:
                self.drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                self.drive_msg.drive.speed = 3.0
                
        self.goal_pub.publish(self.drive_msg)
        self.point_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("simple_goal_taehun", anonymous=True)
    simple_goal = Simple_Goal()
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        simple_goal.main()
        rate.sleep()
