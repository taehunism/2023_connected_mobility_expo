#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np
import itertools,operator
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import concurrent.futures

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan, Joy

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Int8, Int16
import matplotlib.pyplot as plt
import csv
import copy

from drivers_ppc_kkh import PurePursuitDriver

# choose your drivers here (1-4)
drivers = PurePursuitDriver(lookahead_distance=0.4)


prev_steering_angle = 0
t_prev = 0.0
prev_range = []

view_angel = 180 #0-135
view_idx = int(1077/135*view_angel)

Kp = 1
Ki = 1
Kd = 1

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        #self.lidar_sub = rospy.Subscriber('/scan',LaserScan, self.ppc_callback)

        self.driver = drivers
        self.pose_sub = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped , self.ppc_callback)
        self.drive_pub = rospy.Publisher('/vesc/ackermann_cmd',AckermannDriveStamped,queue_size=5)
        self.point_pub = rospy.Publisher("/lookahead_point", Marker, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.x_values = []
        self.y_values = []
        self.min = 100
        self.gap_max = 100 
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.flag_sub = rospy.Subscriber('/flag', Int8, self.flag_callback)
        self.auto_flag_sub = rospy.Subscriber('/auto_flag', Int16, self.auto_flag_callback)
        self.mode_flag_sub = rospy.Subscriber('/mode_flag', Int16, self.mode_flag_callback)
        self.flag_in = 0
        self.auto_flag_in = 0
        self.vel_weight = 0.6
        self.cnt = 0
        self.aeb_dis = 0.0
        self.poses = [0.0, 0.0]
        self.quat = Quaternion()

        self.global_ref = np.genfromtxt('/home/f1tenth/f1tenth_ws/src/F1tenth_bootcamp/racecar/racecar/maps/cut_race_way.csv', delimiter=',', skip_header = 1)
        #self.global_ref = np.genfromtxt('/home/f1tenth/f1tenth_ws/src/F1tenth_bootcamp/racecar/racecar/maps/optimal_raceline.csv', delimiter=',', skip_header = 1)
        ##self.ref[:,1] = -self.ref[:,1]

        self.global_ref[:,0] = self.global_ref[:,0]# - 2.08
        self.global_ref[:,1] = self.global_ref[:,1] # + 9.5
        # data[:,0] = data[:,0]-180.8
        # data[:,1] = -1*data[:,1]+154.2

        self.prev_idx = 0
     
    def points_marker(self,points, frame_id="map", ns="points", id=0, color=(1,0,0)):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.ns = ns
        marker.id = id

        marker.scale.x = 0.2  # specifies the radius of the points
        marker.scale.y = 0.2
 
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0

        marker.points = []

        p = Point()
        p.x, p.y = points
            #print("point: ",point)
            #print("p.x: ",p.x)
            #print("p.y: ",p.y)
            #print("p.z: ",p.z)
        marker.points.append(p)
        #marker.points = p

        return marker
    

    def joy_callback(self, data):
        self.flag_in = data.buttons[5]

        # 0 - auto , else - manual
        #print("flag input:",data.data)
        #if self.flag_in is None:
        #    self.flag = 1
           
        #else:
        #     self.flag = 0

    def flag_callback(self, data):
        #print('=== flag changed !!! ===', data.data)
        pass

    def auto_flag_callback(self, data):
        self.auto_flag_in = data.data
        print('auto_flag : ', self.auto_flag_in)

    def mode_flag_callback(self, data):
        if data.data == 0:
            self.vel_weight = 0.6
        else:
            self.vel_weight = 1.0

    def ppc_callback(self, data):

        # print(dir(data.pose.pose.position))
        #print(self.global_ref)

        self.poses = np.array([data.pose.pose.position.x,data.pose.pose.position.y])
        self.quat = data.pose.pose.orientation

    def scan_callback(self, data):
        if len(data.ranges) == 0:
            return
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()

        front = data.ranges[520:560]
        gap_follow = data.ranges[480:600]

        temp_range = front
        max_gap = np.argmax(temp_range)
        min_gap = np.argmin(temp_range)
        
        self.min = temp_range[min_gap]
        self.gap = temp_range[max_gap]
        
        self.gap_max = np.argmax(gap_follow)

        #print("lidar front :",self.min)

    def main(self):
        #####
        pose_x = self.poses[0]          #Cartesian coordinates of the vehicle in the global frame
        pose_y = self.poses[1]          #Cartesian coordinates of the vehicle in the global frame
        print('pose_x: ',pose_x,'pose_y: ',pose_y)

        # print(poses)
        quat_x = self.quat.x
        quat_y = self.quat.y
        quat_z = self.quat.z
        quat_w = self.quat.w   #vehicle's orientation in the global frame
        (roll, pitch, yaw) = euler_from_quaternion([quat_x, quat_y, quat_z, quat_w])
        pose_theta = yaw
        print('pose_theta: ',pose_theta)


        '''
        distances = np.sqrt((self.ref[:, 0] - pose_x) ** 2 + (self.ref[:, 1] - pose_y) ** 2)
        print('distances: ',distances)
        min_idx = np.argmin(distances)       ##find index of minimum value
        print('distances[min_idx]: ',distances[min_idx])
        lookahead_idx = min_idx
        print('lookahead_idx: ',lookahead_idx)
        print('pose_x: ',pose_x)
        print('pose_y: ',pose_y)
        print('pose_x_ref: ',self.ref[lookahead_idx,0])
        print('pose_y_ref: ',self.ref[lookahead_idx,1])
        print('len(ref): ',len(self.ref))
        print('ref[lookahead_idx]: ',self.ref[lookahead_idx])
        

        while True:
            lookahead_idx += 1
            if lookahead_idx >= len(self.ref):      #len(ref) of Austin map is 1102
                lookahead_idx = 0
            dist_to_lookahead = np.hypot(self.ref[lookahead_idx, 0] - pose_x, self.ref[lookahead_idx, 1] - pose_y)  ##calculate distance between robot pose with ref of lookahead
            if dist_to_lookahead >= 1.0:   ##if waypoint is farther than the lookahead distance
                break
       '''
        #print(self.global_ref)

        local_ref = copy.deepcopy(self.global_ref)

        local_ref[:,0] = math.cos(pose_theta)*(self.global_ref[:,0]- pose_x) + math.sin(pose_theta)*(self.global_ref[:,1] - pose_y)
        local_ref[:,1] = -math.sin(pose_theta)*(self.global_ref[:,0]- pose_x) + math.cos(pose_theta)*(self.global_ref[:,1] - pose_y)
        # print('local_ref[0:5,0]: ',local_ref[0:5,0])
        # print('local_ref[0:5,1]: ',local_ref[0:5,1])
        print('len(ref): ',len(local_ref))
        speed, steering_angle, lookahead_idx = drivers.pure_pursuit_control(pose_x, pose_y, pose_theta, local_ref)

        # dist = np.abs((self.ref[:,0]-poses[0])**2+(self.ref[:,1]-poses[1])**2-L_f**2)

        # idx = np.argmin(dist)

        # if idx > self.prev_idx and idx-self.prev_idx < 10:
        #     self.prev_idx = idx
        
        # lookahead_idx = self.prev_idx
        
        # # Compute the heading to the lookahead point
        # theta = math.atan2(self.ref[lookahead_idx, 1]-poses[1], self.ref[lookahead_idx, 0]-poses[0])-pose_theta

        # theta = math.atan2(math.sin(theta), math.cos(theta))

        # # Compute the steering angle
        # steering_angle = math.atan2(2.0*math.sin(theta), L_f)

        # # Compute the speed (you may use constant speed simply)
        # speed = 1

        #print("idx : ",lookahead_idx)
        #print("steer : ", steering_angle)
        

        #########visualization lookahead point#############################
        lookahead_point2 = self.global_ref[lookahead_idx]
        lookahead_point2 = lookahead_point2[:2]
        print('lookahead_point2: ',lookahead_point2)
        #print(self.global_ref)
        marker = self.points_marker(lookahead_point2)
        ###################################################################
        
        print("lookahead_idx :",lookahead_idx)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
            #drive_msg.drive.steering_angle = 0.0
            #drive_msg.drive.steering_angle = steering_angle
        if self.flag_in == 1 and self.auto_flag_in == 1 and self.min >= self.aeb_dis:
            #drive_msg.drive.steering_angle = 0.0
            if lookahead_idx <= 4:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 5.0 *self.vel_weight

            elif lookahead_idx >= 5 and lookahead_idx <= 9:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 3.0 *self.vel_weight

            elif lookahead_idx >= 10 and lookahead_idx <= 11:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 3.0 *self.vel_weight

            elif lookahead_idx >= 11 and lookahead_idx <= 12:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 2.7 *self.vel_weight

            elif lookahead_idx >= 13 and lookahead_idx <= 13:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 2.2 *self.vel_weight

            elif lookahead_idx >= 14 and lookahead_idx <= 14:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 2.3 *self.vel_weight

            elif lookahead_idx >= 15 and lookahead_idx <= 17:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 3.0 *self.vel_weight

            elif lookahead_idx >= 18 and lookahead_idx <= 24:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 3.5 *self.vel_weight

            elif lookahead_idx >= 25 and lookahead_idx <= 26:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 13.0 *self.vel_weight

            elif lookahead_idx >= 27 and lookahead_idx <= 27:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 7.0 *self.vel_weight

            elif lookahead_idx >= 28 and lookahead_idx <= 37:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 2.5 *self.vel_weight

            #elif lookahead_idx >= 29 and lookahead_idx <= 32:
                #    drive_msg.drive.speed = 3:0
                #    drove_msg.drive.st

            elif lookahead_idx >= 38 and lookahead_idx <= 38:
                drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
                drive_msg.drive.speed = 7.0 *self.vel_weight

            #elif lookahead_idx >= 38 and lookahead_idx <= 38:
            #    drive_msg.drive.steering_angle = steering_angle
                #drive_msg.drive.speed = speed - abs(speed*steering_angle)
            #    drive_msg.drive.speed = 3.0

            else:
                pass
                
            
            #drive_msg.drive.speed = speed - (speed * abs(steering_angle))
            
            #drive_msg.drive.speed = speed
        elif self.flag_in == 1 and self.auto_flag_in == 0:
            #drive_msg.drive.steering_angle = 0.0
            #drive_msg.drive.speed = 0.0
            # (480~ 600) - 480  = 0 ~ 120
            pass
        else:
            pass

        if self.min < self.aeb_dis:
            drive_msg.drive.steering_angle = steering_angle
            drive_msg.drive.speed = 0.0
            self.cnt += 1

            if self.cnt == 40:
                if self.gap_max > 60:
                    drive_msg.drive.steering_angle = ((self.gap_max/4)/180) * math.pi
                    drive_msg.drive.speed = 1.0
                else:
                    drive_msg.drive.steering_angle = -((self.gap_max/4)/180) * math.pi
                    drvie_msg.drive.speed = 1.0
        else:
            self.cnt = 0

             
        #print(data)
        print("speed:", drive_msg.drive.speed)
        print("flag:", self.flag_in)
        print("vel_weight:", self.vel_weight)

        self.drive_pub.publish(drive_msg)
        self.point_pub.publish(marker)







    #     with open("map_data_2-centerline.csv", "r") as csv_file:
    #         csv_reader = csv.reader(csv_file)
    #         next(csv_reader)  # Skip the header row
    #         for row in csv_reader:
    #             self.x_values.append(float(row[0]))
    #             self.y_values.append(-float(row[1]))

                


    # ################plot##############################

    #     # Create the scatter plot
    #     plt.scatter(self.x_values, self.y_values, c='#1f77b4')
    #     plt.xlabel("X Position")
    #     plt.ylabel("Y Position")
    #     plt.title("Point Mapping")

    #     # Display the plot
    #     new_x = poses[0]
    #     new_y = poses[1]
    #     self.update_plot(new_x, new_y)

    #     self.update_plot(self.ref[lookahead_idx,0],self.ref[lookahead_idx,1])

    #     plt.show(block=False)




    # def update_plot(self, x, y):
    #     self.x_values.append(x)
    #     self.y_values.append(y)
    #     plt.scatter(x, y, color='red')  # Add the new point
    #     plt.pause(0.001)  # Pause to update the plot


        




#def main(args):

if __name__ == '__main__':
    #main(sys.argv)
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    while not rospy.is_shutdown():
        rfgs.main()
        rospy.Rate(100).sleep()
        #rospy.spin()

