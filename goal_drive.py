#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import csv
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from math import sqrt
import numpy as np

def calculate_distance(coord1, coord2):
    # 두 좌표 간의 거리 계산
    dx = coord2[0] - coord1[0]
    dy = coord2[1] - coord1[1]
    distance = sqrt(dx**2 + dy**2)
    return distance

def get_closest_waypoint(robot_pose, waypoints):
    # 현재 로봇 위치에서 가장 가까운 웨이포인트 찾기
    min_distance = float('inf')
    closest_waypoint = None

    for i, waypoint in enumerate(waypoints):
        distance = calculate_distance(robot_pose, waypoint)
        if distance < min_distance:
            min_distance = distance
            closest_waypoint = i

    return closest_waypoint

def send_goal(client, goal_pose):
    # 이동 목표 생성
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose = goal_pose

    # 목표 전송
    client.send_goal(goal)

    # 목표 달성까지 대기
    client.wait_for_result()

def odom_callback(msg):
    # 현재 로봇의 위치 업데이트
    global current_robot_pose
    current_robot_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

def read_csv_file(file_path):
    # CSV 파일에서 웨이포인트 읽기
    waypoints = np.genfromtxt(file_path, delimiter=',', skip_header=1, dtype=float)

    return waypoints


def main():
    # 노드 초기화
    rospy.init_node('waypoint_node')

    # MoveBase 액션 클라이언트 생성
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # 서버에 연결될 때까지 대기
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server")

    # CSV 파일에서 웨이포인트 읽기
    csv_file_path = '/home/agilex/taehun_limo/maps/way.csv'
    waypoints_list = read_csv_file(csv_file_path)

    # 로봇의 위치 업데이트 콜백 함수 등록
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, odom_callback)

    # 현재 로봇의 위치 초기화
    global current_robot_pose
    current_robot_pose = [0.0, 0.0]

    try:
        while not rospy.is_shutdown():
            # 가장 가까운 웨이포인트 찾기
            closest_waypoint_index = get_closest_waypoint(current_robot_pose, waypoints_list)

            # 가장 가까운 웨이포인트로 이동
            closest_waypoint_pose = Pose(Point(waypoints_list[closest_waypoint_index][0], waypoints_list[closest_waypoint_index][1], 0.0),
                                         Quaternion(0, 0, 0, 1))
            send_goal(client, closest_waypoint_pose)

            rospy.sleep(1)  # 웨이포인트 간의 간격 조절을 위해 잠시 대기

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

