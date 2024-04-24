#!/usr/bin/env python

import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from geometry_msgs.msg import PoseWithCovarianceStamped

home = expanduser('~')
file = open('/home/agilex/taehun_limo/maps/way.csv', 'w')

def save_waypoint(data):
    pose = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
    file.write('{:.4f}, {:.4f}\n'.format(pose[0], pose[1]))
    print("saving...")
                                    
def shutdown():
    file.close()
    print("done")
    print('good bye')

def listener():
    rospy.init_node('waypoints_logger', anonymous=True)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, save_waypoint)
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('saving...')
    try:
        listener()
    except rospy.ROSInterruptException:
	print("retry plz")

