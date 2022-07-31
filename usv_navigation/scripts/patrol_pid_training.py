#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point, Quaternion
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty
import rosbag
import subprocess
import os
import numpy as np
from navigation_utilities import *

# TODO all of this:
# 1) Choose goal randomly from the workspace (need coord bounds)
# 3) etc

result = Float64()
result.data = 0
maxSimulations = 1
maxTime = 5 * 60

goal_x = np.random.uniform(100, 300, 1)
goal_y = np.random.uniform(100, 300, 1)

marker_pos = (0, 0)


def goal_pose():
    goal_pose = Odometry()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = 'world'

    # TODO goal limits are hardcoded for now, need to change them at random
    goal_pose.pose.pose.position = Point(goal_x, goal_y, 0.)
    return goal_pose


def marker_msg():
    global goal_x, goal_y
    global marker_pos

    x = goal_x
    y = goal_y

    if marker_pos[0] != x or marker_pos[1] != y:
        marker_pos = (x, y)

        msg = ModelState()
        msg.model_name = "waypoint_marker"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 1
        msg.pose.orientation.w = 1
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0

        return msg

    else:
        return None


if __name__ == '__main__':
    global proc
    pub = rospy.Publisher('move_usv/goal', Odometry, queue_size=10)
    pub_marker = rospy.Publisher('/gazebo/set_model_state',
                                 ModelState,
                                 queue_size=10)
    rospy.init_node('patrol')
    rate = rospy.Rate(1)  # 10h
    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.wait_for_service('/gazebo/pause_physics')
    rospy.wait_for_service('/gazebo/reset_simulation')
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    resetSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    unpause()

    simulationNumber = 1
    while not rospy.is_shutdown():
        try:
            goal = goal_pose()
            pub.publish(goal)

            # Only move the marker if the target has changed, otherwise jitters
            mm = marker_msg()
            if mm is not None:
                pub_marker.publish(mm)

            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr("ROS InterruptException! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")
