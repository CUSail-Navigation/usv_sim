#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist, Point, Quaternion
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty
import rosbag
import os
import numpy as np
from navigation_utilities import *

result = Float64()
result.data = 0
maxSimulations = 1
maxTime = 5 * 60

workspace_x_max = 0
workspace_x_min = 0
workspace_y_max = 0
workspace_y_min = 0

goal_x = 0.0
goal_y = 0.0

marker_pos = (0, 0)


def reset_yaw_and_goal(r):

    if r.data:
        randomize_goal()


def axis_angle_to_quaternion(ax, ay, az, angle):
    # angle in radians, unit vector of axis
    s = np.sin(angle / 2)
    qx = ax * s
    qy = ay * s
    qz = az * s
    qw = np.cos(angle / 2)
    return qx, qy, qz, qw


def goal_pose():
    goal_pose = Odometry()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = 'world'

    goal_pose.pose.pose.position = Point(goal_x, goal_y, 0.)
    return goal_pose


def marker_msg():
    global goal_x, goal_y
    global marker_pos

    x = goal_x
    y = goal_y

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


def randomize_goal():
    global goal_x, goal_y
    goal_x = np.random.uniform(workspace_x_min + 10, workspace_x_max - 10)
    goal_y = np.random.uniform(workspace_y_min + 10, workspace_y_max - 10)


if __name__ == '__main__':
    pub = rospy.Publisher('move_usv/goal', Odometry, queue_size=10)
    pub_model_state = rospy.Publisher('/gazebo/set_model_state',
                                      ModelState,
                                      queue_size=10)
    rospy.Subscriber('/sailboat/episode_reset', Bool, reset_yaw_and_goal)
    rospy.init_node('patrol')
    rate = rospy.Rate(0.5)  # 1Hz
    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.wait_for_service('/gazebo/pause_physics')
    rospy.wait_for_service('/gazebo/reset_simulation')
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    resetSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    workspace_x_max = rospy.get_param('/sailboat/training/work_space/x_max')
    workspace_x_min = rospy.get_param('/sailboat/training/work_space/x_min')
    workspace_y_max = rospy.get_param('/sailboat/training/work_space/y_max')
    workspace_y_min = rospy.get_param('/sailboat/training/work_space/y_min')
    randomize_goal()

    unpause()

    simulationNumber = 1
    while not rospy.is_shutdown():
        try:
            goal = goal_pose()
            pub.publish(goal)

            pub_model_state.publish(marker_msg())
            # Only move the marker if the target has changed, otherwise jitters
            # mm = marker_msg()
            # if mm is not None:
            #     pub_marker.publish(mm)

            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr("ROS InterruptException! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")
