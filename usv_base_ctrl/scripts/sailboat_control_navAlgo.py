#!/usr/bin/env python3
# license removed for brevity

import rospy
import math
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Float64
from navigation_utilities import *

initial_pose = Odometry()
target_pose = Odometry()
target_distance = math.inf
rate_value = 10
result = Float64()
result.data = 0
windDir = Float64()
windDir.data = 1.5
currentHeading = Float64()
currentHeading.data = 0
f_distance = 2
current_heading = 0

T = 4  # run the nav algo every T seconds
last_run = -math.inf  # last time the nav algo was run
sim_time = 0
sail_angle = 0
rud_angle = 0
marker_pos = (0, 0)


def get_pose(initial_pose_tmp):
    global initial_pose
    initial_pose = initial_pose_tmp


def get_target(target_pose_tmp):
    global target_pose
    target_pose = target_pose_tmp


def get_time(time_tmp):
    global sim_time
    sim_time = time_tmp.clock.to_sec()


def verify_result():
    global target_distance
    global result
    global f_distance
    if target_distance < f_distance:
        result.data = 1
    if target_distance >= f_distance:
        result.data = 0
    return result


def angle_saturation(sensor):
    if sensor > 180:
        sensor = sensor - 360
    if sensor < -180:
        sensor = sensor + 360
    return sensor


def talker_ctrl():
    global rate_value
    global currentHeading
    global windDir
    global isTacking
    global heeling
    global spHeading

    rospy.init_node('usv_simple_ctrl', anonymous=True)
    rate = rospy.Rate(rate_value)  # 10h
    pub_rudder = rospy.Publisher('joint_setpoint', JointState, queue_size=10)
    pub_result = rospy.Publisher('move_usv/result', Float64, queue_size=10)

    pub_marker = rospy.Publisher('/gazebo/set_model_state',
                                 ModelState,
                                 queue_size=10)

    # subscribe to state and target point topics
    rospy.Subscriber("state", Odometry, get_pose)
    rospy.Subscriber("move_usv/goal", Odometry,
                     get_target)  # get target position
    rospy.Subscriber("/clock", Clock, get_time)

    while not rospy.is_shutdown():
        try:
            pub_rudder.publish(sail_rudder_ctrl_msg())
            pub_result.publish(verify_result())

            # Only move the marker if the target has changed, otherwise jitters
            mm = marker_msg()
            if mm is not None:
                pub_marker.publish(mm)

            rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")


def sail_rudder_ctrl():
    global initial_pose
    global target_pose
    global target_distance
    global current_heading
    global currentHeading
    global heeling

    global sail_angle
    global rud_angle
    global sim_time
    global last_run
    global T

    x1 = initial_pose.pose.pose.position.x
    y1 = initial_pose.pose.pose.position.y
    x2 = target_pose.pose.pose.position.x
    y2 = target_pose.pose.pose.position.y

    target_distance = math.hypot(x2 - x1, y2 - y1)

    if (sim_time - last_run) < T:
        return math.radians(sail_angle), math.radians(rud_angle)

    last_run = sim_time

    x = rospy.get_param('/uwsim/wind/x')
    y = rospy.get_param('/uwsim/wind/y')
    abs_wind_dir = math.degrees(math.atan2(y, x))

    quaternion = (initial_pose.pose.pose.orientation.x,
                  initial_pose.pose.pose.orientation.y,
                  initial_pose.pose.pose.orientation.z,
                  initial_pose.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    target_angle = math.degrees(euler[2])
    target_angle = angle_saturation(target_angle)
    target_angle = -target_angle
    current_heading = math.radians(target_angle)
    currentHeading.data = current_heading

    boat_position = (x1, y1)
    target_position = (x2, y2)
    angle_boat_heading = math.degrees(currentHeading.data)

    intended_angle = newSailingAngleImpl(boat_position, target_position,
                                         angle_boat_heading, abs_wind_dir)

    sail_angle, rud_angle = getServoAnglesImpl(abs_wind_dir,
                                               angle_boat_heading,
                                               intended_angle)

    log_msg = "boat: ({}, {}), target: ({}, {}), target dist: {}, abs wind: {}, heading: {}, intended angle: {}, sail angle: {}, rud angle: {}".format(
        x1, y1, x2, y2, target_distance, abs_wind_dir, angle_boat_heading,
        intended_angle, sail_angle, rud_angle)
    rospy.loginfo(log_msg)

    return math.radians(sail_angle), math.radians(rud_angle)


def sail_rudder_ctrl_msg():
    msg = JointState()
    msg.header = Header()
    msg.name = ['rudder_joint', 'sail_joint']
    sail, rud = sail_rudder_ctrl()
    msg.position = [rud, sail]
    msg.velocity = []
    msg.effort = []

    return msg


def marker_msg():
    global target_pose
    global marker_pos

    x = target_pose.pose.pose.position.x
    y = target_pose.pose.pose.position.y

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
    try:
        talker_ctrl()
    except rospy.ROSInterruptException:
        pass
