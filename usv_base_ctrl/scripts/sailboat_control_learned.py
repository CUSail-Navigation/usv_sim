#!/usr/bin/env python3
# license removed for brevity

import rospy
import math
import tf
import numpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Point, Quaternion
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
import torch
from torch import nn
import os

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

T = 1  # run the nav algo every T seconds
last_run = -math.inf  # last time the nav algo was run
sim_time = 0
sail_angle = 0
rud_angle = 0
marker_pos = (0, 0)

actor = None
prev_x = None
prev_y = None
dec_obs = 2


def get_pose(initial_pose_tmp):
    global initial_pose, prev_x, prev_y
    initial_pose = initial_pose_tmp

    if prev_x is None or prev_y is None:
        prev_x = initial_pose.pose.pose.position.x
        prev_y = initial_pose.pose.pose.position.y


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
    global initial_pose, prev_x, prev_y
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

    odom = initial_pose
    base_position = odom.pose.pose.position
    base_orientation_quat = odom.pose.pose.orientation

    vel_x = (base_position.x - prev_x) / T if prev_x is not None else 0.0
    vel_y = (base_position.y - prev_y) / T if prev_y is not None else 0.0

    prev_x = base_position.x
    prev_y = base_position.y

    base_roll, base_pitch, base_yaw = get_orientation_euler(
        base_orientation_quat)
    base_speed_angular_yaw = odom.twist.twist.angular.z

    wind_rel_x, wind_rel_y = relative_wind_unit_vector(base_yaw)

    distance_x, distance_y = get_distance_from_goal(base_position)
    target_distance = numpy.sqrt(distance_x**2 + distance_y**2)

    observation = []
    observation.append(round(vel_x, dec_obs))
    observation.append(round(vel_y, dec_obs))
    observation.append(round(base_speed_angular_yaw, dec_obs))

    observation.append(round(sail_angle, dec_obs))
    observation.append(round(rud_angle, dec_obs))

    observation.append(round(wind_rel_x, dec_obs))
    observation.append(round(wind_rel_y, dec_obs))

    observation.append(round(distance_x, dec_obs))
    observation.append(round(distance_y, dec_obs))

    if (sim_time - last_run) < T:
        return math.radians(sail_angle), math.radians(rud_angle)

    last_run = sim_time

    a = actor.get_action(observation)
    sail_angle = a[0] * 90.0
    rud_angle = a[1] * 30.0

    log_msg = "boat: ({}, {}), target: ({}, {}), target dist: {}, sail angle: {}, rud angle: {}".format(
        prev_x, prev_y, target_pose.pose.pose.position.x,
        target_pose.pose.pose.position.y, target_distance, sail_angle,
        rud_angle)
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


def get_orientation_euler(quaternion_vector):
    # We convert from quaternions to euler
    orientation_list = [
        quaternion_vector.x, quaternion_vector.y, quaternion_vector.z,
        quaternion_vector.w
    ]

    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    return roll, pitch, yaw


def relative_wind_unit_vector(yaw):
    # all in radians
    x = rospy.get_param('/uwsim/wind/x')
    y = rospy.get_param('/uwsim/wind/y')
    abs_angle = numpy.arctan2(y, x)
    rel_angle = (abs_angle - yaw) % (2 * numpy.pi)

    # make unit vector
    x = numpy.cos(rel_angle)
    y = numpy.sin(rel_angle)
    return x, y


def get_distance_from_goal(current_position):
    goal_x = target_pose.pose.pose.position.x
    goal_y = target_pose.pose.pose.position.y

    displacement = numpy.array(
        [goal_x - current_position.x, goal_y - current_position.y])

    return displacement[0], displacement[1]


if __name__ == '__main__':
    try:
        # path to load from
        result_dir = rospy.get_param('/sailboat/training/path_to_results')

        # Existing actor model to use
        load_actor = rospy.get_param(
            '/sailboat/training/load_models/load_actor')

        hidden_size1 = rospy.get_param("/sailboat/training/hidden_size1")
        hidden_size2 = rospy.get_param("/sailboat/training/hidden_size2")

        # Use GPU if possible
        device = None
        if torch.cuda.is_available():
            device = torch.device("cuda:0")
            rospy.loginfo("Using device {}".format(
                torch.cuda.get_device_name(0)))
        else:
            rospy.logwarn("No GPU detected. Using CPU is not recommended.")
            device = torch.device("cpu")

        class Actor(nn.Module):

            def __init__(self,
                         state_dim=9,
                         action_dim=2,
                         h1=hidden_size1,
                         h2=hidden_size2,
                         init_w=0.003):
                super(Actor, self).__init__()

                self.linear1 = nn.Linear(state_dim, h1)
                self.ln1 = nn.LayerNorm(h1)

                self.linear2 = nn.Linear(h1, h2)
                self.ln2 = nn.LayerNorm(h2)

                self.linear3 = nn.Linear(h2, action_dim)
                self.linear3.weight.data.uniform_(-init_w, init_w)

                self.relu = nn.ReLU()
                self.tanh = nn.Tanh()

            def forward(self, state):
                x = self.linear1(state)
                x = self.ln1(x)
                x = self.relu(x)

                x = self.linear2(x)
                x = self.ln2(x)
                x = self.relu(x)

                x = self.linear3(x)
                x = self.tanh(x)
                return x

            def get_action(self, state):
                state = torch.FloatTensor(state).unsqueeze(0).to(device)
                action = self.forward(state)
                return action.detach().cpu().numpy()[0]

        actor = Actor().to(device)
        actor.eval()

        if load_actor != "":
            actor.load_state_dict(
                torch.load(os.path.join(result_dir, load_actor)))
        else:
            rospy.logerr("No actor provided. Using random.")

        talker_ctrl()
    except rospy.ROSInterruptException:
        pass
