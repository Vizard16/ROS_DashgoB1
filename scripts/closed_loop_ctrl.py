#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseStamped
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from tf.transformations import euler_from_quaternion

robot_control = 1
data_pub = rospy.Publisher("xarm_valor", Int32, queue_size=1)

# Setup Variables to be used
first = True
start_time = 0.0
current_time = 0.0
last_time = 0.0
Error = 0.0
Error_int = 0.0
angle_threshold = 0.1  # Adjust as needed

Error_pos = np.zeros([2])
E_d = 0.0
E_th = 0.0
E_d_int = 0.0
E_th_int = 0.0
control_Omega = 0.0
control_V = 0.0

# Setup the input/output Messages
controlOutput = Twist()
robot_odom = Odometry()
goals = []  # List to store multiple goals

# Define your goals as PoseStamped messages and add them to the 'goals' list
goal1 = PoseStamped()
goal1.pose.position.x = 3.5
goal1.pose.position.y = 0
goal1.pose.orientation.w = 0
goals.append(goal1)

goal2 = PoseStamped()
goal2.pose.position.x = 7
goal2.pose.position.y = 0
goal2.pose.orientation.w = 0
goals.append(goal2)

goal3 = PoseStamped()
goal3.pose.position.x = 8.0
goal3.pose.position.y = 1.0
goal3.pose.orientation.w = 1.0
goals.append(goal3)

goal4 = PoseStamped()
goal4.pose.position.x = 8
goal4.pose.position.y = 4
goal4.pose.orientation.w = 0
goals.append(goal4)

goal5 = PoseStamped()
goal5.pose.position.x = 7
goal5.pose.position.y = 5
goal5.pose.orientation.w = 1.0
goals.append(goal5)

goal6 = PoseStamped()
goal6.pose.position.x = 3
goal6.pose.position.y = 5
goal6.pose.orientation.w = 1.0
goals.append(goal6)

goal7 = PoseStamped()
goal7.pose.position.x = 1.5
goal7.pose.position.y = 3
goal7.pose.orientation.w = 1.0
goals.append(goal7)

goal8 = PoseStamped()
goal8.pose.position.x = 0
goal8.pose.position.y = 0
goal8.pose.orientation.w = 1.0
goals.append(goal8)

# Define a rotation goal
rotation_goal = PoseStamped()
rotation_goal.pose.position.x = goal8.pose.position.x
rotation_goal.pose.position.y = goal8.pose.position.y

# Calculate the desired orientation to face the opposite direction (180 degrees)
desired_yaw = np.pi
quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, desired_yaw)
rotation_goal.pose.orientation.x = quaternion[0]
rotation_goal.pose.orientation.y = quaternion[1]
rotation_goal.pose.orientation.z = quaternion[2]
rotation_goal.pose.orientation.w = quaternion[3]

# Add the rotation goal to the list of goals
goals.append(rotation_goal)

# ... Add more goals as needed

# Define a variable to track the current goal
current_goal_index = 0

# Define the callback functions
def odom_callback(msg):
    global robot_odom
    robot_odom = msg

def control_callback(msg):
    global robot_control
    robot_control = msg.data

# Wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi), (2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

# Stop Condition
def stop():
    # Setup the stop message (can be the same as the control message)
    controlOutput.linear.x = 0.0
    controlOutput.linear.y = 0.0
    controlOutput.linear.z = 0.0
    controlOutput.angular.x = 0.0
    controlOutput.angular.y = 0.0
    controlOutput.angular.z = 0.0

    control_pub.publish(controlOutput)
    total_time = rospy.get_time() - start_time
    print("Stopping", " ", "Total Time", "  ", total_time)

if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Position_Control")

    # Set the parameters of the Controller Node

    # Controller Gains
    _v_gains = rospy.get_param("~v_gains", {'p': 0.11, 'i': 0.0})
    _w_gains = rospy.get_param("~w_gains", {'p': 0.3, 'i': 0.0})

    # Radius around the goal
    _goal_radius = rospy.get_param("~goal_radius", 0.01)

    # Node Parameters
    _sample_time = rospy.get_param("~sample_time", 0.02)
    _node_rate = rospy.get_param("~node_rate", 100)

    # Goal and base frames
    _target_link = rospy.get_param("~base_frame", "base_link")
    _source_link = rospy.get_param("~goal_frame", "world")

    # Configure the Node
    loop_rate = rospy.Rate(_node_rate)
    rospy.on_shutdown(stop)

    # Setup the publishers
    control_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    # Setup the Subscribers
    rospy.Subscriber("odom", Odometry, odom_callback)

    rospy.Subscriber("Flag", Int32, control_callback)

    # Setup the Subscribers
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    print("The Control is Running")

    try:
        while not rospy.is_shutdown():
            if first:
                start_time = rospy.get_time()
                last_time = rospy.get_time()
                current_time = rospy.get_time()
                first = False
            else:
                current_time = rospy.get_time()
                dt = current_time - last_time

                if dt >= _sample_time:
                    try:
                        trans = tfBuffer.lookup_transform(_target_link, _source_link, rospy.Time())
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        loop_rate.sleep()
                        continue

                    if current_goal_index < len(goals):
                        goal = goals[current_goal_index]
                        goal_tf = tf2_geometry_msgs.do_transform_pose(goal, trans)
                        Error_pos[0] = goal_tf.pose.position.x
                        Error_pos[1] = goal_tf.pose.position.y

                        # Retrieve orientation information as Euler angles (yaw)
                        orientation = goal_tf.pose.orientation
                        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

                        # Check if the current goal is the last goal
                        if current_goal_index == len(goals) - 1:
                            # Handle the rotation goal
                            if np.abs(yaw - desired_yaw) > angle_threshold:
                                # Rotate to the desired_yaw
                                control_Omega = _w_gains['p'] * (desired_yaw - yaw)
                            else:
                                # Once the desired orientation is reached, stop rotating
                                control_Omega = 0.0
                                # Move to the next goal (the rotation goal)
                                current_goal_index += 1
                        else:
                            # Handle other goals (your existing logic)
                            E_d = np.sqrt(np.dot(Error_pos, Error_pos.T))
                            E_th = np.arctan2(Error_pos[1], Error_pos[0])
                            E_th = wrap_to_Pi(E_th)

                            if E_d < _goal_radius:
                                print(f"Reached Goal {current_goal_index}")
                                if current_goal_index == 0:
                                    
                                    data_pub.publish(1)

                                elif current_goal_index == 5:
                                    
                                    data_pub.publish(2)

                                control_V = 0.0
                                control_Omega = 0.0
                                # Move to the next goal
                                current_goal_index += 1
                            else:
                                E_d_int += E_d * dt
                                E_th_int += E_th * dt

                                control_V = _v_gains['p'] * E_d + _v_gains['i'] * E_d_int
                                control_Omega = _w_gains['p'] * E_th + _w_gains['i'] * E_th_int

                        last_time = rospy.get_time()

                        controlOutput.linear.x = control_V
                        controlOutput.linear.y = 0.0
                        controlOutput.linear.z = 0.0
                        controlOutput.angular.x = 0.0
                        controlOutput.angular.y = 0.0
                        controlOutput.angular.z = control_Omega


                        if robot_control== 1:
                            control_pub.publish(controlOutput)

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass
