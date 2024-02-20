#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
import tf_conversions

# Setup Variables to be used
first = True
start_time = 0.0
last_time =  0.0
current_time = 0.0

x_dot = 0.0
y_dot = 0.0
omega = 0.0
x_dot_n = 0.0
y_dot_n = 0.0
omega_n = 0.0
wheel_wl = 0.0
wheel_wr = 0.0
wheel_wl_n = 0.0
wheel_wr_n = 0.0

# Declare Messages to be used
cmd_vel = Twist()
robot_vel = Twist()
pose_sim = PoseStamped()
wr = Float32()
wl = Float32()

#Initialise messages (if required)
def init_wheel_speed():
    wr.data = 0.0
    wl.data = 0.0

def init_pose():
    #Transform the angle into a quaternion for the orientation
    pos_orient = tf_conversions.transformations.quaternion_from_euler(0,0,pos_th)
    pose_sim.header.frame_id = _pose_frame
    pose_sim.header.stamp = rospy.Time.now()
    pose_sim.pose.position.x = pos_x
    pose_sim.pose.position.y = pos_y
    pose_sim.pose.position.z = 0.0
    pose_sim.pose.orientation = Quaternion(pos_orient[0], pos_orient[1], pos_orient[2], pos_orient[3])

def init_robot_vel():
    #Declare the velocity output Message
    robot_vel.linear.x = 0.0
    robot_vel.linear.y = 0.0
    robot_vel.linear.z = 0.0
    robot_vel.angular.x = 0.0
    robot_vel.angular.y = 0.0
    robot_vel.angular.z = 0.0

def init_cmd_vel():
    # Declare the input Message
    cmd_vel.linear.x = 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0
    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = 0.0

    #Define the callback functions (if required)
def cmd_callback(msg):
    global cmd_vel
    cmd_vel = msg

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")
    wr.data = 0.0
    wl.data = 0.0
    wr_pub.publish(wr)
    wl_pub.publish(wl)

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Noholonomic_robot_sim")
 
    #Parameters
    #Set the parameters of the system
    _wheel_r = rospy.get_param("~wheel_radius",0.05)
    _robot_l = rospy.get_param("~robot_wheelbase",0.18)

    #Set initial conditions of the system
    pos_x = rospy.get_param("~pos_x0",0.00)
    pos_y = rospy.get_param("~pos_y0",0.00)
    pos_th = rospy.get_param("~pos_th0",0.00)

    #Simulation Parameters
    _pose_frame = rospy.get_param("~pose_frame","world")
    _sample_time = rospy.get_param("~sample_time",0.01)
    _node_rate = rospy.get_param("~node_rate",100)
    _noise = rospy.get_param("~noise", False)
    _noise_kr = rospy.get_param("~noise_kr", 0.01)
    _noise_kl = rospy.get_param("~noise_kl", 0.01)
    _noise_kv = rospy.get_param("~noise_kv", 0.02)
    _noise_kw = rospy.get_param("~noise_kw", 0.02)

    # Configure the Node
    loop_rate = rospy.Rate(_node_rate)
    rospy.on_shutdown(stop)

    #Init messages to be used
    init_cmd_vel()
    init_wheel_speed()
    init_pose()
    init_robot_vel()

    # Setup the Subscribers
    rospy.Subscriber("cmd_vel", Twist, cmd_callback)

    #Setup de publishers
    wr_pub = rospy.Publisher("wr", Float32, queue_size=1)
    wl_pub = rospy.Publisher("wl", Float32, queue_size=1)
    pose_pub = rospy.Publisher("robot_pose", PoseStamped, queue_size=1)
    vel_pub = rospy.Publisher("robot_vel", Twist, queue_size=1)

    #Node Running
    print("The Robot Simulator is Running")

    try:
    #Run the node
        while not rospy.is_shutdown():
            if first == True:
                start_time = rospy.get_time()
                last_time =  rospy.get_time()
                current_time = rospy.get_time()
                first = False
            else: 
                current_time = rospy.get_time()
                dt = current_time - last_time

                if dt >= _sample_time:

                    #Velocity Estimation
                    x_dot = cmd_vel.linear.x * np.cos(pos_th)
                    y_dot = cmd_vel.linear.x * np.sin(pos_th)
                    omega = cmd_vel.angular.z

                    #Pose estimation
                    pos_x += dt * x_dot
                    pos_y += dt * y_dot
                    pos_th += dt * omega

                    #Wheel speeds
                    wheel_wr = (cmd_vel.linear.x + _robot_l * cmd_vel.angular.z / 2.0) / _wheel_r
                    wheel_wl = (cmd_vel.linear.x - _robot_l * cmd_vel.angular.z / 2.0) / _wheel_r

                    if _noise == True:
                        wheel_wr_n = wheel_wr + np.random.normal(0, _noise_kr * np.abs(wheel_wr))
                        wheel_wl_n = wheel_wl + np.random.normal(0, _noise_kl * np.abs(wheel_wl))

                        x_dot_n = x_dot + np.random.normal(0, _noise_kv * np.abs(x_dot))
                        y_dot_n = y_dot + np.random.normal(0, _noise_kv * np.abs(y_dot))
                        omega_n = omega + np.random.normal(0, _noise_kw * np.abs(omega))
                    else: 
                        wheel_wr_n = wheel_wr
                        wheel_wl_n = wheel_wl
                        x_dot_n = x_dot
                        y_dot_n = y_dot
                        omega_n = omega


                    #Fill The messages to be published

                    #Fill the Velocities
                    robot_vel.linear.x = x_dot_n
                    robot_vel.linear.y = y_dot_n
                    robot_vel.angular.z = omega_n

                    #Fill the estimated pose message
                    pos_orient = tf_conversions.transformations.quaternion_from_euler(0,0,pos_th)
                    pose_sim.header.stamp = rospy.Time.now()
                    pose_sim.pose.position.x = pos_x
                    pose_sim.pose.position.y = pos_y
                    pose_sim.pose.orientation = Quaternion(pos_orient[0], pos_orient[1], pos_orient[2], pos_orient[3])
                    
                    #Fill the wheel speed message
                    wr.data = wheel_wr_n
                    wl.data = wheel_wl_n

                    last_time = rospy.get_time()

                    #Publish messages
                    wr_pub.publish(wr)
                    wl_pub.publish(wl)
                    pose_pub.publish(pose_sim)
                    vel_pub.publish(robot_vel)

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass