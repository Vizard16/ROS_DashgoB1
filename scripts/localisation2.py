#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf_conversions
from tf2_ros import TransformBroadcaster

# Setup Variables to be used
first = True
start_time = 0.0
last_time =  0.0
current_time = 0.0

k_r = 0.01
k_l = 0.01

Vel = 0.0
Omega = 0.0

wr_n = 0.0
wl_n = 0.0

x_dot = 0.0
y_dot = 0.0
th_dot = 0.0

# Declare Messages to be used
wr = Float32()
wl = Float32()
calc_odom = Odometry()
robot_odom = Odometry()
robot_tf = TransformStamped()



#Initialise messages (if required)

def init_tf():
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, pos_th)
    #Define Transformations
    robot_tf.header.stamp = rospy.Time.now()
    robot_tf.header.frame_id = _odom_frame
    robot_tf.child_frame_id = _base_frame
    robot_tf.transform.translation.x = pos_x
    robot_tf.transform.translation.y = pos_y
    robot_tf.transform.translation.z = 0.0
    robot_tf.transform.rotation.x = q[0]
    robot_tf.transform.rotation.y = q[1]
    robot_tf.transform.rotation.z = q[2]
    robot_tf.transform.rotation.w = q[3]

def init_calc_odom():
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, pos_th)
    calc_odom.header.stamp = rospy.Time.now()
    calc_odom.header.frame_id = _odom_frame
    calc_odom.child_frame_id = _base_frame
    calc_odom.pose.pose.position.x = pos_x
    calc_odom.pose.pose.position.y = pos_y
    calc_odom.pose.pose.position.z = 0.0
    calc_odom.pose.pose.orientation.x = q[0]
    calc_odom.pose.pose.orientation.y = q[1]
    calc_odom.pose.pose.orientation.z = q[2]
    calc_odom.pose.pose.orientation.w = q[3]
    calc_odom.pose.covariance = [0]*36
    calc_odom.twist.twist.linear.x = 0.0
    calc_odom.twist.twist.linear.y = 0.0
    calc_odom.twist.twist.linear.z = 0.0
    calc_odom.twist.twist.angular.x = 0.0
    calc_odom.twist.twist.angular.y = 0.0
    calc_odom.twist.twist.angular.z = 0.0
    calc_odom.twist.covariance = [0]*36

#Define the callback functions (if required)
#Left Wheel Callback Function
def odom_callback(msg):
    global robot_odom
    robot_odom = msg


#wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("EAI_Robot_Sim")

    #Parameters
    #Set initial conditions of the system
    pos_x = rospy.get_param("~pos_x0",0.00)
    pos_y = rospy.get_param("~pos_y0",0.00)
    pos_th = rospy.get_param("~pos_th0",0.00)

    #Simulation Parameters
    _sample_time = rospy.get_param("~sample_time",0.01)
    _node_rate = rospy.get_param("~node_rate",100)

    #Localisation Frames
    _base_frame = rospy.get_param("~base_frame","base_link")
    _odom_frame = rospy.get_param("~odom_frame","odom_calc")


    # Configure the Node
    loop_rate = rospy.Rate(_node_rate)
    rospy.on_shutdown(stop)

    #Init joints
    init_tf()
    init_calc_odom()

    #Setup Publishers/Subscribers and Transform Broadcasters
    odom_pub = rospy.Publisher('calc_odom', Odometry, queue_size=1)
    
    rospy.Subscriber("odom", Odometry, odom_callback)
    #rospy.Subscriber("wl", Float32, wl_callback)
    
    tf_broadcaster = TransformBroadcaster()

    print("The Localisation is Running")

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

                    #Robot Velocity estimation
                    Vel = robot_odom.twist.twist.linear.x
                    Omega = robot_odom.twist.twist.angular.z

                    #Velocity Estimation
                    x_dot = Vel * np.cos(pos_th)
                    y_dot = Vel * np.sin(pos_th)
                    th_dot= Omega

                    #Pose estimation
                    pos_x += x_dot * dt
                    pos_y += y_dot * dt
                    pos_th = wrap_to_Pi(pos_th + th_dot * dt)

                    #Update last time
                    last_time = rospy.get_time()

                    #Fill the messages/tf/joints to be published 

                    #Transform angles to quaternions
                    q = tf_conversions.transformations.quaternion_from_euler(0, 0, pos_th)

                    calc_odom.header.stamp = rospy.Time.now()
                    calc_odom.pose.pose.position.x = pos_x
                    calc_odom.pose.pose.position.y = pos_y
                    calc_odom.pose.pose.position.z = 0.0
                    calc_odom.pose.pose.orientation.x = q[0]
                    calc_odom.pose.pose.orientation.y = q[1]
                    calc_odom.pose.pose.orientation.z = q[2]
                    calc_odom.pose.pose.orientation.w = q[3]
                    calc_odom.twist.twist.linear.x = Vel
                    calc_odom.twist.twist.angular.z = Omega

                    #Define Transformations/Joints
                    robot_tf.header.stamp = rospy.Time.now()
                    robot_tf.transform.translation.x = calc_odom.pose.pose.position.x
                    robot_tf.transform.translation.y = calc_odom.pose.pose.position.y
                    robot_tf.transform.translation.z = 0.0
                    robot_tf.transform.rotation.x = calc_odom.pose.pose.orientation.x
                    robot_tf.transform.rotation.y = calc_odom.pose.pose.orientation.y
                    robot_tf.transform.rotation.z = calc_odom.pose.pose.orientation.z
                    robot_tf.transform.rotation.w = calc_odom.pose.pose.orientation.w

                    #Publish messages
                    odom_pub.publish(calc_odom)
                    tf_broadcaster.sendTransform(robot_tf)

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass