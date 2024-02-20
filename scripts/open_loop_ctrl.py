#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose
import tf_conversions
 
# Setup Variables to be used
first = True
start_time = 0.0
current_time = 0.0
last_time = 0.0
state = 0
count = 0

controlOutput = Twist()  

#Stop Condition
def stop():
  #Setup the stop message (can be the same as the control message)
    controlOutput.linear.x = 0.0
    controlOutput.linear.y = 0.0
    controlOutput.linear.z = 0.0
    controlOutput.angular.x = 0.0
    controlOutput.angular.y = 0.0
    controlOutput.angular.z = 0.0

    control_pub.publish(controlOutput)
    total_time = rospy.get_time()- start_time
    print("Stopping", " ", "Total Time", "  ", total_time)


if __name__=='__main__':

    #Initialise and Setup node
    rospy.init_node("Open_Loop_Ctrl")

    #Set the parameters of the Controller
    #Controller Speeds
    linearSpeed = rospy.get_param("~linear_speed",0.3)
    angularSpeed = rospy.get_param("~angular_speed",0.3)
    
    #Shape parameters
    _corners = rospy.get_param("~corners",4)
    _edgeLenght = rospy.get_param("~edgeLenght",0.5)
    _node_rate = rospy.get_param("~node_rate",100)
    
    # Configure the Node
    loop_rate = rospy.Rate(_node_rate)
    rospy.on_shutdown(stop)


    control_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    print("The Open Loop Control is Running")    

    try:
        while not rospy.is_shutdown():

            if first == True:
                start_time = rospy.get_time() 
                last_time = rospy.get_time()
                current_time = rospy.get_time()
                first = False
            
            else:
                current_time = rospy.get_time()
                time = current_time - last_time
                print(count)

                # State 0 = moving along a straight line
                if state == 0:
                    print("Straight")
                    controlOutput.linear.x = linearSpeed
                    controlOutput.angular.z = 0.0
                    # If at end of a side
                    if time > _edgeLenght/linearSpeed:
                        #Reset time and go to corner state
                        last_time = rospy.get_time() 
                        state = 1

                
                
                # State 1 = Turning a corner
                elif state == 1:
                    print("Turn")
                    controlOutput.linear.x = 0.0
                    controlOutput.angular.z = angularSpeed
                    # If finished turning through 90 degrees
                    if time > (((2*np.pi)/_corners)/angularSpeed):
                        last_time = rospy.get_time()   
                    # If we have not finished the square
                        if count < _corners-1:
                        # go to corner and count how many sides are done
                            state = 0
                            count += 1
                        # otherwise we have finished a square
                        else:
                            #stop
                            state = 2

                
                # State 2 = Motion completed
                elif state == 2:
                    # Stop and signal
                    controlOutput.linear.x = 0.0
                    controlOutput.angular.z = 0.0
                    print("Motion Completed")
                    rospy.signal_shutdown("Figure Completed")
                    # If invalid state, stop
                
                else:
                    controlOutput.linear.x = 0.0
                    controlOutput.angular.z = 0.0

                # Publish message
                    
                control_pub.publish(controlOutput) 

            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass