#!/usr/bin/env python3

#import anypackages you need
import rospy
import math


#import any ROS messages you need
from std_msgs.msg import Float64

#create a function where all the action happens
def messenger():
    #set up and publisers you want along with creating the ROS node
    j1pub = rospy.Publisher('/rrbot/joint_main_mid_position_controller/command', Float64, queue_size=10)
    j2pub = rospy.Publisher('/rrbot/joint_mid_top_position_controller/command', Float64, queue_size=10)
    j3pub = rospy.Publisher('/rrbot/left_gripper_joint_position_controller/command', Float64, queue_size=10)
    j4pub = rospy.Publisher('/rrbot/right_gripper_joint_position_controller/command', Float64, queue_size=10)
    j5pub = rospy.Publisher('/rrbot/joint_rot_main_position_controller/command', Float64, queue_size=10)

    i = 0
    rospy.init_node('messenger', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    #this standard ROS construct allows us to loop only when ROS is still active
    while not rospy.is_shutdown():
        j = math.sin(0.3*i)
        i += 1
        rospy.loginfo(j)
        j1pub.publish(j)
        j2pub.publish(j)
        j3pub.publish(j)
        j4pub.publish(j)
        j5pub.publish(j)
        rate.sleep()

#This allows the node to exit gracefully if ROS is shut down
if __name__ == '__main__':
    try:
        messenger()
    except rospy.ROSInterruptException:
        pass