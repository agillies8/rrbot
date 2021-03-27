#!/usr/bin/env python3
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math

class Commander:
    def __init__(self):
        self.j1_val=0
        self.j2_val=0
        self.j3_val=0
        self.j4_val=0
        self.j5_val=0
        self.target_seen = False

    def joint_positions(self, data):
        self.j1 = data.position[0]
        self.j2 = data.position[1]
        self.j3 = data.position[2]
        self.j4 = data.position[3]
        self.j5 = data.position[4]

    def callback(self, data):
        if data.detections:
            yaw_error = math.asin(data.detections[0].pose.pose.pose.position.x/data.detections[0].pose.pose.pose.position.z)
            pitch_error = math.asin(data.detections[0].pose.pose.pose.position.y/data.detections[0].pose.pose.pose.position.z)
            
            self.target_seen = True
            self.j1_val=self.j1
            self.j2_val=self.j2 + pitch_error
            self.j3_val=self.j3 - yaw_error
            self.j4_val=1
            self.j5_val=1
            rospy.loginfo("Yaw pose: %s, Pitch pose: %s", self.j3, self.j2)
            rospy.loginfo("Yaw error: %s, Pitch error: %s", yaw_error, pitch_error)
            rospy.loginfo("Yaw cmd: %s, Pitch cmd: %s", self.j3_val, self.j2_val)




        else:
            self.target_seen = False

    def command(self):

        i = 0
        rate = rospy.Rate(0.5) # 1hz

        while not rospy.is_shutdown():
            if not self.target_seen:
                i+=1
                j1pub.publish(-0.1)
                j2pub.publish(2)
                j3pub.publish(0.7+ math.sin(i/10))
                j4pub.publish(0)
                j5pub.publish(0)
               
                rate.sleep()
            else:
                #j1pub.publish(self.j1_val)
                j2pub.publish(self.j2_val)
                j3pub.publish(self.j3_val)
                #j4pub.publish(self.j4_val)
                #j5pub.publish(self.j5_val)
                rate.sleep()

if __name__ == '__main__':

    rospy.init_node('commander', anonymous=False)

    commander = Commander()

    rospy.Subscriber("tag_detections", AprilTagDetectionArray, commander.callback)
    rospy.Subscriber("/rrbot/joint_states", JointState, commander.joint_positions)

    #set up and publisers you want along with creating the ROS node
    j1pub = rospy.Publisher('/rrbot/joint_main_mid_position_controller/command', Float64, queue_size=10)
    j2pub = rospy.Publisher('/rrbot/joint_mid_top_position_controller/command', Float64, queue_size=10)
    j3pub = rospy.Publisher('/rrbot/joint_rot_main_position_controller/command', Float64, queue_size=10)    
    j4pub = rospy.Publisher('/rrbot/left_gripper_joint_position_controller/command', Float64, queue_size=10)
    j5pub = rospy.Publisher('/rrbot/right_gripper_joint_position_controller/command', Float64, queue_size=10)


    rate = rospy.Rate(1) # 1hz

    commander.command()