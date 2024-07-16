#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
#from controller_manager_msgs.srv import *

class PuppeteerRelay:

    def __init__(self):
        self.real_state_publisher = rospy.Publisher("/puppeteer/real_robot_state", Pose, queue_size=1)
        self.real_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.relay_real)
        self.ar_state_subscriber = rospy.Subscriber("/puppeteer/joints", Pose, self.puppeteer)
        self.ar_state_publisher = rospy.Publisher("/ar_teleop_franka_controller/panda/target_joint_positions",PoseStamped, queue_size=1 )

    def relay_real(self, msg):
        real_pose = Pose()
        real_pose.position.x = msg.position[0]
        real_pose.position.y = msg.position[1]
        real_pose.position.z = msg.position[2]
        real_pose.orientation.x = msg.position[3]
        real_pose.orientation.y = msg.position[4]
        real_pose.orientation.z = msg.position[5]
        real_pose.orientation.w = msg.position[6]
        self.real_state_publisher.publish(real_pose)

    def puppeteer(self, msg):
        ar_pose = PoseStamped()
        ar_pose.pose = msg
        self.ar_state_publisher.publish(ar_pose) 

    def puppeteer_ee(self, msg):
        twist = Twist()
        twist.linear.x = msg.position.x
        twist.linear.y = msg.position.y
        twist.linear.z = msg.position.z
        twist.angular.x = msg.orientation.x
        twist.angular.y = msg.orientation.y
        twist.angular.z = msg.orientation.z
        self.ar_state_publisher.publish(twist)  

if __name__ == "__main__":
    rospy.init_node("ar_puppeteer_franka")
    #rate = rospy.Rate(50)
    relay = PuppeteerRelay()
    rospy.spin()