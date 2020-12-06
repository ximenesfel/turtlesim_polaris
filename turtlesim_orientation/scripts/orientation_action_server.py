#!/usr/bin/env python3

import rospy
import actionlib
import math

from geometry_msgs.msg import Twist
from turtlesim_msgs.msg import OrientationAction
from turtlesim_msgs.msg import OrientationResult
from turtlesim.srv import Spawn
from turtlesim.msg import Pose

class OrientationServer:

    def __init__(self):
        self._as = actionlib.SimpleActionServer('orientation', OrientationAction, execute_cb=self.on_goal, auto_start=False)
        self._as.start()
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        rospy.loginfo("Orientation action server has been started ...")
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
    
    def on_goal(self, goal):

        rospy.loginfo("[OrientationServer] Pose received {} ...".format(goal))

        goal_pose = Pose()
        goal_pose.x = goal.referencePose.x
        goal_pose.y = goal.referencePose.y

        vel_msg = Twist()

        while round(self.steering_angle(goal_pose),2) != round(self.pose.theta,2):

            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        result = OrientationResult()
        result.status = "Tutlesim aligned with Polaris"
        self._as.set_succeeded(result)
            

if __name__ == '__main__':

    rospy.init_node('orientation_server')
    server = OrientationServer()
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', Spawn)
    spawner(rospy.get_param('polarisPoseX'), rospy.get_param('polarisPoseY'), 0, 'polaris')
    rospy.spin()