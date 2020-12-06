#!/usr/bin/env python3

import rospy
import actionlib

from turtlesim.msg import Pose
from turtlesim_msgs.msg import OrientationAction
from turtlesim_msgs.msg import OrientationGoal

class PoseListener:

    def __init__(self):
        self._ac = actionlib.SimpleActionClient("/orientation", OrientationAction)
        self._ac.wait_for_server()
        self.polaris_pose_subscriber = rospy.Subscriber('/polaris/pose', Pose, self.handle_polaris_pose)
        rospy.loginfo("Orientation action client has been started ...")

    def handle_polaris_pose(self, data):

        goal_pose = Pose()
        goal_pose.x = data.x
        goal_pose.y = data.y
      
        goal = OrientationGoal()
        goal.referencePose.x= goal_pose.x
        goal.referencePose.y= goal_pose.y

        rospy.loginfo("Polaris has change the pose ...")
        self._ac.send_goal(goal, done_cb=self.done_callback)
        rospy.loginfo("Polaris pose sended to server ...")


    def done_callback(self, status, result):
        rospy.loginfo("Result: " + str(result.status))


if __name__ == '__main__':

    rospy.init_node('pose_listener')
    server = PoseListener()
    rospy.spin()