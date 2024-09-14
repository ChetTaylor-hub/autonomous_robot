#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

class ArmController:
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        rospy.init_node('arm_controller', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.command_sub = rospy.Subscriber('/arm_command', String, self.command_callback)

    def command_callback(self, msg):
        if msg.data == "pick":
            self.pick_object()
        elif msg.data == "place":
            self.place_object()
        elif msg.data == "home":
            self.return_to_home()

    def pick_object(self):
        rospy.loginfo("Picking object")
        # 这里应该实现抓取物体的具体逻辑
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.0
        pose_goal.position.z = 0.2
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.loginfo("Object picked")

    def place_object(self):
        rospy.loginfo("Placing object")
        # 这里应该实现放置物体的具体逻辑
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.2
        pose_goal.position.y = 0.2
        pose_goal.position.z = 0.3
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.loginfo("Object placed")

    def return_to_home(self):
        rospy.loginfo("Returning to home position")
        self.move_group.set_named_target("home")
        self.move_group.go(wait=True)
        self.move_group.stop()
        rospy.loginfo("Returned to home position")

if __name__ == '__main__':
    arm_controller = ArmController()
    rospy.spin()