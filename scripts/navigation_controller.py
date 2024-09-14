#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class NavigationController:
    def __init__(self):
        rospy.init_node('navigation_controller')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)

    def goal_callback(self, goal):
        rospy.loginfo("Received new goal")
        self.move_to_goal(goal)

    def move_to_goal(self, goal_pose):
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose

        self.client.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)
        
        # 获取规划路径并发布
        plan = self.client.get_plan(goal)
        if plan:
            self.path_pub.publish(plan)

    def done_callback(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully")
        else:
            rospy.logwarn("Goal failed with status: %s", str(status))

    def feedback_callback(self, feedback):
        # 可以添加更多的反馈处理逻辑
        pass

if __name__ == '__main__':
    try:
        navigator = NavigationController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation controller terminated")