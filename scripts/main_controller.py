#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from autonomous_robot.msg import ObjectDetection

class MainController:
    def __init__(self):
        rospy.init_node('main_controller')
        self.object_locations = {}
        self.current_pose = None
        self.target_object = None

        # 订阅话题
        rospy.Subscriber('/detected_objects', ObjectDetection, self.object_callback)
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)

        # 发布话题
        self.nav_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.arm_pub = rospy.Publisher('/arm_command', String, queue_size=10)

    def object_callback(self, data):
        self.object_locations[data.class_id] = data.pose
        rospy.loginfo(f"Detected object: {data.class_id} at position: ({data.pose.pose.position.x}, {data.pose.pose.position.y}, {data.pose.pose.position.z})")

    def pose_callback(self, data):
        self.current_pose = data

    def speech_recognition(self):
        # 这里是语音识别的空函数
        recognized_text = "找到苹果"  # 模拟识别结果
        return recognized_text

    def navigate_to_object(self, object_class):
        if object_class in self.object_locations:
            rospy.loginfo(f"Navigating to {object_class}")
            self.nav_pub.publish(self.object_locations[object_class])
        else:
            rospy.loginfo(f"Object {object_class} not found. Searching...")
            # 实现搜索策略

    def pick_object(self):
        rospy.loginfo("Picking object")
        self.arm_pub.publish("pick")

    def return_to_home(self):
        rospy.loginfo("Returning to home")
        home_pose = PoseStamped()
        home_pose.header.frame_id = "map"
        home_pose.pose.position.x = 0
        home_pose.pose.position.y = 0
        home_pose.pose.orientation.w = 1
        self.nav_pub.publish(home_pose)

    def run(self):
        while not rospy.is_shutdown():
            command = self.speech_recognition()
            self.target_object = command.split()[-1]  # 假设命令格式为"找到 [物体]"
            rospy.loginfo(f"Received command: {command}")
            
            self.navigate_to_object(self.target_object)
            rospy.sleep(5)  # 等待导航完成
            
            self.pick_object()
            rospy.sleep(3)  # 等待抓取完成
            
            self.return_to_home()
            rospy.sleep(5)  # 等待返回完成

if __name__ == '__main__':
    try:
        controller = MainController()
        controller.run()
    except rospy.ROSInterruptException:
        pass