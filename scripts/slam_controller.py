#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class SLAMController:
    def __init__(self):
        rospy.init_node('slam_controller')
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
        self.current_pose = PoseStamped()
        
    def map_callback(self, map_data):
        # 处理地图数据
        rospy.loginfo("Map updated. Size: %d x %d, Resolution: %.2f", 
                      map_data.info.width, map_data.info.height, map_data.info.resolution)
    
    def odom_callback(self, odom_data):
        # 更新机器人位姿
        self.current_pose.header = odom_data.header
        self.current_pose.pose = odom_data.pose.pose
        self.pose_pub.publish(self.current_pose)
        
        # 输出当前位置和朝向
        position = self.current_pose.pose.position
        orientation = self.current_pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        rospy.loginfo("Robot position: (%.2f, %.2f, %.2f), Yaw: %.2f", 
                      position.x, position.y, position.z, yaw)

if __name__ == '__main__':
    try:
        slam_controller = SLAMController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass