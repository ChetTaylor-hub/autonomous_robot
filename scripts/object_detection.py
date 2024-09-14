#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from autonomous_robot.msg import ObjectDetection
from geometry_msgs.msg import PoseStamped

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detection')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.object_pub = rospy.Publisher('/detected_objects', ObjectDetection, queue_size=10)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        
    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(cv_image)
        
        for det in results.xyxy[0]:
            class_id = int(det[5])
            confidence = float(det[4])
            if confidence > 0.5:  # 置信度阈值
                obj_msg = ObjectDetection()
                obj_msg.class_id = self.model.names[class_id]
                obj_msg.confidence = confidence
                
                # 考虑相机位置进行坐标转换
                obj_msg.pose = PoseStamped()
                obj_msg.pose.header = data.header
                obj_msg.pose.pose.position.x = 0.22 + (det[0] + det[2]) / 2 / cv_image.shape[1] * 2  # 假设相机视野为2米
                obj_msg.pose.pose.position.y = ((det[0] + det[2]) / 2 - cv_image.shape[1] / 2) / cv_image.shape[1] * 2
                obj_msg.pose.pose.position.z = 0.08 - ((det[1] + det[3]) / 2 - cv_image.shape[0] / 2) / cv_image.shape[0] * 1.5  # 假设相机视野垂直方向为1.5米
                obj_msg.pose.pose.orientation.w = 1
                
                self.object_pub.publish(obj_msg)

if __name__ == '__main__':
    try:
        object_detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass