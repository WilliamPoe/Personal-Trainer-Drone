#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class Tracker(Node):
    def __init__(self):
        super().__init__('tracking')

        self.camera_sub = self.create_subscription(Image, '/bebop/camera/image_raw', self.listener_callback, 10)

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Camera Feed', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

def main(args=None):