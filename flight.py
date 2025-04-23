#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('flight')

        self.takeoff_pub = self.create_publisher(Empty, '/bebop/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/bebop/land', 10)
        self.camera_sub = self.create_subscription(Image, '/bebop/camera/image_raw', self.listener_callback, 10)


        self.timer_count = 0
        self.bridge = CvBridge()
        self.msg = Empty()

        self.get_logger().info('Preparing to takeoff!')
        
        # Takeoff after 1 sec 
        self.create_timer(1.0, self.takeoff)

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Camera Feed', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

    def takeoff(self):
        self.get_logger().info('Taking off!')
        self.takeoff_pub.publish(self.msg)

        # Timer for landing 
        self.countdown_to_land

    def countdown_to_land(self):
        start = time.monotonic()
        elapsed = 0.0

        while elapsed <= 20:
            elapsed = time.monotonic() - start
            self.get_logger().info(f"Time taken: {elapsed:.2f} seconds")
            time.sleep(1)
        
        
        self.get_logger().info('Landing!')
        self.land_pub.publish(self.msg)
        self.get_logger().info('Shutting down.')
        rclpy.shutdown()
            
        ## This is old still needs testing ##
        #if self.timer_count < 30:
        #    self.get_logger().info(f"Time left before landing: {30 - self.timer_count} sec")
        #    self.timer_count += 1
        #else:
        #    self.get_logger().info('Landing!')
        #    self.land_pub.publish(self.msg)
        #    self.get_logger().info('Shutting down.')
        #    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    drone = DroneController()

    rclpy.spin(drone)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()