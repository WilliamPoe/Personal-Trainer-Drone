#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time



class DroneController(Node):
    def __init__(self):
        super().__init__('flight')

        self.takeoff_pub = self.create_publisher(Empty, '/bebop/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/bebop/land', 10)
        self.move_pub = self.create_publisher(Twist, '/bebop/cmd_vel', 10)


        self.timer_count = 0
        self.msg = Empty()


        self.get_logger().info('Preparing to takeoff!')
        
        # Takeoff after 1 sec 
        self.create_timer(1.0, self.takeoff)

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