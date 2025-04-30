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

        self.create_timer(30.0, self.safe_land)

    #Rename to hover possibly
    def stop_movement(self):
        stopmsg = Twist()
        self.get_logger().info('Stopping Movement!')
        self.move_pub.publish(stopmsg)

    def takeoff(self):
        self.get_logger().info('Taking off!')
        self.takeoff_pub.publish(self.msg)

        ascendmsg = Twist()
        ascendmsg.linear.z = 0.5
        self.move_pub.publish(ascendmsg)

        self.create_timer(7.5, self.stop_movement)

        # Timer for landing 
        self.create_timer(20.0, self.land)

        self.takeoff = lambda: None

    def land(self):
        desendmsg = Twist()
        desendmsg.linear.z = -0.5
        self.move_pub.publish(desendmsg)

        self.create_timer(3.5, self.stop_movement)

        self.get_logger().info('Landing!')
        self.land_pub.publish(self.msg)
        self.get_logger().info('Shutting down.')
        self.landed = True
        rclpy.shutdown()

    def safe_land(self):
        if not self.landed:
            self.get_logger().warn('Safe Landing!')
            self.land_pub.publish(self.msg)
            self.get_logger().info('Shutting down.')
            rclpy.shutdown()

    def tracking(self, x_offset, y_offset):
        min_pixels = 50
        movemsg = Twist()

        if abs(x_offset) > min_pixels:
            if x_offset > 0:
                movemsg.linear.x = 0.3
                self.move_pub.publish(movemsg)
            else:
                movemsg.linear.x = -0.3
                self.move_pub.publish(movemsg)
        else:
            self.stop_movement

        if abs(y_offset) > min_pixels:
            if y_offset > 0:
                movemsg.linear.z = -0.3
                self.move_pub.publish(movemsg)
            else:
                movemsg.linear.z = 0.3
                self.move_pub.publish(movemsg)
        else:
            self.stop_movement


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