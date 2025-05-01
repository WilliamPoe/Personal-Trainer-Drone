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


        self.msg = Empty()
        self.landed = False
        self.emerg_land = False


        self.get_logger().info('Preparing to takeoff!')
        
        # Takeoff after 1 sec 
        self.takeoff_timer = self.create_timer(1.0, self.takeoff)
        # Climb after 2 sec
        self.climb_timer = self.create_timer(2, self.climb)
        # Emergency land after 65 sec
        self.create_timer(35.0, self.safe_land)

    def stop_movement(self): # Stops the movement of the drone and starts to hover
        stopmsg = Twist()
        self.get_logger().info('Stopping Movement!')
        self.move_pub.publish(stopmsg)
        if hasattr(self, 'stop_timer') and self.stop_timer is not None: # If there is a stop timer created
            self.stop_timer.cancel()
            self.stop_timer = None

    def takeoff(self): # Drone takeoff call
        self.get_logger().info('Taking off!')
        self.takeoff_pub.publish(self.msg)
        self.takeoff_timer.cancel()

        # Timer for landing 
        self.create_timer(30.0, self.land)
    
    def climb(self): # Drone climb to a safe height
        self.get_logger().info('Climbing!')
        ascendmsg = Twist()
        ascendmsg.linear.z = 0.5
        self.move_pub.publish(ascendmsg)
        self.climb_timer.cancel()

        self.stop_timer = self.create_timer(5.0, self.stop_movement)

    def land(self): # Drone land
        if not self.landed:
            desendmsg = Twist()
            desendmsg.linear.z = -0.5
            self.move_pub.publish(desendmsg)

            self.stop_timer = self.create_timer(1.5, self.stop_movement)

            self.get_logger().info('Landing!')
            self.land_pub.publish(self.msg)
            self.get_logger().info('Shutting down.')
            self.landed = True

    def safe_land(self): # Drone safe land
        self.emerg_land = True
        self.get_logger().warn('Safe Landing!')
        self.land_pub.publish(self.msg)
        self.landed = True
        self.get_logger().info('Shutting down.')
        rclpy.shutdown()

    def find_turn(self): # Was not able to get this implemented 
        turnmsg = Twist()
        angle_rad = 1.57
        speed = 0.3
        self.get_logger().info('Turning to find person!')
        # Assuming this is a right turn
        turnmsg.angular.z = -speed
        self.time_start = time.monotonic()
        while time.monotonic() - self.time_start < angle_rad / speed:
            self.move_pub.publish(turnmsg)
            time.sleep(0.1)
        self.stop_movement()
    
    def tracking(self, x_offset, y_offset): # Tracking function to be called keeps person in the. x_offset, y_offset are offsets from center
        min_pixels = 35
        movemsg = Twist()
        if self.emerg_land: # Will not move if safe landing is started
            self.get_logger().warn('Safe landing started!')
            return
        # Left and right
        if abs(x_offset) > min_pixels:
            if x_offset > 0:
                movemsg.linear.y = -.75
            else:
                movemsg.linear.y = .75
        # Up and down
        if abs(y_offset) > min_pixels:
            if y_offset > 0:
                movemsg.linear.x = -.75
            else:
                movemsg.linear.x = .75

        if movemsg.linear.x == 0.0 and movemsg.linear.y == 0.0: # Stop movement of the drone if centered
            self.stop_movement()

        else:
            self.move_pub.publish(movemsg) # Publish movements


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
