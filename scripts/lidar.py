import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import sol_text as sol
import servofix as servo

class MyNode(Node):
    def __init__(self):
            super().__init__('lidar')
            self.subscription = self.create_subscription(LaserScan,'scan',self.listener_callback,qos_profile_sensor_data)
            self.subscription
    def listener_callback(self, msg):
        # create numpy array
            laser_range = np.array(msg.ranges)
        # replace 0's with nan
            laser_range[laser_range==0] = np.nan
        # find index with minimum value
        #lr2i = np.nanargmin(laser_range)
            self.distance = laser_range[0]
        # log the distance
            self.get_logger().info('Distance: '+str(self.distance))
            if 0.9<=self.distance<=1.1:
                servo
                solenoid
    def solenoid(self):
        sol.call()

    def servo(self):
        servo.turnin()


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
