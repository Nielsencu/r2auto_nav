#import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Shooter(Node):

    def __init__(self,string1):
        super().__init__('shooter')
        self.publisher_ = self.create_publisher(String, 'shooter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.string1 = string1

    def timer_callback(self):
        msg = String()
        msg.data = self.string1
        #self.string1 = "now working"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
       
 
def main(args=None):
    rclpy.init(args=args)
    shooter = Shooter("Stop moving")
   # shooter.string1 = "Shooter now working"
    rclpy.spin_once(shooter)
    time.sleep(2)
    print("resume")
    shooter.string1 = "Continue moving"
    rclpy.spin_once(shooter)
    print("hello")
    shooter.destroy_node()
    rclpy.shutdown()
    
#main()
