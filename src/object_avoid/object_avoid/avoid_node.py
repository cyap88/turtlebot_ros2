import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import csv

class AvoidNode(Node):
  def __init__(self, node_name):
    super().__init__(node_name)
    self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
    self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb,10) #/cmd_vel is velocity command topic
    timer_period = 0.5 # seconds
    self.time = self.create_timer(timer_period, self.time_callback)
    self.i = 0
  def scan_cb(self, msg):
    scan = LaserScan()
    if min(msg.ranges) < 0.3:
      scan.angular.z = 0.5
    else:
      scan.linear.x = 0.1
  def cmd_vel_cb(self, msg):
    cmd_vel = Twist()

def main(args = None):
  rclpy.init(args = args)
  avoid = AvoidNode()

  rclpy.spin(avoid)

  avoid.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()