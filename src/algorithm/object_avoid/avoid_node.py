import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import csv
import os
import time 

class AvoidNode(Node):
  def __init__(self, node_name):
    super().__init__(node_name)

    directory = None
    directory = os.path.join(os.getcwd(), 'src', 'object_avoid', 'test_data')
    file_path = os.path.join(directory, "test_data.csv")
    global writer
    fields = ['timestamp'] + [f'scan_{i}' for i in range(360)] + ['linear_x', 'angular_z']

    with open(file_path, 'w', newline='') as file:
      writer = csv.DictWriter(file, fieldnames=fields)
      writer.writeheader()

    self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
    self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb,10) #/cmd_vel is velocity command topic
    self.last_scan = None
    self.last_cmd_vel = None
    
  def scan_cb(self, msg):
    self.last_scan = msg.ranges

  def cmd_vel_cb(self, msg):
    if self.last_scan is not None:
      d = {
        'timestamp': time.time(), **{f'scan_{i}': v for i, v in enumerate(self.last_scan)},
        'linear_x': msg.linear.x, 'angular_z': msg.angular.z
      }
    writer.writerow(d)
    self.last_scan = None

def main(args = None):
  rclpy.init(args = args)
  avoid = AvoidNode()
  rclpy.spin(avoid)
  avoid.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()


    # scan = LaserScan()
    # if min(msg.ranges) < 0.3:
    #   scan.angular.z = 0.5
    # else:
    #   scan.linear.x = 0.1