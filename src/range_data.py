#!/usr/bin/env python
# BEGIN ALL
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
  global range_ahead_min, range_ahead_max, range_ahead_center
  range_ahead_min = msg.ranges[0]
  range_ahead_center = msg.ranges[len(msg.ranges)/2]
  range_ahead_max = msg.ranges[len(msg.ranges)-10]
  print(range_ahead_min)
  print(range_ahead_center)
  print(range_ahead_max)
  print(msg.angle_min)
  print(msg.angle_max)

rospy.init_node('range_ahead')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.spin()