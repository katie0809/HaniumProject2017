#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
  global g_range_ahead
  g_range_ahead = (msg.range_min + msg.range_max) / 2

g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.init_node('move')
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():
  if driving_forward:
    # BEGIN FORWARD
    if (g_range_ahead < 1 ):
      driving_forward = False
    else: 
      if (g_range_ahead < 0.5) :
        driving_forward = False

  twist = Twist()
  if driving_forward:
    twist.linear.x = 0.1
  else:
    twist.angular.z = 0.1
    driving_forward = True
  cmd_vel_pub.publish(twist)
  print (g_range_ahead)

  rate.sleep()
# END ALL
