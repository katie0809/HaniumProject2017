#!/usr/bin/env python
import rospy
import sys
from nav_msgs.srv import GetMap

rospy.init_node('map_client')
rospy.wait_for_service('static_map')
mapsrv = rospy.ServiceProxy('static_map', GetMap)
result = mapsrv()
print(result)
