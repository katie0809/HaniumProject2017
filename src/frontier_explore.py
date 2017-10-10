#!/usr/bin/env python

import rospy
from time import sleep
from geometry_msgs.msg import Point32, Point, PointStamped, PolygonStamped
from nav_msgs.srv import GetMap
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from frontier_exploration.srv import UpdateBoundaryPolygon
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskGoal
#from gmapping.srv import GetMapMetaData
import actionlib
import frontier_exploration.msg

def getAndWaitForActionServer(seviceName, actionClass):
    client = actionlib.SimpleActionClient(seviceName, actionClass)

    # Wait for explore service
    rospy.loginfo("Waiting for service " + seviceName + ".")
    client.wait_for_server()
    rospy.loginfo("Found service " + seviceName + ".")

    return client

def getMapMetadata(mapMetaDataServiceName):
    # Create service handle
#    mapMetaDataService = rospy.ServiceProxy(mapMetaDataServiceName, GetMapMetaData)
    mapMetaDataService = rospy.ServiceProxy(mapMetaDataServiceName, GetMap)

    response = None
    numTries = 0
    rospy.loginfo("Trying to get the map information.")
    while response is None and numTries < maxTries and not rospy.is_shutdown():
        numTries += 1
        try:
            response = mapMetaDataService()
        except rospy.ServiceException as exc:
            rospy.logwarn(exc)
            rospy.logwarn("Try [%s / %s]. Map service did not process request: \"%s\"", numTries, maxTries, exc)
            if numTries < maxTries:
                # Don't use blocking rospy.sleep(..)
                rospy.loginfo("Waiting for " + str(tryDelay)  + " seconds.")
                sleep(tryDelay)

    if rospy.is_shutdown():
        rospy.loginfo("ROS has been shut down... Exiting...")
        return None

    if numTries >= maxTries:
        rospy.logfatal("Map service did not process request within " + str(maxTries) + " tries. Exiting...")
        return None

    rospy.loginfo("Got map in " + str(numTries) + " tries.")

    # Get metadata
    return response.metadata

def getBoundaryPoints(mapMetaData):
    # Corner points
    origin_x, origin_y = mapMetaData.origin.position.x, mapMetaData.origin.position.y
    width, height = mapMetaData.width * mapMetaData.resolution, mapMetaData.height * mapMetaData.resolution

    # Pad the boundary of the map
    points = [
        (origin_x + padding, origin_y + padding),
        (origin_x + width - padding, origin_y + padding),
        (origin_x + width - padding, origin_y + height - padding),
        (origin_x + padding, origin_y + height - padding),
    ]
    return points

def createExploreTaskGoal(points):
    polygon = PolygonStamped()
    polygon.header.stamp = rospy.Time.now()
    polygon.header.frame_id = mapFrame
    polygon.polygon.points = []

    # Make explore corner points into a correct ROS message
    for point in points:
        x, y = point
        polygon.polygon.points.append(Point32(x=x, y=y))

    # Add start point for exploration
    startPoint = PointStamped()
    startPoint.header.stamp = rospy.Time.now()
    startPoint.header.frame_id = mapFrame
    startPoint.point = Point(x=0, y=0)

    return ExploreTaskGoal(explore_boundary = polygon, explore_center = startPoint)

def exploreNode():
    # Wait for map service
    rospy.loginfo("Waiting for service " + mapMetaDataServiceName + ".")
    rospy.wait_for_service(mapMetaDataServiceName)
    rospy.loginfo("Found service " + mapMetaDataServiceName + ".")

    mapMetaData = getMapMetadata(mapMetaDataServiceName)
    if mapMetaData == None:
        rospy.logerr("No map metadata received. Exiting...")
        return

    points = getBoundaryPoints(mapMetaData)

    # Create service handle
    exploreClient = getAndWaitForActionServer(exploreServiceName, ExploreTaskAction)

    # Create an explore task
    exploreTask = createExploreTaskGoal(points)
    # Set goal to explore action server
    exploreClient.send_goal(exploreTask)
    rospy.loginfo('Sent exploration goal. Waiting for result...')
    exploreClient.wait_for_result()
    rospy.logwarn('Exploration finished')

    # Send movement command to to initial position
    returnToInitialPosition()

def returnToInitialPosition():
    rospy.loginfo('Sending return goal to initial position')

    moveBaseClient = getAndWaitForActionServer(moveBaseServiceName, MoveBaseAction)

    # Create task
    moveGoal = MoveBaseGoal()
    moveGoal.target_pose.header.frame_id = mapFrame
    moveGoal.target_pose.header.stamp = rospy.Time.now()

    # Initial position
    moveGoal.target_pose.pose.position.x = 0.0
    moveGoal.target_pose.pose.position.y = 0.0
    moveGoal.target_pose.pose.orientation.w = 1.0;

    # Send movement goal. In case of failures: just ignore them.
    moveBaseClient.send_goal(moveGoal)
    moveBaseClient.wait_for_result()

    rospy.loginfo('Movement done')

if __name__ == '__main__':
    rospy.init_node('exploration')

    rospy.loginfo('Initializing exploration')

    # Set global constants
#    mapMetaDataServiceName = rospy.get_param('~mapMetaDataServiceName', '/dynamic_map_metadata')
    mapMetaDataServiceName = rospy.get_param('~mapMetaDataServiceName', '/dynamic_map')
    exploreServiceName = rospy.get_param('~exploreServiceName', '/explore_server')
    moveBaseServiceName = rospy.get_param('~moveBaseServiceName', '/move_base')
    mapFrame = rospy.get_param('~mapFrame', 'map')
    robotFrame = rospy.get_param('~robotFrame', 'base_link')

    maxTries = rospy.get_param('~maxTries', 15)
    tryDelay = rospy.get_param('~tryDelay', 3.0)
    padding = rospy.get_param('~padding', 0.35)

    # Run node
    exploreNode()

    rospy.spin()

    rospy.loginfo('Exit')
