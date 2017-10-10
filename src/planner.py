#!/usr/bin/env python

import tf, rospy
from nodes import *
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point as ROSPoint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from rbe3002.srv import *

costMapExpansion = 85


# converts a pose (Pose) to a point (Point) in the world grid
def pose2point(pose, grid):
    assert isinstance(pose, Pose)

    poseStamped = PoseStamped()
    poseStamped.pose = pose
    poseStamped.header.frame_id = 'map'
    poseStamped.header.stamp = rospy.Time(0)

    pose = transformer.transformPose(grid.frame_id, poseStamped).pose

    dX = pose.position.x - grid.map_info.origin.position.x
    dY = pose.position.y - grid.map_info.origin.position.y

    return Point(int(dX / grid.map_info.resolution), int(dY / grid.map_info.resolution))


# converts a node object to a PoseStamped object for use in a path
def node2pose(node, grid):
    pose = Pose()

    pose.position.x = ((node.point.x + .5) * grid.map_info.resolution) + grid.map_info.origin.position.x
    pose.position.y = ((node.point.y + .5) * grid.map_info.resolution) + grid.map_info.origin.position.y

    # convert to quaternian
    tempOri = node.orientation
    if (tempOri >= 3): tempOri -= 4
    tempOri -= 1
    nodeYaw = tempOri * (math.pi / 2)
    quat = quaternion_from_euler(0, 0, nodeYaw)

    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    poseStamped = PoseStamped()
    poseStamped.pose = pose
    poseStamped.header.frame_id = grid.frame_id
    poseStamped.header.stamp = rospy.Time(0)

    # pose = transformer.transformPose('map',poseStamped).pose
    return poseStamped


# calculates and returns path to the goal point
# start and end are poses
# passes back a list of PoseStamped wayPoints to get from star to end
def aStar(start, goal, grid, wayPub):
    global cost_map

    # convert from poses to points + init orientation (1,2,3, or 4)
    startPoint = pose2point(start, grid)
    goalPoint = pose2point(goal, grid)
    quat = [start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    initYaw = yaw  # returns the yaw

    cutoffVal = costMapExpansion
    if grid.getVal(startPoint.x, startPoint.y) > cutoffVal:
        cutoffVal = grid.getVal(startPoint.x, startPoint.y)

    if grid.getVal(goalPoint.x, goalPoint.y) > cutoffVal:
        print
        "   a* -> Goal is unreachable"
        return None

    # convert from euler yaw to 1,2,3,4, as used by node
    initOri = round(initYaw / (math.pi / 2)) + 1
    if initOri <= 0: initOri += 4

    ######################################################################################################

    # A* here
    curNode = Node(startPoint, initOri, goalPoint, None)
    path = {curNode.key: curNode}
    frontier = [curNode]

    # keep searching the frontiers based on the lowest cost until goal is reached
    while (not curNode.point.equals(goalPoint)):
        nodeKids = curNode.createNewNodes(path, grid, cutoffVal)  # create new nodes that are neighbors to current node
        for kid in nodeKids:
            # add the nodes to frontier based on cost
            for ind in range(0, len(frontier)):
                if (kid.cost < frontier[ind].cost):
                    frontier.insert(ind, kid)
                    break
                elif (ind == len(frontier) - 1):
                    frontier.append(kid)

            # add the new node to the dictionary of nodes
            path[kid.key()] = kid

            # add kids' costs to the cost_map
            cost_map.setVal(kid.point.x, kid.point.y, int(kid.cost))

        frontier.remove(curNode)
        if not frontier:
            print
            "   a* -> No more frontier"
            return None

        curNode = frontier[0]  # curNode becomes the frontier node with the lowest cost

    ######################################################################################################

    # store the best path, determined by a*
    path = []

    # displayPath is a GridCells, and is used to display the path in rviz
    displayPath = GridCells()
    displayPath.cell_height = grid.map_info.resolution
    displayPath.cell_width = grid.map_info.resolution
    displayPath.header.frame_id = grid.frame_id
    displayPath.header.stamp = rospy.get_rostime()

    # put the chosen path into an array of nodes (path), and a GridCells (displayPath)
    while curNode and curNode.prevNode:
        path.insert(0, curNode)
        rosPoint = ROSPoint()
        rosPoint.x = (curNode.point.x + .5) * displayPath.cell_width + grid.map_info.origin.position.x
        rosPoint.y = (curNode.point.y + .5) * displayPath.cell_height + grid.map_info.origin.position.y
        rosPoint.z = displayPath.cell_height * .125  # offset above path
        displayPath.cells.append(rosPoint)
        curNode = curNode.prevNode

    path_pub.publish(displayPath)

    ######################################################################################################

    # figure out important waypoints, store them
    wayPoints = []

    # displayWays is a GridCells, and is used to display the waypoints in rviz
    displayWays = GridCells()
    displayWays.cell_height = grid.map_info.resolution
    displayWays.cell_width = grid.map_info.resolution
    displayWays.header.frame_id = grid.frame_id
    displayWays.header.stamp = rospy.get_rostime()

    # put the vital waypoints into an array of poseStamped (waypoints), and a GridCells (displayWays) to be displayed
    maxCount = .75 / grid.map_info.resolution  # the max number of gridcells between wayPoints
    minCount = 2  # the min number of gridcells between wayPoints
    count = 0
    for curNode in path:
        if not curNode.prevNode:  # if on the start node
            isVitalWaypoint = False
        elif curNode.point.equals(goalPoint):  # if on the goal node
            isVitalWaypoint = True
        elif not len(wayPoints):  # if there is nothing in the waypoints array
            isntTooClose = (
            math.sqrt((node2pose(curNode, grid).pose.position.x - node2pose(path[1], grid).pose.position.x) ** 2 +
                      (node2pose(curNode, grid).pose.position.y - node2pose(path[1],
                                                                            grid).pose.position.y) ** 2) > minCount * grid.map_info.resolution)
            isVitalWaypoint = (
                                  count >= maxCount or not curNode.orientation == curNode.prevNode.orientation) and isntTooClose
        else:
            isntTooClose = (math.sqrt((node2pose(curNode, grid).pose.position.x - wayPoints[-1].pose.position.x) ** 2 +
                                      (node2pose(curNode, grid).pose.position.y - wayPoints[
                                          -1].pose.position.y) ** 2) > minCount * grid.map_info.resolution)
            isVitalWaypoint = (
                                  count >= maxCount or not curNode.orientation == curNode.prevNode.orientation) and isntTooClose

        if isVitalWaypoint:
            wayPoints.append(node2pose(curNode, grid))
            temp = ROSPoint()
            temp.x = (curNode.point.x + .5) * displayPath.cell_width + grid.map_info.origin.position.x
            temp.y = (curNode.point.y + .5) * displayPath.cell_height + grid.map_info.origin.position.y
            temp.z = displayPath.cell_height * .25  # offset above costmap
            displayWays.cells.append(temp)
            count = 0
        count += 1
    if path:
        wayPoints.append(node2pose(path[-1], grid))

    wayPub.publish(displayWays)

    return wayPoints


# finds the path to goal from start and returns the waypoints to reach there
def calcWaypoints(start, goal, grid, wayPub):
    global cost_map

    cost_map = Grid(grid.width, grid.height, [0] * len(grid.data), grid.map_info, grid.frame_id)

    dao = aStar(start, goal, grid, wayPub)  # dao = way in Chinese

    if not dao:
        print
        "   a* -> no path found"
        return

    way = Path()
    for waypoint in dao:
        way.poses.append(waypoint)

    waypoints_pub.publish(way)
    return way


# subscriber callbacks

# data_map is an OccupancyGrid
# stores the incoming global map
def globalMapCallback(data_map):
    global global_map
    global global_current_map_pub

    assert isinstance(data_map, OccupancyGrid)

    global_map = Grid(data_map.info.width, data_map.info.height, data_map.data, data_map.info, data_map.header.frame_id)


# data_map is an OccupancyGrid
# stores the incoming local map
def localMapCallback(data_map):
    global local_map
    global local_current_map_pub

    assert isinstance(data_map, OccupancyGrid)

    local_map = Grid(data_map.info.width, data_map.info.height, data_map.data, data_map.info, data_map.header.frame_id)


# service handler. Takes in a start and end pose then returns a path
def globalCalcPath(req):
    global cost_map

    start = req.start
    goal = req.end
    path = calcWaypoints(start, goal, global_map, global_way_pub)

    cost_map.publish(global_costmap_pub)
    cost_map = None

    return CalcPathResponse(path)


# service handler. Takes in a start and end pose then returns a path
def localCalcPath(req):
    start = req.start
    goal = req.end
    path = calcWaypoints(start, goal, local_map, local_way_pub)

    return CalcPathResponse(path)


def localUpdateCallback(update):
    global local_map
    local_map.update(update)


def globalUpdateCallback(update):
    global gobal_map
    global_map.update(update)


def printUpdatedMaps(event):
    global local_map
    global global_map

    if local_map and global_map:
        # print "recieved and updated local map"
        local_map.publish(local_current_map_pub)
        # print "recieved global updated map"
        # global_map.publish(global_current_map_pub)


def run():
    rospy.init_node('planning_node')
    init()

    global_serv = rospy.Service('global_path', CalcPath, globalCalcPath)
    local_serv = rospy.Service('local_path', CalcPath, localCalcPath)

    rospy.sleep(1)
    print
    "READY TO NAVIGATE"

    # rospy.Timer(rospy.Duration(1), printUpdatedMaps) # for debugging

    # this handles updating the local_cost_map
    while not rospy.is_shutdown():
        if (cost_map != None):
            cost_map.publish(local_costmap_pub)
        rospy.sleep(.125)


def init():
    global global_map
    global local_map
    global cost_map

    # make publishers global so that they can be used anywhere
    global local_costmap_pub
    global path_pub
    global waypoints_pub
    global local_way_pub
    global global_costmap_pub
    global global_way_pub
    global transformer
    global local_current_map_pub
    global global_current_map_pub
    global global_update_sub
    global local_update_sub

    cost_map = None

    transformer = tf.TransformListener()

    # publishers
    local_costmap_pub = rospy.Publisher('/robot_cost_map', OccupancyGrid, queue_size=1)
    local_way_pub = rospy.Publisher('/robot_waypoints', GridCells, queue_size=1)
    local_current_map_pub = rospy.Publisher('/local_cur_map', OccupancyGrid, queue_size=1)
    global_costmap_pub = rospy.Publisher('/global_cost_map', OccupancyGrid, queue_size=1)
    global_way_pub = rospy.Publisher('/global_waypoints', GridCells, queue_size=1)
    global_current_map_pub = rospy.Publisher('/global_cur_map', OccupancyGrid, queue_size=1)
    path_pub = rospy.Publisher('/robot_path', GridCells, queue_size=1)
    waypoints_pub = rospy.Publisher('/waypoints', Path, queue_size=1)

    # subscribers
    global_grid_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, globalMapCallback,
                                       queue_size=1)
    local_grid_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, localMapCallback, queue_size=1)

    local_update_sub = rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate,
                                        localUpdateCallback, queue_size=1)
    global_update_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate,
                                         globalUpdateCallback, queue_size=1)


if __name__ == '__main__':
    run()
