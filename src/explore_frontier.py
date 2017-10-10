#!/usr/bin/env python

import planner, nav
import rospy, tf, math, copy, os
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Point as ROSPoint
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from Queue import Queue
import math, rospy, copy
from nav_msgs.msg import OccupancyGrid


# represents a 2D point in a map
class Point:
    # creates a new point with location (x,y)
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # returns the key to use to hash this point
    def key(self):
        return str(self.x) + "," + str(self.y)

    # returns if this point is at the same location as compPoint
    def equals(self, compPoint):
        return self.x == compPoint.x and self.y == compPoint.y


# represents a single cell in a map
class Node:
    point = None
    orientation = None
    prevNode = None
    g_cost = -1
    cost = -1
    endPoint = None

    # constructor
    # nodePoint is the grid location of this node
    # initOri is the orientation of the node (1 to 4)
    # endPoint is the goal point being navigated to
    # previousNode is the node that was before this node in the path
    def __init__(self, nodePoint, initOri, endPoint, previousNode):
        self.point = nodePoint
        self.prevNode = previousNode
        self.orientation = initOri
        self.endPoint = endPoint
        self.calcCost(endPoint)

    # calculates the total cost for this node
    def calcCost(self, endPoint):
        # calculates the manhattan g_cost of this node
        # g_cost of a node is equal to the g_cost of the previous node plus 1 plus a weighting based on rotations
        # if no previous node, this is first node in the search
        if self.prevNode:
            # calculates rotation cost
            rotCost = abs(self.orientation - self.prevNode.orientation)
            if (rotCost > 2):
                rotCost = 4 - rotCost

            # calculates driving cost based on distance between this node and the next node
            if self.orientation % 1:  # non-zero =true = decimal = diagonal mflag Corrected from previous
                goCost = 1.414  # also changed to ori instead of rotCost
            else:
                goCost = 1

            self.g_cost = self.prevNode.g_cost + goCost + rotCost
        else:
            self.g_cost = 0

        # calculates euclidian h_cost
        h_cost = math.sqrt((self.point.x - endPoint.x) ** 2 + (self.point.y - endPoint.y) ** 2)

        # adds g and h cost to get the total cost
        self.cost = self.g_cost + h_cost

    # gets the key to use for hashing this node
    def key(self):
        return self.point.key()

    # creates and returns all nodes that are neighbors to this node and are not either created or blocked
    # curNodes is a dictionary of all created nodes
    # world is a Grid representing the map of the world
    # blockedThresh is the cutoff to use for determining if a location is blocked
    def createNewNodes(self, curNodes, world, blockedThresh):
        # arrays to easily create new neighbor nodes
        dx = [1, 1, 0, -1, -1, -1, 0, 1]
        dy = [0, 1, 1, 1, 0, -1, -1, -1]
        theta = [1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5]
        newNodes = []

        # iterate through each node
        for i in range(0, 8):
            tempX = self.point.x + dx[i]
            tempY = self.point.y + dy[i]
            key = str(tempX) + "," + str(tempY)

            # if the neighbor node is in the grid, is not blocked, and has not been created, create it
            if (not ((key in curNodes) or tempX < 0 or tempX >= world.width or tempY < 0 or tempY >= world.height or (
            world.getVal(tempX, tempY)) >= blockedThresh)):
                newNodes.append(Node(Point(tempX, tempY), theta[i], self.endPoint, self))
        return newNodes


# represents a 2D map of the world
class Grid:
    # creates a new grid with inputed width and height using data
    # data should be a 1D array with a length of width*height
    def __init__(self, width, height, data, info, frame_id):
        self.width = width
        self.height = height
        self.data = list(data)
        self.map_info = info
        self.frame_id = frame_id

    def getValFromPoint(self, point):
        return self.getVal(point.x, point.y)

    # gets the stored value that corresponds to the inputted (x,y) location
    def getVal(self, x, y):
        return self.data[x + y * self.width]

    # sets the stored value that corresponds to the inputted (x,y) location to val
    def setVal(self, x, y, val):
        self.data[x + y * self.width] = val

    def publish(self, publisher, real=False):
        costGrid = OccupancyGrid()

        costGrid.header.frame_id = self.frame_id
        costGrid.header.stamp = rospy.Time.now()
        costGrid.info = self.map_info
        temparr = copy.deepcopy(self.data)  # protect ourselves from volitility

        if not real:
            # map cost_map to between 0 and 127 for fancy colors in rviz
            maxVal = max(temparr)

            minVal = float('inf')
            for cost in temparr:
                if not (cost == 0) and (cost < minVal): minVal = cost

            minVal *= 1.01

            factor = 90.0 / (maxVal - minVal)

            if (maxVal == minVal): return

            # costGrid.data = [(int((i - minVal) * factor) if (i != 0) else 0) for i in cost_map.data]
            costGrid.data = []
            for i in temparr:
                if (i != 0):
                    costGrid.data.append(int((i - minVal) * factor))
                else:
                    costGrid.data.append(0)
        else:
            costGrid.data = temparr

        publisher.publish(costGrid)

    # this updates the map with the update sent by the robot
    def update(self, update):

        xOff = update.x
        yOff = update.y

        # assert isinstance(self.data, list)
        # assert isinstance(update.data, list)

        # loop through the x and y, replacing the corresponding values with updated values
        for x in range(0, update.width):
            for y in range(0, update.height):
                self.setVal(xOff + x, yOff + y, update.data[x + y * update.width])




def getNextFrontier():
    startPoint = planner.pose2point(nav.navBot.cur.pose, global_map)
    curNode = Node(startPoint, 1, startPoint, None)
    nodes = {curNode.key(): curNode}
    frontier = Queue()
    frontier.put(curNode)
    nextFrontier = []
    tempMap = copy.deepcopy(global_map)
    cells = GridCells()
    cells.cell_height = global_map.map_info.resolution
    cells.cell_width = global_map.map_info.resolution
    cells.header.frame_id = global_map.frame_id
    cells.header.stamp = rospy.Time(0)

    # while there are still nodes in the frontier
    while not frontier.empty():
        node = frontier.get()
        # if the node is a frontier, expand the frontier
        if tempMap.getValFromPoint(node.point) == -1:
            front = expandFrontier(node, nodes)
            # if the frontier was valid, return it
            if front:
                print
                "found Frontier"
                return front
            continue
        # if the node was no a frontier, expand the node and add it to the back of the queue
        tempNodes = node.createNewNodes(nodes, tempMap, planner.costMapExpansion)
        for n in tempNodes:
            nodes[n.key()] = n
            p = ROSPoint()
            tPose = planner.node2pose(n, global_map)
            p.x = tPose.pose.position.x
            p.y = tPose.pose.position.y
            cells.cells.append(p)
            frontier_pub.publish(cells)

            frontier.put(n)

    print
    "No Frontiers found"
    return None


def getNextWaypoint():
    node = getNextFrontier()
    if node:
        return planner.node2pose(node, global_map)
    return None


def expandFrontier(start, nodeDict):
    print
    "expanding frontier"
    nodes = {start.key(): start}
    fullFrontier = []
    curFrontier = [start]
    endPoints = []
    cells = GridCells()
    cells.cell_height = global_map.map_info.resolution
    cells.cell_width = global_map.map_info.resolution
    cells.header.frame_id = global_map.frame_id
    cells.header.stamp = rospy.Time(0)

    # while there are still nodes to explore
    while curFrontier:
        curNode = curFrontier[0]
        # expand the node
        for node in curNode.createNewNodes(nodes, global_map, planner.costMapExpansion):
            # check if the expanded nodes are unexplored and is manahttan
            if global_map.getValFromPoint(node.point) == -1 and node.orientation % 1 == 0:
                # check that at least one child node of the current node is explored, then add it to the expansion
                for tempNode in node.createNewNodes(nodes, global_map, 101):
                    if global_map.getValFromPoint(tempNode.point) != -1 and global_map.getValFromPoint(
                            tempNode.point) <= planner.costMapExpansion:
                        curFrontier.append(node)
                        nodes[node.key()] = node
                        break
        p = ROSPoint()
        tPose = planner.node2pose(curNode, global_map)
        p.x = tPose.pose.position.x
        p.y = tPose.pose.position.y
        cells.cells.append(p)
        frontier_pub.publish(cells)
        fullFrontier.append(curNode)
        curFrontier.remove(curNode)

    print
    "len = " + str(len(fullFrontier))
    if len(fullFrontier) > .4 / global_map.map_info.resolution:
        for n in fullFrontier:
            bot = nav.navBot.cur  # planner.node2pose(nav.navBot.cur)
            node = planner.node2pose(n, global_map)
            squaredDistance = (bot.pose.position.x - node.pose.position.x) ** 2 + (
                                                                                  bot.pose.position.y - node.pose.position.y) ** 2
            point1 = planner.pose2point(nav.navBot.cur.pose, global_map)
            point2 = n.point
            splitPoint = Point(int((point1.x - point2.x) * .25 + point2.x), int((point1.y - point2.y) * .25 + point2.y))
            if squaredDistance >= 1:
                if planner.global_map.getValFromPoint(n.point) < planner.costMapExpansion:
                    globalPathServ = getGlobalPath(nav.navBot.cur.pose, planner.node2pose(n, global_map).pose)
                # elif planner.global_map.getValFromPoint(splitPoint) < planner.costMapExpansion:
                #                    globalPathServ = getGlobalPath(nav.navBot.cur.pose, planner.node2pose(Node(splitPoint,1,splitPoint,None),global_map).pose)
                else:
                    continue
                globalPath = globalPathServ.path
                if (globalPath.poses):
                    print
                    "returning node"
                    return n
                for n2 in fullFrontier:
                    nodeDict[n2.key()] = n2
                print
                "Failed Path"
                return None

        # os.system("spd-say \"Breadth First\"")
        curNode = fullFrontier[int(len(fullFrontier) / 2)]
        nodes = {curNode.key(): curNode}
        frontier = Queue()
        frontier.put(curNode)
        nextFrontier = []
        tempMap = copy.deepcopy(global_map)

        # while there are still nodes in the frontier
        while not frontier.empty():
            node = frontier.get()
            if tempMap.getValFromPoint(node.point) == -1:
                continue
            if planner.global_map.getValFromPoint(node.point) < planner.costMapExpansion:
                return node
            # if the node was no a frontier, expand the node and add it to the back of the queue
            tempNodes = node.createNewNodes(nodes, tempMap, 101)
            for n in tempNodes:
                nodes[n.key()] = n
                frontier.put(n)

        print
        "no valid node in frontier"
        # if len(endPoints) >= 2:
        #     print "found bounded edge"
        #     tempNode =  Node(Point((endPoints[1].point.x+endPoints[0].point.x)/2,(endPoints[1].point.y+endPoints[0].point.y)/2),1,start.point,None)
        #     if planner.global_map.getValFromPoint(tempNode.point) > 60:
        #         print "blocked node"
        #         return None
        # else:
        #     print "found unbounded edge"
        #     return start
    else:
        print
        "no Frontier"
        return None


def exploreMap():
    global reachedGoal

    # os.system("spd-say \"started Mapping\"")
    # nav.navBot.rotateCircle()
    rospy.sleep(5)

    print
    "starting search"

    waypoint = getNextWaypoint()

    while waypoint and not rospy.is_shutdown():
        goal_pub.publish(waypoint)
        print
        "Navigating to: " + str(waypoint.pose.position.x) + "," + str(waypoint.pose.position.y)
        nav.stopDrive = False
        nav.navToPose(waypoint)
        if not nav.stopDrive:
            pass
            # nav.navBot.rotateCircle()
        waypoint = getNextWaypoint()

    os.system("spd-say \"Done Mapping\"")
    print
    "finished exploring map"


def mapCallback(data_map):
    global global_map
    global_map = Grid(data_map.info.width, data_map.info.height, data_map.data, data_map.info, data_map.header.frame_id)
    nav.stopDrive = True
    print("New Map!")


def run():
    global goal_pub
    global frontier_pub
    global getGlobalPath

    rospy.init_node('exploration_node')
    navType = rospy.get_param('~nav', default='rbe')
    if navType != 'rbe': rospy.delete_param('~nav')
    if navType == 'rbe':
        print("Using rbe nav")
        goal_pub = rospy.Publisher('/rbe_goal', PoseStamped, queue_size=1)
    else:
        print
        "using gMapping nav"
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    planner.init()
    nav.init()

    frontier_pub = rospy.Publisher('/frontier/frontier', GridCells, queue_size=1)
    getGlobalPath = rospy.ServiceProxy('global_path', CalcPath)

    # status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, statusCallback, queue_size=1)
    map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=1)

    exploreMap()


if __name__ == '__main__':
    run()
