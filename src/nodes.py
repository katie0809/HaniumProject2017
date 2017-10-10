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

