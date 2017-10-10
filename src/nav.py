#!/usr/bin/env python

import rospy, math, tf, numpy
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

# Turtlebot Dimension Constants
wheel_rad = 3.5 / 100.0  # cm to m
wheel_base = 23.0 / 100.0  # cm to m

stopDrive = False


class Navigate:
    runPID = True
    start = PoseStamped()
    goal = PoseStamped()
    cur = PoseStamped()
    goalAngle = 0.0
    angVel = 0.0
    linVel = 0.0
    angleThresh = 0.0
    distThresh = 0.0

    # PID Constants

    turnKp = 0.0
    turnKi = 0.0
    turnKd = 0.0

    driveKp = 0.0
    driveKi = 0.0
    driveKd = 0.0

    curTurnDelta = 0.0
    prevTurnDelta = 0.0
    totalTurnDelta = 0.0

    curDriveDelta = 0.0
    prevDriveDelta = 0.0
    totalDriveDelta = 0.0

    def __init__(self, angleThresh, distThresh, turtleBot):
        if turtleBot == "cap'n":
            self.turnKp = 2  # 1.25
            self.driveKp = .5  # .5
        elif turtleBot == "hawk-i":
            self.turnKp = 2.25
            self.driveKp = 1
            self.driveKi = 0.1
        elif turtleBot == "sim":
            self.turnKp = 2  # 1.25
            self.driveKp = .5  # .5
        else:
            turtleBot = "not a"

        print
        "You're on the *" + turtleBot + "* bot!"

        self.angleThresh = angleThresh
        self.distThresh = distThresh
        self.resetPID()
        pass

    # Publishes Twist messages
    def pubTwist(self):
        global pub
        msg = Twist()
        linVel = numpy.clip(self.linVel, 0, 0.3)
        msg.linear.x = linVel
        aVel = numpy.clip(self.angVel, -0.3, 0.3)
        msg.angular.z = aVel
        pub.publish(msg)

    # drives to the pose of goal
    # goal is a poseStamped
    def goToPose(self, goal):

        self.start = self.cur
        self.goal = goal

        # get current pose and orientation
        self.rotateTowardPose(goal)
        rospy.sleep(2)
        self.driveStraight()
        rospy.sleep(2)

    def getCurrentAngle(self):

        quat = self.cur.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        return yaw

    # This function accepts a speed and a distance for the robot to move in a straight line
    def driveStraight(self):

        timeDelta = 0.05

        # initialize position
        goalX = self.goal.pose.position.x
        goalY = self.goal.pose.position.y
        atGoal = False

        while (not atGoal and not rospy.is_shutdown()):
            self.prevDriveDelta = self.curDriveDelta
            tempCur = transformer.transformPose(self.goal.header.frame_id, self.cur)
            curX = tempCur.pose.position.x
            curY = tempCur.pose.position.y
            self.curDriveDelta = math.sqrt((goalX - curX) ** 2 + (goalY - curY) ** 2)
            self.totalDriveDelta += self.curDriveDelta
            self.goalAngle = math.atan2(goalY - curY, goalX - curX)
            atGoal = (self.curDriveDelta <= self.distThresh)

            self.linVel = (
                          self.driveKp * self.curDriveDelta + self.driveKi * self.totalDriveDelta * timeDelta - self.driveKd * abs(
                              self.prevDriveDelta - self.curDriveDelta) / timeDelta) * (1 - abs(self.angVel) / 0.5)
            rospy.sleep(timeDelta)

        # stop when goal is reached
        self.resetPID()
        self.linVel = 0.0
        self.angVel = 0.0

    def rotateTowardPose(self, goal):

        tempCur = transformer.transformPose(goal.header.frame_id, self.cur)
        curX = tempCur.pose.position.x
        curY = tempCur.pose.position.y

        assert isinstance(goal, PoseStamped)

        goalX = goal.pose.position.x
        goalY = goal.pose.position.y

        dX = goalX - curX
        dY = goalY - curY

        self.rotateTo(math.atan2(dY, dX))

    # Accepts an angle and makes the robot rotate around it. Assume there's no reason for
    def rotateTo(self, angle):
        self.goalAngle = angle
        timeRes = 0.05

        # initialize position
        atGoal = False

        self.curTurnDelta = math.atan2(math.sin(self.goalAngle - self.getCurrentAngle()),
                                       math.cos(self.goalAngle - self.getCurrentAngle()))

        while (not atGoal and not rospy.is_shutdown()):
            # print "turning: " + str(self.curTurnDelta)
            atGoal = (abs(self.curTurnDelta) <= self.angleThresh)

            self.linVel = 0.0
            rospy.sleep(timeRes)

        # stop when goal is reached
        self.angVel = 0.0
        self.linVel = 0.0
        self.resetPID()

    # Turns in a circle, returns when done
    def rotateCircle(self):
        self.runPID = False
        # initialize position
        initAng = self.getCurrentAngle()
        atGoal = False
        print("Circle Start")

        self.linVel = 0.0
        self.angVel = 2
        rospy.sleep(2)
        while (not atGoal and not rospy.is_shutdown()):
            if not abs(self.getCurrentAngle() - initAng) < self.angleThresh:
                # keep going at given speed

                self.linVel = 0.0
                # self.angVel = 1
                rospy.sleep(0.1)
            else:
                atGoal = True
        # stop when goal is reached
        print("Circle Done")
        self.runPID = True
        self.resetPID()

    def updatePID(self, event):
        if not self.runPID:
            self.pubTwist()
            return

        self.prevTurnDelta = self.curTurnDelta
        timeDelta = .05  # abs(event.current_real - event.last_real)
        self.curTurnDelta = math.atan2(math.sin(self.goalAngle - self.getCurrentAngle()),
                                       math.cos(self.goalAngle - self.getCurrentAngle()))
        self.totalTurnDelta += self.curTurnDelta * timeDelta

        self.angVel = (
            self.turnKp * self.curTurnDelta + self.turnKi * self.totalTurnDelta * timeDelta - self.turnKd * abs(
                self.prevTurnDelta - self.curTurnDelta) / timeDelta)

        self.pubTwist()

    def resetPID(self):
        self.goalAngle = self.getCurrentAngle()

        self.curTurnDelta = 0.0
        self.totalTurnDelta = 0.0
        self.prevTurnDelta = 0.0

        self.curDriveDelta = 0.0
        self.totalDriveDelta = 0.0
        self.prevDriveDelta = 0.0


# Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        print
        "Bumper pressed!"
        # do bumper stuff here


# handles the current position of the robot
def odomCallback(event):
    global pose_pub
    global navBot

    assert isinstance(navBot, Navigate)
    assert isinstance(event, Odometry)

    robPose = PoseStamped()
    robPose.header = event.header
    robPose.header.stamp = rospy.Time(0)
    robPose.pose = event.pose.pose

    navBot.cur = transformer.transformPose('map', robPose)

    pose_pub.publish(robPose)


def getAngleFromPose(pose):
    quat = pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    return yaw


# creates a path and uses that path to move to the location
def navToPose(goal):
    print
    "Starting Navigation!"
    # get path from A*
    while not stopDrive:
        # find the global path
        globalPathServ = getGlobalPath(navBot.cur.pose, goal.pose)
        globalPath = globalPathServ.path
        if globalPath.poses:  # if there's something in the list of poses
            print
            " Global Navigation"
            navBot.rotateTowardPose(globalPath.poses[0])  # rotate to update local map
            localPathServ = getLocalPath(navBot.cur.pose, globalPath.poses[0].pose)
            localPath = localPathServ.path
            while not localPath.poses:  # if there's nothing in the list of poses
                print
                " - blocked global waypoint, moving to next"
                del globalPath.poses[0]
                if not globalPath.poses:
                    print
                    " - no valid path, ending navigation"
                    return
                localPathServ = getLocalPath(navBot.cur.pose, globalPath.poses[0].pose)
                localPath = localPathServ.path
            print
            " Local Navigation"
            navBot.goToPose(localPath.poses[0])
        else:
            print
            " - goal not navigable, ending navigation"
            break
        gridDist = math.sqrt((globalPath.poses[-1].pose.position.x - navBot.cur.pose.position.x) ** 2 + (
        globalPath.poses[-1].pose.position.y - navBot.cur.pose.position.y) ** 2)
        if (len(globalPath.poses) == 1 and len(localPath.poses) == 1) or gridDist <= navBot.distThresh * 3:
            print
            " Finished All Navigation!"
            break
    navBot.rotateTo(getAngleFromPose(goal.pose))
    statuses = GoalStatusArray()
    status = GoalStatus()
    status.status = 3
    statuses.status_list.append(status)
    status_pub.publish(statuses)


def init():
    global pub
    global odom_tf
    global odom_list
    global start_pub
    global getGlobalPath
    global getLocalPath
    global transformer
    global pose_pub
    global status_pub

    global navBot

    navBot = Navigate(.1, .025, "hawk-i")  # pass these the resolutions (meters, radians) that you want.

    transformer = tf.TransformListener()

    rospy.sleep(1)

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None,
                          queue_size=10)  # Publisher for commanding robot motion
    pose_pub = rospy.Publisher('/robot_pose', PoseStamped, None, queue_size=10)
    status_pub = rospy.Publisher('/move_base/status', GoalStatusArray, None, queue_size=1)
    # start_pub = rospy.Publisher('/rviz_start',PoseWithCovarianceStamped,queue_size = 1)
    # bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events

    odom_sub = rospy.Subscriber('/odom', Odometry, odomCallback, queue_size=1)
    rospy.Timer(rospy.Duration(.05), navBot.updatePID)

    getGlobalPath = rospy.ServiceProxy('global_path', CalcPath)
    getLocalPath = rospy.ServiceProxy('local_path', CalcPath)


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('rbe3002_nav')

    init()
    goal_sub = rospy.Subscriber('/rbe_nav_goal', PoseStamped, navToPose, queue_size=3)

    rospy.sleep(rospy.Duration(1, 0))

    print
    "Starting navigation node"
    rospy.spin()

    print
    "Ended navigation node"
