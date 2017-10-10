#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.srv import GetMap, GetPlan
import tf
from math import pi, cos, sin, sqrt, atan2

import numpy as np


class Mapper:
    unknown_cell = -1
    free_cell = 0
    obstacle_cell = 100

    def __init__(self, base_link, map_frame):
        self.base_link = base_link
        self.map_frame = map_frame
        self.listener = tf.TransformListener()
        # The map is accessed by calling the service dynamic_map()
        rospy.wait_for_service("dynamic_map")
        self.map_service = rospy.ServiceProxy('dynamic_map', GetMap)
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.pose_origin = None
        self.map_data = None

    def get_current_xyt(self):
        '''
        Returns the current position/orientation (x,y,theta) of the robot in the map frame
        '''
        t0 = rospy.Time(0)
        self.listener.waitForTransform(self.map_frame, self.base_link, t0, rospy.Duration(1))
        ((x_robot, y_robot, z), rot) = self.listener.lookupTransform(self.map_frame, self.base_link, t0)
        euler = tf.transformations.euler_from_quaternion(rot)
        theta_robot = euler[2]
        return (x_robot, y_robot, theta_robot)

    def get_dynamic_map(self):
        '''
        Updates the map and returns an array with :
            -1 : unknown
            0 : free cell
            100 : walls
        '''
        m = self.map_service().map
        self.map_width = m.info.width
        self.map_height = m.info.height
        self.map_resolution = m.info.resolution
        self.map_data = np.array(m.data).reshape((self.map_height,
                                                  self.map_width))
        return self.map_data


class Explorer:
    def __init__(self, base_link, map_frame):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # We will use the make_plan service to test if a selected goal is reachable
        rospy.wait_for_service("/move_base/NavfnROS/make_plan")
        self.plan_service = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
        self.map_frame = map_frame
        self.goal_number = 0
        # We need a mapper to select the next goal
        self.mapper = Mapper(base_link, map_frame)
        # We keep track of the aborted goal taken as unreachable and never reconsider them
        self.aborted_goals = []

    def test_reachability(self, g):
        '''
        Tests if a goal is reachable. To do so, we invoke the make_plan service of move_base
        and check that the length of the plan is > 0 which means a plan exists
        and therefore the goal can be reached
        '''
        (px0, py0, pt0) = self.mapper.get_current_xyt()

        start_pose = PoseStamped()
        start_pose.header.stamp = rospy.Time.now()
        start_pose.header.frame_id = self.map_frame
        start_pose.pose.position.x = px0
        start_pose.pose.position.y = py0
        start_pose.pose.position.z = 0
        q = tf.transformations.quaternion_from_euler(0, 0, pt0)
        start_pose.pose.orientation.x = q[0]
        start_pose.pose.orientation.y = q[1]
        start_pose.pose.orientation.z = q[2]
        start_pose.pose.orientation.w = q[3]

        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.pose.position.x = g[0]
        goal_pose.pose.position.y = g[1]
        goal_pose.pose.position.z = 0
        q = tf.transformations.quaternion_from_euler(0, 0, g[2])
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        res = self.plan_service(start=start_pose, goal=goal_pose, tolerance=0.)
        return len(res.plan.poses) > 0

    def explore(self):
        # Let us first move around
        print("Move forward 1m.")
        x, y, t = self.mapper.get_current_xyt()
        result = self.reach_goal((x + cos(t), y + sin(t), t))

        print("Let us turn around by 180 deg.")
        x, y, t = self.mapper.get_current_xyt()
        result = self.reach_goal((x, y, t + pi))

        # We now look for one goal
        g = self.next_goal()
        while (g is not None):
            print("I want to move to {}".format(g))
            status = self.reach_goal(g)
            if (status == GoalStatus.SUCCEEDED):
                print("I successfully reached the goal")
            else:
                print("Move base failed with status : {}".format(status))
                p = self.mapper.pose_to_pix(g)
                self.aborted_goals.append((p[0], p[1]))
                print("Status : {}".format(status))
            g = self.next_goal()

    def next_goal(self):
        # We must return a pose (x,y,theta) in the map frame which is the next goal to reach
        # If no such goal exists, we must return None
        ################################################
        ### YOUR CODE GOES HERE
        # You must find a reachable goal. In order to prune unreachable goals
        # you can :
        # 1) diffuse from the current position of the robot
        #    along free_cells. You would find places that have good chances
        #    to be reachable
        # 2) discard the goals (in pixels) filled in the aborted_goals
        #    list. When move_base fails, the current goal is appended to
        #    this list. cf. the "explore" method above
        # 3) test the reachability of a pose in the map frame by
        #    invoking the test_reachability method.

        # Below is the current map with 3 cell states (free, obst., unkn)
        # you can get/set a cell state in this map by calling
        # Mapper.set_cell_status,  Mapper.get_cell_status
        current_map = self.mapper.get_map()

        ################################################
        return None

    def reach_goal(self, goal_pose):
        ''' Invokes move_base to move the robot to the pose goal_pose
            in the map_frame
        '''
        x, y, theta = goal_pose
        target_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        t0 = rospy.Time.now()
        goal = MoveBaseGoal()
        goal.target_pose.header.seq = self.goal_number
        goal.target_pose.header.stamp = t0
        goal.target_pose.header.frame_id = self.map_frame

        goal.target_pose.pose.position = Point(x, y, 0)
        goal.target_pose.pose.orientation.x = target_quat[0]
        goal.target_pose.pose.orientation.y = target_quat[1]
        goal.target_pose.pose.orientation.z = target_quat[2]
        goal.target_pose.pose.orientation.w = target_quat[3]

        # Sends the goal to the action server.
        self.client.send_goal(goal)

        # Waits for the server to finish performing the action.
        # let us wait 1 minute to reach the goal and abort otherwise
        reached_the_goal = self.client.wait_for_result(rospy.Duration.from_sec(60))
        if not reached_the_goal:
            rospy.logwarn("I was not able to reach the goal within the\
                          allocated time")
            self.client.cancel_goal()

        self.goal_number += 1

        # Prints out the result of executing the action
        return self.client.get_state()



if __name__ == "__main__":
    rospy.init_node('explore_client')

    ######################################### This code is for testing the API
    base_link = 'odom'
    target_frame = 'map'
    map_frame = 'map'

    mapper = Mapper(base_link, map_frame)
    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        map = mapper.get_dynamic_map()
        print(map)
        ########## Let us go for the exploration
        explorer = Explorer(base_link, map_frame)
        explorer.explore()
        rate.sleep()






