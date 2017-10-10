#!/usr/bin/env python
# coding: utf-8

# This node communicates with the actionlib server of move_base
# by sending it goals as a Pose in the /map frame

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

    def get_map(self):
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
        self.pose_origin = (m.info.origin.position.x, m.info.origin.position.y, (
        tf.transformations.euler_from_quaternion((m.info.origin.orientation.x, m.info.origin.orientation.y,
                                                  m.info.origin.orientation.z, m.info.origin.orientation.w)))[2])
        self.map_data = np.array(m.data).reshape((self.map_height,
                                                  self.map_width))
        return self.map_data

    @staticmethod
    def plot_map(map_data, filename, robot_pose_pix=None):
        map_height, map_width = map_data.shape

        image_array = np.zeros((map_height, map_width, 3), dtype=int)
        image_array[map_data == Mapper.unknown_cell] = (255, 255, 255)
        image_array[map_data == Mapper.free_cell] = (125, 125, 125)
        image_array[map_data == Mapper.obstacle_cell] = (0, 0, 0)

        # Plot the current position of the robot
        if (robot_pose_pix is not None):
            x_im, y_im, theta_im = robot_pose_pix
            for i in range(-3, 4):
                for j in range(-3, 4):
                    image_array[y_im + i, x_im + j] = (255, 0, 0)

            ## We draw a ray to indicate the orientation of the robot
            for i in range(10):
                image_array[int(y_im + i * sin(theta_im)), int(x_im + i * cos(theta_im))] = (0, 0, 255)

        # Save the file as uncompressed ppm
        print("Saving the image into %s.ppm" % filename)
        f = open(filename + ".ppm", 'w')
        f.write("P3\n")
        f.write("{} {}\n".format(map_height, map_width))
        f.write("255\n")
        data = ""
        for i in range(map_height):
            for j in range(map_width):
                data += " ".join(map(str, list(image_array[i, j, :]))) + " "
            data += "\n"
        f.write(data)
        f.close()

    def pose_to_pix(self, pose_robot):
        '''
        Given a position (x,y,theta) in the map frame, returns the position (x,y,theta) in the image frame
        Be sure to call get_map before calling pose_to_pix
        because pose_to_pix makes use of map metadata that are gathered
        when calling get_map
        '''
        if (self.map_height == None):
            raise Exception("I cannot perform conversion if no map has been updated !")

        # We first convert pose_robot from the "map" frame to the image frame
        x_robot, y_robot, theta_robot = pose_robot
        x_origin, y_origin, theta_origin = self.pose_origin
        ###### For the position
        # The translation and scaling
        xr_in_im = (x_robot - x_origin) / self.map_resolution * cos(-theta_origin) - (
                                                                                     y_robot - y_origin) / self.map_resolution * sin(
            -theta_origin)
        yr_in_im = (y_robot - y_origin) / self.map_resolution * sin(-theta_origin) + (
                                                                                     y_robot - y_origin) / self.map_resolution * cos(
            -theta_origin)
        # And apply a rotation
        theta_in_im = theta_robot - theta_origin
        return (int(xr_in_im), int(yr_in_im), theta_in_im)

    def pix_to_pose(self, pose_robot_in_im):
        '''
        Given a position (x,y,theta) in the image frame, returns the position (x,y,theta) in the map_frame
        Be sure to call get_map before calling pix_to_pose
        because pix_to_pose makes use of map metadata that are gathered
        when calling get_map
        '''
        if (self.map_height == None):
            raise Exception("I cannot perform conversion if no map has been updated !")

        # We first convert pose_robot from the "map" frame to the image frame
        x_robot_in_im, y_robot_in_im, theta_robot_in_im = pose_robot_in_im
        x_origin, y_origin, theta_origin = self.pose_origin

        x_robot = x_robot_in_im * self.map_resolution * cos(theta_origin) - y_robot_in_im * self.map_resolution * sin(
            theta_origin) + x_origin
        y_robot = x_robot_in_im * self.map_resolution * sin(theta_origin) + y_robot_in_im * self.map_resolution * cos(
            theta_origin) + y_origin
        theta_robot = theta_robot_in_im + theta_origin
        return (x_robot, y_robot, theta_robot)

    @staticmethod
    def get_cell_status(map_image, pix_pose):
        return map_image[pix_pose[1], pix_pose[0]]

    @staticmethod
    def set_cell_status(map_image, pix_pose, new_status):
        map_image[pix_pose[1], pix_pose[0]] = new_status


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


rospy.init_node('explore_client')

######################################### This code is for testing the API
base_link = 'Pioneer_p3dx'
target_frame = 'map'
map_frame = 'map'
mapper = Mapper(base_link, map_frame)

############ Tests the utilities of the Mapper class
print("I'm requesting the map and saving it into map.ppm")
m = mapper.get_map()
Mapper.plot_map(m, 'map')

########### Test the conversions between the map frame of reference and the image frame of reference
print("I'm testing the conversions between poses in the map frame and poses in the image frame")

# We first call get_map because the conversion methods pix_to_pose
# and pose_to_pix make use of map metadata got when calling get_map
mapper.get_map()

print("Current position of the robot in the {} frame".format(map_frame))
x, y, t = mapper.get_current_xyt()
print("x: {} m, y : {} m, theta: {} rad \n".format(x, y, t))

print("Which means the following coordinates in the image space (in pixels)")
x_im, y_im, theta_im = mapper.pose_to_pix(mapper.get_current_xyt())
print("x: {} px , y : {} pix , theta: {} rad\n".format(x_im, y_im, theta_im))

print("Which means the following coordinates in the map frame if I project back the pixels coordinates:")
x, y, t = mapper.pix_to_pose((x_im, y_im, theta_im))
print("x: {} m, y : {} m, theta: {} rad \n".format(x, y, t))
print("These do not match exactly the true position because the map discretizes the positions")

######################################### ^^^^^^^^^^^^^^^^^^^^^^^^^^

####################################################################
########## Let us go for the exploration
explorer = Explorer(base_link, map_frame)
explorer.explore()

# Once the exploration is finished, we save the final map
m = mapper.get_map()
Mapper.plot_map(m, 'final_map')
