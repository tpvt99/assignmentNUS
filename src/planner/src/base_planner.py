#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np

from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from const import *
from math import *
import copy
import argparse
import json
import time

# My import
import pandas as pd
import pickle

ROBOT_SIZE = 0.2552  # width and height of robot in terms of stage unit


def dump_action_table(action_table, filename):
    """dump the MDP policy into a json file

    Arguments:
        action_table {dict} -- your mdp action table. It should be of form {'1,2,0': (1, 0), ...}
        filename {str} -- output filename
    """
    tab = dict()
    for k, v in action_table.items():
        key = [str(i) for i in k]
        key = ','.join(key)
        tab[key] = v

    with open(filename, 'w') as fout:
        json.dump(tab, fout)


class Planner(object):
    def __init__(self, world_width, world_height, world_resolution, inflation_ratio=3):
        """init function of the base planner. You should develop your own planner
        using this class as a base.

        For standard mazes, width = 200, height = 200, resolution = 0.05. 
        For COM1 map, width = 2500, height = 983, resolution = 0.02

        Arguments:
            world_width {int} -- width of map in terms of pixels
            world_height {int} -- height of map in terms of pixels
            world_resolution {float} -- resolution of map

        Keyword Arguments:
            inflation_ratio {int} -- [description] (default: {3})
        """
        rospy.init_node('planner')
        self.map = None
        self.pose = None
        self.goal = None
        self.path = None
        self.action_seq = None  # output
        self.aug_map = None  # occupancy grid with inflation
        self.action_table = {}

        self.world_width = world_width
        self.world_height = world_height
        self.resolution = world_resolution
        self.inflation_ratio = inflation_ratio
        self.map_callback()
        self.sb_obs = rospy.Subscriber('/scan', LaserScan, self._obs_callback)
        self.sb_pose = rospy.Subscriber(
            '/base_pose_ground_truth', Odometry, self._pose_callback)
        self.sb_goal = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self._goal_callback)
        self.controller = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=10)
        rospy.sleep(1)

    def map_callback(self):
        """Get the occupancy grid and inflate the obstacle by some pixels. You should implement the obstacle inflation yourself to handle uncertainty.
        """
        self.map = rospy.wait_for_message('/map', OccupancyGrid).data

        # TODO: FILL ME! implement obstacle inflation function and define self.aug_map = new_mask

        # FOR EXP, output map as 2d
        #pd.DataFrame(np.array(self.map).reshape(self.world_height,self.world_width)).to_csv("original_map.csv", index=False)
        #pd.DataFrame(np.flipud(np.array(self.map).reshape(self.world_height,self.world_width))).to_csv("original_map_flip.csv", index=False)

        # Step 1. Convert to numpy 2d array
        new_map = np.array(self.map).reshape(self.world_height, self.world_width)

        # Step 2. Get the obstacles
        print(np.unique(new_map))
        #assert np.array_equal(np.unique(new_map), np.array([-1, 100])) # Ensuring map has only 2 values: -1 for Unknown, 100 for obstacles
        obstacles = np.where(new_map == 100)

        # Step 3. Inflation
        flip_type = 1 # 1 mean inflate left and right same size as inflation_ratio, #2 means divide inflation equally on both side
        if flip_type == 1:
            for ob_x, ob_y in zip(*obstacles):
                mask_row_top = ob_x - self.inflation_ratio
                mask_row_bot = ob_x + self.inflation_ratio+1
                mask_col_left = ob_y - self.inflation_ratio
                mask_col_right = ob_y + self.inflation_ratio+1
                if mask_row_top < 0:
                    mask_row_top = 0
                if mask_col_left < 0:
                    mask_col_left = 0
                if mask_row_bot > self.world_height: # Index is equal to width because in python, the last index is not get
                    mask_row_bot = self.world_height
                if mask_col_right > self.world_width:
                    mask_col_right = self.world_width
                new_map[mask_row_top:mask_row_bot, mask_col_left:mask_col_right] = 100 # 100 means obstacle
        else:
            for ob_x, ob_y in zip(*obstacles):
                mask_row_top = ob_x - self.inflation_ratio//2
                mask_row_bot = ob_x + self.inflation_ratio//2+1
                mask_col_left = ob_y - self.inflation_ratio//2
                mask_col_right = ob_y + self.inflation_ratio//2+1
                if mask_row_top < 0:
                    mask_row_top = 0
                if mask_col_left < 0:
                    mask_col_left = 0
                if mask_row_bot > self.world_height: # Index is equal to width because in python, the last index is not get
                    mask_row_bot = self.world_height
                if mask_col_right > self.world_width:
                    mask_col_right = self.world_width
                new_map[mask_row_top:mask_row_bot, mask_col_left:mask_col_right] = 100 # 100 means obstacle

        # Because some map does not have borders, inflate the border by the inflation_rate

        for col in range(self.world_width):# Inflate the top border
            new_map[0:self.inflation_ratio, col] = 100
        for col in range(self.world_width):# Inflate the bottom border
            new_map[-self.inflation_ratio:, col] = 100
        for row in range(self.world_height):# Inflate the left border
            new_map[row, 0:self.inflation_ratio] = 100
        for row in range(self.world_height): # Inflate the right border
            new_map[row, -self.inflation_ratio:] = 100



        self.aug_map_2d = new_map # Easier to manipulate
        self.aug_map_2d = np.flipud(self.aug_map_2d)
        new_map = tuple(new_map.reshape(self.world_width*self.world_height))

        # FOR EXP, output map as 2d
        #pd.DataFrame(np.array(new_map).reshape(self.world_height,self.world_width)).to_csv("inflated_map.csv", index=False)
        #pd.DataFrame(np.flipud(np.array(new_map).reshape(self.world_height,self.world_width))).to_csv("inflated_map_flip.csv", index=False)

        # you should inflate the map to get self.aug_map
        self.aug_map = copy.deepcopy(new_map)


    def _pose_callback(self, msg):
        """get the raw pose of the robot from ROS

        Arguments:
            msg {Odometry} -- pose of the robot from ROS
        """
        self.pose = msg

    def _goal_callback(self, msg):
        self.goal = msg
        self.generate_plan()

    def _get_goal_position(self):
        goal_position = self.goal.pose.position
        return (goal_position.x, goal_position.y)

    def set_goal(self, x, y, theta=0):
        """set the goal of the planner

        Arguments:
            x {int} -- x of the goal
            y {int} -- y of the goal

        Keyword Arguments:
            theta {int} -- orientation of the goal; we don't consider it in our planner (default: {0})
        """
        a = PoseStamped()
        a.pose.position.x = x
        a.pose.position.y = y
        a.pose.orientation.z = theta
        self.goal = a

    def _obs_callback(self, msg):
        """get the observation from ROS; currently not used in our planner; researve for the next assignment

        Arguments:
            msg {LaserScan} -- LaserScan ROS msg for observations
        """
        self.last_obs = msg

    def _d_from_goal(self, pose):
        """compute the distance from current pose to the goal; only for goal checking

        Arguments:
            pose {list} -- robot pose

        Returns:
            float -- distance to the goal
        """
        goal = self._get_goal_position()
        return sqrt((pose[0] - goal[0])**2 + (pose[1] - goal[1])**2)

    def _check_goal(self, pose):
        """Simple goal checking criteria, which only requires the current position is less than 0.25 from the goal position. The orientation is ignored

        Arguments:
            pose {list} -- robot post

        Returns:
            bool -- goal or not
        """
        if self._d_from_goal(pose) < 0.25:
            return True
        else:
            return False

    def create_control_msg(self, x, y, z, ax, ay, az):
        """a wrapper to generate control message for the robot.

        Arguments:
            x {float} -- vx
            y {float} -- vy
            z {float} -- vz
            ax {float} -- angular vx
            ay {float} -- angular vy
            az {float} -- angular vz

        Returns:
            Twist -- control message
        """
        message = Twist()
        message.linear.x = x
        message.linear.y = y
        message.linear.z = z
        message.angular.x = ax
        message.angular.y = ay
        message.angular.z = az
        return message

    def index_from_map_to_real(self, x, y, is_discrete = True):
        '''
        Function to map from pixel of aug_map to real stage map
        :param x: is a pixel (integer type)
        :param y: is a pixel (integer type)
        :return value: a floating point if is_discrete False, else a round integer
        '''
        real_x = x * self.resolution
        real_y = y * self.resolution
        if is_discrete:
            real_x = int(round(real_x))
            real_y = int(round(real_y))
        return real_x, real_y

    def index_from_real_to_map(self, x, y):
        '''
        Function to map from stage map to pixel on aug_map
        :param x is an index of real map, can be integer or float
        :param y is an index of real map, can be integer or float
        :return an integer list
        '''
        map_x = x / self.resolution
        map_y = y / self.resolution
        return int(round(map_x)), int(round(map_y))

    def generate_plan(self):
        """TODO: FILL ME! This function generates the plan for the robot, given a goal.
        You should store the list of actions into self.action_seq.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree

        In continuous case (task 2), the robot can have arbitrary orientations

        Each action could be: (v, \omega) where v is the linear velocity and \omega is the angular velocity
        """
        #raise NotImplementedError
        pass


    def get_current_continuous_state(self):
        """Our state is defined to be the tuple (x,y,theta). 
        x and y are directly extracted from the pose information. 
        Theta is the rotation of the robot on the x-y plane, extracted from the pose quaternion. For our continuous problem, we consider angles in radians

        Returns:
            tuple -- x, y, \theta of the robot
        """
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        orientation = self.pose.pose.pose.orientation
        ori = [orientation.x, orientation.y, orientation.z,
               orientation.w]

        phi = np.arctan2(2 * (ori[0] * ori[1] + ori[2] * ori[3]), 1 - 2 *
                         (ori[1] ** 2 + ori[2] ** 2))
        return (x, y, phi)

    def get_current_discrete_state(self):
        """Our state is defined to be the tuple (x,y,theta). 
        x and y are directly extracted from the pose information. 
        Theta is the rotation of the robot on the x-y plane, extracted from the pose quaternion. For our continuous problem, we consider angles in radians

        Returns:
            tuple -- x, y, \theta of the robot in discrete space, e.g., (1, 1, 1) where the robot is facing north
        """
        x, y, phi = self.get_current_continuous_state()
        def rd(x): return int(round(x))
        return rd(x), rd(y), rd(phi / (np.pi / 2))

    def collision_checker(self, x, y):
        """TODO: FILL ME!
        You should implement the collision checker.
        Hint: you should consider the augmented map and the world size
        
        Arguments:
            x {float} -- current x of robot
            y {float} -- current y of robot
        
        Returns:
            bool -- True for collision, False for non-collision
        """

        # Step 1. Return true position from world map to augmented map
        x, y = self.index_from_real_to_map(x, y)
        robot_inflation = int(round(ROBOT_SIZE / self.resolution)) # loosely the size a bit :')
        true_x, true_y = self.world_height - y, x
        inflate_row_top = true_x - robot_inflation
        inflate_row_bot = true_x + robot_inflation
        inflate_col_left = true_y - robot_inflation
        inflate_col_right = true_y + robot_inflation

        if inflate_row_top < 0:
            inflate_row_top = 0
        if inflate_row_bot > self.world_height:
            inflate_row_bot = self.world_height
        if inflate_col_left < 0:
            inflate_col_left = 0
        if inflate_col_right > self.world_width:
            inflate_col_right = self.world_width

        #mask = self.aug_map_2d[true_x-robot_inflation:true_x+robot_inflation, true_y-robot_inflation:true_y+robot_inflation]
        mask = self.aug_map_2d[inflate_row_top:inflate_row_bot, inflate_col_left:inflate_col_right]
        indexOfCollision = np.where(mask == 100)
        if indexOfCollision[0].size == 0:
            return False
        return True


    def motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        """predict the next pose of the robot given controls. Returns None if the robot collide with the wall
        The robot dynamics are provided in the homework description

        Arguments:
            x {float} -- current x of robot
            y {float} -- current y of robot
            theta {float} -- current theta of robot
            v {float} -- linear velocity 
            w {float} -- angular velocity

        Keyword Arguments:
            dt {float} -- time interval. DO NOT CHANGE (default: {0.5})
            frequency {int} -- simulation frequency. DO NOT CHANGE (default: {10})

        Returns:
            tuple -- next x, y, theta; return None if has collision
        """
        num_steps = int(dt * frequency)
        dx = 0
        dy = 0
        for i in range(num_steps):
            if w != 0:
                dx = - v / w * np.sin(theta) + v / w * \
                    np.sin(theta + w / frequency)
                dy = v / w * np.cos(theta) - v / w * \
                    np.cos(theta + w / frequency)
            else:
                dx = v*np.cos(theta)/frequency
                dy = v*np.sin(theta)/frequency
            x += dx
            y += dy

            if self.collision_checker(x, y):
                return None
            theta += w / frequency
        return x, y, theta

    def discrete_motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        """discrete version of the motion predict. Note that since the ROS simulation interval is set to be 0.5 sec
        and the robot has a limited angular speed, to achieve 90 degree turns, we have to execute two discrete actions
        consecutively. This function wraps the discrete motion predict.

        Please use it for your discrete planner.

        Arguments:
            x {int} -- current x of robot
            y {int} -- current y of robot
            theta {int} -- current theta of robot
            v {int} -- linear velocity
            w {int} -- angular velocity (0, 1, 2, 3)

        Keyword Arguments:
            dt {float} -- time interval. DO NOT CHANGE (default: {0.5})
            frequency {int} -- simulation frequency. DO NOT CHANGE (default: {10})

        Returns:
            tuple -- next x, y, theta; return None if has collision
        """
        w_radian = w * np.pi/2
        first_step = self.motion_predict(x, y, theta*np.pi/2, v, w_radian)
        if first_step:
            second_step = self.motion_predict(
                first_step[0], first_step[1], first_step[2], v, w_radian)
            if second_step:
                return (round(second_step[0]), round(second_step[1]), round(second_step[2] / (np.pi / 2)) % 4)
        return None

    def publish_control(self):
        """publish the continuous controls (task 2)
        """
        for action in self.action_seq:
            msg = self.create_control_msg(action[0], 0, 0, 0, 0, action[1])
            self.controller.publish(msg)
            rospy.sleep(0.6)

    def publish_discrete_control(self):
        """publish the discrete controls (task 1)
        """
        for action in self.action_seq:
            msg = self.create_control_msg(
                action[0], 0, 0, 0, 0, action[1]*np.pi/2)
            self.controller.publish(msg)
            rospy.sleep(0.6)
            self.controller.publish(msg)
            rospy.sleep(0.6)

    def publish_stochastic_control(self):
        """publish stochastic controls in MDP.  (task 3)
        In MDP, we simulate the stochastic dynamics of the robot as described in the assignment description.
        Please use this function to publish your controls in task 3, MDP. DO NOT CHANGE THE PARAMETERS :)
        We will test your policy using the same function.
        """
        current_state = self.get_current_state()
        actions = []
        new_state = current_state
        while not self._check_goal(current_state):
            current_state = self.get_current_state()
            action = self.action_table[current_state[0],
                                       current_state[1], current_state[2] % 4]
            if action == (1, 0):
                r = np.random.rand()
                if r < 0.9:
                    action = (1, 0)
                elif r < 0.95:
                    action = (np.pi/2, 1)
                else:
                    action = (np.pi/2, -1)
                print("Real action is: ({}, {}) but Sending actions: ({} {})".format(1 , 0, action[0], action[1] * np.pi / 2))
            else:
                print("Real action is: ({}, {}) but Sending actions: ({} {})".format(action[0] , action[1], action[0], action[1] * np.pi / 2))
            msg = self.create_control_msg(action[0], 0, 0, 0, 0, action[1]*np.pi/2)
            self.controller.publish(msg)
            rospy.sleep(0.6)
            self.controller.publish(msg)
            rospy.sleep(0.6)
            time.sleep(1)
            current_state = self.get_current_state()
            print(current_state)


if __name__ == "__main__":
    # TODO: You can run the code using the code below
    from task1_dsda_planner import DSDAPlanner
    from task2_csda_planner import CSDAPlanner
    from task3_mdp_planner import MDPPlanner

    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', type=str, default='1,8',
                        help='goal position')
    parser.add_argument('--com', type=int, default=0,
                        help="if the map is com1 map")
    parser.add_argument('--task', type=int, default=3) # 1 mean DSDA, 2 mean CSDA, 3 mean MD
    args = parser.parse_args()

    try:
        goal = [int(pose) for pose in args.goal.split(',')]
    except:
        raise ValueError("Please enter correct goal format")

    if args.com:
        width = 2500
        height = 983
        resolution = 0.02
    else:
        width = 200
        height = 200
        resolution = 0.05

    # TODO: You should change this value accordingly
    inflation_ratio = 3
    if args.task == 1:
        planner = DSDAPlanner(width, height, resolution, inflation_ratio=inflation_ratio)
    elif args.task == 2:
        inflation_ratio=3 # for maze2.jpg, goal (9,9)
        planner = CSDAPlanner(width, height, resolution, inflation_ratio=inflation_ratio)
    elif args.task == 3:
        planner = MDPPlanner(width, height, resolution, inflation_ratio=inflation_ratio)

    planner.set_goal(goal[0], goal[1])

    if planner.goal is not None:
        planner.generate_plan()

    # You could replace this with other control publishers
    if args.task == 1:
        planner.publish_discrete_control()
    elif args.task == 2:
        planner.publish_control()
    elif args.task == 3:
        planner.publish_stochastic_control()

    # save your action sequence
    if args.task == 1:
        result = np.array(planner.action_seq)
        np.savetxt("task1_actions/{0}_{1}_{2}_{3}.txt".format(1, 'com1', goal[0], goal[1]), result, fmt="%.2e")
    elif args.task == 2:
        result = np.array(planner.action_seq)
        np.savetxt("task2_actions/{0}_{1}_{2}_{3}.txt".format(2, 'com1', goal[0], goal[1]), result, fmt="%.2e")

    # for MDP, please dump your policy table into a json file
    if args.task == 3:
        dump_action_table(planner.action_table, 'task3_actions/{0}_{1}_{2}_{3}.json'.format(3, 'com1', goal[0], goal[1]))

    # spin the ros
    print('Done')
    rospy.spin()
