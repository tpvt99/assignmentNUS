from __future__ import division
from itertools import product
import numpy as np
from base_planner import Planner
from util import PriorityQueue
from util import euclideanHeuristic, manhattanHeuristic
import math

class MDPPlanner(Planner):
    def __init__(self, world_width, world_height, world_resolution, inflation_ratio):
        super(MDPPlanner, self).__init__(world_width, world_height, world_resolution, inflation_ratio)
        self.GAMMA = 0.9
        self.ACTION_SEQUENCES = [(0,0), (1, 0), (0, -1), (0, 1)]
        self.total_discrete_y = int(math.ceil(self.world_height * self.resolution)) # Round up in com1
        self.total_discrete_x = int(self.world_width * self.resolution)
        self.total_discrete_theta = 4  # 4 angles
        self.total_states = self.total_discrete_x * self.total_discrete_y * self.total_discrete_theta
        self.possible_states = list(product(range(self.total_discrete_x), range(self.total_discrete_y),
                                            range(self.total_discrete_theta)))
        self.values = np.zeros(shape=(self.total_discrete_x,  self.total_discrete_y, len(self.ACTION_SEQUENCES)))
        p = self.motion_predict(1, 1, np.pi/2, np.pi/2, 1)
        pass

    def discretize_state_actions(self, x, y, theta, MAX_ANGLE=4, RESOLUTION=2):
        '''
        x: position x of robot
        y:
        theta: direction
        MAX_ANGLE: the allowable angle turn of robot
        RESOLUTION: Number of discrete square per real unit stage
        '''
        dis_x = int(round(x/(1/RESOLUTION)))
        dis_y = int(round(y/(1/RESOLUTION)))
        dis_theta = int(round(theta / (np.pi/(MAX_ANGLE/2)))) % MAX_ANGLE
        return (dis_x, dis_y, dis_theta)

    def get_current_state(self):
        state = self.get_current_discrete_state()
        return tuple([int(x) for x in state])


    def build_transition(self):
        self.transition = {}
        for state in self.possible_states:
            if self.collision_checker(state[0], state[1]):
                continue
            self.transition[state] = {}
            for action in self.ACTION_SEQUENCES:
                self.transition[state][action] = {}

                if action == (0, -1) or action == (0, 1):
                    next_state = self.discrete_motion_predict(state[0], state[1], state[2], action[0], action[1])
                    if next_state is not None and self.collision_checker(next_state[0], next_state[1]) == False:
                        next_state = tuple([int(x) for x in next_state])  # just discrete
                        self.transition[state][action][next_state] = 1
                elif action == (1,0):
                    next_state = self.discrete_motion_predict(state[0], state[1], state[2], action[0], action[1])
                    if next_state is not None and self.collision_checker(next_state[0], next_state[1]) == False:
                        next_state = tuple([int(x) for x in next_state])
                        self.transition[state][(1,0)][next_state] = 0.9

                    # probability 0.05
                    for action in [(np.pi / 2, 1), (np.pi / 2, -1)]:
                        next_state = self.discrete_motion_predict(state[0], state[1], state[2], action[0], action[1])
                        if next_state is not None and self.collision_checker(next_state[0], next_state[1]) == False:
                            next_state = tuple([int(x) for x in next_state])
                            self.transition[state][(1,0)][next_state] = 0.05

    def build_rewards(self):
        self.rewards = {}
        for state in self.possible_states:
            if self.collision_checker(state[0], state[1]):
                continue
            self.rewards[state] = {}
            for action in self.ACTION_SEQUENCES:
                self.rewards[state][action] = {}

                if action == (0, -1) or action == (0, 1):
                    next_state = self.discrete_motion_predict(state[0], state[1], state[2], action[0], action[1])
                    if next_state is not None and self.collision_checker(next_state[0], next_state[1]) == False:
                        next_state = tuple([int(x) for x in next_state])  # just discrete
                        if self._check_goal((next_state[0], next_state[1])):
                            self.rewards[state][action][next_state] = 1
                        else:
                            self.rewards[state][action][next_state] = 0
                elif action == (1,0):
                    for action in [(np.pi / 2, 1), (np.pi / 2, -1), (1,0)]:
                        next_state = self.discrete_motion_predict(state[0], state[1], state[2], action[0], action[1])
                        if next_state is not None and self.collision_checker(next_state[0], next_state[1]) == False:
                            next_state = tuple([int(x) for x in next_state])
                            if self._check_goal((next_state[0], next_state[1])):
                                self.rewards[state][(1,0)][next_state] = 1
                            else:
                                self.rewards[state][(1,0)][next_state] = 0


    def generate_plan(self):
        """TODO: FILL ME! This function generates the plan for the robot, given a goal.
        You should store the list of actions into self.action_seq.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree

        In continuous case (task 2), the robot can have arbitrary orientations

        Each action could be: (v, \omega) where v is the linear velocity and \omega is the angular velocity
        """

        self.build_transition()
        self.build_rewards()

        policy = {}
        for state in self.possible_states:
            policy[state] = (0,0) # Initialize policy as STAY action

        iteration = 300
        itr = 0

        while itr < iteration:
            newValues = self.values.copy()
            for state in self.possible_states:
                if state not in self.transition.keys():
                    newValues[state] = 0
                    continue

                max_value_action = []
                transition = self.transition[state]
                reward = self.rewards[state]

                for action in self.ACTION_SEQUENCES:
                    value_action = 0
                    for next_state in transition[action].keys():
                        # print('----')
                        # print(next_state)
                        # print(action)
                        # print(transition)
                        # print(reward)
                        # print(transition[action][next_state])
                        # print(reward[action][next_state])
                        value_action += transition[action][next_state] * (reward[action][next_state] + self.GAMMA * self.values[next_state])
                    max_value_action.append(value_action)
                newValues[state] = np.max(np.array(max_value_action))
                argmax = int(np.argmax(np.array(max_value_action)))
                policy[state] = self.ACTION_SEQUENCES[argmax]

            if np.mean(np.abs(self.values-newValues)) < 1e-8:
                break
            itr+=1
            print('Iteration{} with mean values: {}', itr,np.mean(np.abs(self.values-newValues)))
            self.values = newValues.copy()

        # Build action table
        self.action_table = policy.copy()
