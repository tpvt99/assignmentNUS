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

    def get_current_state(self):
        state = self.get_current_discrete_state()
        return tuple([int(x) for x in state])

    #LEGACY code
    # def build_transitions_v2(self):
    #     self.transitions = {}
    #     for state in self.possible_states:
    #         if self.collision_checker(state[0], state[1]):
    #             continue
    #         self.transitions[state] = {}
    #         for action in self.ACTION_SEQUENCES:
    #             self.transitions[state][action] = {}
    #
    #             if action == (0, -1) or action == (0, 1):
    #                 _, next_state = self.discrete_motion_predict_v2(state[0], state[1], state[2], action[0], action[1])
    #                 #next_state = tuple([int(x) for x in next_state])  # just discrete
    #                 self.transitions[state][action][next_state] = 1
    #             elif action == (1,0):
    #                 _, next_state = self.discrete_motion_predict_v2(state[0], state[1], state[2], action[0], action[1])
    #                 #next_state = tuple([int(x) for x in next_state])
    #                 self.transitions[state][(1,0)][next_state] = 0.9
    #
    #                 # probability 0.05
    #                 for action in [(np.pi / 2, 1), (np.pi / 2, -1)]:
    #                     _, next_state = self.discrete_motion_predict_v2(state[0], state[1], state[2], action[0], action[1])
    #                     #next_state = tuple([int(x) for x in next_state])
    #                     self.transitions[state][(1,0)][next_state] = 0.05
    #
    # def build_rewards_v2(self):
    #     self.rewards = {}
    #     for state in self.possible_states:
    #         if self.collision_checker(state[0], state[1]):
    #             continue
    #         self.rewards[state] = {}
    #         for action in self.ACTION_SEQUENCES:
    #             self.rewards[state][action] = {}
    #
    #             if action == (0, -1) or action == (0, 1):
    #                 isCollision, next_state = self.discrete_motion_predict_v2(state[0], state[1], state[2], action[0], action[1])
    #                 if not isCollision:
    #                     #next_state = tuple([int(x) for x in next_state])  # just discrete
    #                     if self._check_goal((next_state[0], next_state[1])):
    #                         self.rewards[state][action][next_state] = 1
    #                     else:
    #                         self.rewards[state][action][next_state] = 0
    #                 else:
    #                     #next_state = tuple([int(x) for x in next_state])
    #                     self.rewards[state][action][next_state] = -1
    #             elif action == (1,0):
    #                 for action in [(np.pi / 2, 1), (np.pi / 2, -1), (1,0)]:
    #                     isCollision, next_state = self.discrete_motion_predict_v2(state[0], state[1], state[2], action[0], action[1])
    #                     if not isCollision:
    #                         #next_state = tuple([int(x) for x in next_state])
    #                         if self._check_goal((next_state[0], next_state[1])):
    #                             self.rewards[state][(1,0)][next_state] = 1
    #                         else:
    #                             self.rewards[state][(1,0)][next_state] = 0
    #                     else:
    #                         #next_state = tuple([int(x) for x in next_state])
    #                         self.rewards[state][(1,0)][next_state] = -1

    def build_transitions(self):
        self.transitions = {}
        for state in self.possible_states:
            if self.collision_checker(state[0], state[1]):
                continue
            self.transitions[state] = {}
            for action in self.ACTION_SEQUENCES:
                self.transitions[state][action] = {}

                if action == (0, -1) or action == (0, 1):
                    next_state = self.discrete_motion_predict(state[0], state[1], state[2], action[0], action[1])
                    if next_state is not None and self.collision_checker(next_state[0], next_state[1]) == False:
                        next_state = tuple([int(x) for x in next_state])  # just discrete
                        if next_state not in self.transitions[state][action]:
                            self.transitions[state][action][next_state] = [1]
                        else:
                            self.transitions[state][action][next_state].append(1)
                elif action == (1,0):
                    next_state = self.discrete_motion_predict(state[0], state[1], state[2], action[0], action[1])
                    if next_state is not None and self.collision_checker(next_state[0], next_state[1]) == False:
                        next_state = tuple([int(x) for x in next_state])
                        if next_state not in self.transitions[state][(1, 0)]:
                            self.transitions[state][(1,0)][next_state] = [0.9]
                        else:
                            self.transitions[state][(1,0)][next_state].append(0.9)

                    # probability 0.05
                    for action in [(np.pi / 2, 1), (np.pi / 2, -1)]:
                        next_state = self.discrete_motion_predict(state[0], state[1], state[2], action[0], action[1])
                        if next_state is not None and self.collision_checker(next_state[0], next_state[1]) == False:
                            next_state = tuple([int(x) for x in next_state])
                            if next_state not in self.transitions[state][(1, 0)]:
                                self.transitions[state][(1, 0)][next_state] = [0.05]
                            else:
                                self.transitions[state][(1, 0)][next_state].append(0.05)

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
                            if next_state not in self.rewards[state][action]:
                                self.rewards[state][action][next_state] = [1]
                            else:
                                self.rewards[state][action][next_state].append(1)
                        else:
                            if next_state not in self.rewards[state][action]:
                                self.rewards[state][action][next_state] = [0]
                            else:
                                self.rewards[state][action][next_state].append(0)
                elif action == (1,0):
                    for action in [(np.pi / 2, 1), (np.pi / 2, -1), (1,0)]:
                        next_state = self.discrete_motion_predict(state[0], state[1], state[2], action[0], action[1])
                        if next_state is not None and self.collision_checker(next_state[0], next_state[1]) == False:
                            next_state = tuple([int(x) for x in next_state])
                            if self._check_goal((next_state[0], next_state[1])):
                                if next_state not in self.rewards[state][(1,0)]:
                                    self.rewards[state][(1,0)][next_state] = [1]
                                else:
                                    self.rewards[state][(1,0)][next_state].append(1)
                            else:
                                if next_state not in self.rewards[state][(1,0)]:
                                    self.rewards[state][(1, 0)][next_state] = [0]
                                else:
                                    self.rewards[state][(1, 0)][next_state].append(0)


    def generate_plan(self):
        """TODO: FILL ME! This function generates the plan for the robot, given a goal.
        You should store the list of actions into self.action_seq.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree

        In continuous case (task 2), the robot can have arbitrary orientations

        Each action could be: (v, \omega) where v is the linear velocity and \omega is the angular velocity
        """

        self.build_transitions()
        self.build_rewards()

        policy = {}
        for state in self.possible_states:
            policy[state] = (0,0) # Initialize policy as STAY action

        iteration = 1000
        itr = 0

        while itr < iteration:
            newValues = self.values.copy()
            for state in self.possible_states:
                if state not in self.transitions.keys():
                    newValues[state] = 0
                    continue

                max_value_action = []
                transition = self.transitions[state]
                reward = self.rewards[state]

                for action in self.ACTION_SEQUENCES:
                    value_action = 0
                    for next_state in transition[action].keys():
                        for zzz in range(len(transition[action][next_state])):
                            value_action += transition[action][next_state][zzz] * (
                                        reward[action][next_state][zzz] + self.GAMMA * self.values[next_state])
                    max_value_action.append(value_action)
                newValues[state] = np.max(np.array(max_value_action))
                argmax = int(np.argmax(np.array(max_value_action)))
                policy[state] = self.ACTION_SEQUENCES[argmax]

            if np.mean(np.abs(self.values-newValues)) < 1e-15:
                break
            itr+=1
            print('Iteration{} with mean values: {}'.format(itr,np.mean(np.abs(self.values-newValues))))
            self.values = newValues.copy()

        # Build action table
        self.action_table = policy.copy()
        #self.plot_matrix()

    def plot_matrix(self):
        import numpy as np
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(1,1, figsize=(15,15))

        xxx = np.flipud(np.transpose(self.values, (1,0, 2)))
        action_codes=["LEFT", "TOP", "RIGHT", "BOTTOM"]

        intersection_matrix = xxx[:,:,0]

        ax.matshow(intersection_matrix, cmap=plt.cm.Blues)
        for j in range(10):
            for i in range(10):
                c = "{}".format(round(intersection_matrix[j, i],2))
                ax.text(i, j, str(c), va='center', ha='center')

        plt.show()