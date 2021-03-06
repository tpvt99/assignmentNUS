from __future__ import division

import numpy as np
from base_planner import Planner
from util import PriorityQueue
from util import euclideanHeuristic, manhattanHeuristic

class CSDAPlanner(Planner):
    def __init__(self, world_width, world_height, world_resolution, inflation_ratio):
        super(CSDAPlanner, self).__init__(world_width, world_height, world_resolution, inflation_ratio)
        # state = (1, 1, 0)
        # next_state1 = self.motion_predict(state[0], state[1], state[2], 1, np.pi/2)
        # next_state2 = self.motion_predict(state[0], state[1], state[2], 1, -np.pi/2)
        # a = self.discretize_state_actions(next_state1[0],next_state1[1],next_state1[2])
        # b = self.discretize_state_actions(next_state2[0],next_state2[1],next_state2[2])
        # pass


    def heuristic(self, currentState, goalState):
        #return np.sqrt((currentState[0]-goalState[0])**2 + (currentState[1] - goalState[1])**2)
        return abs(currentState[0] - goalState[0]) + abs(currentState[1] - goalState[1])

    def distance(self, prevState, currentState):
        return abs(currentState[0]-prevState[0]) + abs(currentState[1] - prevState[1])


    def discretize_state_actions(self, x, y, theta, MAX_ANGLE=8, RESOLUTION=4):
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


    def generate_plan(self):
        """TODO: FILL ME! This function generates the plan for the robot, given a goal.
        You should store the list of actions into self.action_seq.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree

        In continuous case (task 2), the robot can have arbitrary orientations

        Each action could be: (v, \omega) where v is the linear velocity and \omega is the angular velocity
        """

        #ACTION_SEQUENCES = [[(1,0)], [(1, np.pi/2)], [(1, -np.pi/2)], [(0,1)], [(0,-1)]]
        ACTION_SEQUENCES = [[(1, 0)], [(1, np.pi / 2)], [(1, -np.pi / 2)], [(0,1)]]
        COST_ACTION_SEQUENCES = [0, 0, 0, 0]


        start_cont_state = self.get_current_continuous_state()
        start_dist_state = self.discretize_state_actions(*start_cont_state)
        goalState = self._get_goal_position()
        closedSet = [] # To not add expanded state into the path
        fringe = PriorityQueue()
        seq_state_acts = [(start_cont_state, start_dist_state, "", 0)] # contState, disState, action, cost
        fringe.push(seq_state_acts, 0)
        self.action_seq = []

        gScore={}

        count = 0
        while True:
            if fringe.isEmpty():
                self.action_seq = []
                return

            currentPath = fringe.pop() # currentPath is a list where each element is (contState, disState, action_from_prev_to_this, totalCost)
            last_cont_state, last_dist_state, last_action, last_cost = currentPath[-1]
            print('Iteration {} Fring length: {}'.format(count, fringe.count))
            print('Iteration {} Best cost: {} Last path'.format(count, currentPath[-1]))
            #for path in currentPath:
            #    print(path)
            # If this is goal
            if self._check_goal(last_cont_state):
                for path in currentPath:
                    print(path)
                # Build the action sequences
                for item in currentPath:
                    actions = item[2]
                    if actions != "":
                        for action in actions:
                            self.action_seq.append(action)
                return

            if last_dist_state not in closedSet:
                closedSet.append(last_dist_state)
                #print('Iteration {} ClosedSet: {}'.format(count, closedSet))
                # Get the neighbors by executing actions
                for actions, actions_cost in zip(ACTION_SEQUENCES, COST_ACTION_SEQUENCES):
                    current_cont_state = last_cont_state[0], last_cont_state[1], last_cont_state[2]
                    next_cont_state = None
                    for action in actions:
                        x, y, theta, v, w = current_cont_state[0], current_cont_state[1], current_cont_state[2], action[0], action[1]
                        next_cont_state = self.motion_predict(x, y, theta, v, w)
                        if next_cont_state is None: # if either turn left or go forward fails, stop this action sequences
                            break
                        current_cont_state = next_cont_state
                    if next_cont_state is None:
                        continue
                    #Convert state (real) to aug_map state
                    map_last_state = (last_cont_state[0], last_cont_state[1], last_cont_state[2]) # Continuous
                    map_goal_state = (goalState[0], goalState[1], 0) # Discrete
                    map_next_state = (next_cont_state[0], next_cont_state[1], next_cont_state[2]) # Continuous
                    #Then discretize
                    map_last_state = self.discretize_state_actions(map_last_state[0], map_last_state[1], map_last_state[2])  # Continuous
                    map_goal_state = self.discretize_state_actions(map_goal_state[0], map_goal_state[1], 0)  # Discrete
                    map_next_state = self.discretize_state_actions(map_next_state[0], map_next_state[1], map_next_state[2])  # Continuous
                    # Calculate the cost
                    cost_to_come = last_cost + 2 #self.distance(map_last_state, map_next_state)  # 1 is cost-to-come from last to current node
                    cost_to_go = self.heuristic(map_next_state, map_goal_state)
                    newCost = cost_to_come + cost_to_go + actions_cost
                    # Discretize next_state
                    next_dist_state = self.discretize_state_actions(next_cont_state[0], next_cont_state[1], next_cont_state[2])

                    newPath = currentPath + [(next_cont_state, next_dist_state, actions, newCost-cost_to_go)]
                    # if map_next_state in gScore.keys():
                    #     if cost_to_come < gScore[map_next_state]:
                    #         fringe.push(newPath, newCost)
                    #         gScore[map_next_state] = cost_to_come
                    # else:
                    #     gScore[map_next_state] = cost_to_come
                    #     fringe.push(newPath, newCost)
                    fringe.push(newPath, newCost)
                    print('Iteration {} Actions: {} Cost: {} lead to Path: {}'.format(count, actions, newCost, newPath))

                count+=1
                print('-------------------')
