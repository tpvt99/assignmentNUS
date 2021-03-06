
from base_planner import Planner
from util import PriorityQueue
from util import euclideanHeuristic, manhattanHeuristic

class DSDAPlanner(Planner):
    def __init__(self, world_width, world_height, world_resolution, inflation_ratio):
        super(DSDAPlanner, self).__init__(world_width, world_height, world_resolution, inflation_ratio)
        self.heuristic = manhattanHeuristic

    def generate_plan(self):
        """TODO: FILL ME! This function generates the plan for the robot, given a goal.
        You should store the list of actions into self.action_seq.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree

        In continuous case (task 2), the robot can have arbitrary orientations

        Each action could be: (v, \omega) where v is the linear velocity and \omega is the angular velocity
        """
        # [(1,0)] mean forward
        # [(0,1), (1,0)] mean turn left, forward
        # [(0,-1), (1,0)] mean turn right, forward
        # [(0,1), (0,1), (1,0)] mean turn left, left, forward (Go backward)
        ACTION_SEQUENCES = [[(1,0)], [(0,1), (1,0)], [(0, -1), (1,0)], [(0,1),(0,1),(1,0)]]
        COST_ACTION_SEQUENCES = [0, 0.5, 0.5, 1]


        startState = self.get_current_discrete_state()
        goalState = self._get_goal_position()
        closedSet = [] # To not add expanded state into the path
        fringe = PriorityQueue()
        seq_state_acts = [(startState, "", 0)]
        fringe.push(seq_state_acts, 0)
        self.action_seq = []

        while True:
            if fringe.isEmpty():
                self.action_seq = []
                return

            currentPath = fringe.pop() # currentPath is a list where each element is (state, action_from_prev_state_to_this_state, cost_from_start_to_this_state)
            last_state, last_action, last_cost = currentPath[-1]

            # If this is goal
            if self._check_goal(last_state):
                print(currentPath)
                # Build the action sequences
                for item in currentPath:
                    actions = item[1]
                    if actions != "":
                        for action in actions:
                            self.action_seq.append(action)
                return
            if last_state not in closedSet:
                closedSet.append(last_state)
                # Get the neighbors by executing actions
                for index, actions in enumerate(ACTION_SEQUENCES):
                    current_state = last_state[0], last_state[1], last_state[2]
                    next_state = None
                    for action in actions:
                        x, y, theta, v, w = current_state[0], current_state[1], current_state[2], action[0], action[1]
                        next_state = self.discrete_motion_predict(x, y, theta, v, w)
                        if next_state is None: # if either turn left or go forward fails, stop this action sequences
                            break
                        next_state = tuple(int(x) for x in next_state)
                        current_state = next_state
                    if next_state is None:
                        continue
                    #Convert state (real) to aug_map state
                    map_last_state = (last_state[0], last_state[1])
                    map_goal_state = (goalState[0], goalState[1])
                    map_next_state = (next_state[0], next_state[1])
                    cost_to_come = last_cost - self.heuristic(map_last_state, map_goal_state) + 1 # 1 is cost-to-come from last to current node
                    cost_to_go = self.heuristic(map_next_state, map_goal_state)
                    newCost = cost_to_come + cost_to_go + COST_ACTION_SEQUENCES[index]
                    newPath = currentPath + [(next_state, actions, newCost)]
                    fringe.push(newPath, newCost)