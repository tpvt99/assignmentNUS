import numpy as np
import argparse
import pdb
import pickle
import copy


class Evaluator(object):
    def __init__(self, filename, aug_map, start, goal):
        self.filename = filename
        self.aug_map = aug_map
        self.start = np.array(start) 
        self.goal = np.array(goal) 
        self.gen_policy()

    def gen_policy(self):
        self.policy = np.genfromtxt(self.filename)

    def distance_function(self, v):
        return v * 0.5

    def simulate(self, actions):
        pose = self.start

        distance = 0
        for v, w in actions:
            pose = self.motion_predict(pose[0], pose[1], pose[2], v, w, aug_map)
            pose = np.array(pose)
            distance += self.distance_function()
            if pose is None:
                break
            
        if pose is None:
            # print("Collision!")
            return -1
        elif np.sum((pose[:2] - self.goal)**2) < 0.04:
            # print('Reach the goal, distance traveled: {}'.format(distance))
            return distance
        else:
            pass

    def evaluate(self):
        actions = self.policy
        distance = self.simulate(actions)
        return 1 / distance

    def motion_predict(self, x, y, theta, v, w, aug_map, dt=0.5, frequency=10):
        """
        This function predicts the next state when transitioning from the current state (x,y,theta) by action (v,w).
        Returns None if the motion results in collision.
        """
        num_steps = int(dt * frequency)
        dx = 0
        dy = 0
        for i in range(num_steps):
            if w!=0:
                dx = - v / w * np.sin(theta) + v / w * np.sin(theta + w / frequency)
                dy = v / w * np.cos(theta) - v / w * np.cos(theta + w / frequency)
            else:
                dx = v*np.cos(theta)/frequency
                dy = v*np.sin(theta)/frequency
            x += dx
            y += dy
            if x>=10 or x<=0 or y>=10 or y<=0:
                return None
            if (y*4000+x*20>39999) or (aug_map[int(y*20)*200 +int(x*20)]) == 100:
                return None
            theta += w / frequency
            if theta >= np.pi * 2:
                theta = theta / (np.pi * 2) * 360
                theta = theta % 360 / 360 * np.pi * 2

        return x, y, theta


class DiscreteEvaluator(Evaluator):
    def __init__(self, filename, aug_map, start, goal):
        super(Evaluator, self).__init__(filename, aug_map, start, goal)
    
    def gen_policy(self):
        self.policy = np.genfromtxt(self.filename)
        self.policy = self.generate_action_sequence_discrete(self.policy)

    def distance_function(self, v):
        return 1

    def generate_action_sequence_discrete(self, actions):
        controls = []
        for act in actions:
            act = [act[0], act[1]*np.pi / 2]
            controls += [act, act]

        return controls

class MDPEvaluator(Evaluator):
    def __init__(self, filename, aug_map, start, goal):
        super(MDPEvaluator, self).__init__(filename, aug_map, start, goal)

    def gen_policy(self):
        try:
            with open(self.filename, 'r') as fin:
                import json
                self.policy = json.load(fin)
        except:
            raise ValueError("incorrect policy file")
    
    def get_action(self, pose):
        position = copy.deepcopy(pose)
        position[0] = round(position[0])
        position[1] = round(position[1])
        position[2] = round(position[2] / (np.pi / 2)) % 4

        position = [str(int(s)) for s in position]
        position = ','.join(position)
        act = copy.deepcopy(self.policy[position])

        if act[0] != 0:
            r = np.random.rand()
            if r < 0.9:
                pass
            elif r < 0.95:
                act = [np.pi / 2, 1]
            else:
                act = [np.pi / 2, -1]
        
        act[1] = act[1] * np.pi / 2

        return [act, act]

    def simulate(self):
        pose = copy.deepcopy(self.start)
        steps = 0

        while True:
            action = self.get_action(pose)
            for act in action:
                pose = self.motion_predict(pose[0], pose[1], pose[2], act[0], act[1], self.aug_map)
                if pose is None:
                    break
                pose = np.array(pose)

            steps += 1

            if pose is None:
                return -1
            elif np.sum((pose[:2] - np.array(self.goal)) ** 2) < 0.04:
                return steps
            else:
                pass

    def evaluate(self):
        seed = 0
        records = []

        for i in range(50):
            step = self.simulate()
            spl = float(step > 0) / step
            records.append(spl)

        return np.mean(records)

parser = argparse.ArgumentParser()
parser.add_argument('--map', type=int, default=8, help="map file")
parser.add_argument('--start', type=str, default='1,1,0', help="start pos")
parser.add_argument('--goal', type=str, default='1,7', help="goal pos")
parser.add_argument('--actions', type=str, default='mdp_policy.json', help='action file')
args = parser.parse_args()

start = args.start.split(',')
start = [int(pos) for pos in start]
goal = args.goal.split(',')
goal = np.array([int(pos) for pos in goal])

# save the augmented map as a pickle and load it here
with open('./map{}.pkl'.format(args.map), 'rb') as fin:
    # -1 empty space, 100 obstacle
    aug_map = pickle.load(fin)

evaluator = MDPEvaluator(args.actions, aug_map, start, goal)
spl = evaluator.evaluate()
print(spl)
