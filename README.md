# CS4278/CS5478 Homework Assignment: Mobile Robot Motion Planning

In this assignment, you will be working on planning algorithms for controlling the robot to navigate on a 2D map. 


## Setup the environment

We developed the simulator using ROS Kinetic. The simulator is adjusted from [ROS TurtleBot Stage package](http://wiki.ros.org/turtlebot_stage). As your first step, you should install the ROS kinetic according to assignment 1. After installation, install TurtleBot Stage with
```
sudo apt install ros-kinetic-turtlebot-stage
sudo apt install ros-kinetic-joint-state-publisher-gui
```

Next, clone our repo and setup your environment with
```
catkin_make
source devel/setup.bash
```

Check your setup by 
```
roslaunch planner turtlebot_in_stage.launch
```
You should be able to see the RViz and ROS stage. 

## What you need to do
We split the robot navigation task into three parts: simulator, controller and planner. We use a very simple controller defined in the planner file. In this assignment, we provide you with the simulator and all the necessary functions for 1) ROS nodes 2) data communication 3) controlling the robot. You only need to implement the planning algorithms and collision checking mechanism, and fill in the template marked with `# TODO: FILL ME!` in `base_planner.py`. 

We would recomment you to use `base_planner.py` as a base class, then impelement your planners as derived classes.

## How to run the code

You can simply use the following instructions to launch the simulator and set the configurations. 
```
roscd planner/src
sh run.sh [your map name] start_x start_y start_theta
python your_planner.py --goal 'goal_x,goal_y' --com use_com_1_map_or_not
```

Specifically, suppose we want to load *maze1.png*, set the start pose of the robot as (x=2, y=2, \theta=0), and set the goal to be (x'=8, y'=8). You should:
1. Go into the source directory `roscd planner/src`
2. Launch the simulator and set the start of the robot `sh run.sh maze1.png 1 1 0`
3. Open a new shell and launch the planner script with goal specified `python your_planner.py --goal '5,5' --com 0`. Here, `--com 0` flag indicates that we are not using the `com1.jpg` map. This is because the environment parameter changes from other mazes to the com1 map. 

## Notes
For visualization,  use ROS stage. RViz provides the 2.5D visualization but has certain noise due to ROS asynchronous communication.

## Submission

We provide you with 5 maps, including 4 handcrafted maps (maze0.png to maze3.png) and an illustrative COM1 level 1 floorplan (com1.jpg). You can find them [here](./src/planner/maps/). Each map has a list of corresponding testcases, with the goals specified [here](./files/goals.json). For all cases, we assume the robot starts with pose (1, 1, 0).

You should implement the planners, test them, generate controls for each testcase, and submit all of them. For discrete and continuous actions (task 1 and task 2),  save them in `.txt` files. For MDP policy,  save it into a json file. We have provided functions in base_planner.py.

The naming should follow `{task}_{map}_{goal}.txt` for task 1 and task 2; `{task}_{map}_{goal_x}_{goal_y}.json` for task 3. For example, 

- `1_maze2_5_5.txt` for the discrete planner on maze2.png with [5, 5] as the goal.
- `2_maze3_9_9.txt` for the continuous planner on maze3.png with [9, 9] as the goal.
- `3_com1_43_10.json` for the mdp policy on com1.jpg with [43, 10] as the goal.

Some example control files can be found [here](./files/).

In summary, you should submit:
- Your code
- The control files for each map and each goal
- Your report
