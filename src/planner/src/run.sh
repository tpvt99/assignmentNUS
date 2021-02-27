# map x y \theta

# map_file="../maze${1}.png"
map_file="../${1}"

python world_edit.py ../maps/stage/maze.world ${map_file} "[ $2 $3 0.0 $4 ]" 
python yaml_edit.py ../maps/maze.yaml ${map_file}

roslaunch planner turtlebot_in_stage.launch \
    initial_pose_x:=$2 \
    initial_pose_y:=$3 \
    initial_pose_z:=$4
