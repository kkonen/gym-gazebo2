source /opt/ros/dashing/setup.bash
source ~/test/ros2_mara/install/setup.bash
source /usr/share/gazebo-9/setup.sh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH=$PYTHONPATH:~/ros2_mara_ws/install/lib/python3/dist-packages
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_mara_ws/install/share
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_mara_ws/install/lib
