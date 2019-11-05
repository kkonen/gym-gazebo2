source /opt/ros/dashing/setup.bash
source ~/phantomx_ws/install/setup.bash
source /usr/share/gazebo-9/setup.sh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH=$PYTHONPATH:~/phantomx_ws/install/lib/python3/dist-packages
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/phantomx_ws/install/share
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/phantomx_ws/install/lib
