#!/usr/bin/env bash

cleanup() {
  echo "Cleaning up perception..."
  trap - INT TERM EXIT
  kill 0
}

# Set up cleanup trap
trap cleanup INT TERM EXIT

echo "Starting perception framework..."

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 

export PYTHONPATH=/home/ziga/miniconda3/envs/ros2_ml/lib/python3.12/site-packages:$PYTHONPATH

echo "Exported Python path"
echo $PYTHONPATH


ros2 launch realsense2_camera rs_align_depth_launch.py &
sleep 5
ros2 run perception perception_avoid &


echo "Successfully launched Perception framework"

wait