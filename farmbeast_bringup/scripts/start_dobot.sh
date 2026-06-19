#!/usr/bin/env bash
# Script is responsible for launching the dobot platform with all the necessary processes.

set -e

echo "Launching Dobot..."

export IP_address="192.168.5.1"
export DOBOT_TYPE="me6"

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash 

cleanup() {
  echo "Cleaning up dobot..."
  trap - INT TERM EXIT
  kill 0
}

trap cleanup INT TERM EXIT

ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py &
PID1=$!

sleep 5

ros2 launch dobot_moveit dobot_moveit.launch.py &
PID2=$!

echo "Successfully launched Dobot"

wait