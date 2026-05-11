#!/usr/bin/env bash
# Script is responsible for launching the dobot platform with all the necessary processes.

set -e
export IP_address="192.168.5.1"
export DOBOT_TYPE="me6"

cleanup() {
  echo "Cleaning up..."
  sleep 5.0
  trap - INT TERM EXIT
  kill 0
}

trap cleanup INT TERM EXIT

ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py &
PID1=$!

sleep 10

ros2 launch dobot_moveit dobot_moveit.launch.py &
PID2=$!

wait