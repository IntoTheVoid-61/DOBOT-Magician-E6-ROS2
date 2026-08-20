#!/usr/bin/env bash

echo "Launching entire framewok..."
bash ~/ros2_ws/src/DOBOT-Magician-E6-ROS2/farmbeast_bringup/scripts/start_dobot.sh &
sleep 5
bash ~/ros2_ws/src/DOBOT-Magician-E6-ROS2/farmbeast_bringup/scripts/start_perception_loop.sh &
echo "Successfully launched framework"

wait
