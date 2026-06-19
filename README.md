# **Description**
Application allows for safe, perception based weeds removal while avoiding asparagus. The framework is based on YOLO for perception and MoveIt Task Constructor for object avoidance path planning. The goal is to detect weeds and subsequently mechanically remove them without damaging the crop.

Framework was awarded the first place at the **Field Robot Event 2026** event in freestyle category. It is part of the FarmBeast project - an AMR agriculture platform being actively developed at University of Maribor.

---

## **Prerequisites**
1. **ROS 2-Jazzy installed with necessary dependencies / Ubuntu 24.04**
    - WSL Ubuntu guide: https://documentation.ubuntu.com/wsl/stable/howto/install-ubuntu-wsl2/
    - MAC Ubuntu guide: https://www.youtube.com/watch?v=1PL-0-5BNXs&t=160s
    - ROS 2 installation guide: https://automaticaddison.com/how-to-install-ros-2-jazzy/

2. **Network configuration**
    - **Wired connection:** Controller IP is `192.168.5.1`
    - **Wireless Connection**: Controller IP is `192.168.1.6`  

---

## **Configuration steps**
1. **Create workspace**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   ```

2. **Configure robot specific variables**
    ```bash
    echo "export IP_address=192.168.5.1" >> ~/.bashrc
    echo "export DOBOT_TYPE=me6" >> ~/.bashrc
    ```

3. **Download and compile the source code**
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/IntoTheVoid-61/DOBOT-Magician-E6-ROS2
    vcs import < DOBOT-Magician-E6-ROS2/upstream.repos
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    ```

---

## **Framework Launch**
     ~/ros2_ws/src/DOBOT-Magician-E6-ROS2/dobot_e6_bringup/scripts/start_framework.sh

---
## **FarmBeast specific**
**Packages containing FarmBeast specific implementation**
 - dobot_msgs_fb
 - dobot_system_tests
 - dobot_mtc_tasks
 - farmbeast_utils
 - gripper_hardware_interface
 - gripper_system_tests
 - perception


## Notes
1. Ensure connection is established via ping.
2. Ensure the robotic arm is in remote TCP mode and enabled. This can be configured via DobotStudio: https://www.scribd.com/document/723793818/DobotStudio-Pro-User-Guide-MG400-M1-Pro-V2-7
3. When running perception segment make sure ultralytics is available.

---
## Other
- **Last Updated**: June 19, 2026
- **More Information**: https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/tree/main
