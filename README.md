# **Description**
This repository contains all the necessary files used to integrate the Dobot-E6-Magician with the FarmBeast - an AMR agriculture platform being actively developed at University of Maribor.

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
1. **Create work space and set environmental variables**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   echo "source ~/ros2_ws/install/local_setup.sh" >> ~/.bashrc
   echo "alias cb='cd ~/ros2_ws && colcon build && source ~/.bashrc'" >> ~/.bashrc
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
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    cb #Build alias, set in the first step
    ```

---

## **Features**
1. **Robot visualization**
    ```bash
    ros2 launch dobot_e6_bringup robot_state_publisher.launch.py
    ```
2. **Basic ROS 2 control**
    ```bash
    ros2 launch dobot_e6_bringup ros2_control.py
    ```
3. **MoveIt Virtual Demo**
    ```bash
    ros2 launch dobot_moveit moveit_demo.launch.py
    ```

4. **Moveit Robot Control**
    - Terminal 1 (Connect the Robotic Arm)
    ```bash
    ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
    ```
    - Terminal 2 (Launch MoveIt)
    ```bash
    ros2 launch dobot_moveit dobot_moveit.launch.py
    ```

---

## Notes
1. Ensure connection is established via ping.
2. Ensure the robotic arm is in remote TCP mode and enabled. This can be configured via DobotStudio: https://www.scribd.com/document/723793818/DobotStudio-Pro-User-Guide-MG400-M1-Pro-V2-7
3. This are the recovered files.

- **Last Updated**: March 25, 2026
- **More Information**: https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4/tree/main
