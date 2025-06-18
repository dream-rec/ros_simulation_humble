[前置操作]

[Install Gazebo]

$ sudo apt install ros-humble-gazebo-*

[Install Cartographer]

$ sudo apt install ros-humble-cartographer
$ sudo apt install ros-humble-cartographer-ros
[Install Navigation2]
$ sudo apt install ros-humble-navigation2
$ sudo apt install ros-humble-nav2-bringup

[Install TurtleBot3 Packages]

$ source /opt/ros/humble/setup.bash
$ mkdir -p ~/turtlebot3_ws/src
$ cd ~/turtlebot3_ws/src/
$ https://github.com/dream-rec/ros_simulation_humble.git
$ sudo apt install python3-colcon-common-extensions
$ cd ~/turtlebot3_ws
$ colcon build --symlink-install
$ echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

[Environment Configuration]

$ echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
$ echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
$ echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
$ echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
$ source ~/.bashrc

[gazebo simulation]
$ ros2 launch turtlebot3_gazebo empty_world.launch.py

[键盘控制]

$ ros2 run turtlebot3_teleop teleop_keyboard

[slam simulation]

$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
$ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
$ ros2 run turtlebot3_teleop teleop_keyboard

[map save]

$ ros2 run nav2_map_server map_saver_cli -f ~/map

[navigation simulation]

$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
$ ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
$ ros2 run turtlebot3_teleop teleop_keyboard #位资匹配
