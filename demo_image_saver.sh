#!/bin/bash

# TurtleBot3 图像保存演示脚本

echo "========================================"
echo "TurtleBot3 图像保存演示"
echo "========================================"

# 设置TurtleBot3模型为waffle_pi（带摄像头）
export TURTLEBOT3_MODEL=waffle_pi
echo "设置机器人模型为: $TURTLEBOT3_MODEL"

# 进入工作空间
cd /home/ros/turtlebot3_ws

# 编译和加载环境
echo "加载ROS2环境..."
source install/setup.bash

echo ""
echo "使用说明："
echo "1. 首先在一个新终端中启动Gazebo仿真："
echo "   export TURTLEBOT3_MODEL=waffle_pi"
echo "   cd /home/ros/turtlebot3_ws"
echo "   source install/setup.bash"
echo "   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo ""
echo "2. 等待仿真完全加载后，在另一个终端运行图像保存器："
echo "   cd /home/ros/turtlebot3_ws"
echo "   source install/setup.bash"
echo "   ros2 launch image_saver image_saver.launch.py"
echo ""
echo "3. 或者使用自定义参数："
echo "   ros2 launch image_saver image_saver.launch.py \\"
echo "       image_topic:=/camera/image_raw \\"
echo "       save_path:=~/my_robot_images \\"
echo "       save_interval:=2.0 \\"
echo "       max_images:=50"
echo ""
echo "4. 检查可用的图像话题："
echo "   ros2 topic list | grep image"
echo ""
echo "图像将保存到: ~/turtlebot3_images/"
echo "========================================"
