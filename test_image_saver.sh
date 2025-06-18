#!/bin/bash

# TurtleBot3 图像保存完整测试脚本

echo "========================================"
echo "TurtleBot3 图像保存功能测试"
echo "========================================"

# 设置环境
export TURTLEBOT3_MODEL=waffle_pi
cd /home/ros/turtlebot3_ws
source install/setup.bash

echo "当前TurtleBot3模型: $TURTLEBOT3_MODEL"
echo ""

# 检查依赖
echo "检查系统依赖..."
if ! dpkg -l | grep -q python3-opencv; then
    echo "❌ python3-opencv 未安装"
    echo "安装命令: sudo apt install python3-opencv"
    exit 1
else
    echo "✅ python3-opencv 已安装"
fi

if ! dpkg -l | grep -q ros-humble-cv-bridge; then
    echo "❌ ros-humble-cv-bridge 未安装"
    echo "安装命令: sudo apt install ros-humble-cv-bridge"
    exit 1
else
    echo "✅ ros-humble-cv-bridge 已安装"
fi

echo ""

# 检查包是否编译成功
echo "检查image_saver包..."
if [ -f "install/image_saver/lib/image_saver/image_saver_node" ]; then
    echo "✅ image_saver_node 可执行文件存在"
else
    echo "❌ image_saver_node 可执行文件不存在，正在重新编译..."
    colcon build --packages-select image_saver --symlink-install
    source install/setup.bash
fi

if [ -f "install/image_saver/lib/image_saver/interactive_image_saver" ]; then
    echo "✅ interactive_image_saver 可执行文件存在"
else
    echo "❌ interactive_image_saver 可执行文件不存在，正在重新编译..."
    colcon build --packages-select image_saver --symlink-install
    source install/setup.bash
fi

echo ""

# 创建保存目录
echo "创建图像保存目录..."
mkdir -p ~/turtlebot3_images
echo "✅ 图像保存目录: ~/turtlebot3_images"

echo ""
echo "========================================"
echo "使用指南"
echo "========================================"
echo ""
echo "步骤1: 启动TurtleBot3仿真 (在新终端中运行)"
echo "----------------------------------------"
echo "export TURTLEBOT3_MODEL=waffle_pi"
echo "cd /home/ros/turtlebot3_ws"
echo "source install/setup.bash"
echo "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo ""
echo "步骤2: 等待仿真完全加载，然后选择以下方式之一保存图像："
echo ""
echo "方式A: 自动保存模式 (每1秒保存一张)"
echo "----------------------------------------"
echo "ros2 launch image_saver image_saver.launch.py"
echo ""
echo "方式B: 交互式保存模式 (实时显示+手动保存)"
echo "----------------------------------------"
echo "ros2 launch image_saver interactive_image_saver.launch.py"
echo "然后在显示窗口中按 's' 保存图片，按 'q' 退出"
echo ""
echo "方式C: 自定义参数"
echo "----------------------------------------"
echo "ros2 launch image_saver image_saver.launch.py \\"
echo "    image_topic:=/camera/image_raw \\"
echo "    save_path:=~/my_images \\"
echo "    save_interval:=2.0 \\"
echo "    max_images:=20"
echo ""
echo "步骤3: 检查可用的图像话题"
echo "----------------------------------------"
echo "ros2 topic list | grep image"
echo "ros2 topic echo /camera/image_raw --once  # 测试话题"
echo ""
echo "步骤4: 远程触发保存 (适用于交互式模式)"
echo "----------------------------------------"
echo "ros2 topic pub /save_image_command std_msgs/msg/String \"data: 'save'\""
echo ""
echo "========================================"
echo "故障排除"
echo "========================================"
echo ""
echo "问题1: 没有图像话题"
echo "解决: 确保使用waffle_pi模型启动仿真"
echo ""
echo "问题2: 图像保存失败"
echo "解决: 检查保存目录权限: chmod 755 ~/turtlebot3_images"
echo ""
echo "问题3: OpenCV显示问题"
echo "解决: 确保有X11显示支持"
echo ""
echo "保存的图像位置: ~/turtlebot3_images/"
echo "========================================"
