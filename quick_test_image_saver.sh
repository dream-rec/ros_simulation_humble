#!/bin/bash

# 简化的图像保存测试脚本

echo "========================================"
echo "TurtleBot3 图像保存功能 - 快速测试"
echo "========================================"

cd /home/ros/turtlebot3_ws
source install/setup.bash

echo "当前摄像头话题状态："
echo "检查 /camera/image_raw 话题..."
if ros2 topic list | grep -q "/camera/image_raw"; then
    echo "✅ 摄像头话题 /camera/image_raw 存在"
    
    echo ""
    echo "开始保存图像（将保存5张图片）..."
    echo "保存位置: ~/turtlebot3_images/"
    echo ""
    
    # 直接运行节点，保存5张图片
    timeout 8 python3 /home/ros/turtlebot3_ws/install/image_saver/bin/image_saver_node \
        --ros-args \
        -p max_images:=5 \
        -p save_interval:=1.0 \
        -p save_path:=~/turtlebot3_images
    
    echo ""
    echo "========================================"
    echo "保存结果："
    ls -la ~/turtlebot3_images/ | tail -n +2
    echo "========================================"
    echo ""
    echo "测试交互式保存器（按Ctrl+C退出）："
    echo "启动命令："
    echo "python3 /home/ros/turtlebot3_ws/install/image_saver/bin/interactive_image_saver"
    echo ""
    echo "使用方法："
    echo "- 在显示窗口按 's' 保存图片"
    echo "- 按 'q' 退出"
    echo "- 或发布话题: ros2 topic pub /save_image_command std_msgs/msg/String \"data: 'save'\""
    
else
    echo "❌ 摄像头话题 /camera/image_raw 不存在"
    echo ""
    echo "请先启动TurtleBot3仿真："
    echo "export TURTLEBOT3_MODEL=waffle_pi"
    echo "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
fi

echo "========================================"
