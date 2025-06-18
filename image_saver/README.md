# TurtleBot3 图像保存器

这个包提供了一个ROS2节点，用于保存TurtleBot3 waffle_pi仿真环境中摄像头传感器的图像。

## 功能特性

- 自动从摄像头话题订阅图像数据
- 可配置的保存间隔和最大图片数量
- 自动生成带时间戳的文件名
- 支持不同的图像话题
- **交互式图像保存器**：实时显示摄像头画面，支持手动保存
- 支持通过ROS话题远程触发保存

## 安装和编译

1. 确保你已经安装了所需的依赖：
```bash
sudo apt install python3-opencv ros-humble-cv-bridge
```

2. 在workspace中编译包：
```bash
cd ~/turtlebot3_ws
colcon build --packages-select image_saver
source install/setup.bash
```

## 使用方法

### 1. 启动TurtleBot3仿真环境

首先启动TurtleBot3 waffle_pi仿真：

```bash
# 设置机器人模型
export TURTLEBOT3_MODEL=waffle_pi

# 启动Gazebo仿真
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
``

### 2. 检查摄像头话题

查看可用的图像话题：
```bash
ros2 topic list | grep image
```

常见的摄像头话题包括：
- `/camera/image_raw` - 原始图像
- `/intel_realsense_r200_depth/image_raw` - 深度图像（如果可用）

### 3. 启动图像保存节点

#### 方式1：自动保存模式（推荐）
使用launch文件启动，每隔指定时间自动保存图片：
```bash
# 使用默认参数（每1秒保存一张，最多100张）
ros2 launch image_saver image_saver.launch.py

# 自定义参数
ros2 launch image_saver image_saver.launch.py \
    image_topic:=/camera/image_raw \
    save_path:=~/my_robot_images \
    save_interval:=2.0 \
    max_images:=50
```

#### 方式2：交互式保存模式（推荐）
启动实时显示窗口，可手动控制保存：
```bash
ros2 launch image_saver interactive_image_saver.launch.py

# 自定义参数
ros2 launch image_saver interactive_image_saver.launch.py \
    image_topic:=/camera/image_raw \
    save_path:=~/my_images \
    show_image:=true
```

在GUI显示窗口中：
- 按 **'s'** 键保存当前图像
- 按 **'q'** 键退出程序

或者通过话题远程触发保存：
```bash
ros2 topic pub /save_image_command std_msgs/msg/String "data: 'save'"
```

#### 方式3：直接运行节点（备用方案）
如果launch文件有问题，可以直接运行：
```bash
# 自动保存节点
python3 /home/ros/turtlebot3_ws/install/image_saver/bin/image_saver_node \
    --ros-args -p max_images:=5

# 交互式保存节点
python3 /home/ros/turtlebot3_ws/install/image_saver/bin/interactive_image_saver
```

#### 方式4：快速测试
运行我们提供的测试脚本：
```bash
/home/ros/turtlebot3_ws/quick_test_image_saver.sh
```

## 参数说明

### 自动保存模式参数
- `image_topic` (默认: `/camera/image_raw`): 要订阅的图像话题
- `save_path` (默认: `~/turtlebot3_images`): 图像保存目录
- `save_interval` (默认: `1.0`): 保存间隔（秒）
- `max_images` (默认: `100`): 最大保存图片数量

### 交互式保存模式参数
- `image_topic` (默认: `/camera/image_raw`): 要订阅的图像话题
- `save_path` (默认: `~/turtlebot3_images`): 图像保存目录
- `show_image` (默认: `true`): 是否显示摄像头窗口

## 输出文件

### 自动保存模式
保存的图像文件格式为：
```
turtlebot3_image_XXXX_YYYYMMDD_HHMMSS_mmm.jpg
```

### 交互式保存模式
保存的图像文件格式为：
```
turtlebot3_manual_XXXX_YYYYMMDD_HHMMSS_mmm.jpg
```

其中：
- `XXXX`: 4位数字序号（从0000开始）
- `YYYYMMDD_HHMMSS_mmm`: 时间戳（年月日_时分秒_毫秒）

## 故障排除

### 1. 没有接收到图像
检查话题是否存在：
```bash
ros2 topic list | grep image
ros2 topic echo /camera/image_raw --once
```

### 2. Launch文件问题
如果遇到launch文件错误，请确保包已正确编译：
```bash
cd ~/turtlebot3_ws
colcon build --packages-select image_saver
source install/setup.bash
```

### 3. GUI窗口无法显示
确保X11显示支持：
```bash
echo $DISPLAY  # 应该显示类似 :1 的值
xdpyinfo | head -5  # 检查X11服务器状态
```

### 4. 保存目录问题
确保指定的保存目录有写权限：
```bash
mkdir -p ~/turtlebot3_images
chmod 755 ~/turtlebot3_images
```

### 5. 依赖问题
如果遇到cv_bridge相关错误，安装对应的包：
```bash
sudo apt install ros-humble-cv-bridge python3-opencv
```

### 6. 图像编码问题
节点自动处理RGB和BGR编码转换，支持常见的图像格式。

## 进阶使用

### 保存深度图像
如果要保存深度相机的图像：
```bash
ros2 launch image_saver image_saver.launch.py \
    image_topic:=/intel_realsense_r200_depth/image_raw \
    save_path:=~/depth_images
```

### 同时保存多个话题
可以启动多个实例来保存不同的图像话题：
```bash
# 终端1 - RGB图像
ros2 launch image_saver image_saver.launch.py \
    image_topic:=/camera/image_raw \
    save_path:=~/rgb_images

# 终端2 - 深度图像
ros2 launch image_saver image_saver.launch.py \
    image_topic:=/intel_realsense_r200_depth/image_raw \
    save_path:=~/depth_images
```

### 交互式远程控制保存
启动交互式保存器后，可以通过话题远程触发保存：
```bash
# 启动交互式保存器
ros2 launch image_saver interactive_image_saver.launch.py

# 在另一个终端中远程触发保存
ros2 topic pub /save_image_command std_msgs/msg/String "data: 'save'"
```

### 快速验证功能
```bash
# 验证自动保存（保存3张图片）
ros2 launch image_saver image_saver.launch.py max_images:=3

# 验证交互式保存
ros2 launch image_saver interactive_image_saver.launch.py
```

## 注意事项

1. 确保TurtleBot3模型设置为`waffle_pi`，因为只有这个模型配备了摄像头
2. 图像保存会占用磁盘空间，注意设置合适的`max_images`参数
3. 如果仿真环境中没有摄像头数据，检查机器人模型是否正确加载
