# TurtleBot3 ROS2 Humble 仿真环境

这个仓库包含了用于 ROS2 Humble 的 TurtleBot3 仿真和导航功能包。

## 目录结构

```
├── turtlebot3/                    # TurtleBot3 主要功能包
├── turtlebot3_msgs/               # TurtleBot3 自定义消息类型
└── turtlebot3_simulations/        # TurtleBot3 仿真相关功能包
```

## 前置操作

在开始使用 TurtleBot3 仿真之前，需要安装以下依赖包。

### 安装 Gazebo

```bash
sudo apt install ros-humble-gazebo-*
```

### 安装 Cartographer (SLAM功能)

```bash
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```

### 安装 Navigation2 (导航功能)

```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

## 安装 TurtleBot3 功能包

### 1. 创建工作空间并克隆代码

```bash
# 设置 ROS2 环境
source /opt/ros/humble/setup.bash

# 创建工作空间
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/

# 克隆本仓库
git clone https://github.com/dream-rec/ros_simulation_humble.git
```

### 2. 安装编译工具

```bash
sudo apt install python3-colcon-common-extensions
```

### 3. 编译功能包

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
```

### 4. 配置环境变量

```bash
# 添加工作空间到环境变量
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 环境配置

配置必要的环境变量以确保 TurtleBot3 仿真正常运行：

```bash
# 设置 ROS 域 ID (避免网络冲突)
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc

# 配置 Gazebo 环境
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

# 配置 ROS2 环境
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# 设置 TurtleBot3 模型 (可选: burger, waffle, waffle_pi)
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

# 重新加载环境变量
source ~/.bashrc
```

## 使用方法

### 基础 Gazebo 仿真

启动空白世界中的 TurtleBot3：

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 键盘控制

在新终端中启动键盘控制节点：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

控制说明：
- `w/a/s/d`: 前进/左转/后退/右转
- `x`: 停止
- `space`: 强制停止

### SLAM 建图仿真

#### 1. 启动 TurtleBot3 世界

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### 2. 启动 Cartographer SLAM

在新终端中：

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

#### 3. 启动键盘控制进行建图

在新终端中：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### 保存地图

建图完成后，保存地图到本地：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

这将在用户主目录下生成 `map.pgm` 和 `map.yaml` 文件。

### 导航仿真

#### 1. 启动 TurtleBot3 世界

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### 2. 启动导航功能

在新终端中，使用之前保存的地图启动导航：

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

#### 3. 位置初始化 (可选)

如需手动设置初始位置，可以启动键盘控制进行位置匹配：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

也可以在 RViz 中使用 "2D Pose Estimate" 工具设置机器人初始位置。

## 功能包说明

### turtlebot3/
- `turtlebot3_bringup`: 机器人启动相关配置
- `turtlebot3_cartographer`: Cartographer SLAM 配置
- `turtlebot3_description`: 机器人 URDF 描述文件
- `turtlebot3_example`: 示例代码
- `turtlebot3_navigation2`: Navigation2 配置
- `turtlebot3_node`: 机器人核心节点
- `turtlebot3_teleop`: 遥控功能

### turtlebot3_msgs/
包含 TurtleBot3 使用的自定义消息、服务和动作定义。

### turtlebot3_simulations/
- `turtlebot3_fake_node`: 虚拟机器人节点
- `turtlebot3_gazebo`: Gazebo 仿真环境
- `turtlebot3_manipulation_gazebo`: 机械臂仿真

## 故障排除

1. **Gazebo 启动失败**: 确保已正确安装 Gazebo 并配置环境变量
2. **找不到 TurtleBot3 模型**: 检查 `TURTLEBOT3_MODEL` 环境变量是否正确设置
3. **编译错误**: 确保所有依赖包已正确安装
4. **网络通信问题**: 检查 `ROS_DOMAIN_ID` 设置

## 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo 11
- Python 3.10+

## 许可证

请参考各个功能包中的 LICENSE 文件。

## 贡献

欢迎提交 Issues 和 Pull Requests！

## 联系方式

如有问题，请在 GitHub 仓库中提交 Issue。
