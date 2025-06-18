# TurtleBot3 ROS2 Humble 仿真环境

这个仓库包含了用于 ROS2 Humble 的 TurtleBot3 仿真和导航功能包。

## 目录结构

```
├── turtlebot3/                              # TurtleBot3 主要功能包集合
│   ├── turtlebot3/                          # 元功能包 (Meta Package)
│   ├── turtlebot3_bringup/                  # 机器人启动配置
│   │   ├── launch/                          # 启动文件 (robot.launch.py, camera.launch.py 等)
│   │   ├── param/                           # 参数配置文件 (burger.yaml, waffle.yaml 等)
│   │   └── script/                          # 脚本文件 (udev 规则等)
│   ├── turtlebot3_cartographer/             # Cartographer SLAM 配置
│   │   ├── config/                          # Cartographer 配置文件
│   │   ├── launch/                          # SLAM 启动文件
│   │   └── rviz/                            # RViz 配置文件
│   ├── turtlebot3_description/              # 机器人模型描述
│   │   ├── meshes/                          # 3D 网格文件 (STL/DAE)
│   │   ├── rviz/                            # RViz 可视化配置
│   │   └── urdf/                            # URDF 机器人描述文件
│   ├── turtlebot3_example/                  # 示例代码和算法
│   │   └── turtlebot3_example/              # Python 示例包
│   │       ├── turtlebot3_absolute_move/    # 绝对位置移动示例
│   │       ├── turtlebot3_interactive_marker/# 交互式标记示例
│   │       ├── turtlebot3_obstacle_detection/# 障碍物检测示例
│   │       ├── turtlebot3_patrol/           # 巡逻算法示例
│   │       └── turtlebot3_relative_move/    # 相对位置移动示例
│   ├── turtlebot3_navigation2/              # Navigation2 导航配置
│   │   ├── launch/                          # 导航启动文件
│   │   ├── map/                             # 示例地图文件
│   │   ├── param/                           # 导航参数配置
│   │   └── rviz/                            # 导航 RViz 配置
│   ├── turtlebot3_node/                     # 核心硬件驱动节点 (C++)
│   │   ├── include/                         # 头文件
│   │   ├── param/                           # 节点参数配置
│   │   └── src/                             # 源代码 (传感器驱动、差分控制器等)
│   └── turtlebot3_teleop/                   # 遥控操作功能包
│       └── turtlebot3_teleop/               # Python 遥控包
│           └── script/                      # 键盘控制脚本
│
├── turtlebot3_msgs/                         # TurtleBot3 自定义消息定义
│   ├── action/                              # 动作定义 (Patrol.action)
│   ├── msg/                                 # 消息定义 (SensorState.msg, Sound.msg 等)
│   └── srv/                                 # 服务定义 (Dqn.srv, Goal.srv 等)
│
└── turtlebot3_simulations/                  # TurtleBot3 仿真环境
    ├── turtlebot3_fake_node/                # 虚拟机器人节点 (无 Gazebo)
    │   ├── include/                         # C++ 头文件
    │   ├── launch/                          # 虚拟机器人启动文件
    │   ├── param/                           # 虚拟机器人参数
    │   └── src/                             # 虚拟机器人源代码
    ├── turtlebot3_gazebo/                   # Gazebo 仿真环境
    │   ├── include/                         # Gazebo 插件头文件
    │   ├── launch/                          # Gazebo 世界启动文件
    │   ├── models/                          # Gazebo 模型定义
    │   │   ├── turtlebot3_burger/           # Burger 模型
    │   │   ├── turtlebot3_waffle/           # Waffle 模型
    │   │   ├── turtlebot3_waffle_pi/        # Waffle Pi 模型
    │   │   ├── turtlebot3_world/            # TurtleBot3 测试世界
    │   │   ├── turtlebot3_house/            # 房屋环境模型
    │   │   ├── turtlebot3_autorace_2020/    # 自动驾驶赛道模型
    │   │   └── turtlebot3_dqn_world/        # 强化学习环境模型
    │   ├── rviz/                            # RViz 可视化配置
    │   ├── src/                             # Gazebo 插件源代码
    │   ├── urdf/                            # 仿真用 URDF 文件
    │   └── worlds/                          # Gazebo 世界文件
    ├── turtlebot3_manipulation_gazebo/      # 机械臂仿真 (与 OpenMANIPULATOR-X 集成)
    │   ├── config/                          # 控制器配置
    │   ├── gazebo/                          # Gazebo 特定配置
    │   ├── launch/                          # 机械臂仿真启动文件
    │   ├── models/                          # 机械臂相关模型
    │   ├── ros2_control/                    # ROS2 控制配置
    │   ├── rviz/                            # 机械臂 RViz 配置
    │   ├── urdf/                            # 机械臂 URDF 文件
    │   └── worlds/                          # 机械臂仿真世界
    └── turtlebot3_simulations/              # 仿真元功能包
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

## 功能包详细说明

### turtlebot3/ - 主要功能包集合

#### 核心功能包
- **turtlebot3_bringup**: 机器人启动和配置管理
  - 提供不同型号机器人的启动文件和参数配置
  - 包含相机、激光雷达等传感器的启动配置
  - 支持 burger、waffle、waffle_pi 三种型号

- **turtlebot3_node**: 硬件驱动和核心功能 (C++)
  - 差分驱动控制器 (DiffDriveController)
  - 传感器数据处理 (IMU、电池状态、关节状态等)
  - Dynamixel 舵机控制接口
  - 里程计计算和发布

- **turtlebot3_description**: 机器人物理模型定义
  - URDF 机器人描述文件，定义机器人的几何结构
  - 3D 网格文件 (STL/DAE 格式)，用于可视化和碰撞检测
  - 支持不同传感器配置 (LDS 激光雷达、摄像头等)

#### 应用功能包
- **turtlebot3_teleop**: 遥控操作
  - 键盘控制接口 (WASD 控制方案)
  - 支持线速度和角速度调节
  - 安全停止机制

- **turtlebot3_example**: 编程示例和算法演示
  - **absolute_move**: 绝对坐标移动控制
  - **relative_move**: 相对位置移动控制
  - **obstacle_detection**: 基于激光雷达的障碍物检测
  - **patrol**: 自动巡逻算法实现
  - **interactive_marker**: 交互式标记控制示例

#### 导航和 SLAM
- **turtlebot3_cartographer**: Google Cartographer SLAM 集成
  - 2D 激光雷达 SLAM 配置
  - 实时建图和定位
  - 支持不同环境的参数调优

- **turtlebot3_navigation2**: Navigation2 导航栈集成
  - 全局路径规划 (A* 算法)
  - 局部路径规划 (DWB 控制器)
  - 代价地图配置 (静态层、障碍物层、膨胀层)
  - AMCL 自适应蒙特卡罗定位

### turtlebot3_msgs/ - 消息和接口定义

#### 消息类型 (msg/)
- **SensorState.msg**: 传感器状态信息
  - 电池电压、电流
  - 按钮状态
  - 碰撞传感器状态
  - 电机编码器数据

- **Sound.msg**: 声音控制消息
  - 蜂鸣器音调控制
  - 持续时间设置

- **VersionInfo.msg**: 版本信息
  - 硬件版本
  - 固件版本
  - 软件版本

#### 服务类型 (srv/)
- **Dqn.srv**: 深度 Q 网络强化学习接口
- **Goal.srv**: 目标点设置服务
- **Sound.srv**: 声音播放服务

#### 动作类型 (action/)
- **Patrol.action**: 巡逻任务动作定义
  - 巡逻路径点列表
  - 执行状态反馈
  - 结果反馈

### turtlebot3_simulations/ - 仿真环境

#### 基础仿真
- **turtlebot3_fake_node**: 轻量级虚拟机器人
  - 无需 Gazebo 的快速仿真
  - 适用于算法测试和调试
  - 模拟机器人动力学和传感器数据

#### Gazebo 仿真
- **turtlebot3_gazebo**: 完整的 Gazebo 仿真环境
  - 物理引擎仿真 (碰撞、摩擦、重力)
  - 传感器仿真 (激光雷达、IMU、摄像头)
  - 多种预设环境:
    - **empty_world**: 空白测试环境
    - **turtlebot3_world**: 基础测试场景
    - **turtlebot3_house**: 室内房屋环境
    - **turtlebot3_autorace_2020**: 自动驾驶赛道
    - **turtlebot3_dqn_world**: 强化学习训练环境

#### 高级仿真
- **turtlebot3_manipulation_gazebo**: 机械臂集成仿真
  - 与 OpenMANIPULATOR-X 机械臂集成
  - ROS2 Control 框架支持
  - 运动学和动力学仿真
  - 抓取和操作任务仿真

## 技术特性

### 支持的机器人型号
- **TurtleBot3 Burger**: 入门级，紧凑设计
- **TurtleBot3 Waffle**: 标准版，配备摄像头
- **TurtleBot3 Waffle Pi**: 树莓派版本

### 传感器支持
- LDS-01/LDS-02 360° 激光雷达
- IMU (惯性测量单元)
- 编码器 (轮式里程计)
- RGB 摄像头 (仅 Waffle 系列)
- 碰撞传感器
- 电池监控

### 软件框架
- ROS2 Humble 原生支持
- Navigation2 导航栈
- Gazebo 11 物理仿真
- RViz2 可视化
- Cartographer SLAM
- ROS2 Control 硬件抽象

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
