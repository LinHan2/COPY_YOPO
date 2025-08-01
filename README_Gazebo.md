# YOPO Gazebo仿真集成指南

本文档说明如何将YOPO项目集成到Gazebo仿真环境中进行测试。

## 环境要求

### 必需软件
- ROS Noetic
- Gazebo 11
- PX4 Autopilot
- MAVROS
- OpenCV
- PCL (Point Cloud Library)

### 安装依赖
```bash
# 安装ROS和Gazebo
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# 安装MAVROS
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# 安装PX4
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl gazebo
```

## 文件结构

仿真相关文件位于YOPO项目根目录：

```
YOPO/
├── yopo_gazebo.launch          # Gazebo仿真启动文件
├── forest_environment.world    # Gazebo世界文件
├── iris_depth_camera.urdf     # 无人机模型文件
├── start_px4_gazebo.sh        # PX4启动脚本
├── test_yopo_gazebo.py        # Gazebo测试脚本
└── README_Gazebo.md           # 本说明文档
```

## 启动步骤

### 1. 启动Gazebo仿真
```bash
cd /home/hl-fang/Desktop/quadcopter/YOPO
roslaunch yopo_gazebo.launch
```

这个命令会启动：
- Gazebo仿真环境（森林场景）
- 带深度相机的iris无人机模型
- MAVROS通信节点
- SO3控制器
- YOPO路径规划器

### 2. 验证连接
在新终端中检查节点和话题：
```bash
# 检查ROS节点
rostopic list

# 检查关键话题
rostopic echo /mavros/state
rostopic echo /camera/depth/image_raw
rostopic echo /mavros/local_position/pose
```

### 3. 运行测试脚本
```bash
cd /home/hl-fang/Desktop/quadcopter/YOPO
python3 test_yopo_gazebo.py
```

## 系统架构

### 数据流
```
Gazebo世界 → 深度相机 → /camera/depth/image_raw
                    → /camera/depth/points
                    
YOPO网络 → 路径规划 → /network/pos_cmd → NetworkControl
                                      ↓
PX4固件 ← MAVROS ← SO3控制器 ← /so3_command
```

### 关键话题
- `/camera/depth/image_raw`: 深度图像数据
- `/camera/depth/points`: 点云数据
- `/mavros/local_position/pose`: 无人机位置姿态
- `/mavros/state`: 无人机状态
- `/network/pos_cmd`: YOPO路径规划命令
- `/so3_command`: SO3控制器命令

## 配置说明

### 1. 世界环境 (forest_environment.world)
- 包含树木、障碍物和围墙
- 模拟森林飞行环境
- 可根据需要修改障碍物位置和数量

### 2. 无人机模型 (iris_depth_camera.urdf)
- 基于PX4的iris模型
- 集成深度相机传感器
- 包含IMU、电机和MAVROS插件

### 3. 控制参数
控制器参数在以下文件中配置：
- `/Controller/src/so3_control/config/gains_hummingbird.yaml`
- `/YOPO/config/config.py`

## 测试内容

### 1. 传感器验证
- 深度相机数据获取
- 点云数据处理
- IMU数据读取
- 位置估计精度

### 2. 控制系统测试
- 解锁和起飞
- 位置控制精度
- 航点导航能力
- 姿态稳定性

### 3. YOPO路径规划
- 深度图像处理
- 障碍物检测
- 智能路径生成
- 实时避障能力

## 故障排除

### 常见问题

1. **Gazebo启动失败**
   - 检查Gazebo版本是否兼容
   - 确认模型路径设置正确
   - 查看错误日志：`~/.gazebo/gzserver.log`

2. **MAVROS连接失败**
   - 确认PX4 SITL正在运行
   - 检查端口配置（14560, 14550）
   - 验证MAVROS参数设置

3. **深度相机无数据**
   - 检查Gazebo插件加载
   - 确认话题名称正确
   - 验证相机参数设置

4. **控制器无响应**
   - 检查SO3控制器启动
   - 确认MAVROS模式设置
   - 验证解锁状态

### 调试命令
```bash
# 查看所有话题
rostopic list

# 监控MAVROS状态
rostopic echo /mavros/state

# 检查深度图像
rosrun image_view image_view image:=/camera/depth/image_raw

# 可视化点云
rosrun rviz rviz -d yopo.rviz

# 检查控制器日志
rosnode info /so3_control_nodelet
```

## 性能优化

### 1. Gazebo优化
- 调整物理引擎步长
- 减少不必要的视觉效果
- 优化模型复杂度

### 2. 传感器优化
- 调整深度相机分辨率
- 设置合适的更新频率
- 优化点云处理算法

### 3. 控制优化
- 调整控制器增益
- 优化航点间距
- 设置合适的速度限制

## 扩展功能

### 1. 多机仿真
- 修改launch文件支持多机
- 配置不同namespace
- 实现集群控制

### 2. 复杂环境
- 增加动态障碍物
- 模拟风场干扰
- 添加GPS拒止环境

### 3. 传感器融合
- 集成激光雷达
- 添加双目相机
- 实现SLAM算法

## 实验建议

### 1. 基础测试
- 单点悬停测试
- 简单航点飞行
- 传感器数据验证

### 2. 避障测试
- 静态障碍物避障
- 复杂环境导航
- 紧急避障能力

### 3. 性能评估
- 轨迹跟踪精度
- 计算资源占用
- 实时性能分析

## 参考资料

- [PX4 User Guide](https://docs.px4.io/)
- [MAVROS Documentation](http://wiki.ros.org/mavros)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)

## 联系支持

如有问题，请查看：
1. 系统日志文件
2. ROS话题状态
3. Gazebo仿真状态
4. PX4飞控日志
