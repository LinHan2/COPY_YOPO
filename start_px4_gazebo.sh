#!/bin/bash

# PX4启动脚本 - Gazebo仿真
# 用于YOPO项目的Gazebo集成

echo "正在启动PX4 SITL for Gazebo..."

# 设置PX4环境变量
export PX4_HOME_LAT=40.072842
export PX4_HOME_LON=-74.165607
export PX4_HOME_ALT=40.0

# 设置Gazebo环境变量
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/models

# 检查PX4是否安装
if [ ! -d "/opt/ros/noetic/share/px4" ]; then
    echo "警告: PX4未找到，请确保已安装PX4"
    echo "可以通过以下命令安装:"
    echo "git clone https://github.com/PX4/PX4-Autopilot.git"
    echo "cd PX4-Autopilot"
    echo "make px4_sitl gazebo"
fi

# 启动PX4 SITL
cd /opt/ros/noetic/share/px4 || {
    echo "错误: 无法找到PX4目录"
    exit 1
}

# 设置机型为iris
export PX4_SIM_MODEL=iris

# 启动PX4 SITL
make px4_sitl gazebo_iris__forest_environment
