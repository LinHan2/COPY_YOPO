#!/bin/bash

# 终端1: 启动Realsense相机
gnome-terminal --title="Realsense Camera" -- bash -c "
roslaunch realsense2_camera rs_camera.launch \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=linear_interpolation \
    color_width:=640 \
    color_height:=480 \
    color_fps:=30;
exec bash"
sleep 10  # 等待相机启动
# 终端2: 启动VINS-Mono
gnome-terminal --title="VINS-Mono" -- bash -c "
cd ~/catkin_ws/src/VINS-Mono;
roslaunch vins_estimator realsense_color.launch;
exec bash"
sleep 5
# 终端3: 启动控制器
gnome-terminal --title="Controller" -- bash -c "
cd ~/Desktop/YOPO/Controller;
source devel/setup.bash;
roslaunch so3_control controller_network.launch hover_thrust:=0.4;
exec bash"
# 终端4: 运行YOPO程序
# gnome-terminal --title="YOPO Inference" -- bash -c "
# cd ~/Desktop/YOPO/YOPO;
# conda activate yopo;
# source /home/nvidia/Desktop/YOPO/Controller/devel/setup.bash;
# python test_yopo_ros.py --use_tensorrt=1;
# exec bash"
# 终端5: 启动RVIZ可视化
gnome-terminal --title="Visualization" -- bash -c "
roslaunch vins_estimator vins_rviz.launch 
exec bash"

# 终端6: 启动MAVROS
gnome-terminal --title="MAVROS" -- bash -c "
roslaunch mavros px4.launch;
exec bash"

# 终端7: 运行控制示例程序
gnome-terminal --title="Control Example" -- bash -c "
cd ~/Desktop/YOPO/Controller;
source devel/setup.bash;
rosrun so3_control control_example;
exec bash"

