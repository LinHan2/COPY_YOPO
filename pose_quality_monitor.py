#!/usr/bin/env python3
"""
位姿估计质量监控工具
用于评估无人机定位系统的可靠性和精度
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf.transformations as tf_trans
from collections import deque
import threading
import time

class PoseEstimationMonitor:
    def __init__(self):
        rospy.init_node('pose_estimation_monitor')
        
        # 数据存储
        self.pose_history = deque(maxlen=1000)  # 位置历史
        self.velocity_history = deque(maxlen=1000)  # 速度历史
        self.acceleration_history = deque(maxlen=200)  # 加速度历史
        self.time_history = deque(maxlen=1000)  # 时间戳历史
        
        # 质量评估指标
        self.position_variance = np.array([0.0, 0.0, 0.0])
        self.velocity_variance = np.array([0.0, 0.0, 0.0])
        self.last_update_time = rospy.Time.now()
        self.update_frequency = 0.0
        
        # 订阅器
        self.odom_sub = rospy.Subscriber('/vins_estimator/imu_propagate', Odometry, 
                                        self.odom_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data_raw', Imu, 
                                       self.imu_callback, queue_size=1)
        
        # 发布器 - 质量指标
        self.pos_var_pub = rospy.Publisher('/pose_quality/position_variance', Float64, queue_size=1)
        self.vel_var_pub = rospy.Publisher('/pose_quality/velocity_variance', Float64, queue_size=1)
        self.freq_pub = rospy.Publisher('/pose_quality/update_frequency', Float64, queue_size=1)
        
        # 状态变量
        self.last_pose = None
        self.last_velocity = None
        self.last_imu_acc = None
        self.lock = threading.Lock()
        
        # 启动监控定时器
        self.monitor_timer = rospy.Timer(rospy.Duration(2.0), self.publish_quality_metrics)
        
        print("=== 位姿估计质量监控器启动 ===")
        print("监控话题:")
        print("  - /vins_estimator/imu_propagate (VINS里程计)")
        print("  - /mavros/imu/data_raw (IMU数据)")
        print("发布质量指标:")
        print("  - /pose_quality/position_variance")
        print("  - /pose_quality/velocity_variance") 
        print("  - /pose_quality/update_frequency")
        
        rospy.spin()
    
    def odom_callback(self, msg):
        """处理里程计数据并评估质量"""
        current_time = rospy.Time.now()
        
        with self.lock:
            # 提取位置和速度
            position = np.array([msg.pose.pose.position.x, 
                               msg.pose.pose.position.y, 
                               msg.pose.pose.position.z])
            
            velocity = np.array([msg.twist.twist.linear.x,
                               msg.twist.twist.linear.y,
                               msg.twist.twist.linear.z])
            
            # 计算更新频率
            if self.last_update_time is not None:
                dt = (current_time - self.last_update_time).to_sec()
                if dt > 0:
                    self.update_frequency = 1.0 / dt
            
            # 存储历史数据
            self.pose_history.append(position)
            self.velocity_history.append(velocity)
            self.time_history.append(current_time.to_sec())
            
            # 计算位置方差（稳定性指标）
            if len(self.pose_history) > 10:
                recent_poses = np.array(list(self.pose_history)[-10:])
                self.position_variance = np.var(recent_poses, axis=0)
            
            # 计算速度方差（平滑性指标）
            if len(self.velocity_history) > 10:
                recent_velocities = np.array(list(self.velocity_history)[-10:])
                self.velocity_variance = np.var(recent_velocities, axis=0)
            
            # 检测异常跳跃
            if self.last_pose is not None:
                position_jump = np.linalg.norm(position - self.last_pose)
                if position_jump > 0.5:  # 位置跳跃超过50cm
                    rospy.logwarn(f"检测到位置异常跳跃: {position_jump:.3f}m")
            
            if self.last_velocity is not None:
                velocity_jump = np.linalg.norm(velocity - self.last_velocity)
                if velocity_jump > 2.0:  # 速度跳跃超过2m/s
                    rospy.logwarn(f"检测到速度异常跳跃: {velocity_jump:.3f}m/s")
            
            # 更新状态
            self.last_pose = position.copy()
            self.last_velocity = velocity.copy()
            self.last_update_time = current_time
            
            # 实时显示关键信息
            self.print_status(position, velocity)
    
    def imu_callback(self, msg):
        """处理IMU数据"""
        with self.lock:
            # 提取加速度
            acceleration = np.array([msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z])
            
            self.acceleration_history.append(acceleration)
            self.last_imu_acc = acceleration.copy()
    
    def print_status(self, position, velocity):
        """实时打印状态信息"""
        # 每秒打印一次
        if rospy.Time.now().to_sec() % 1.0 < 0.05:
            pos_var_norm = np.linalg.norm(self.position_variance)
            vel_var_norm = np.linalg.norm(self.velocity_variance)
            
            status = "🟢 良好" if pos_var_norm < 0.001 else "🟡 一般" if pos_var_norm < 0.01 else "🔴 较差"
            
            print(f"位置: ({position[0]:+6.3f}, {position[1]:+6.3f}, {position[2]:+6.3f}) | "
                  f"速度: ({velocity[0]:+5.2f}, {velocity[1]:+5.2f}, {velocity[2]:+5.2f}) | "
                  f"频率: {self.update_frequency:4.1f}Hz | "
                  f"稳定性: {status} (σ={pos_var_norm:.4f})")
    
    def publish_quality_metrics(self, event):
        """发布质量评估指标"""
        with self.lock:
            # 发布位置方差
            pos_var_msg = Float64()
            pos_var_msg.data = float(np.linalg.norm(self.position_variance))
            self.pos_var_pub.publish(pos_var_msg)
            
            # 发布速度方差
            vel_var_msg = Float64()
            vel_var_msg.data = float(np.linalg.norm(self.velocity_variance))
            self.vel_var_pub.publish(vel_var_msg)
            
            # 发布更新频率
            freq_msg = Float64()
            freq_msg.data = self.update_frequency
            self.freq_pub.publish(freq_msg)
    
    def analyze_quality(self):
        """分析位姿估计质量"""
        with self.lock:
            if len(self.pose_history) < 50:
                return "数据不足"
            
            # 计算质量评分
            pos_var_norm = np.linalg.norm(self.position_variance)
            vel_var_norm = np.linalg.norm(self.velocity_variance)
            
            quality_score = 100.0
            
            # 位置稳定性评分 (30%)
            if pos_var_norm > 0.01:
                quality_score -= 30
            elif pos_var_norm > 0.001:
                quality_score -= 15
            
            # 速度平滑性评分 (25%)
            if vel_var_norm > 1.0:
                quality_score -= 25
            elif vel_var_norm > 0.1:
                quality_score -= 12
            
            # 更新频率评分 (25%)
            if self.update_frequency < 20:
                quality_score -= 25
            elif self.update_frequency < 30:
                quality_score -= 12
            
            # 数据连续性评分 (20%)
            if len(self.pose_history) < len(self.time_history):
                quality_score -= 20
            
            return f"质量评分: {quality_score:.1f}/100"

if __name__ == '__main__':
    try:
        monitor = PoseEstimationMonitor()
    except rospy.ROSInterruptException:
        print("监控器被中断")
