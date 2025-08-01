#!/usr/bin/env python3
"""
轨迹跟踪性能验证工具
用于测试无人机轨迹跟踪精度和响应性能
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Float64
import tf.transformations as tf_trans
from collections import deque
import threading
import time
import math

class TrajectoryTrackingValidator:
    def __init__(self):
        rospy.init_node('trajectory_tracking_validator')
        
        # 数据存储
        self.desired_positions = deque(maxlen=2000)
        self.actual_positions = deque(maxlen=2000)
        self.desired_velocities = deque(maxlen=2000)
        self.actual_velocities = deque(maxlen=2000)
        self.timestamps = deque(maxlen=2000)
        
        # 跟踪性能指标
        self.position_errors = deque(maxlen=1000)
        self.velocity_errors = deque(maxlen=1000)
        self.max_position_error = 0.0
        self.rms_position_error = 0.0
        self.steady_state_error = 0.0
        
        # 当前状态
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.desired_pos = np.array([0.0, 0.0, 0.0])
        self.desired_vel = np.array([0.0, 0.0, 0.0])
        
        # 订阅器
        self.odom_sub = rospy.Subscriber('/vins_estimator/imu_propagate', Odometry, 
                                        self.odom_callback, queue_size=1)
        self.cmd_sub = rospy.Subscriber('/position_cmd', PositionCommand, 
                                       self.cmd_callback, queue_size=1)
        
        # 发布器 - 路径可视化
        self.desired_path_pub = rospy.Publisher('/trajectory_tracking/desired_path', Path, queue_size=1)
        self.actual_path_pub = rospy.Publisher('/trajectory_tracking/actual_path', Path, queue_size=1)
        self.error_pub = rospy.Publisher('/trajectory_tracking/position_error', Float64, queue_size=1)
        
        # 测试轨迹发布器
        self.test_cmd_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=1)
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 启动性能评估定时器
        self.eval_timer = rospy.Timer(rospy.Duration(1.0), self.evaluate_tracking_performance)
        
        print("=== 轨迹跟踪性能验证器启动 ===")
        print("订阅话题:")
        print("  - /vins_estimator/imu_propagate (实际位置)")
        print("  - /position_cmd (期望轨迹)")
        print("发布话题:")
        print("  - /trajectory_tracking/desired_path (期望路径)")
        print("  - /trajectory_tracking/actual_path (实际路径)")
        print("  - /trajectory_tracking/position_error (位置误差)")
        print("可用测试轨迹:")
        print("  - rosservice call /start_circle_test")
        print("  - rosservice call /start_figure8_test") 
        print("  - rosservice call /start_step_test")
        
        # 创建测试服务
        from std_srvs.srv import Empty
        self.circle_srv = rospy.Service('/start_circle_test', Empty, self.start_circle_test)
        self.figure8_srv = rospy.Service('/start_figure8_test', Empty, self.start_figure8_test)
        self.step_srv = rospy.Service('/start_step_test', Empty, self.start_step_test)
        
        rospy.spin()
    
    def odom_callback(self, msg):
        """处理里程计数据"""
        with self.lock:
            # 提取当前位置和速度
            self.current_pos = np.array([msg.pose.pose.position.x,
                                       msg.pose.pose.position.y,
                                       msg.pose.pose.position.z])
            
            self.current_vel = np.array([msg.twist.twist.linear.x,
                                       msg.twist.twist.linear.y,
                                       msg.twist.twist.linear.z])
            
            # 存储历史数据
            current_time = rospy.Time.now().to_sec()
            self.actual_positions.append(self.current_pos.copy())
            self.actual_velocities.append(self.current_vel.copy())
            self.timestamps.append(current_time)
            
            # 计算跟踪误差
            pos_error = np.linalg.norm(self.desired_pos - self.current_pos)
            vel_error = np.linalg.norm(self.desired_vel - self.current_vel)
            
            self.position_errors.append(pos_error)
            self.velocity_errors.append(vel_error)
            
            # 更新最大误差
            if pos_error > self.max_position_error:
                self.max_position_error = pos_error
            
            # 发布当前误差
            error_msg = Float64()
            error_msg.data = pos_error
            self.error_pub.publish(error_msg)
            
            # 更新路径可视化
            self.publish_paths()
    
    def cmd_callback(self, msg):
        """处理位置命令"""
        with self.lock:
            # 提取期望位置和速度
            self.desired_pos = np.array([msg.position.x, msg.position.y, msg.position.z])
            self.desired_vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
            
            # 存储期望轨迹
            self.desired_positions.append(self.desired_pos.copy())
            self.desired_velocities.append(self.desired_vel.copy())
    
    def publish_paths(self):
        """发布路径可视化"""
        if len(self.actual_positions) < 2:
            return
        
        # 期望路径
        desired_path = Path()
        desired_path.header.frame_id = "world"
        desired_path.header.stamp = rospy.Time.now()
        
        for i, pos in enumerate(list(self.desired_positions)[-100:]):  # 最近100个点
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1] 
            pose.pose.position.z = pos[2]
            pose.pose.orientation.w = 1.0
            desired_path.poses.append(pose)
        
        self.desired_path_pub.publish(desired_path)
        
        # 实际路径
        actual_path = Path()
        actual_path.header.frame_id = "world"
        actual_path.header.stamp = rospy.Time.now()
        
        for i, pos in enumerate(list(self.actual_positions)[-100:]):  # 最近100个点
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]
            pose.pose.orientation.w = 1.0
            actual_path.poses.append(pose)
        
        self.actual_path_pub.publish(actual_path)
    
    def evaluate_tracking_performance(self, event):
        """评估跟踪性能"""
        with self.lock:
            if len(self.position_errors) < 10:
                return
            
            # 计算RMS误差
            recent_errors = list(self.position_errors)[-50:]  # 最近50个误差
            self.rms_position_error = np.sqrt(np.mean(np.square(recent_errors)))
            
            # 计算稳态误差
            if len(recent_errors) >= 20:
                self.steady_state_error = np.mean(recent_errors[-20:])  # 最近20个的平均值
            
            # 性能评估
            performance_level = self.assess_performance()
            
            # 打印性能报告
            print(f"轨迹跟踪性能: {performance_level}")
            print(f"  位置误差 - RMS: {self.rms_position_error:.4f}m, 最大: {self.max_position_error:.4f}m, 稳态: {self.steady_state_error:.4f}m")
            print(f"  当前位置: ({self.current_pos[0]:+6.3f}, {self.current_pos[1]:+6.3f}, {self.current_pos[2]:+6.3f})")
            print(f"  期望位置: ({self.desired_pos[0]:+6.3f}, {self.desired_pos[1]:+6.3f}, {self.desired_pos[2]:+6.3f})")
            print(f"  瞬时误差: {np.linalg.norm(self.desired_pos - self.current_pos):.4f}m")
            print("-" * 60)
    
    def assess_performance(self):
        """评估跟踪性能等级"""
        if self.rms_position_error < 0.05:
            return "🟢 优秀 (RMS < 5cm)"
        elif self.rms_position_error < 0.1:
            return "🟡 良好 (RMS < 10cm)"  
        elif self.rms_position_error < 0.2:
            return "🟠 一般 (RMS < 20cm)"
        else:
            return "🔴 较差 (RMS > 20cm)"
    
    # 测试轨迹生成函数
    def start_circle_test(self, req):
        """启动圆形轨迹测试"""
        rospy.loginfo("开始圆形轨迹跟踪测试...")
        
        def circle_trajectory():
            rate = rospy.Rate(20)  # 20Hz
            start_time = rospy.Time.now().to_sec()
            radius = 1.0
            height = 1.0
            period = 10.0  # 10秒一圈
            
            for i in range(400):  # 20秒测试
                if rospy.is_shutdown():
                    break
                
                t = (rospy.Time.now().to_sec() - start_time) / period * 2 * math.pi
                
                cmd = PositionCommand()
                cmd.header.stamp = rospy.Time.now()
                cmd.header.frame_id = "world"
                
                # 圆形轨迹
                cmd.position.x = radius * math.cos(t)
                cmd.position.y = radius * math.sin(t)
                cmd.position.z = height
                
                # 圆形轨迹速度
                omega = 2 * math.pi / period
                cmd.velocity.x = -radius * omega * math.sin(t)
                cmd.velocity.y = radius * omega * math.cos(t)
                cmd.velocity.z = 0.0
                
                # 圆形轨迹加速度
                cmd.acceleration.x = -radius * omega * omega * math.cos(t)
                cmd.acceleration.y = -radius * omega * omega * math.sin(t)
                cmd.acceleration.z = 0.0
                
                cmd.yaw = t  # 朝向切线方向
                cmd.yaw_dot = omega
                
                self.test_cmd_pub.publish(cmd)
                rate.sleep()
        
        # 在后台线程中运行
        import threading
        thread = threading.Thread(target=circle_trajectory)
        thread.daemon = True
        thread.start()
        
        from std_srvs.srv import EmptyResponse
        return EmptyResponse()
    
    def start_figure8_test(self, req):
        """启动8字轨迹测试"""
        rospy.loginfo("开始8字轨迹跟踪测试...")
        
        def figure8_trajectory():
            rate = rospy.Rate(20)  # 20Hz
            start_time = rospy.Time.now().to_sec()
            scale = 1.0
            height = 1.0
            period = 16.0  # 16秒一个8字
            
            for i in range(640):  # 32秒测试
                if rospy.is_shutdown():
                    break
                
                t = (rospy.Time.now().to_sec() - start_time) / period * 2 * math.pi
                
                cmd = PositionCommand()
                cmd.header.stamp = rospy.Time.now()
                cmd.header.frame_id = "world"
                
                # 8字轨迹 (Lemniscate)
                cmd.position.x = scale * math.sin(t)
                cmd.position.y = scale * math.sin(t) * math.cos(t)
                cmd.position.z = height
                
                # 8字轨迹速度
                omega = 2 * math.pi / period
                cmd.velocity.x = scale * omega * math.cos(t)
                cmd.velocity.y = scale * omega * (math.cos(t) * math.cos(t) - math.sin(t) * math.sin(t))
                cmd.velocity.z = 0.0
                
                cmd.yaw = 0.0
                cmd.yaw_dot = 0.0
                
                self.test_cmd_pub.publish(cmd)
                rate.sleep()
        
        import threading
        thread = threading.Thread(target=figure8_trajectory)
        thread.daemon = True
        thread.start()
        
        from std_srvs.srv import EmptyResponse
        return EmptyResponse()
    
    def start_step_test(self, req):
        """启动阶跃响应测试"""
        rospy.loginfo("开始阶跃响应测试...")
        
        def step_trajectory():
            rate = rospy.Rate(20)  # 20Hz
            
            # 序列：起始点 -> 各轴阶跃
            waypoints = [
                [0.0, 0.0, 1.0],  # 起始
                [1.0, 0.0, 1.0],  # X轴阶跃
                [1.0, 1.0, 1.0],  # Y轴阶跃
                [1.0, 1.0, 2.0],  # Z轴阶跃
                [0.0, 0.0, 1.0],  # 返回起始
            ]
            
            for waypoint in waypoints:
                # 保持每个航点5秒
                for i in range(100):
                    if rospy.is_shutdown():
                        break
                    
                    cmd = PositionCommand()
                    cmd.header.stamp = rospy.Time.now()
                    cmd.header.frame_id = "world"
                    
                    cmd.position.x = waypoint[0]
                    cmd.position.y = waypoint[1]
                    cmd.position.z = waypoint[2]
                    
                    cmd.velocity.x = 0.0
                    cmd.velocity.y = 0.0
                    cmd.velocity.z = 0.0
                    
                    cmd.yaw = 0.0
                    cmd.yaw_dot = 0.0
                    
                    self.test_cmd_pub.publish(cmd)
                    rate.sleep()
        
        import threading
        thread = threading.Thread(target=step_trajectory)
        thread.daemon = True
        thread.start()
        
        from std_srvs.srv import EmptyResponse
        return EmptyResponse()

if __name__ == '__main__':
    try:
        validator = TrajectoryTrackingValidator()
    except rospy.ROSInterruptException:
        print("轨迹跟踪验证器被中断")
