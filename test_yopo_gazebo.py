#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOPO Gazebo仿真测试脚本
用于在Gazebo环境中测试YOPO智能路径规划系统
"""

import rospy
import numpy as np
import time
import tf.transformations as tf_trans
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Twist
import cv2
from cv_bridge import CvBridge

class YOPOGazeboTest:
    def __init__(self):
        rospy.init_node('yopo_gazebo_test', anonymous=True)
        
        # 初始化变量
        self.current_pose = None
        self.depth_image = None
        self.point_cloud = None
        self.bridge = CvBridge()
        
        # 发布器  
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.vel_cmd_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        
        # 订阅器
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.cloud_sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.cloud_callback)
        
        # 服务客户端
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        # 等待连接
        rospy.loginfo("等待MAVROS连接...")
        time.sleep(5)
        
    def pose_callback(self, msg):
        """位置回调函数"""
        self.current_pose = msg
        
    def depth_callback(self, msg):
        """深度图像回调函数"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            rospy.logerr(f"深度图像转换错误: {e}")
            
    def cloud_callback(self, msg):
        """点云回调函数"""
        self.point_cloud = msg
        
    def arm_and_takeoff(self, target_altitude=2.0):
        """解锁并起飞"""
        rospy.loginfo("正在解锁无人机...")
        
        # 设置为OFFBOARD模式
        try:
            if self.set_mode_client(custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo("OFFBOARD模式设置成功")
            else:
                rospy.logerr("OFFBOARD模式设置失败")
                return False
        except Exception as e:
            rospy.logerr(f"设置模式错误: {e}")
            return False
            
        # 解锁
        try:
            if self.arming_client(True).success:
                rospy.loginfo("无人机解锁成功")
            else:
                rospy.logerr("无人机解锁失败")
                return False
        except Exception as e:
            rospy.logerr(f"解锁错误: {e}")
            return False
            
        # 起飞到目标高度
        rospy.loginfo(f"正在起飞到{target_altitude}米高度...")
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = 0
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = target_altitude
        target_pose.pose.orientation.w = 1.0
        
        # 发送目标位置
        rate = rospy.Rate(20)  # 20Hz
        for i in range(100):  # 发送5秒
            target_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(target_pose)
            rate.sleep()
            
        return True
        
    def send_position_command(self, x, y, z, yaw=0):
        """发送位置命令到MAVROS"""
        cmd = PoseStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"
        
        # 设置位置
        cmd.pose.position.x = x
        cmd.pose.position.y = y
        cmd.pose.position.z = z
        
        # 设置方向（简单的偏航角转换为四元数）
        quat = tf_trans.quaternion_from_euler(0, 0, yaw)
        cmd.pose.orientation.x = quat[0]
        cmd.pose.orientation.y = quat[1]
        cmd.pose.orientation.z = quat[2]
        cmd.pose.orientation.w = quat[3]
        
        self.local_pos_pub.publish(cmd)
        rospy.loginfo(f"发送位置命令: ({x:.2f}, {y:.2f}, {z:.2f})")
        
    def check_sensors(self):
        """检查传感器数据"""
        rospy.loginfo("检查传感器数据...")
        
        # 检查位置信息
        if self.current_pose is not None:
            pos = self.current_pose.pose.position
            rospy.loginfo(f"当前位置: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        else:
            rospy.logwarn("未收到位置信息")
            
        # 检查深度图像
        if self.depth_image is not None:
            height, width = self.depth_image.shape
            rospy.loginfo(f"深度图像尺寸: {width}x{height}")
            rospy.loginfo(f"深度范围: {np.nanmin(self.depth_image):.2f} - {np.nanmax(self.depth_image):.2f}m")
        else:
            rospy.logwarn("未收到深度图像")
            
        # 检查点云
        if self.point_cloud is not None:
            rospy.loginfo(f"点云数据点数: {self.point_cloud.width * self.point_cloud.height}")
        else:
            rospy.logwarn("未收到点云数据")
            
    def test_waypoint_navigation(self):
        """测试航点导航"""
        rospy.loginfo("开始航点导航测试...")
        
        # 定义测试航点
        waypoints = [
            (0, 0, 2),      # 起飞点
            (3, 0, 2),      # 向前
            (3, 3, 2),      # 向右
            (0, 3, 2),      # 向后
            (0, 0, 2),      # 返回起点
        ]
        
        rate = rospy.Rate(1)  # 1Hz
        
        for i, (x, y, z) in enumerate(waypoints):
            rospy.loginfo(f"导航到航点 {i+1}: ({x}, {y}, {z})")
            
            # 发送航点命令
            for _ in range(10):  # 发送10秒
                self.send_position_command(x, y, z)
                rate.sleep()
                
                # 检查是否到达
                if self.current_pose is not None:
                    pos = self.current_pose.pose.position
                    distance = np.sqrt((pos.x - x)**2 + (pos.y - y)**2 + (pos.z - z)**2)
                    if distance < 0.5:  # 50cm容差
                        rospy.loginfo(f"到达航点 {i+1}")
                        break
                        
        rospy.loginfo("航点导航测试完成")
        
    def run_gazebo_test(self):
        """运行Gazebo仿真测试"""
        rospy.loginfo("=== YOPO Gazebo仿真测试开始 ===")
        
        try:
            # 1. 检查传感器
            self.check_sensors()
            time.sleep(2)
            
            # 2. 解锁并起飞
            if not self.arm_and_takeoff():
                rospy.logerr("起飞失败，测试终止")
                return
                
            time.sleep(5)  # 等待起飞稳定
            
            # 3. 再次检查传感器
            self.check_sensors()
            time.sleep(2)
            
            # 4. 测试航点导航
            self.test_waypoint_navigation()
            
            # 5. 降落
            rospy.loginfo("开始降落...")
            self.send_position_command(0, 0, 0.5)  # 降落到0.5米
            time.sleep(10)
            
            rospy.loginfo("=== YOPO Gazebo仿真测试完成 ===")
            
        except KeyboardInterrupt:
            rospy.loginfo("测试被用户中断")
        except Exception as e:
            rospy.logerr(f"测试过程中发生错误: {e}")

def main():
    try:
        tester = YOPOGazeboTest()
        tester.run_gazebo_test()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")

if __name__ == '__main__':
    main()
