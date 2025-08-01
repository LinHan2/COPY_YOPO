#!/usr/bin/env python
"""
里程计姿态转换工具
用于监控和验证无人机的位姿估计质量
将四元数姿态转换为欧拉角（度），便于人类理解和调试
"""

import rospy
import numpy as np
import tf
from tf import transformations as tfs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Joy

pub = None
pub1 = None

def callback(odom_msg):
    """
    里程计回调函数：将四元数姿态转换为欧拉角
    用于实时监控飞行器的姿态变化
    """
    q = np.array([odom_msg.pose.pose.orientation.x,
                  odom_msg.pose.pose.orientation.y,
                  odom_msg.pose.pose.orientation.z,
                  odom_msg.pose.pose.orientation.w])

    # 转换为欧拉角 (Roll, Pitch, Yaw)
    e = tfs.euler_from_quaternion(q, 'rzyx')

    euler_msg = Vector3Stamped()
    euler_msg.header = odom_msg.header
    euler_msg.vector.z = e[0]*180.0/3.14159  # Yaw (偏航角)
    euler_msg.vector.y = e[1]*180.0/3.14159  # Pitch (俯仰角)
    euler_msg.vector.x = e[2]*180.0/3.14159  # Roll (翻滚角)

    pub.publish(euler_msg)
    
    # 实时打印位姿信息用于调试
    pos = odom_msg.pose.pose.position
    vel = odom_msg.twist.twist.linear
    print(f"位置: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) | "
          f"姿态: R={euler_msg.vector.x:.1f}° P={euler_msg.vector.y:.1f}° Y={euler_msg.vector.z:.1f}° | "
          f"速度: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")

def imu_callback(imu_msg):
    q = np.array([imu_msg.orientation.x,
                  imu_msg.orientation.y,
                  imu_msg.orientation.z,
                  imu_msg.orientation.w])

    e = tfs.euler_from_quaternion(q, 'rzyx')

    euler_msg = Vector3Stamped()
    euler_msg.header = imu_msg.header
    euler_msg.vector.z = e[0]*180.0/3.14159
    euler_msg.vector.y = e[1]*180.0/3.14159
    euler_msg.vector.x = e[2]*180.0/3.14159

    pub1.publish(euler_msg)

def joy_callback(joy_msg):
    out_msg = Vector3Stamped()
    out_msg.header = joy_msg.header
    out_msg.vector.z = -joy_msg.axes[3]
    out_msg.vector.y = joy_msg.axes[1]
    out_msg.vector.x = joy_msg.axes[0]

    pub2.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("odom_to_euler")

    pub = rospy.Publisher("~euler", Vector3Stamped, queue_size=10)
    sub = rospy.Subscriber("~odom", Odometry, callback)

    pub1 = rospy.Publisher("~imueuler", Vector3Stamped, queue_size=10)
    sub1 = rospy.Subscriber("~imu", Imu, imu_callback)

    pub2 = rospy.Publisher("~ctrlout", Vector3Stamped, queue_size=10)
    sub2 = rospy.Subscriber("~ctrlin", Joy, joy_callback)

    rospy.spin()
