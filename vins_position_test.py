#!/usr/bin/env python3
"""
简单位置控制测试 - 验证NetworkControl + 视觉里程计
不依赖YOPO，直接使用VINS定位和NetworkControl
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np

class SimplePositionTest:
    def __init__(self):
        rospy.init_node('simple_position_test')
        
        # 状态变量
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.odom_received = False
        
        # 发布器：通过RViz设置目标点，NetworkControl会响应
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # 订阅器：监听VINS里程计状态
        self.odom_sub = rospy.Subscriber('/vins_estimator/imu_propagate', Odometry, self.odom_callback)
        
        print("=== 简单位置控制测试启动 ===")
        print("监听话题: /vins_estimator/imu_propagate (VINS里程计)")
        print("发布话题: /move_base_simple/goal (目标位置)")
        print("等待VINS里程计数据...")
        
        # 测试航点序列
        self.test_waypoints = [
            [1.0, 0.0, 1.5],    # 向前1米
            [1.0, 1.0, 1.5],    # 向右1米
            [0.0, 1.0, 1.5],    # 向左1米
            [0.0, 0.0, 1.5],    # 回原点
            [0.0, 0.0, 2.0],    # 上升0.5米
        ]
        self.current_waypoint = 0
        self.waypoint_timer = None
        
        rospy.spin()
    
    def odom_callback(self, msg):
        """VINS里程计回调"""
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        if not self.odom_received:
            self.odom_received = True
            print(f"VINS里程计数据接收成功!")
            print(f"当前位置: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f})")
            print("5秒后开始航点测试...")
            
            # 5秒后开始测试
            self.waypoint_timer = rospy.Timer(rospy.Duration(5.0), self.start_waypoint_test, oneshot=True)
        
        # 每2秒打印一次当前位置
        if rospy.Time.now().to_sec() % 2.0 < 0.1:
            print(f"当前位置: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f})")
    
    def start_waypoint_test(self, event):
        """开始航点测试"""
        if self.current_waypoint < len(self.test_waypoints):
            waypoint = self.test_waypoints[self.current_waypoint]
            self.send_goal(waypoint)
            
            # 8秒后发送下一个航点
            self.current_waypoint += 1
            if self.current_waypoint < len(self.test_waypoints):
                self.waypoint_timer = rospy.Timer(rospy.Duration(8.0), self.start_waypoint_test, oneshot=True)
            else:
                print("所有航点测试完成!")
    
    def send_goal(self, position):
        """发送目标位置"""
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.position.z = position[2]
        
        # 姿态设为默认（无旋转）
        goal.pose.orientation.w = 1.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        
        self.goal_pub.publish(goal)
        
        distance = np.linalg.norm(np.array(position) - self.current_pos)
        print(f"发送目标: ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}), 距离: {distance:.2f}m")

if __name__ == '__main__':
    try:
        test = SimplePositionTest()
    except rospy.ROSInterruptException:
        print("测试被中断")
