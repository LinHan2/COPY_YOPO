#include <cmath>
#include <Eigen/Eigen>
#include <iostream>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "quad_sim_example");
  ros::NodeHandle nh("~");

  ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);

  ros::Duration(2.0).sleep();

  double max_velocity = 1.0;     // 最大速度限制 (m/s)
  double max_acceleration = 0.5; // 最大加速度限制 (m/s^2)

  while (ros::ok())
  {
    std::cout << "\033[42m选择模式: 1-单点位置控制, 2-画圆圈, 3-画八字\033[0m" << std::endl;
    int mode;
    std::cin >> mode;

    if (mode == 1) // 单点位置控制
    {
      double x, y, z;
      std::cout << "输入目标点 (x, y, z): ";
      std::cin >> x >> y >> z;

      for (int i = 0; i < 500; i++)
      {
        quadrotor_msgs::PositionCommand cmd;
        cmd.position.x = x;
        cmd.position.y = y;
        cmd.position.z = z;
        cmd.velocity.x = std::min(max_velocity, std::abs(x) / 2.0);
        cmd.velocity.y = std::min(max_velocity, std::abs(y) / 2.0);
        cmd.velocity.z = std::min(max_velocity, std::abs(z) / 2.0);
        cmd.acceleration.x = std::min(max_acceleration, std::abs(x) / 4.0);
        cmd.acceleration.y = std::min(max_acceleration, std::abs(y) / 4.0);
        cmd.acceleration.z = std::min(max_acceleration, std::abs(z) / 4.0);
        cmd_pub.publish(cmd);

        ros::Duration(0.01).sleep();
        ros::spinOnce();
      }
    }
    else if (mode == 2) // 画圆圈
    {
      double radius;
      std::cout << "输入圆的半径: ";
      std::cin >> radius;

      for (int i = 0; i < 500; i++)
      {
        double angle = (i / 500.0) * 2 * M_PI; // 当前角度
        quadrotor_msgs::PositionCommand cmd;
        cmd.position.x = radius * cos(angle);
        cmd.position.y = radius * sin(angle);
        cmd.position.z = 1.0; // 固定高度
        cmd.velocity.x = -radius * sin(angle) * max_velocity;
        cmd.velocity.y = radius * cos(angle) * max_velocity;
        cmd.velocity.z = 0.0;
        cmd.acceleration.x = -radius * cos(angle) * max_acceleration;
        cmd.acceleration.y = -radius * sin(angle) * max_acceleration;
        cmd.acceleration.z = 0.0;
        cmd_pub.publish(cmd);

        ros::Duration(0.01).sleep();
        ros::spinOnce();
      }
    }
    else if (mode == 3) // 画八字
    {
      double scale;
      std::cout << "输入八字的大小: ";
      std::cin >> scale;

      for (int i = 0; i < 500; i++)
      {
        double t = (i / 500.0) * 2 * M_PI; // 当前时间参数
        quadrotor_msgs::PositionCommand cmd;
        cmd.position.x = scale * sin(t);
        cmd.position.y = scale * sin(2 * t) / 2.0;
        cmd.position.z = 1.0; // 固定高度
        cmd.velocity.x = scale * cos(t) * max_velocity;
        cmd.velocity.y = scale * cos(2 * t) * max_velocity;
        cmd.velocity.z = 0.0;
        cmd.acceleration.x = -scale * sin(t) * max_acceleration;
        cmd.acceleration.y = -2 * scale * sin(2 * t) * max_acceleration;
        cmd.acceleration.z = 0.0;
        cmd_pub.publish(cmd);

        ros::Duration(0.01).sleep();
        ros::spinOnce();
      }
    }
    else
    {
      std::cout << "无效的模式选择，请重新输入。" << std::endl;
    }
  }

  return 0;
}
