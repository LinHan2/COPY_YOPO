#include <Eigen/Eigen>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <limits>

class QuadrotorController {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    std::vector<Eigen::Vector3d> waypoints_;
    bool running_;
    bool use_conservative_limits_;
    std::thread input_thread_;
    
    // 速度和加速度限制参数
    double max_velocity_;
    double max_acceleration_;
    double max_yaw_rate_;
    
    // 当前状态（用于速度和加速度控制）
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    bool position_initialized_;

public:
    QuadrotorController() : nh_("~"), running_(true), position_initialized_(false) {
        cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/so3_control/pos_cmd", 10);
        
        // 从参数服务器读取限制参数
        nh_.param("max_velocity", max_velocity_, 3.0);           
        nh_.param("max_acceleration", max_acceleration_, 5.0);   
        nh_.param("max_yaw_rate", max_yaw_rate_, 1.57);         
        
        // 从参数读取是否使用保守限制
        nh_.param("use_conservative_limits", use_conservative_limits_, false);
        
        // 显示当前配置
        std::cout << "🚁 SO3四旋翼控制器配置:" << std::endl;
        std::cout << "   控制话题: /so3_control/pos_cmd" << std::endl;
        std::cout << "   控制模式: SO3位置/速度/加速度控制" << std::endl;
        
        // 应用限制策略
        if (use_conservative_limits_) {
            std::cout << "⚠️  使用保守限制模式" << std::endl;
            max_velocity_ = std::min(max_velocity_, 2.0);
            max_acceleration_ = std::min(max_acceleration_, 3.0);
        } else {
            std::cout << "🚀 使用标准限制模式" << std::endl;
        }
        
        // 初始化当前状态
        current_position_ = Eigen::Vector3d::Zero();
        current_velocity_ = Eigen::Vector3d::Zero();
        
        std::cout << "📋 速度和加速度限制已设置:" << std::endl;
        std::cout << "   最大速度: " << max_velocity_ << " m/s" << std::endl;
        std::cout << "   最大加速度: " << max_acceleration_ << " m/s²" << std::endl;
        std::cout << "   最大偏航速度: " << max_yaw_rate_ << " rad/s" << std::endl;
        
        ros::Duration(2.0).sleep();
        
        // 启动输入线程
        input_thread_ = std::thread(&QuadrotorController::inputThread, this);
        
        printHelp();
    }

    ~QuadrotorController() {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

    void printHelp() {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "🚁 SO3四旋翼控制器交互界面 (带速度/加速度限制)" << std::endl;
        std::cout << "控制模式: SO3位置/速度/加速度控制" << std::endl;
        std::cout << "限制模式: " << (use_conservative_limits_ ? "保守" : "标准") << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "基本命令:" << std::endl;
        std::cout << "  goto x y z         - 飞到指定位置 (例: goto 5 0 2)" << std::endl;
        std::cout << "  takeoff h          - 起飞到高度h (例: takeoff 2)" << std::endl;
        std::cout << "  land               - 降落到地面" << std::endl;
        std::cout << "  hover              - 原地悬停" << std::endl;
        std::cout << "  emergency          - 紧急降落" << std::endl;
        std::cout << "\n轨迹命令:" << std::endl;
        std::cout << "  figure8            - 执行八字飞行" << std::endl;
        std::cout << "  circle r           - 画圆，半径r (例: circle 3)" << std::endl;
        std::cout << "  square s           - 画正方形，边长s (例: square 4)" << std::endl;
        std::cout << "\n航点命令:" << std::endl;
        std::cout << "  waypoint x y z     - 添加航点 (例: waypoint 5 5 2)" << std::endl;
        std::cout << "  execute            - 执行航点任务" << std::endl;
        std::cout << "  clear              - 清空航点列表" << std::endl;
        std::cout << "\n速度/加速度控制:" << std::endl;
        std::cout << "  vel x y z          - 设置速度 (例: vel 1 0 0)" << std::endl;
        std::cout << "  acc x y z          - 设置加速度 (例: acc 0.5 0 0)" << std::endl;
        std::cout << "  stop               - 立即停止 (速度归零)" << std::endl;
        std::cout << "\n参数设置:" << std::endl;
        std::cout << "  set_max_vel v      - 设置最大速度 (例: set_max_vel 2.0)" << std::endl;
        std::cout << "  set_max_acc a      - 设置最大加速度 (例: set_max_acc 3.0)" << std::endl;
        std::cout << "  toggle_limits      - 切换保守/标准限制模式" << std::endl;
        std::cout << "  show_limits        - 显示当前限制参数" << std::endl;
        std::cout << "  show_status        - 显示系统状态" << std::endl;
        std::cout << "\n其他:" << std::endl;
        std::cout << "  help               - 显示帮助" << std::endl;
        std::cout << "  quit               - 退出程序" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
    }

    void inputThread() {
        std::string input;
        while (running_ && ros::ok()) {
            std::cout << "\n🚁 请输入命令 (输入 help 查看帮助): ";
            std::getline(std::cin, input);
            
            if (!running_) break;
            
            processCommand(input);
        }
    }

    Eigen::Vector3d limitVector(const Eigen::Vector3d& vec, double max_magnitude) {
        double magnitude = vec.norm();
        if (magnitude > max_magnitude) {
            return vec * (max_magnitude / magnitude);
        }
        return vec;
    }

    void processCommand(const std::string& input) {
        std::istringstream iss(input);
        std::string command;
        iss >> command;

        if (command == "goto") {
            double x, y, z;
            if (iss >> x >> y >> z) {
                std::cout << "✈️  飞向位置: (" << x << ", " << y << ", " << z << ")" << std::endl;
                flyToPosition(x, y, z);
            } else {
                std::cout << "❌ 错误: 请输入 goto x y z" << std::endl;
            }
        }
        else if (command == "vel") {
            double x, y, z;
            if (iss >> x >> y >> z) {
                Eigen::Vector3d desired_vel(x, y, z);
                Eigen::Vector3d limited_vel = limitVector(desired_vel, max_velocity_);
                std::cout << "🏃 设置速度: (" << limited_vel.x() << ", " << limited_vel.y() << ", " << limited_vel.z() << ") m/s" << std::endl;
                if (desired_vel.norm() > max_velocity_) {
                    std::cout << "⚠️  速度已限制到最大值: " << max_velocity_ << " m/s" << std::endl;
                }
                publishVelocityCommand(limited_vel.x(), limited_vel.y(), limited_vel.z());
            } else {
                std::cout << "❌ 错误: 请输入 vel x y z" << std::endl;
            }
        }
        else if (command == "acc") {
            double x, y, z;
            if (iss >> x >> y >> z) {
                Eigen::Vector3d desired_acc(x, y, z);
                Eigen::Vector3d limited_acc = limitVector(desired_acc, max_acceleration_);
                std::cout << "🚀 设置加速度: (" << limited_acc.x() << ", " << limited_acc.y() << ", " << limited_acc.z() << ") m/s²" << std::endl;
                if (desired_acc.norm() > max_acceleration_) {
                    std::cout << "⚠️  加速度已限制到最大值: " << max_acceleration_ << " m/s²" << std::endl;
                }
                publishAccelerationCommand(limited_acc.x(), limited_acc.y(), limited_acc.z());
            } else {
                std::cout << "❌ 错误: 请输入 acc x y z" << std::endl;
            }
        }
        else if (command == "stop") {
            std::cout << "🛑 立即停止 (速度归零)" << std::endl;
            publishVelocityCommand(0, 0, 0);
        }
        else if (command == "set_max_vel") {
            double max_vel;
            if (iss >> max_vel && max_vel > 0) {
                max_velocity_ = max_vel;
                if (use_conservative_limits_) {
                    max_velocity_ = std::min(max_velocity_, 2.0);
                    std::cout << "⚠️  保守模式下最大速度被限制为 2.0 m/s" << std::endl;
                }
                std::cout << "✅ 最大速度设置为: " << max_velocity_ << " m/s" << std::endl;
            } else {
                std::cout << "❌ 错误: 请输入有效的正数" << std::endl;
            }
        }
        else if (command == "set_max_acc") {
            double max_acc;
            if (iss >> max_acc && max_acc > 0) {
                max_acceleration_ = max_acc;
                if (use_conservative_limits_) {
                    max_acceleration_ = std::min(max_acceleration_, 3.0);
                    std::cout << "⚠️  保守模式下最大加速度被限制为 3.0 m/s²" << std::endl;
                }
                std::cout << "✅ 最大加速度设置为: " << max_acceleration_ << " m/s²" << std::endl;
            } else {
                std::cout << "❌ 错误: 请输入有效的正数" << std::endl;
            }
        }
        else if (command == "toggle_limits") {
            use_conservative_limits_ = !use_conservative_limits_;
            std::cout << "🔄 切换到" << (use_conservative_limits_ ? "保守" : "标准") << "限制模式" << std::endl;
            
            // 重新应用限制
            if (use_conservative_limits_) {
                max_velocity_ = std::min(max_velocity_, 2.0);
                max_acceleration_ = std::min(max_acceleration_, 3.0);
            }
            
            std::cout << "   新的最大速度: " << max_velocity_ << " m/s" << std::endl;
            std::cout << "   新的最大加速度: " << max_acceleration_ << " m/s²" << std::endl;
        }
        else if (command == "show_limits") {
            std::cout << "\n📊 当前限制参数:" << std::endl;
            std::cout << "   最大速度: " << max_velocity_ << " m/s" << std::endl;
            std::cout << "   最大加速度: " << max_acceleration_ << " m/s²" << std::endl;
            std::cout << "   最大偏航速度: " << max_yaw_rate_ << " rad/s" << std::endl;
            std::cout << "   限制模式: " << (use_conservative_limits_ ? "保守" : "标准") << std::endl;
        }
        else if (command == "show_status") {
            std::cout << "\n📡 系统状态:" << std::endl;
            std::cout << "   控制话题: /so3_control/pos_cmd" << std::endl;
            std::cout << "   控制器类型: SO3控制器" << std::endl;
            std::cout << "   限制模式: " << (use_conservative_limits_ ? "保守" : "标准") << std::endl;
            std::cout << "   航点数量: " << waypoints_.size() << std::endl;
            std::cout << "   运行状态: " << (running_ ? "✅ 运行中" : "❌ 已停止") << std::endl;
        }
        else if (command == "takeoff") {
            double height = 2.0;
            iss >> height;
            std::cout << "🛫 起飞到高度: " << height << "m (使用位置控制)" << std::endl;
            flyToPosition(0, 0, height);
        }
        else if (command == "land") {
            std::cout << "🛬 降落中... (使用位置控制)" << std::endl;
            flyToPosition(0, 0, 0.1);
        }
        else if (command == "emergency") {
            std::cout << "🚨 紧急降落!" << std::endl;
            flyToPosition(0, 0, 0.0);
        }
        else if (command == "hover") {
            std::cout << "🚁 原地悬停" << std::endl;
            publishVelocityCommand(0, 0, 0);
        }
        else if (command == "figure8") {
            if (use_conservative_limits_) {
                std::cout << "⚠️  保守模式下建议谨慎执行复杂轨迹，确认安全后继续..." << std::endl;
                std::cout << "输入 'yes' 确认执行: ";
                std::string confirm;
                std::getline(std::cin, confirm);
                if (confirm != "yes") {
                    std::cout << "❌ 已取消八字飞行" << std::endl;
                    return;
                }
            }
            std::cout << "♾️  执行八字飞行..." << std::endl;
            executeFigure8();
        }
        else if (command == "circle") {
            double radius = 3.0;
            iss >> radius;
            
            if (use_conservative_limits_) {
                radius = std::max(1.0, std::min(5.0, radius));
                std::cout << "⚠️  保守模式下半径限制为1-5米" << std::endl;
            }
            
            std::cout << "⭕ 画圆，半径: " << radius << "m" << std::endl;
            executeCircle(radius);
        }
        else if (command == "square") {
            double side = 4.0;
            iss >> side;
            
            if (use_conservative_limits_) {
                side = std::max(1.0, std::min(5.0, side));
                std::cout << "⚠️  保守模式下边长限制为1-5米" << std::endl;
            }
            
            std::cout << "⬜ 画正方形，边长: " << side << "m" << std::endl;
            executeSquare(side);
        }
        else if (command == "waypoint") {
            double x, y, z;
            if (iss >> x >> y >> z) {
                waypoints_.push_back(Eigen::Vector3d(x, y, z));
                std::cout << "📍 添加航点 " << waypoints_.size() << ": (" << x << ", " << y << ", " << z << ")" << std::endl;
            } else {
                std::cout << "❌ 错误: 请输入 waypoint x y z" << std::endl;
            }
        }
        else if (command == "execute") {
            if (waypoints_.empty()) {
                std::cout << "❌ 航点列表为空，请先添加航点" << std::endl;
            } else {
                std::cout << "🎯 执行航点任务 (" << waypoints_.size() << " 个航点)" << std::endl;
                executeWaypoints();
            }
        }
        else if (command == "clear") {
            waypoints_.clear();
            std::cout << "🗑️  航点列表已清空" << std::endl;
        }
        else if (command == "help") {
            printHelp();
        }
        else if (command == "quit" || command == "exit") {
            std::cout << "👋 再见!" << std::endl;
            running_ = false;
            ros::shutdown();
        }
        else if (!command.empty()) {
            std::cout << "❌ 未知命令: " << command << " (输入 help 查看帮助)" << std::endl;
        }
    }

    void publishCommand(double x, double y, double z) {
        quadrotor_msgs::PositionCommand cmd;
        cmd.position.x = x;
        cmd.position.y = y;
        cmd.position.z = z;
        cmd.velocity.x = 0;
        cmd.velocity.y = 0;
        cmd.velocity.z = 0;
        cmd.acceleration.x = 0;
        cmd.acceleration.y = 0;
        cmd.acceleration.z = 0;
        cmd.yaw = 0;
        cmd.yaw_dot = 0;
        cmd_pub_.publish(cmd);
    }

    void publishVelocityCommand(double vx, double vy, double vz) {
        Eigen::Vector3d velocity(vx, vy, vz);
        velocity = limitVector(velocity, max_velocity_);
        
        quadrotor_msgs::PositionCommand cmd;
        cmd.position.x = std::numeric_limits<float>::quiet_NaN();
        cmd.position.y = std::numeric_limits<float>::quiet_NaN();
        cmd.position.z = std::numeric_limits<float>::quiet_NaN();
        cmd.velocity.x = velocity.x();
        cmd.velocity.y = velocity.y();
        cmd.velocity.z = velocity.z();
        cmd.acceleration.x = 0;
        cmd.acceleration.y = 0;
        cmd.acceleration.z = 0;
        cmd.yaw = 0;
        cmd.yaw_dot = 0;
        
        for (int i = 0; i < 100; i++) {
            cmd_pub_.publish(cmd);
            ros::Duration(0.02).sleep();
            ros::spinOnce();
            if (!running_) break;
        }
    }

    void publishAccelerationCommand(double ax, double ay, double az) {
        Eigen::Vector3d acceleration(ax, ay, az);
        acceleration = limitVector(acceleration, max_acceleration_);
        
        quadrotor_msgs::PositionCommand cmd;
        cmd.position.x = std::numeric_limits<float>::quiet_NaN();
        cmd.position.y = std::numeric_limits<float>::quiet_NaN();
        cmd.position.z = std::numeric_limits<float>::quiet_NaN();
        cmd.velocity.x = std::numeric_limits<float>::quiet_NaN();
        cmd.velocity.y = std::numeric_limits<float>::quiet_NaN();
        cmd.velocity.z = std::numeric_limits<float>::quiet_NaN();
        cmd.acceleration.x = acceleration.x();
        cmd.acceleration.y = acceleration.y();
        cmd.acceleration.z = acceleration.z();
        cmd.yaw = 0;
        cmd.yaw_dot = 0;
        
        for (int i = 0; i < 100; i++) {
            cmd_pub_.publish(cmd);
            ros::Duration(0.02).sleep();
            ros::spinOnce();
            if (!running_) break;
        }
    }

    void flyToPosition(double x, double y, double z) {
        int duration = use_conservative_limits_ ? 500 : 300;
        double sleep_time = use_conservative_limits_ ? 0.05 : 0.02;
        
        for (int i = 0; i < duration; i++) {
            publishCommand(x, y, z);
            ros::Duration(sleep_time).sleep();
            ros::spinOnce();
            if (!running_) break;
        }
    }

    void executeFigure8() {
        double center_x = use_conservative_limits_ ? 2 : 10;
        double center_y = use_conservative_limits_ ? 2 : 5;
        double center_z = use_conservative_limits_ ? 2 : 3;
        double a = use_conservative_limits_ ? 1.5 : 4.0;
        double b = use_conservative_limits_ ? 1.0 : 2.0;
        int steps = use_conservative_limits_ ? 300 : 200;
        
        std::cout << "开始八字飞行，中心: (" << center_x << ", " << center_y << ", " << center_z << ")" << std::endl;
        
        for (int i = 0; i < steps && running_; i++) {
            double t = 2 * M_PI * i / steps;
            
            double x = center_x + a * sin(t);
            double y = center_y + b * sin(2 * t);
            double z = center_z;
            
            publishCommand(x, y, z);
            
            if (i % 30 == 0) {
                std::cout << "♾️  八字进度: " << (i * 100 / steps) << "%" << std::endl;
            }
            
            ros::Duration(use_conservative_limits_ ? 0.1 : 0.05).sleep();
            ros::spinOnce();
        }
        
        std::cout << "✅ 八字飞行完成!" << std::endl;
    }

    void executeCircle(double radius) {
        double center_x = use_conservative_limits_ ? 2 : 5;
        double center_y = use_conservative_limits_ ? 2 : 5;
        double center_z = use_conservative_limits_ ? 2 : 2;
        int steps = use_conservative_limits_ ? 200 : 150;
        
        std::cout << "开始画圆，中心: (" << center_x << ", " << center_y << ", " << center_z << "), 半径: " << radius << std::endl;
        
        for (int i = 0; i < steps && running_; i++) {
            double t = 2 * M_PI * i / steps;
            double x = center_x + radius * cos(t);
            double y = center_y + radius * sin(t);
            double z = center_z;
            
            publishCommand(x, y, z);
            
            if (i % 20 == 0) {
                std::cout << "⭕ 画圆进度: " << (i * 100 / steps) << "%" << std::endl;
            }
            
            ros::Duration(use_conservative_limits_ ? 0.08 : 0.05).sleep();
            ros::spinOnce();
        }
        
        std::cout << "✅ 画圆完成!" << std::endl;
    }

    void executeSquare(double side) {
        double start_x = use_conservative_limits_ ? 1 : 5;
        double start_y = use_conservative_limits_ ? 1 : 5;
        double start_z = use_conservative_limits_ ? 2 : 2;
        
        std::vector<Eigen::Vector3d> corners = {
            Eigen::Vector3d(start_x, start_y, start_z),
            Eigen::Vector3d(start_x + side, start_y, start_z),
            Eigen::Vector3d(start_x + side, start_y + side, start_z),
            Eigen::Vector3d(start_x, start_y + side, start_z),
            Eigen::Vector3d(start_x, start_y, start_z)
        };
        
        std::cout << "开始画正方形，边长: " << side << std::endl;
        
        for (size_t i = 0; i < corners.size() && running_; i++) {
            std::cout << "⬜ 飞向角点 " << (i + 1) << ": (" 
                     << corners[i].x() << ", " << corners[i].y() << ", " << corners[i].z() << ")" << std::endl;
            
            flyToPosition(corners[i].x(), corners[i].y(), corners[i].z());
            
            if (!running_) break;
            ros::Duration(use_conservative_limits_ ? 3.0 : 1.0).sleep();
        }
        
        std::cout << "✅ 正方形飞行完成!" << std::endl;
    }

    void executeWaypoints() {
        for (size_t i = 0; i < waypoints_.size() && running_; i++) {
            std::cout << "🎯 飞向航点 " << (i + 1) << "/" << waypoints_.size() 
                     << ": (" << waypoints_[i].x() << ", " << waypoints_[i].y() << ", " << waypoints_[i].z() << ")" << std::endl;
            
            flyToPosition(waypoints_[i].x(), waypoints_[i].y(), waypoints_[i].z());
            
            if (!running_) break;
            ros::Duration(use_conservative_limits_ ? 4.0 : 2.0).sleep();
        }
        
        std::cout << "✅ 航点任务完成!" << std::endl;
    }

    void spin() {
        while (running_ && ros::ok()) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "quad_sim_interactive");
    
    std::cout << "🚁 SO3四旋翼交互式控制器启动..." << std::endl;
    
    QuadrotorController controller;
    controller.spin();
    
    return 0;
}
// #include <Eigen/Eigen>
// #include <quadrotor_msgs/PositionCommand.h>
// #include <ros/ros.h>

// int main(int argc, char **argv)
// {

//   ros::init(argc, argv, "quad_sim_example");
//   ros::NodeHandle nh("~");

//   ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);

//   ros::Duration(2.0).sleep();

//   while (ros::ok())
//   {

//     /*** example 1: position control ***/
//     std::cout << "\033[42m"
//               << "Position Control to (2,0,1) meters"
//               << "\033[0m" << std::endl;
//     for (int i = 0; i < 500; i++)
//     {
//       quadrotor_msgs::PositionCommand cmd;
//       cmd.position.x = 2.0;
//       cmd.position.y = 0.0;
//       cmd.position.z = 1.0;
//       cmd_pub.publish(cmd);

//       ros::Duration(0.01).sleep();
//       ros::spinOnce();
//     }

//     /*** example 2: velocity control ***/
//     std::cout << "\033[42m"
//               << "Velocity Control to (-1,0,0) meters/second"
//               << "\033[0m" << std::endl;
//     for (int i = 0; i < 500; i++)
//     {
//       quadrotor_msgs::PositionCommand cmd;
//       cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
//       cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
//       cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
//       cmd.velocity.x = -1.0;
//       cmd.velocity.y = 0.0;
//       cmd.velocity.z = 0.0;
//       cmd_pub.publish(cmd);

//       ros::Duration(0.01).sleep();
//       ros::spinOnce();
//     }

//     /*** example 3: accelleration control ***/
//     std::cout << "\033[42m"
//               << "Accelleration Control to (1,0,0) meters/second^2"
//               << "\033[0m" << std::endl;
//     for (int i = 0; i < 500; i++)
//     {
//       quadrotor_msgs::PositionCommand cmd;
//       cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
//       cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
//       cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
//       cmd.velocity.x = std::numeric_limits<float>::quiet_NaN();
//       cmd.velocity.y = std::numeric_limits<float>::quiet_NaN();
//       cmd.velocity.z = std::numeric_limits<float>::quiet_NaN();
//       cmd.acceleration.x = 1.0;
//       cmd.acceleration.y = 0.0;
//       cmd.acceleration.z = 0.0;
//       cmd_pub.publish(cmd);

//       ros::Duration(0.01).sleep();
//       ros::spinOnce();
//     }

//   }

//   return 0;
// }
