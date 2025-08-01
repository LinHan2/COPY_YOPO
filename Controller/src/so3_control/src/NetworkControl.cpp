#include "so3_control/NetworkControl.h"

/**
 * @brief 初始化日志记录器
 * 
 * 该函数创建一个新的CSV日志文件用于记录飞行数据。
 * 文件名格式: {logger_file_name}{序号}_Net_logger_{时间戳}.csv
 * 通过检查现有文件的序号来确定新文件的序号，避免文件名冲突。
 */
void NetworkControl::initLogRecorder()
{   
    // 使用文件计数作为名称来避免日期混乱
    std::cout << "logger_file_name: " << logger_file_name << std::endl;
    int max_number = -1;
    boost::filesystem::path dir(logger_file_name);
    
    // 检查日志目录是否存在
    if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir)) {
        std::regex filename_pattern(R"(^(\d+)_.*$)"); // 匹配以数字开头，并以"_"分隔的文件名
        // 遍历目录中的所有文件，找到最大序号
        for (const auto& entry : boost::filesystem::directory_iterator(dir)) {
            if (boost::filesystem::is_regular_file(entry)) {
                std::string filename = entry.path().filename().string();
                std::smatch match;
                if (std::regex_match(filename, match, filename_pattern)) {
                    int number = std::stoi(match[1]); // 提取第一个"_"前的数字
                    max_number = std::max(max_number, number);
                }
            }
        }
    } else {
        std::cerr << "Error: invalid logger path" << std::endl;
    }
    
    // 构建新的日志文件名
    std::string fileCountStr = std::to_string(max_number + 1);
    std::string temp_file_name = logger_file_name + fileCountStr + "_Net_logger_";
    time_t timep;
    timep = time(0);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y_%m_%d_%H_%M_%S", localtime(&timep));
    temp_file_name += tmp;
    temp_file_name += ".csv";
    
    // 关闭之前打开的日志文件
    if (logger.is_open())
    {
        logger.close();
    }
    
    // 打开新的日志文件
    logger.open(temp_file_name.c_str(), std::ios::out);
    std::cout << "logger: " << temp_file_name << std::endl;
    if (!logger.is_open())
    {
        std::cout << "cannot open the logger." << std::endl;
    }
    else
    {
        // 写入CSV文件的表头
        logger << "timestamp" << ',';         // 时间戳
        logger << "cur_px" << ',';           // 当前位置x
        logger << "cur_py" << ',';           // 当前位置y
        logger << "cur_pz" << ',';           // 当前位置z
        logger << "cur_vx" << ',';           // 当前速度x
        logger << "cur_vy" << ',';           // 当前速度y
        logger << "cur_vz" << ',';           // 当前速度z
        logger << "cur_ax" << ',';           // 当前加速度x
        logger << "cur_ay" << ',';           // 当前加速度y
        logger << "cur_az" << ',';           // 当前加速度z
        logger << "des_px" << ',';           // 期望位置x
        logger << "des_py" << ',';           // 期望位置y
        logger << "des_pz" << ',';           // 期望位置z
        logger << "des_vx" << ',';           // 期望速度x
        logger << "des_vy" << ',';           // 期望速度y
        logger << "des_vz" << ',';           // 期望速度z
        logger << "des_ax" << ',';           // 期望加速度x
        logger << "des_ay" << ',';           // 期望加速度y
        logger << "des_az" << ',';           // 期望加速度z
        logger << "dis_ax" << ',';           // 扰动观测器估计的扰动加速度x
        logger << "dis_ay" << ',';           // 扰动观测器估计的扰动加速度y
        logger << "dis_az" << ',';           // 扰动观测器估计的扰动加速度z
        logger << "px4_ax" << ',';           // 发送给PX4的实际加速度x（期望加速度-扰动加速度）
        logger << "px4_ay" << ',';           // 发送给PX4的实际加速度y
        logger << "px4_az" << ',';           // 发送给PX4的实际加速度z
        logger << "thrust" << ',';           // 推力值
        logger << "cur_yaw" << ',';          // 当前偏航角
        logger << "des_yaw" << std::endl;    // 期望偏航角
    }
}

/**
 * @brief 记录飞行日志数据
 * 
 * 将当前的飞行状态、期望状态、扰动信息等记录到CSV文件中，用于后续的数据分析。
 * 
 * @param cur_v 当前速度向量
 * @param cur_a 当前加速度向量
 * @param des_a 期望加速度向量
 * @param dis_a 扰动观测器估计的扰动加速度向量
 * @param cur_yaw 当前偏航角
 * @param des_yaw 期望偏航角
 */
void NetworkControl::recordLog(Eigen::Vector3d &cur_v, Eigen::Vector3d &cur_a, Eigen::Vector3d &des_a, Eigen::Vector3d &dis_a, double cur_yaw, double des_yaw)
{
    if (logger.is_open())
    {
        logger << ros::Time::now().toNSec() << ',';     // 当前时间戳（纳秒）
        logger << cur_pos_(0) << ',';                   // 当前位置x
        logger << cur_pos_(1) << ',';                   // 当前位置y
        logger << cur_pos_(2) << ',';                   // 当前位置z
        logger << cur_v(0) << ',';                      // 当前速度x
        logger << cur_v(1) << ',';                      // 当前速度y
        logger << cur_v(2) << ',';                      // 当前速度z
        logger << cur_a(0) << ',';                      // 当前加速度x
        logger << cur_a(1) << ',';                      // 当前加速度y
        logger << cur_a(2) << ',';                      // 当前加速度z
        logger << des_pos_(0) << ',';                   // 期望位置x
        logger << des_pos_(1) << ',';                   // 期望位置y
        logger << des_pos_(2) << ',';                   // 期望位置z
        logger << des_vel_(0) << ',';                   // 期望速度x
        logger << des_vel_(1) << ',';                   // 期望速度y
        logger << des_vel_(2) << ',';                   // 期望速度z
        logger << des_a(0) << ',';                      // 期望加速度x
        logger << des_a(1) << ',';                      // 期望加速度y
        logger << des_a(2) << ',';                      // 期望加速度z
        logger << dis_a(0) << ',';                      // 扰动加速度x
        logger << dis_a(1) << ',';                      // 扰动加速度y
        logger << dis_a(2) << ',';                      // 扰动加速度z
        logger << des_a(0) - dis_a(0) << ',';           // 发送给PX4的实际加速度x
        logger << des_a(1) - dis_a(1) << ',';           // 发送给PX4的实际加速度y
        logger << des_a(2) - dis_a(2) << ',';           // 发送给PX4的实际加速度z
        logger << last_thrust_ << ',';                  // 最后的推力值
        logger << cur_yaw << ',';                       // 当前偏航角
        logger << des_yaw << std::endl;                 // 期望偏航角
    }
}

/**
 * @brief 发布悬停SO3控制指令
 * 
 * 该函数实现了基于位置、速度、加速度的四旋翼控制。
 * 使用SO3控制器计算所需的力和姿态，然后发布SO3命令并通过MAVROS发送给飞控。
 * 
 * @param des_pos 期望位置向量
 * @param des_vel 期望速度向量  
 * @param des_acc 期望加速度向量
 * @param des_yaw 期望偏航角
 * @param des_yaw_dot 期望偏航角速度
 * @return Eigen::Vector3d 计算得到的姿态加速度（用于扰动观测器）
 */
Eigen::Vector3d NetworkControl::publishHoverSO3Command(Eigen::Vector3d des_pos, Eigen::Vector3d des_vel, 
                                                       Eigen::Vector3d des_acc, double des_yaw, double des_yaw_dot)
{
    // 设置位置和速度的PD控制增益
    Eigen::Vector3d kx(kx_xy, kx_xy, kx_z);    // 位置增益 [kx_xy, kx_xy, kx_z]
    Eigen::Vector3d kv(kv_xy, kv_xy, kv_z);    // 速度增益 [kv_xy, kv_xy, kv_z]
    
    // 使用SO3控制器计算控制量
    so3_controller_.calculateControl(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot, kx, kv);

    // 获取计算结果
    Eigen::Vector3d force = so3_controller_.getComputedForce();          // 所需力向量
    Eigen::Quaterniond orientation = so3_controller_.getComputedOrientation();  // 所需姿态四元数

    // 构建SO3控制命令消息
    quadrotor_msgs::SO3Command::Ptr so3_command(new quadrotor_msgs::SO3Command); //! @note memory leak?
    so3_command->header.stamp = ros::Time::now();
    // 设置力向量
    so3_command->force.x = force(0);
    so3_command->force.y = force(1);
    so3_command->force.z = force(2);
    // 设置姿态四元数
    so3_command->orientation.x = orientation.x();
    so3_command->orientation.y = orientation.y();
    so3_command->orientation.z = orientation.z();
    so3_command->orientation.w = orientation.w();
    // 设置姿态控制增益
    so3_command->kR[0] = 1.5;   // Roll增益
    so3_command->kR[1] = 1.5;   // Pitch增益
    so3_command->kR[2] = 1.0;   // Yaw增益
    // 设置角速度控制增益
    so3_command->kOm[0] = 0.13; // Roll角速度增益
    so3_command->kOm[1] = 0.13; // Pitch角速度增益
    so3_command->kOm[2] = 0.1;  // Yaw角速度增益
    so3_command->aux.current_yaw = cur_yaw_;      // 当前偏航角
    so3_command->aux.enable_motors = true;       // 使能电机
    
    // 发布SO3控制命令
    so3_command_pub_.publish(so3_command);

    // 计算归一化推力并通过MAVROS发送给PX4
    double thrust_norm = force.norm() / (mass_ * ONE_G) * hover_thrust_;
    mavros_interface_.pub_att_thrust_cmd(orientation, thrust_norm);
    last_thrust_ = thrust_norm;

    // 计算姿态产生的实际加速度（用于扰动观测器）
    double thrust = force.norm() / mass_;
    Eigen::Matrix3d Cbn;    
    get_dcm_from_q(Cbn, orientation);  // 从四元数获取旋转矩阵
    Eigen::Vector3d att_acc = Eigen::Vector3d(0, 0, thrust);  // 机体系下的加速度
    att_acc = Cbn * att_acc;  // 转换到世界坐标系
    att_acc(2) -= ONE_G;      // 减去重力加速度
    // std::cout<<"att_acc"<<att_acc.transpose()<<std::endl;
    return att_acc;
}

/**
 * @brief 从期望加速度计算四元数和力向量
 * 
 * 根据期望的加速度和偏航角，计算出四旋翼需要的姿态四元数和推力向量。
 * 该函数实现了从期望加速度到姿态的几何解算，包括倾斜角度限制。
 * 
 * @param ref_acc 期望加速度向量（世界坐标系，不包含重力）
 * @param ref_yaw 期望偏航角
 * @param quat_des 输出：计算得到的期望姿态四元数
 * @param force_des 输出：计算得到的期望力向量
 */
void NetworkControl::get_Q_from_ACC(const Eigen::Vector3d &ref_acc, double ref_yaw, Eigen::Quaterniond &quat_des, Eigen::Vector3d &force_des)
{
    // 计算总力向量（期望加速度 + 重力补偿）
    Eigen::Vector3d force_ = mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    force_.noalias() += mass_ * ref_acc;

    // 限制控制角度到theta度以内（安全限制）
    double theta = M_PI / 4;  // 45度
    double c = cos(theta);
    Eigen::Vector3d f;
    f.noalias() = force_ - mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    
    // 检查是否超出倾斜角度限制
    if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c)
    {
        // 如果超出限制，重新计算力向量使其在限制范围内
        double nf = f.norm();
        double A = c * c * nf * nf - f(2) * f(2);
        double B = 2 * (c * c - 1) * f(2) * mass_ * ONE_G;
        double C = (c * c - 1) * mass_ * mass_ * ONE_G * ONE_G;
        double s = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
        force_.noalias() = s * f + mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    }

    // 计算机体坐标系的三个轴
    Eigen::Vector3d b1c, b2c, b3c;
    Eigen::Vector3d b1d(cos(ref_yaw), sin(ref_yaw), 0);  // 期望的机头方向

    // b3轴（推力方向）= 力向量的单位向量
    if (force_.norm() > 1e-6)
        b3c.noalias() = force_.normalized();
    else
        b3c.noalias() = Eigen::Vector3d(0, 0, 1);

    // b2轴 = b3 × b1d（右手法则）
    b2c.noalias() = b3c.cross(b1d).normalized();
    // b1轴 = b2 × b3（右手法则）
    b1c.noalias() = b2c.cross(b3c).normalized();

    // 构建旋转矩阵并转换为四元数
    Eigen::Matrix3d R;
    R << b1c, b2c, b3c;

    quat_des = Eigen::Quaterniond(R);
    force_des = force_;
}

/**
 * @brief 发布SO3控制命令
 * 
 * 基于期望加速度和偏航角发布SO3控制命令。这是一个简化版本的控制函数，
 * 直接从加速度计算姿态，适用于纯加速度控制模式。
 * 
 * @param ref_acc 期望加速度向量（世界坐标系，已包含重力补偿）
 * @param ref_yaw 期望偏航角
 * @param cur_yaw 当前偏航角
 */
void NetworkControl::pub_SO3_command(Eigen::Vector3d ref_acc, double ref_yaw, double cur_yaw)
{
    Eigen::Vector3d force;
    Eigen::Quaterniond quat_des;
    
    // 从期望加速度计算期望姿态和力
    get_Q_from_ACC(ref_acc, ref_yaw, quat_des, force);
    
    // 构建SO3控制命令
    quadrotor_msgs::SO3Command::Ptr so3_command(new quadrotor_msgs::SO3Command);
    so3_command->header.stamp = ros::Time::now();
    
    // 设置力向量
    so3_command->force.x = force(0);
    so3_command->force.y = force(1);
    so3_command->force.z = force(2);
    
    // 设置姿态四元数
    so3_command->orientation.x = quat_des.x();
    so3_command->orientation.y = quat_des.y();
    so3_command->orientation.z = quat_des.z();
    so3_command->orientation.w = quat_des.w();
    
    // 设置控制增益
    so3_command->kR[0] = 1.5;   // Roll姿态增益
    so3_command->kR[1] = 1.5;   // Pitch姿态增益
    so3_command->kR[2] = 1.0;   // Yaw姿态增益
    so3_command->kOm[0] = 0.13; // Roll角速度增益
    so3_command->kOm[1] = 0.13; // Pitch角速度增益
    so3_command->kOm[2] = 0.1;  // Yaw角速度增益
    
    so3_command->aux.current_yaw = cur_yaw;     // 当前偏航角
    so3_command->aux.enable_motors = true;      // 使能电机
    
    // 发布SO3命令
    so3_command_pub_.publish(so3_command);

    // 计算并发送归一化推力给MAVROS
    double thrust_norm = force.norm() / (mass_ * ONE_G) * hover_thrust_;
    mavros_interface_.pub_att_thrust_cmd(quat_des, thrust_norm);
    last_thrust_ = thrust_norm;
}

void NetworkControl::limite_acc(Eigen::Vector3d &acc){
    return;  // limited if needed
    acc[0] = std::max(-8.0, std::min(acc[0], 8.0));
    acc[1] = std::max(-8.0, std::min(acc[1], 8.0));
    // 限制z方向加速度在±4.0 m/s²以内
    acc[2] = std::max(-4.0, std::min(acc[2], 4.0));
}

/**
 * @brief 网络控制命令回调函数
 * 
 * 这是NetworkControl的核心回调函数，处理来自上层规划器（如YOPO）的位置命令。
 * 根据命令的轨迹标志，选择不同的控制模式：
 * - TRAJECTORY_STATUS_READY: 纯加速度控制模式（适用于YOPO等路径规划器）
 * - 其他状态: 位置-速度-加速度综合控制模式
 * 
 * @param cmd 位置命令消息，包含期望的位置、速度、加速度和偏航信息
 */
void NetworkControl::network_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
    // 检查控制器是否有效
    if (!ctrl_valid_)
        return;

    // 检查飞行器状态（是否解锁且处于offboard模式）
    bool arm_state = false;
    bool ofb_enable = false;
    mavros_interface_.get_status(arm_state, ofb_enable);
    if (!arm_state || !ofb_enable)
        return;

    position_cmd_init_ = true;

    // 提取期望加速度并进行限制
    Eigen::Vector3d des_acc = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
    limite_acc(des_acc);

    double des_yaw = cmd->yaw;
    
    // 使用扰动观测器估计外部扰动
    disturbance_observer_.HGDO_ext_force_ob(last_des_acc_, cur_vel_, dis_acc_);
    ROS_INFO_THROTTLE(0.5, "dis_acc: %.3f, %.3f, %.3f", dis_acc_.x(), dis_acc_.y(), dis_acc_.z());
    // std::cout << "dis_acc: " << dis_acc_.transpose() << std::endl;

    Eigen::Vector3d att_acc;
    
    // 根据轨迹标志选择控制模式
    if (cmd->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY)
    {    
        // 纯加速度控制模式（YOPO等规划器使用）
        if (use_disturbance_observer_)
            att_acc = des_acc - dis_acc_;  // 减去估计的扰动
        else
            att_acc = des_acc;
            
        pub_SO3_command(att_acc, des_yaw, cur_yaw_);
        // std::cout<<"acc: "<<des_acc.transpose()<<"   yaw:"<<des_yaw<<std::endl;
        
        if (record_log_)
            recordLog(cur_vel_, cur_acc_, des_acc, dis_acc_, cur_yaw_, des_yaw);
    }
    else
    {
        // 位置-速度-加速度综合控制模式
        Eigen::Vector3d des_pos = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
        Eigen::Vector3d des_vel = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
        double des_yaw = cmd->yaw;
        double des_yaw_dot = cmd->yaw_dot;
        
        // 使用悬停控制器（包含位置和速度反馈）
        att_acc = publishHoverSO3Command(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot);
        
        if (record_log_)
            recordLog(cur_vel_, cur_acc_, att_acc, dis_acc_, cur_yaw_, des_yaw);
    }

    last_des_acc_ = att_acc;  // 保存加速度用于扰动观测器
}

/**
 * @brief 里程计数据回调函数
 * 
 * 处理来自里程计的位置和姿态数据，这是系统的状态反馈源。
 * 里程计数据可能来自：
 * - 仿真环境：/sim/odom（由仿真器提供的真实状态）
 * - 真实飞行：视觉里程计、激光SLAM或其他定位系统
 * 
 * @param odom 里程计消息，包含位置、姿态、速度等状态信息
 */
void NetworkControl::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    // 提取偏航角（从四元数转换）
    cur_yaw_ = tf::getYaw(odom->pose.pose.orientation);
    
    // 提取线速度
    cur_vel_ = Eigen::Vector3d(odom->twist.twist.linear.x, 
                               odom->twist.twist.linear.y, 
                               odom->twist.twist.linear.z);

    // 提取位置
    cur_pos_ = Eigen::Vector3d(odom->pose.pose.position.x, 
                               odom->pose.pose.position.y, 
                               odom->pose.pose.position.z);
    
    // 提取姿态四元数
    cur_att_.w() = odom->pose.pose.orientation.w;
    cur_att_.x() = odom->pose.pose.orientation.x;
    cur_att_.y() = odom->pose.pose.orientation.y;
    cur_att_.z() = odom->pose.pose.orientation.z;

    // 注意：在仿真中，某些系统可能通过angular部分传递加速度信息
    // if(!is_simulation_)
    //     cur_acc_ = Eigen::Vector3d(odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z);

    // 更新SO3控制器的状态
    so3_controller_.setPosition(cur_pos_);
    so3_controller_.setVelocity(cur_vel_);
    
    // 标记状态已初始化
    if (!state_init_)
        ROS_INFO("Odom Recived! Ready to TakeOff...");
    state_init_ = true;
}

/**
 * @brief IMU数据回调函数
 * 
 * 处理来自IMU的加速度数据，将机体坐标系的加速度转换到世界坐标系。
 * 这些加速度数据主要用于扰动观测器，帮助估计外部扰动。
 * 
 * @param imu IMU消息，包含线加速度和角速度数据
 */
void NetworkControl::imu_callback(const sensor_msgs::Imu &imu)
{
    // 提取机体坐标系下的加速度
    Eigen::Vector3d acc(imu.linear_acceleration.x,
                        imu.linear_acceleration.y,
                        imu.linear_acceleration.z);
    
    // 将加速度从机体坐标系转换到世界坐标系
    Eigen::Vector3d acc_world = cur_att_ * acc;
    
    // 减去重力加速度，得到真实的运动加速度
    acc_world(2) -= 9.8;
    cur_acc_ = acc_world;
    
    // 可选：更新SO3控制器的加速度状态
    // so3_controller_.setAcc(acc_world);
}

/**
 * @brief 定时器回调函数
 * 
 * 当没有外部控制命令时，该函数提供默认的悬停控制。
 * 主要用于以下场景：
 * - 起飞后等待外部控制命令
 * - 外部控制命令中断后的安全悬停
 * - 保持当前位置的悬停控制
 * 
 * @param 定时器事件（未使用）
 */
void NetworkControl::timerCallback(const ros::TimerEvent &)
{
    // 检查基本状态
    if (!state_init_ || !ref_valid_)
        return;
        
    // 如果有外部位置命令且控制有效，则不执行定时器控制
    if (position_cmd_init_ && ctrl_valid_)
        return;

    // 获取当前期望位置（线程安全）
    mutex_.lock();
    Eigen::Vector3d des_pos_temp = des_pos_;
    mutex_.unlock();

    // 执行悬停控制
    Eigen::Vector3d att_acc = publishHoverSO3Command(des_pos_temp, des_vel_, des_acc_, des_yaw_, des_yaw_dot_);

    // 如果起飞命令已初始化，更新扰动观测器
    if (takeoff_cmd_init_)
    {
        disturbance_observer_.HGDO_ext_force_ob(last_des_acc_, cur_vel_, dis_acc_);
        // ROS_INFO_THROTTLE(1.0, " dis_acc: (%f, %f, %f)", dis_acc_.x(), dis_acc_.y(), dis_acc_.z());
    }
    last_des_acc_ = att_acc;
    
    // 记录日志
    if (record_log_)
        recordLog(cur_vel_, cur_acc_, att_acc, dis_acc_, cur_yaw_, des_yaw_);
        
    takeoff_cmd_init_ = true;
}

/**
 * @brief 起飞/降落线程函数
 * 
 * 处理起飞和降落的完整流程，包括：
 * - 起飞：解锁飞行器→缓慢上升到指定高度→启用控制
 * - 降落：禁用控制→缓慢下降→检测着陆→上锁飞行器
 * 
 * @param req 起飞/降落请求，包含起飞高度和起飞/降落标志
 */
void NetworkControl::takeoff_land_thread(quadrotor_msgs::SetTakeoffLand::Request &req)
{
    // 初始化期望状态
    mutex_.lock();
    float takeoff_altitude = req.takeoff_altitude;
    des_pos_ = cur_pos_;
    des_pos_(2) -= 0.2;  // 起始位置稍微降低0.2m
    des_yaw_ = cur_yaw_;
    mutex_.unlock();
    ref_valid_ = true;

    if (req.takeoff)
    {
        // 起飞流程
        std::cout << "takeoff process start" << std::endl;
        
        // 1. 解锁飞行器
        if (!arm_disarm_vehicle(true))
        {
            std::cout << "Service failed because cannot Arm!" << std::endl;
            return;
        }
        sleep(1);

        double takeoff_vel = 0.8;
        double takeoff_ddz = takeoff_vel * control_dt_;
        ros::Rate takeoff_loop(1 / control_dt_);
        
        std::cout << "takeoff altitude: " << takeoff_altitude << " m" << std::endl;
        std::cout << "takeoff velocity: " << takeoff_vel << " m/s" << std::endl;
        
        ros::Time start_takeoff_task_time = ros::Time::now();
        
        // 起飞循环（最多8秒超时保护）
        while (ros::ok() && ros::Time::now() - start_takeoff_task_time < ros::Duration(8.0))
        {       
            mutex_.lock();
            des_pos_(2) += takeoff_ddz;  // 增加期望高度
            mutex_.unlock();

            // 检查是否达到目标高度
            if (des_pos_(2) > takeoff_altitude)
            {
                ROS_INFO("TakeOff Done! Ready to Flight...");
                ctrl_valid_ = true;  // 启用外部控制
                break;
            }
            takeoff_loop.sleep();
        }
    }
    else
    {
        // 降落流程
        ctrl_valid_ = false;  // 禁用外部控制
        
        double land_vel = -0.4;      // 降落速度 -0.4 m/s
        double land_ddz = land_vel * control_dt_;  // 每个控制周期的高度减量
        ros::Rate land_loop(1 / control_dt_);
        
        ros::Time start_land_task_time = ros::Time::now();
        
        // 降落循环（最多8秒超时保护）
        while (ros::ok() && ros::Time::now() - start_land_task_time < ros::Duration(8.0))
        {
            mutex_.lock();
            des_pos_(2) += land_ddz;  // 减少期望高度
            mutex_.unlock();

            // 检测是否着陆（高度<0.1m且垂直速度<1.0m/s）
            if (fabs(cur_pos_(2)) < 0.1f && fabs(cur_vel_(2)) < 1.0f)
            {
                ROS_INFO("detect land: disarm");
                arm_disarm_vehicle(false);  // 上锁飞行器
                break;
            }
            land_loop.sleep();
        }
    }
    ROS_INFO("take off thread out");
    return;
}

/**
 * @brief 飞行器解锁/上锁函数
 * 
 * 控制飞行器的解锁（ARM）和上锁（DISARM）状态，同时管理日志记录。
 * 区分仿真和真实飞行环境，使用不同的MAVROS接口。
 * 
 * @param arm true=解锁飞行器，false=上锁飞行器
 * @return bool 操作是否成功
 */
bool NetworkControl::arm_disarm_vehicle(bool arm)
{
    if (arm)
    {   
        // 解锁前检查状态是否已初始化
        if (!state_init_){
            ROS_WARN("State timeout, will not arm!");
            return false;
        }

        ROS_INFO("UAV will be armed!");
        
        // 根据仿真/真实环境选择不同的解锁方式
        if (is_simulation_)
            mavros_interface_.set_arm_and_offboard_manually();  // 仿真环境：手动设置
        else if (mavros_interface_.set_arm_and_offboard())      // 真实飞行：通过MAVROS服务
            ROS_INFO("Arm done!");
        else{
            ROS_ERROR("Arm failure!");
            return false;
        }
        
        // 解锁成功后初始化日志记录
        if (record_log_)
            initLogRecorder();
    }
    else
    {
        ROS_INFO("UAV will be disarmed!");
        
        // 根据仿真/真实环境选择不同的上锁方式
        if (is_simulation_)
            mavros_interface_.set_disarm_manually();    // 仿真环境：手动设置
        else if (mavros_interface_.set_disarm())        // 真实飞行：通过MAVROS服务
            ROS_INFO("Disarm done!");
        else {
            ROS_ERROR("Disarm failure!");
            return false;
        }
        
        // 上锁后关闭日志记录
        if (record_log_)
            logger.close();
    }
    return true;
}