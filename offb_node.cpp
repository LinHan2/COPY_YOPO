/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped curr_pos;   // 无人机当前位置

// 获取无人机当前状态
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// 获取无人机当前位置
void arrive_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    // 订阅无人机的位置话题
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose",10,arrive_pos);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;  // 目标位置
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int count = 0;  // 计时
    bool flag = true;
    while(ros::ok()){
    	if( flag ) { 
	        if( current_state.mode != "OFFBOARD" &&
	            (ros::Time::now() - last_request > ros::Duration(5.0))){
	            if( set_mode_client.call(offb_set_mode) &&
	                offb_set_mode.response.mode_sent){
	                ROS_INFO("Offboard enabled");
	            }
	            last_request = ros::Time::now();
	        } else {
	            if( !current_state.armed &&
	                (ros::Time::now() - last_request > ros::Duration(5.0))){
	                if( arming_client.call(arm_cmd) &&
	                    arm_cmd.response.success){
	                    ROS_INFO("Vehicle armed");
	                }   // arm
	                last_request = ros::Time::now();
	            }
	        } // else
		} // flag

		// 当检测到与目标高度相差 ± 0.3 就开始悬停 30s
        if(fabs(curr_pos.pose.position.z - pose.pose.position.z) <= 0.3){
            count++;
            if(count >= 600)  // 30s
            {
                mavros_msgs::SetMode land_set_mode;
                land_set_mode.request.custom_mode = "AUTO.LAND";  // 发送降落命令
                if(set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
                    flag = false;
                }
            }  
        }
	  //任务结束,无人机降落完成并关闭该节点
     if ( !flag && current_state.mode == "AUTO.LAND" && current_state.armed == false ) {
          ROS_INFO("Drone has landed");
          ros::shutdown();
      }
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

