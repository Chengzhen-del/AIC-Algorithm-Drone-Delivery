#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>

#define VEL_P 1.0
#define VEL_I 0.0
#define VEL_D 0.0

double getLengthBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b,
                              double *out_err_x = nullptr, double *out_err_y = nullptr, double *out_err_z = nullptr) {
    double err_x = a.x - b.x;
    double err_y = a.y - b.y;
    double err_z = a.z - b.z;
    if (out_err_x != nullptr) *out_err_x = err_x;
    if (out_err_y != nullptr) *out_err_y = err_y;
    if (out_err_z != nullptr) *out_err_z = err_z;
    return sqrt(err_x * err_x + err_y * err_y + err_z * err_z);
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
geometry_msgs::Vector3 current_rpy;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose = *msg;
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg->pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(current_rpy.x, current_rpy.y, current_rpy.z);
}

// 环形中心位置
geometry_msgs::Point ring_center;
bool ring_center_received = false;
void ring_center_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ring_center = msg->pose.position;
    ring_center_received = true;
}

geometry_msgs::Point last_err;
geometry_msgs::Point err_sum;
double last_yaw_err = 0.;
double yaw_err_sum = 0.;
ros::Time last_pid_control_time;
geometry_msgs::Twist get_pid_vel(geometry_msgs::Point target) {
    ros::Time currentStamp = current_pose.header.stamp;
    ros::Duration dt = currentStamp - last_pid_control_time;
    if (dt.toSec() > 0.2) {
        err_sum.x = 0.;
        err_sum.y = 0.;
        err_sum.z = 0.;
        yaw_err_sum = 0.;
    }
    
    geometry_msgs::Point err;
    double absErr = getLengthBetweenPoints(target, current_pose.pose.position, &err.x, &err.y, &err.z);

    double y_err = 0. - current_rpy.z;
    double dy_err = (y_err - last_yaw_err) / dt.toSec();

    geometry_msgs::Twist ret;
    ret.angular.z = VEL_P * y_err + VEL_I * yaw_err_sum + VEL_D * dy_err;
    
    if (absErr > 0.8) {
        ret.linear.x = err.x * 0.8 / absErr;
        ret.linear.y = err.y * 0.8 / absErr;
        ret.linear.z = err.z * 0.8 / absErr;
        
        err_sum.x = .0;
        err_sum.y = .0;
        err_sum.z = .0;
    } else {
        geometry_msgs::Point d_err;
        d_err.x = (err.x - last_err.x) / dt.toSec();
        d_err.y = (err.y - last_err.y) / dt.toSec();
        d_err.z = (err.z - last_err.z) / dt.toSec();

        ret.linear.x = VEL_P * err.x + VEL_I * err_sum.x + VEL_D * d_err.x;
        ret.linear.y = VEL_P * err.y + VEL_I * err_sum.y + VEL_D * d_err.y;
        ret.linear.z = VEL_P * err.z + VEL_I * err_sum.z + VEL_D * d_err.z;

        err_sum.x += err.x * dt.toSec();
        err_sum.y += err.y * dt.toSec();
        err_sum.z += err.z * dt.toSec();
    }
    
    last_err = err;
    last_yaw_err = y_err;
    yaw_err_sum += y_err * dt.toSec();
    last_pid_control_time = currentStamp;
    return ret;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_control_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pose_cb);
    ros::Subscriber ring_center_sub = nh.subscribe<geometry_msgs::PoseStamped>("/ring_center", 1, ring_center_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 定义航点路径
    std::vector<geometry_msgs::Point> waypoints;
    geometry_msgs::Point waypoint;
    
    // 添加路径点 (0, 1.6, 1.0)
    waypoint.x = 0.0; waypoint.y = 1.6; waypoint.z = 1.0;
    waypoints.push_back(waypoint);
    
    // 添加路径点 (6, 1.6, 1.0)
    waypoint.x = 6.0; waypoint.y = 1.6; waypoint.z = 1.0;
    waypoints.push_back(waypoint);

    // 添加路径点 (6, 1.6, 1.0)
    waypoint.x = 6.0; waypoint.y = 0; waypoint.z = 1.0;
    waypoints.push_back(waypoint);

    // Wait for FCU connection
    ros::Rate rate(20.0);
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    int fsm_state = 0;
    int current_waypoint_index = 0;
    ros::Time last_srv_request = ros::Time::now();
    double waypoint_reach_threshold = 0.2; // 到达航点的距离阈值
    bool ring_navigation_started = false;
    
    // 添加航点等待状态变量
    bool waiting_at_waypoint = false;
    ros::Time waypoint_wait_start_time;
    double waypoint_wait_time = 1.0; // 在每个航点停留1秒
    
    while (ros::ok()) {
        geometry_msgs::TwistStamped twist;
        switch (fsm_state) {
            case 0:  // Before offboard state
                if (current_state.mode == "OFFBOARD") {
                    fsm_state = 1;  // goto before armed state
                } else {
                    if (ros::Time::now() - last_srv_request > ros::Duration(1.0)) {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                            ROS_INFO("Offboard enabled");
                        } else {
                            ROS_WARN("Failed to enable offboard");
                        }
                        last_srv_request = ros::Time::now();
                    }
                }
                break;
            case 1:  // After offboard, before armed state
                if (current_state.armed) {
                    fsm_state = 2;  // goto takeoff state
                } else {
                    if (ros::Time::now() - last_srv_request > ros::Duration(1.0)) {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                            ROS_INFO("Vehicle armed");
                        } else {
                            ROS_WARN("Failed to arm vehicle");
                        }
                        last_srv_request = ros::Time::now();
                    }
                }
                break;
            case 2:  // Takeoff state
                if (current_pose.pose.position.z > 0.8) {
                    fsm_state = 3;  // goto waypoint navigation state
                    last_srv_request = ros::Time::now();
                    current_waypoint_index = 0;
                    ROS_INFO("Starting waypoint navigation. Moving to waypoint 1: (%.1f, %.1f, %.1f)", 
                             waypoints[current_waypoint_index].x, 
                             waypoints[current_waypoint_index].y, 
                             waypoints[current_waypoint_index].z);
                } else {
                    twist.twist.linear.z = 0.4;
                }
                break;
            case 3:  // Waypoint navigation state
                // 检查是否到达当前航点
                if (!waiting_at_waypoint && getLengthBetweenPoints(waypoints[current_waypoint_index], current_pose.pose.position) < waypoint_reach_threshold) {
                    // 已到达当前航点，开始等待
                    waiting_at_waypoint = true;
                    waypoint_wait_start_time = ros::Time::now();
                    ROS_INFO("Reached waypoint %d. Waiting for %.1f seconds...", 
                             current_waypoint_index + 1, waypoint_wait_time);
                }
                
                if (waiting_at_waypoint) {
                    // 正在航点处等待
                    if (ros::Time::now() - waypoint_wait_start_time > ros::Duration(waypoint_wait_time)) {
                        // 等待时间结束，前进到下一个航点
                        waiting_at_waypoint = false;
                        current_waypoint_index++;
                        
                        if (current_waypoint_index < waypoints.size()) {
                            ROS_INFO("Moving to waypoint %d: (%.1f, %.1f, %.1f)", 
                                     current_waypoint_index + 1,
                                     waypoints[current_waypoint_index].x,
                                     waypoints[current_waypoint_index].y,
                                     waypoints[current_waypoint_index].z);
                        } else {
                            // 所有预设航点都已完成，检查环形中心点并生成新航点
                            if (ring_center_received && !ring_navigation_started) {
                                // 清空现有航点列表
                                waypoints.clear();
                                
                                // 添加新的航点 (center.x, center.y+0.5, center.z)
                                waypoint.x = ring_center.x;
                                waypoint.y = ring_center.y + 0.5;
                                waypoint.z = ring_center.z;
                                waypoints.push_back(waypoint);
                                
                                // 添加新的航点 (center.x, center.y-1, center.z)
                                waypoint.x = ring_center.x;
                                waypoint.y = ring_center.y - 0.5;
                                waypoint.z = ring_center.z;
                                waypoints.push_back(waypoint);
                                
                                // 重置航点索引
                                current_waypoint_index = 0;
                                ring_navigation_started = true;
                                
                                ROS_INFO("All predefined waypoints reached. Ring center detected at (%.1f, %.1f, %.1f)",
                                         ring_center.x, ring_center.y, ring_center.z);
                                ROS_INFO("Moving to ring position 1: (%.1f, %.1f, %.1f)",
                                         waypoints[current_waypoint_index].x,
                                         waypoints[current_waypoint_index].y,
                                         waypoints[current_waypoint_index].z);
                            } else if (ring_navigation_started) {
                                // 环形导航已完成，准备降落
                                fsm_state = 4;
                                ROS_INFO("Ring navigation completed. Preparing to land.");
                            } else {
                                // 未收到环形中心点，等待接收
                                ROS_WARN("Waiting for ring center position...");
                                // 原地悬停
                                waypoint = current_pose.pose.position;
                                twist.twist = get_pid_vel(waypoint);
                            }
                        }
                    } else {
                        // 等待未结束，原地悬停
                        twist.twist = get_pid_vel(current_pose.pose.position);
                    }
                } else {
                    // 还没到达当前航点，继续使用PID控制移动
                    twist.twist = get_pid_vel(waypoints[current_waypoint_index]);
                }
                break;
            case 4:  // Land state
                if (current_state.mode == "AUTO.LAND") {
                    fsm_state = -1;  // goto do nothing state
                } else if (current_pose.pose.position.z < 0.1) {
                    if (ros::Time::now() - last_srv_request > ros::Duration(0.5)) {
                        mavros_msgs::SetMode land_set_mode;
                        land_set_mode.request.custom_mode = "AUTO.LAND";
                        set_mode_client.call(land_set_mode);
                        last_srv_request = ros::Time::now();
                    }
                } else {
                    twist.twist.linear.z = -0.2;
                }
                break;
            default:
                fsm_state = -1;
                break;
        }
        vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
