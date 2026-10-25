#include <vector>
#include <unordered_set>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <actionlib_msgs/GoalID.h>

#include <tutorial_vision_feng/CircleDetectResult.h>
#include <tutorial_vision_feng/StringStamped.h>
#include "std_srvs/Empty.h"

#define VEL_P 1.0
#define VEL_I 0.0
#define VEL_D 0.0
#define YAW_VEL_P 1.0
#define YAW_VEL_I 0.0
#define YAW_VEL_D 0.0
#define PIX_VEL_P 0.001
#define PIX_VEL_I 0.0
#define PIX_VEL_D 0.0

// 圆周运动参数
#define CIRCLE_RADIUS 1.1
#define CIRCLE_ANGULAR_SPEED 0.5
#define CIRCLE_KP_DIST 5.0
#define CIRCLE_KI_DIST 0.0
#define CIRCLE_KD_DIST 2.0

// 用于执行圆周运动所需的变量
double circle_pole_x = 4.3;  // 圆心X坐标
double circle_pole_y = 0.0;  // 圆心Y坐标
double circle_error_dist_sum = 0.0;
double circle_last_error_dist = 0.0;
ros::Time circle_last_time;
double circle_current_angle = 0.0;  // 当前角度
ros::Time circle_start_time;        // 开始圆周运动的时间
bool circle_move_to_start = true;   // 是否需要先移动到起始点

// 圆周速度控制参数
double circle_initial_speed = 0.1;      // 初始角速度（缓慢开始）
double circle_current_speed = 0.0;      // 当前角速度
double circle_acceleration = 0.05;      // 加速度 (rad/s²)
int circle_state = 0;                   // 圆周运动子状态: 0-移动到起始点, 1-慢速开始, 2-加速, 3-正常速度
double circle_total_angle = 0.0;        // 累计旋转角度
double circle_last_angle = 0.0;         // 上次的角度
double circle_laps;                     // 目标圈数
bool circle_started_counting = false;   // 是否开始累计角度

double getLengthBetweenPoints(geometry_msgs::Point a, double x, double y, double z,
                              double *out_err_x = nullptr, double *out_err_y = nullptr, double *out_err_z = nullptr) {
    double err_x = a.x - x;
    double err_y = a.y - y;
    double err_z = a.z - z;
    if (out_err_x != nullptr) *out_err_x = err_x;
    if (out_err_y != nullptr) *out_err_y = err_y;
    if (out_err_z != nullptr) *out_err_z = err_z;
    return sqrt(err_x * err_x + err_y * err_y + err_z * err_z);
}

double getLengthBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b,
                              double *out_err_x = nullptr, double *out_err_y = nullptr, double *out_err_z = nullptr) {
    return getLengthBetweenPoints(a, b.x, b.y, b.z, out_err_x, out_err_y, out_err_z);
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

geometry_msgs::Twist move_base_twist;
void move_base_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg) {
    move_base_twist = *msg;
}

tutorial_vision_feng::CircleDetectResult parking_detect_result;
void parking_cb(const tutorial_vision_feng::CircleDetectResult::ConstPtr &msg) {
    parking_detect_result = *msg;
    if (msg->header.stamp == ros::Time(0)) parking_detect_result.header.stamp = ros::Time::now();
}

tutorial_vision_feng::CircleDetectResult deliver_detect_result;
void deliver_cb(const tutorial_vision_feng::CircleDetectResult::ConstPtr &msg) {
    deliver_detect_result = *msg;
    if (msg->header.stamp == ros::Time(0)) deliver_detect_result.header.stamp = ros::Time::now();
}

tutorial_vision_feng::StringStamped qr_result;
void qr_cb(const tutorial_vision_feng::StringStamped::ConstPtr &msg) {
    qr_result = *msg;
    if (msg->header.stamp == ros::Time(0)) qr_result.header.stamp = ros::Time::now();
}

tutorial_vision_feng::StringStamped sign_result;
void yolo_cb(const tutorial_vision_feng::StringStamped::ConstPtr &msg) {
    sign_result = *msg;
    if (msg->header.stamp == ros::Time(0)) sign_result.header.stamp = ros::Time::now();
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
    ret.angular.z = YAW_VEL_P * y_err + YAW_VEL_I * yaw_err_sum + YAW_VEL_D * dy_err;

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

geometry_msgs::Point pix_last_err;
geometry_msgs::Point pix_err_sum;
geometry_msgs::Twist get_pix_pid_vel(geometry_msgs::Point err) {
    ros::Time currentStamp = current_pose.header.stamp;
    ros::Duration dt = currentStamp - last_pid_control_time;
    if (dt.toSec() > 0.2) {
        pix_err_sum.x = 0.;
        pix_err_sum.y = 0.;
        yaw_err_sum = 0.;
    }

    double y_err = 0. - current_rpy.z;
    double dy_err = (y_err - last_yaw_err) / dt.toSec();

    geometry_msgs::Twist ret;
    ret.angular.z = YAW_VEL_P * y_err + YAW_VEL_I * yaw_err_sum + YAW_VEL_D * dy_err;

    geometry_msgs::Point d_err;
    d_err.x = (err.x - pix_last_err.x) / dt.toSec();
    d_err.y = (err.y - pix_last_err.y) / dt.toSec();

    ret.linear.x = PIX_VEL_P * err.y + PIX_VEL_I * pix_err_sum.y + PIX_VEL_D * d_err.y;
    ret.linear.y = PIX_VEL_P * err.x + PIX_VEL_I * pix_err_sum.x + PIX_VEL_D * d_err.x;

    pix_err_sum.x += err.x * dt.toSec();
    pix_err_sum.y += err.y * dt.toSec();

    pix_last_err = err;
    last_yaw_err = y_err;
    yaw_err_sum += y_err * dt.toSec();
    last_pid_control_time = currentStamp;
    
    return ret;
}

void analyseQrMessage(const std::string &str, std::unordered_set<std::string> &post_target, std::string &land_target) {
    std::vector<std::string> split;
    boost::split(split, str, boost::is_any_of(","), boost::token_compress_on);
    post_target.insert(split[0]);
    post_target.insert(split[1]);
    land_target = split[2];
}

bool begin_ring_center_calculation = false;
geometry_msgs::Point ring_center;
std::vector<geometry_msgs::Point> ring_center_buffer;
bool ring_center_received = false;
void ring_center_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if(begin_ring_center_calculation) {
        ring_center_buffer.push_back(msg->pose.position);
        // ROS_INFO("Ring Center Buffer Size: %zu", ring_center_buffer.size());
        if(ring_center_buffer.size() > 20) {
            // 计算环形中心点的平均位置
            geometry_msgs::Point avg_center;
            avg_center.x = 0;
            avg_center.y = 0; 
            avg_center.z = 0;

            for(const auto& point : ring_center_buffer) {
                avg_center.x += point.x;
                avg_center.y += point.y;
                avg_center.z += point.z;
            }

            avg_center.x /= ring_center_buffer.size();
            avg_center.y /= ring_center_buffer.size();
            avg_center.z /= ring_center_buffer.size();

            ring_center = avg_center;
            ring_center_received = true;
            ring_center_buffer.clear();
            begin_ring_center_calculation = false;
        }
    }
}

// 圆周运动控制函数，添加速度参数
geometry_msgs::Twist get_circle_vel(double working_altitude, double speed) {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - circle_last_time).toSec();
    if (dt <= 0) dt = 0.01; // 防止除零错误
    circle_last_time = current_time;
    
    // 计算当前距离圆心的平面距离
    double dx = current_pose.pose.position.x - circle_pole_x;
    double dy = current_pose.pose.position.y - circle_pole_y;
    double current_dist = sqrt(dx*dx + dy*dy);
    
    // 防止除零错误
    if (current_dist < 0.01) current_dist = 0.01;
    
    // 计算当前角度（相对于圆心）
    double current_angle = atan2(dy, dx);
    
    // 计算角度增量并累计总角度（考虑角度跳变）
    if (circle_started_counting) {
        double angle_diff = current_angle - circle_last_angle;
        
        // 处理角度跳变（从+π到-π或从-π到+π）
        if (angle_diff > M_PI) angle_diff -= 2*M_PI;
        if (angle_diff < -M_PI) angle_diff += 2*M_PI;
        
        // 由于是逆时针旋转，角度应该是减小的
        circle_total_angle -= angle_diff;
    } else {
        circle_started_counting = true;
    }
    
    // 记录当前角度，用于下次计算
    circle_last_angle = current_angle;
    
    // 计算已完成的圈数
    double completed_laps = circle_total_angle / (2*M_PI);
    
    // PID控制器 - 距离控制
    double error_dist = CIRCLE_RADIUS - current_dist;
    circle_error_dist_sum += error_dist * dt;
    // 限制积分项以防止积分饱和
    circle_error_dist_sum = std::max(-1.0, std::min(circle_error_dist_sum, 1.0));
    double error_dist_diff = (error_dist - circle_last_error_dist) / dt;
    circle_last_error_dist = error_dist;
    
    //ROS_INFO("Circle: Radius error: %.3f m (current: %.3f m, desired: %.3f m)", error_dist, current_dist, CIRCLE_RADIUS);
    //ROS_INFO("Completed laps: %.2f / %.2f", completed_laps, circle_laps);
    
    // 径向速度（向内/向外控制半径）
    double radial_vel = CIRCLE_KP_DIST * error_dist + CIRCLE_KI_DIST * circle_error_dist_sum + CIRCLE_KD_DIST * error_dist_diff;
    
    // 高度控制 - 简单P控制
    double z_vel = std::max(-0.5, std::min(0.5, working_altitude - current_pose.pose.position.z));
    
    // 计算圆周运动的切向速度，使用传入的速度参数
    double tangential_vel = speed * current_dist;
    
    // 计算无人机到圆心的方向向量（归一化）
    double direction_x = dx / current_dist;
    double direction_y = dy / current_dist;
    
    // 计算切向方向（垂直于径向方向）
    double tangent_x = direction_y;
    double tangent_y = -direction_x;
    
    // 创建速度命令
    geometry_msgs::Twist vel_cmd;
    
    // 径向速度分量
    vel_cmd.linear.x = direction_x * radial_vel;
    vel_cmd.linear.y = direction_y * radial_vel;
    
    // 切向速度分量
    vel_cmd.linear.x += tangent_x * tangential_vel;
    vel_cmd.linear.y += tangent_y * tangential_vel;
    
    // 高度控制
    vel_cmd.linear.z = z_vel;
    
    return vel_cmd;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "2025craic_task_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pose_cb);
    ros::Subscriber move_base_cmd_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, move_base_cmd_vel_cb);
    ros::Subscriber parking_sub = nh.subscribe<tutorial_vision_feng::CircleDetectResult>("parking_detect_result", 1, parking_cb);
    ros::Subscriber deliver_sub = nh.subscribe<tutorial_vision_feng::CircleDetectResult>("deliver_detect_result", 1, deliver_cb);
    ros::Subscriber qr_sub = nh.subscribe<tutorial_vision_feng::StringStamped>("qr_detect_result", 1, qr_cb);
    ros::Subscriber yolo_sub = nh.subscribe<tutorial_vision_feng::StringStamped>("yolo_detect", 1, yolo_cb);
    ros::Subscriber ring_center_sub = nh.subscribe<geometry_msgs::PoseStamped>("ring_center", 1, ring_center_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);
    // ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    // ros::Publisher cancel_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    std::vector<ros::Publisher> catapult_pubs;  // 将在main函数中初始化
    bool front_left_available = true;
    bool front_right_available = true;
    ros::Publisher front_left_open_pub = nh.advertise<std_msgs::Empty>("servo/front_left/open", 1);
    ros::Publisher front_right_open_pub = nh.advertise<std_msgs::Empty>("servo/front_right/open", 1);
    ros::Publisher back_left_open_pub = nh.advertise<std_msgs::Empty>("servo/back_left/open", 1);
    ros::Publisher back_right_open_pub = nh.advertise<std_msgs::Empty>("servo/back_right/open", 1);
    ros::Publisher all_servo_close_pub = nh.advertise<std_msgs::Empty>("servo/all/close", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient toggle_yolo_detect_client = nh.serviceClient<std_srvs::Empty>("toggle_yolo_detect");
    bool yolo_detecting = false;
    std_srvs::Empty empty;
    
    ros::NodeHandle param_nh("~");
    double drop_altitude = param_nh.param("drop_altitude", 0.4);
    double working_altitude = param_nh.param("working_altitude", 1.2);
    circle_laps = param_nh.param("circle_laps", 1.0);

    // Wait for FCU connection
    ros::Rate rate(20.0);
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    
    geometry_msgs::Point qr_position;
    qr_position.x = 1.8;
    qr_position.y = 0.0;
    qr_position.z = working_altitude;
    geometry_msgs::Point deliver_position[4];
    deliver_position[0].x = 1.8;
    deliver_position[0].y = -1.6;
    deliver_position[0].z = working_altitude;
    deliver_position[1].x = 3.6;
    deliver_position[1].y = -1.6;
    deliver_position[1].z = working_altitude;
    deliver_position[2].x = 1.8;
    deliver_position[2].y = 1.6;
    deliver_position[2].z = working_altitude;
    deliver_position[3].x = 3.6;
    deliver_position[3].y = 1.6;
    deliver_position[3].z = working_altitude;
    geometry_msgs::Point special_deliver_position;
    special_deliver_position.x = 6.0;
    special_deliver_position.y = 1.0;
    special_deliver_position.z = working_altitude;
    geometry_msgs::Point left_land_position, right_land_position, current_land_position;
    left_land_position.x = 0.0;
    left_land_position.y = 1.6;
    left_land_position.z = working_altitude;
    right_land_position.x = 0.0;
    right_land_position.y = -1.6;
    right_land_position.z = working_altitude;
    
    int fsm_state = 0;
    int checking_deliver_point = 0;
    int posted_object = 0;
    std::unordered_set<std::string> post_target;
    std::string land_target;
    ros::Time last_srv_request = ros::Time::now();
    std::cout << "\033[32mReached Offboard State.\033[0m" << std::endl;
    while (ros::ok()) {
        //std::cout << posted_object << "/2 " << checking_deliver_point << "/4" << std::endl;
        geometry_msgs::TwistStamped twist;
        switch (fsm_state) {
            case 00:  // Offboard state
                if (current_state.mode == "OFFBOARD") {
                    fsm_state = 10;  // goto arm state
                    std::cout << "\033[32mReached Arm State.\033[0m" << std::endl;
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
            case 10:  // Arm state
                if (current_state.armed) {
                    fsm_state = 20;  // goto takeoff state
                    std::cout << "\033[32mReached Takeoff State.\033[0m" << std::endl;
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
            case 20:  // Takeoff state
                all_servo_close_pub.publish(std_msgs::Empty());
                if (current_pose.pose.position.z > working_altitude) {
                    fsm_state = 30;  // goto scan qr state
                    std::cout << "\033[32mReached Scan QR State.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist.linear.z = 0.4;
                }
                break;
            case 30:  // Scan QR state
                 if (ros::Time::now() - last_srv_request > ros::Duration(1.0) &&
                        qr_result.header.stamp > last_srv_request &&
                        qr_result.data.size() > 0) {

                    std::cout << "\033[34mQR Detect: " << qr_result.data[0] << "\033[0m" << std::endl;
                    analyseQrMessage(qr_result.data[0], post_target, land_target);
                    fsm_state = 35;  // 修改为先进入圆周运动状态
                    std::cout << "\033[32mReached Circle Motion State.\033[0m" << std::endl;
                    // 初始化圆周运动状态
                    circle_state = 0;  // 从移动到起始点开始
                    circle_total_angle = 0.0;  // 重置累计角度
                    circle_started_counting = false;  // 重置角度累计标志
                    circle_start_time = ros::Time::now();
                    circle_last_time = ros::Time::now();
                    circle_error_dist_sum = 0.0;
                    circle_last_error_dist = 0.0;
                    last_srv_request = ros::Time::now();
                } else if (ros::Time::now() - last_srv_request > ros::Duration(60.0)) {  // Timeout, detect failed
                    land_target = "left";
                    fsm_state = 70;  // goto navigate to special sign state
                    ROS_ERROR("Scan QR Timeout, Reached Navigate to Special Sign State.");
                } else {
                    if (getLengthBetweenPoints(current_pose.pose.position, qr_position) > 0.1) {
                        last_srv_request = ros::Time::now();
                    }
                    twist.twist = get_pid_vel(qr_position);
                }
                break;
            case 35:  // Circle Motion State
                switch (circle_state) {
                    case 0:  // 移动到起始点
                        {
                            // 移动到圆周起始点 (圆心x-半径, 圆心y, 高度z)
                            geometry_msgs::Point start_point;
                            start_point.x = circle_pole_x - CIRCLE_RADIUS;
                            start_point.y = circle_pole_y;
                            start_point.z = working_altitude;
                            
                            double distance = getLengthBetweenPoints(current_pose.pose.position, start_point);
                            if (distance < 0.1) {
                                // 到达起始点，开始缓慢圆周运动
                                circle_state = 1;
                                circle_current_speed = circle_initial_speed;
                                circle_start_time = ros::Time::now();
                                circle_last_time = ros::Time::now();
                                circle_error_dist_sum = 0.0;
                                circle_last_error_dist = 0.0;
                                std::cout << "\033[32mReached circle start point, beginning slow circular motion...\033[0m" << std::endl;
                            } else {
                                // 继续移动到起始点
                                twist.twist = get_pid_vel(start_point);
                            }
                        }
                        break;
                        
                    case 1:  // 慢速开始圆周运动
                        {
                            // 使用慢速进行圆周运动
                            twist.twist = get_circle_vel(working_altitude, circle_current_speed);
                            
                            // 在慢速圆周运动2秒后开始加速
                            if ((ros::Time::now() - circle_start_time).toSec() > 2.0) {
                                circle_state = 2;
                                circle_start_time = ros::Time::now();
                                std::cout << "\033[32mStarting acceleration to desired speed...\033[0m" << std::endl;
                            }
                        }
                        break;
                        
                    case 2:  // 加速阶段
                        {
                            // 计算当前时间差
                            double time_elapsed = (ros::Time::now() - circle_start_time).toSec();
                            
                            // 根据加速度更新当前角速度
                            circle_current_speed = circle_initial_speed + circle_acceleration * time_elapsed;
                            
                            // 限制不超过期望角速度
                            if (circle_current_speed >= CIRCLE_ANGULAR_SPEED) {
                                circle_current_speed = CIRCLE_ANGULAR_SPEED;
                                circle_state = 3;
                                std::cout << "\033[32mReached desired angular speed, switching to normal circle mode...\033[0m" << std::endl;
                            }
                            
                            // 使用当前角速度执行圆周运动
                            twist.twist = get_circle_vel(working_altitude, circle_current_speed);
                            
                            // 输出当前角速度
                            //ROS_INFO("Current angular speed: %.2f rad/s", circle_current_speed);
                        }
                        break;
                        
                    case 3:  // 正常速度圆周运动
                        {
                            // 计算当前位置与起始点的距离
                            geometry_msgs::Point start_point;
                            start_point.x = circle_pole_x - CIRCLE_RADIUS;
                            start_point.y = circle_pole_y;
                            start_point.z = working_altitude;
                            double distance_to_start = getLengthBetweenPoints(current_pose.pose.position, start_point);
                            
                            // 计算已完成的圈数
                            double completed_laps = circle_total_angle / (2*M_PI);
                            
                            // 检查是否完成了目标圈数且接近起始点
                            if (completed_laps >= circle_laps && distance_to_start < 0.2) {
                                fsm_state = 40;  // goto check deliver point state
                                std::cout << "\033[32mCompleted " << completed_laps << " laps, reached start point. Moving to Check Deliver Point State.\033[0m" << std::endl;
                                last_srv_request = ros::Time::now();
                            } else {
                                // 使用正常速度执行圆周运动
                                twist.twist = get_circle_vel(working_altitude, CIRCLE_ANGULAR_SPEED);
                            }
                        }
                        break;
                }
                break;
            case 40:  // Check deliver point state
                if (ros::Time::now() - last_srv_request > ros::Duration(1.0) &&
                    sign_result.header.stamp > last_srv_request &&
                    sign_result.data.size() > 0) {  // detect succeeded
                    // 修改判断条件：在后两个投放点且未完成两次投放时强制投放
                    if (post_target.count(sign_result.data[0]) || 
                        (checking_deliver_point >= 2 && posted_object < 2)) {  
                        fsm_state = 50;
                        if(yolo_detecting) {
                            toggle_yolo_detect_client.call(empty);
                            yolo_detecting = false;
                        }
                        std::cout << "\033[32mReached Deliver State. Target: " 
                                    << sign_result.data[0] << "\033[0m" << std::endl;
                        last_srv_request = ros::Time::now();
                    } else {  // 非目标且不在后两个投放点
                        checking_deliver_point++;
                        if (checking_deliver_point >= 4) {
                            fsm_state = 70;
                            if(yolo_detecting) {
                                toggle_yolo_detect_client.call(empty);
                                yolo_detecting = false;
                            }
                            std::cout << "\033[32mAll points checked.\033[0m" << std::endl;
                        }
                        last_srv_request = ros::Time::now();
                    }
                } else if (ros::Time::now() - last_srv_request > ros::Duration(10.0)) {  // Timeout
                    // 后两个点超时强制投放
                    if (checking_deliver_point >= 2 && posted_object < 2) {
                        fsm_state = 50;
                        if(yolo_detecting) {
                            toggle_yolo_detect_client.call(empty);
                            yolo_detecting = false;
                        }
                        std::cout << "\033[32mTimeout in later points, force delivery.\033[0m" << std::endl;
                        last_srv_request = ros::Time::now();
                    } else {
                        checking_deliver_point++;
                        if (checking_deliver_point >= 4) {
                            fsm_state = 70;
                            if(yolo_detecting) {
                                toggle_yolo_detect_client.call(empty);
                                yolo_detecting = false;
                            }
                            std::cout << "\033[32mAll points checked after timeout.\033[0m" << std::endl;
                        }
                        last_srv_request = ros::Time::now();
                    }
                } else {
                    // 保持移动至当前投放点逻辑
                    if (getLengthBetweenPoints(current_pose.pose.position, deliver_position[checking_deliver_point]) > 0.1) {
                        last_srv_request = ros::Time::now();
                        if(yolo_detecting) {
                            toggle_yolo_detect_client.call(empty);
                            yolo_detecting = false;
                        }
                    } else {
                        if(!yolo_detecting) {
                            toggle_yolo_detect_client.call(empty);
                            yolo_detecting = true;
                        }
                    }
                    twist.twist = get_pid_vel(deliver_position[checking_deliver_point]);
                }
                break;
            case 50:  // Deliver state
                if (ros::Time::now() - last_srv_request > ros::Duration(2.0)) {  // release catapult
                    // 投放前部弹射器（front_left 和 front_right）
                    if(front_left_available) {
                        front_left_open_pub.publish(std_msgs::Empty());
                        ROS_INFO("Released front left catapult");
                        front_left_available = false;
                    } else if(front_right_available) {
                        front_right_open_pub.publish(std_msgs::Empty());
                        ROS_INFO("Released front right catapult");
                        front_right_available = false;
                    } else {
                        ROS_ERROR("No front catapult available");
                    }

                    fsm_state = 60;  // goto wait for object drop state
                    std::cout << "\033[32mReached Wait for Object Drop State.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    if (deliver_detect_result.header.stamp > last_srv_request - ros::Duration(1.0)) {
                        geometry_msgs::Point err;
                        err.x = deliver_detect_result.width / 2.0 - deliver_detect_result.circles[0].center_x;
                        err.y = deliver_detect_result.height / 2.0 - deliver_detect_result.circles[0].center_y;
                        twist.twist = get_pix_pid_vel(err);
                    }

                    //实物中使用
                    // geometry_msgs::Point target = deliver_position[checking_deliver_point];
                    // target.z = drop_altitude;
                    // twist.twist = get_pid_vel(target);

                    //仿真环境中使用
                    twist.twist.linear.z = std::max(-0.5, std::min(0.5, drop_altitude - current_pose.pose.position.z));
                }
                break;
            case 60:  // Wait for object drop state
                if (ros::Time::now() - last_srv_request > ros::Duration(1.0)) {
                    posted_object++;
                    checking_deliver_point++;
                    // if 2 blocks are both delivered or 4 deliver point are all checked,
                    //     goto navigate to special sign state,
                    // else go back to check deliver point state to check the next.
                    if (posted_object == 2 || checking_deliver_point == 4) {
                        std::cout << "\033[32mReached Navigate to Before Special Sign State.\033[0m" << std::endl;
                        fsm_state = 70;
                    } else {
                        std::cout << "\033[32mReached Check Deliver Point State.\033[0m" << std::endl;
                        fsm_state = 40;
                        last_srv_request = ros::Time::now();
                    }
                }
                break;
            case 70:  // Navigate to before special sign state
                {
                    geometry_msgs::Point target;
                    target.x = 3;
                    target.y = 1.6; 
                    target.z = working_altitude;
                    if (getLengthBetweenPoints(current_pose.pose.position, target) > 0.1) {
                        twist.twist = get_pid_vel(target);
                    } else {
                        fsm_state = 80;  // goto navigate to special sign state
                        std::cout << "\033[32mReached Navigate to Special Sign State.\033[0m" << std::endl;
                    }
                }
                break;
            case 80:  // Navigate to special sign state
                {
                    if (getLengthBetweenPoints(current_pose.pose.position, special_deliver_position) > 0.1) {
                        twist.twist = get_pid_vel(special_deliver_position);
                    } else {
                        fsm_state = 100;
                        std::cout << "\033[32mReached Special Deliver State.\033[0m" << std::endl;
                        last_srv_request = ros::Time::now();
                    }
                }
                break;
            case 100:  // Special deliver state
                if (ros::Time::now() - last_srv_request > ros::Duration(2.0)) {  // release catapult
                    // 投放后部弹射器（back_left 和 back_right）
                    back_left_open_pub.publish(std_msgs::Empty());
                    back_right_open_pub.publish(std_msgs::Empty());
                    ROS_INFO("Released back catapults for special target");
                    
                    fsm_state = 110;  // goto special wait for object drop state
                    std::cout << "\033[32mReached Special Wait for Object Drop State.\033[0m" << std::endl;
                    last_srv_request = ros::Time::now();
                } else {
                    if (deliver_detect_result.header.stamp > last_srv_request - ros::Duration(1.0)) {
                        geometry_msgs::Point err;
                        err.x = deliver_detect_result.width / 2.0 - deliver_detect_result.circles[0].center_x;
                        err.y = deliver_detect_result.height / 2.0 - deliver_detect_result.circles[0].center_y;
                        twist.twist = get_pix_pid_vel(err);
                    }
                    //在实物中使用
                    // geometry_msgs::Point target = special_deliver_position;
                    // target.z = drop_altitude;
                    // twist.twist = get_pid_vel(target);

                    //仿真环境中使用
                    twist.twist.linear.z = std::max(-0.5, std::min(0.5, drop_altitude - current_pose.pose.position.z));
                }
                break;
            case 110:  // Special wait for object drop state
                if (ros::Time::now() - last_srv_request > ros::Duration(1.0)) {
                    fsm_state = 111;
                    std::cout << "\033[32mReached Calculate Ring Center Position State.\033[0m" << std::endl;
                }
                break;
            case 111:  // Calculate ring_center position state
                {
                    geometry_msgs::Point target;
                    target.x = 6;
                    target.y = 0;
                    target.z = 1;
                    twist.twist = get_pid_vel(target);
                    if(getLengthBetweenPoints(target, current_pose.pose.position) < 0.05 && !begin_ring_center_calculation) {
                        begin_ring_center_calculation = true;
                        std::cout << "\033[32mBegin Calculate Ring Center Position.\033[0m" << std::endl;
                    }
                    if(ring_center_received) {
                        fsm_state = 112;
                        std::cout << "\033[32mReached Before Through Ring Center State. Ring Center: " << ring_center.x << ", " << ring_center.y << ", " << ring_center.z << "\033[0m" << std::endl;
                    }
                }
                break;
            case 112:  // Before Through Ring Center State
                {
                    geometry_msgs::Point target;
                    target.x = ring_center.x;
                    target.y = ring_center.y+0.5;
                    target.z = ring_center.z+0.1;
                    twist.twist = get_pid_vel(target);
                    if(getLengthBetweenPoints(target, current_pose.pose.position) < 0.05) {
                        fsm_state = 113;
                        std::cout << "\033[32mReached Through Ring Center State.\033[0m" << std::endl;
                    }
                }
                break;
            case 113:  // Through Ring Center State
                {
                    geometry_msgs::Point target;
                    target.x = ring_center.x;
                    target.y = ring_center.y-0.5;
                    target.z = ring_center.z+0.05;
                    twist.twist = get_pid_vel(target);
                    if(getLengthBetweenPoints(target, current_pose.pose.position) < 0.05) {
                        fsm_state = 114;
                        std::cout << "\033[32mReached Before Land State.\033[0m" << std::endl;
                    }
                }
                break;
            case 114:  // Before Land State
                {
                    geometry_msgs::Point target;
                    target.x = 4.5;
                    target.y = -2.1;
                    target.z = working_altitude;
                    twist.twist = get_pid_vel(target);
                    if(getLengthBetweenPoints(target, current_pose.pose.position) < 0.1) {
                        if(land_target == "left") {
                            current_land_position = left_land_position;
                        } else {
                            current_land_position = right_land_position;
                        }
                        fsm_state = 120;
                        std::cout << "\033[32mReached Going to land position state State.\033[0m" << std::endl;
                    }
                }   
                break;
            case 120:  //Going to land position state
                {
                    geometry_msgs::Point target = current_land_position;
                    twist.twist = get_pid_vel(target);
                    if(getLengthBetweenPoints(target, current_pose.pose.position) < 0.1) {
                        fsm_state = 130;  // goto accurately land state
                        std::cout << "\033[32mReached Accurately Land State.\033[0m" << std::endl;
                        last_srv_request = ros::Time::now();
                    }
                }
                break;
            case 130:  // Accurately land state
                {
                    geometry_msgs::Point target = current_land_position;
                    target.z = 0.15;
                    twist.twist = get_pid_vel(target);
                    if(getLengthBetweenPoints(target, current_pose.pose.position) < 0.05) {
                        fsm_state = 1000;  // goto land state
                        std::cout << "\033[32mReached Land State.\033[0m" << std::endl;
                        last_srv_request = ros::Time::now();
                    }
                }
                break;
            case 1000:  // Land state
                if (current_state.mode == "AUTO.LAND") {
                    fsm_state = -1;  // goto do nothing state
                } else if (current_pose.pose.position.z < 0.1 && (ros::Time::now() - last_srv_request > ros::Duration(2))) {
                    mavros_msgs::SetMode land_set_mode;
                    land_set_mode.request.custom_mode = "AUTO.LAND";
                    set_mode_client.call(land_set_mode);
                    last_srv_request = ros::Time::now();
                } else {
                    twist.twist.linear.z = -0.3;
                }
                break;
            default:
                if (fsm_state != -1) {
                    ROS_FATAL("FATAL ERROR: FSM reaches an invalid state: %d, emergency landing.", fsm_state);
                    fsm_state = 1000;
                }
                break;
        }
        vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
