#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <cmath>

// PID控制参数默认值（现在从参数服务器获取实际值）
#define KP_POS 0.5  // 位置控制PID参数
#define KI_POS 0.01
#define KD_POS 0.2
#define KP_HEIGHT 0.3
#define KI_HEIGHT 0.01
#define KD_HEIGHT 0.1

// 状态机状态
#define STATE_WAIT_FOR_CONNECT 0    // 等待连接FCU
#define STATE_WAIT_FOR_OFFBOARD 1   // 等待切换到OFFBOARD模式
#define STATE_WAIT_FOR_ARM 2        // 等待解锁
#define STATE_TAKEOFF 3             // 起飞
#define STATE_MOVE_TO_CIRCLE 4      // 移动到圆周路径上的初始点
#define STATE_SLOW_CIRCLE 5         // 缓慢开始圆周运动
#define STATE_ACCELERATE 6          // 逐渐加速到期望速度
#define STATE_CIRCLE 7              // 执行圆周运动

// 全局变量
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::Vector3 current_rpy;

// 柱子和圆的参数
double pole_x = 4.3;  // 柱子X坐标
double pole_y = 0.0;  // 柱子Y坐标
double desired_radius = 1.0;  // 期望圆半径
double desired_height = 1.0;  // 期望飞行高度
double angular_speed = 1.0;   // 角速度
double takeoff_height = 1.0;  // 起飞高度

// PID控制器参数
double kp_dist = 3.0;
double ki_dist = 0.5;
double kd_dist = 0.5;

// 速度控制参数
double initial_angular_speed = 0.1;  // 初始角速度（缓慢开始）
double current_angular_speed = 0.0;  // 当前角速度
double acceleration = 0.05;          // 加速度 (rad/s²)

// 初始点参数
double target_x = 0.0;  // 目标点X坐标
double target_y = 0.0;  // 目标点Y坐标

// PID控制相关变量
double error_dist_sum = 0.0;
double last_error_dist = 0.0;
double error_height_sum = 0.0;
double last_error_height = 0.0;
double error_x_sum = 0.0;
double last_error_x = 0.0;
double error_y_sum = 0.0;
double last_error_y = 0.0;
ros::Time last_time;
ros::Time state_start_time;  // 状态开始时间

// 回调函数：更新无人机状态
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 回调函数：更新无人机位置姿态
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(msg->pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(current_rpy.x, current_rpy.y, current_rpy.z);
}

// 计算圆周上的初始点
void calculate_circle_initial_point() {
    // 选择柱子右侧的点（0度位置）
    target_x = pole_x - desired_radius;
    target_y = pole_y;
    ROS_INFO("initial point: (%.2f, %.2f)", target_x, target_y);
}

// 执行移动到指定点的函数
geometry_msgs::Twist execute_move_to_point(double x, double y, double z) {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    if (dt <= 0) dt = 0.01; // 防止除零错误
    last_time = current_time;
    
    // 计算位置误差
    double error_x = x - current_pose.pose.position.x;
    double error_y = y - current_pose.pose.position.y;
    double error_z = z - current_pose.pose.position.z;
    
    // 累积误差
    error_x_sum += error_x * dt;
    error_y_sum += error_y * dt;
    error_height_sum += error_z * dt;
    
    // 限制积分项
    error_x_sum = std::max(-1.0, std::min(error_x_sum, 1.0));
    error_y_sum = std::max(-1.0, std::min(error_y_sum, 1.0));
    error_height_sum = std::max(-1.0, std::min(error_height_sum, 1.0));
    
    // 计算微分项
    double error_x_diff = (error_x - last_error_x) / dt;
    double error_y_diff = (error_y - last_error_y) / dt;
    double error_z_diff = (error_z - last_error_height) / dt;
    
    // 更新上一次误差
    last_error_x = error_x;
    last_error_y = error_y;
    last_error_height = error_z;
    
    // PID控制
    double vx = KP_POS * error_x + KI_POS * error_x_sum + KD_POS * error_x_diff;
    double vy = KP_POS * error_y + KI_POS * error_y_sum + KD_POS * error_y_diff;
    double vz = KP_HEIGHT * error_z + KI_HEIGHT * error_height_sum + KD_HEIGHT * error_z_diff;
    
    // 限制最大速度
    double max_vel = 1.0;
    double vel_norm = sqrt(vx*vx + vy*vy);
    if (vel_norm > max_vel) {
        vx = vx * max_vel / vel_norm;
        vy = vy * max_vel / vel_norm;
    }
    
    // 创建速度命令
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = vx;
    vel_cmd.linear.y = vy;
    vel_cmd.linear.z = vz;
    
    // 打印距离信息（每次迭代都输出）
    double distance = sqrt(error_x*error_x + error_y*error_y + error_z*error_z);
    ROS_INFO("distance: %.2f m", distance);
    
    return vel_cmd;
}

// 执行起飞操作的函数
geometry_msgs::Twist execute_takeoff() {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    if (dt <= 0) dt = 0.01; // 防止除零错误
    last_time = current_time;
    
    // PID控制 - 高度控制
    double error_height = takeoff_height - current_pose.pose.position.z;
    error_height_sum += error_height * dt;
    // 限制积分项以防止积分饱和
    error_height_sum = std::max(-1.0, std::min(error_height_sum, 1.0));
    double error_height_diff = (error_height - last_error_height) / dt;
    last_error_height = error_height;
    
    // 高度控制（Z轴速度）
    double z_vel = KP_HEIGHT * error_height + KI_HEIGHT * error_height_sum + KD_HEIGHT * error_height_diff;
    
    // 创建速度命令
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = z_vel;
    
    return vel_cmd;
}

// 执行圆周运动的函数（带速度控制）
geometry_msgs::Twist execute_circle_with_speed(double speed) {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    if (dt <= 0) dt = 0.01; // 防止除零错误
    last_time = current_time;
    
    // 计算当前距离柱子的平面距离
    double dx = current_pose.pose.position.x - pole_x;
    double dy = current_pose.pose.position.y - pole_y;
    double current_dist = sqrt(dx*dx + dy*dy);
    
    // 防止除零错误
    if (current_dist < 0.01) current_dist = 0.01;
    
    // PID控制器 - 距离控制
    double error_dist = desired_radius - current_dist;
    error_dist_sum += error_dist * dt;
    // 限制积分项以防止积分饱和
    error_dist_sum = std::max(-1.0, std::min(error_dist_sum, 1.0));
    double error_dist_diff = (error_dist - last_error_dist) / dt;
    last_error_dist = error_dist;
    
    ROS_INFO("Radius error: %.3f m (current: %.3f m, desired: %.3f m)", error_dist, current_dist, desired_radius);
    
    // 径向速度（向内/向外控制半径）
    double radial_vel = kp_dist * error_dist + ki_dist * error_dist_sum + kd_dist * error_dist_diff;
    
    // PID控制器 - 高度控制
    double error_height = desired_height - current_pose.pose.position.z;
    error_height_sum += error_height * dt;
    // 限制积分项以防止积分饱和
    error_height_sum = std::max(-1.0, std::min(error_height_sum, 1.0));
    double error_height_diff = (error_height - last_error_height) / dt;
    last_error_height = error_height;
    
    // 高度控制（Z轴速度）
    double z_vel = KP_HEIGHT * error_height + KI_HEIGHT * error_height_sum + KD_HEIGHT * error_height_diff;
    
    // 使用传入的角速度计算切向速度
    double tangential_vel = speed * current_dist;
    
    // 计算无人机到柱子的方向向量（归一化）
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

// 标准圆周运动函数（使用全局角速度）
geometry_msgs::Twist execute_circle() {
    return execute_circle_with_speed(angular_speed);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 从参数服务器获取参数
    private_nh.param("pole_x", pole_x, 4.3);
    private_nh.param("pole_y", pole_y, 0.0);
    private_nh.param("desired_radius", desired_radius, 1.0);
    private_nh.param("desired_height", desired_height, 1.0);
    private_nh.param("angular_speed", angular_speed, 1.0);
    private_nh.param("takeoff_height", takeoff_height, desired_height);
    private_nh.param("initial_angular_speed", initial_angular_speed, 0.1);
    private_nh.param("acceleration", acceleration, 0.05);
    
    // 获取PID控制参数
    private_nh.param("kp_dist", kp_dist, 3.0);
    private_nh.param("ki_dist", ki_dist, 0.5);
    private_nh.param("kd_dist", kd_dist, 0.5);
    
    // 输出加载的PID参数
    ROS_INFO("Loaded PID parameters: kp_dist=%.2f, ki_dist=%.2f, kd_dist=%.2f", kp_dist, ki_dist, kd_dist);
    
    // 初始化订阅和发布
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    // 初始化变量
    last_time = ros::Time::now();
    state_start_time = ros::Time::now();
    
    // 设置控制频率
    ros::Rate rate(20.0);
    
    // 初始化状态机
    int fsm_state = STATE_WAIT_FOR_CONNECT;
    ros::Time last_request = ros::Time::now();
    
    // 计算圆周上的初始点
    calculate_circle_initial_point();
    
    ROS_INFO("Circle control node initialized, waiting for drone to connect...");
    
    // 主循环
    while (ros::ok()) {
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.stamp = ros::Time::now();
        
        // 状态机
        switch (fsm_state) {
            case STATE_WAIT_FOR_CONNECT:
                // 等待连接FCU
                if (current_state.connected) {
                    ROS_INFO("FCU connected, waiting for OFFBOARD mode...");
                    fsm_state = STATE_WAIT_FOR_OFFBOARD;
                    last_request = ros::Time::now();
                }
                break;
                
            case STATE_WAIT_FOR_OFFBOARD:
                // 等待切换到OFFBOARD模式
                if (current_state.mode == "OFFBOARD") {
                    ROS_INFO("OFFBOARD mode enabled, waiting for arming...");
                    fsm_state = STATE_WAIT_FOR_ARM;
                } else {
                    if (ros::Time::now() - last_request > ros::Duration(1.0)) {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                            ROS_INFO("Offboard mode sent");
                        } else {
                            ROS_WARN("Failed to set OFFBOARD mode");
                        }
                        last_request = ros::Time::now();
                    }
                }
                break;
                
            case STATE_WAIT_FOR_ARM:
                // 等待解锁
                if (current_state.armed) {
                    ROS_INFO("Drone armed, starting takeoff...");
                    fsm_state = STATE_TAKEOFF;
                    // 重置PID控制器
                    error_height_sum = 0.0;
                    last_error_height = 0.0;
                    last_time = ros::Time::now();
                } else {
                    if (ros::Time::now() - last_request > ros::Duration(1.0)) {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                            ROS_INFO("Vehicle arming sent");
                        } else {
                            ROS_WARN("Failed to arm drone");
                        }
                        last_request = ros::Time::now();
                    }
                }
                break;
                
            case STATE_TAKEOFF:
                // 起飞状态
                if (fabs(takeoff_height - current_pose.pose.position.z) < 0.1) {
                    ROS_INFO("Takeoff complete, moving to circle initial point...");
                    fsm_state = STATE_MOVE_TO_CIRCLE;
                    // 重置位置PID控制器
                    error_x_sum = 0.0;
                    error_y_sum = 0.0;
                    last_error_x = 0.0;
                    last_error_y = 0.0;
                    last_time = ros::Time::now();
                } else {
                    twist_msg.twist = execute_takeoff();
                }
                break;
                
            case STATE_MOVE_TO_CIRCLE:
                // 移动到圆周路径上的初始点
                {
                    double dx = target_x - current_pose.pose.position.x;
                    double dy = target_y - current_pose.pose.position.y;
                    double distance = sqrt(dx*dx + dy*dy);
                    
                    if (distance < 0.1) {
                        ROS_INFO("Reached initial point, starting slow circle motion...");
                        fsm_state = STATE_SLOW_CIRCLE;
                        current_angular_speed = initial_angular_speed;
                        state_start_time = ros::Time::now();
                        // 重置距离PID控制器
                        error_dist_sum = 0.0;
                        last_error_dist = 0.0;
                        last_time = ros::Time::now();
                    } else {
                        twist_msg.twist = execute_move_to_point(target_x, target_y, desired_height);
                    }
                }
                break;
                
            case STATE_SLOW_CIRCLE:
                // 缓慢开始圆周运动
                {
                    // 使用缓慢的角速度执行圆周运动
                    twist_msg.twist = execute_circle_with_speed(current_angular_speed);
                    
                    // 在慢速圆周运动2秒后开始加速
                    if ((ros::Time::now() - state_start_time).toSec() > 2.0) {
                        ROS_INFO("Starting acceleration to desired speed...");
                        fsm_state = STATE_ACCELERATE;
                        state_start_time = ros::Time::now();
                    }
                }
                break;
                
            case STATE_ACCELERATE:
                // 逐渐加速到期望速度
                {
                    // 计算当前时间差
                    double time_elapsed = (ros::Time::now() - state_start_time).toSec();
                    
                    // 根据加速度更新当前角速度
                    current_angular_speed = initial_angular_speed + acceleration * time_elapsed;
                    
                    // 限制不超过期望角速度
                    if (current_angular_speed >= angular_speed) {
                        current_angular_speed = angular_speed;
                        ROS_INFO("Reached desired angular speed, switching to normal circle mode...");
                        fsm_state = STATE_CIRCLE;
                    }
                    
                    // 使用当前角速度执行圆周运动
                    twist_msg.twist = execute_circle_with_speed(current_angular_speed);
                    
                    // 输出当前角速度（每次迭代都输出）
                    ROS_INFO("Current angular speed: %.2f rad/s", current_angular_speed);
                }
                break;
                
            case STATE_CIRCLE:
                // 圆周飞行状态
                twist_msg.twist = execute_circle();
                break;
                
            default:
                fsm_state = STATE_WAIT_FOR_CONNECT;
                break;
        }
        
        // 发布速度命令
        vel_pub.publish(twist_msg);
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
