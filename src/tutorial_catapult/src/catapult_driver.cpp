#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

class CatapultDriver {
private:
    ros::NodeHandle nh;
    
    // Publishers for servo control
    ros::Publisher back_right_pub_;
    ros::Publisher back_left_pub_;
    ros::Publisher front_right_pub_;
    ros::Publisher front_left_pub_;
    ros::Publisher host_up_pub_;
    ros::Publisher host_down_pub_;
    
    // Subscribers for commands
    ros::Subscriber back_right_open_sub_;
    ros::Subscriber back_right_close_sub_;
    ros::Subscriber back_left_open_sub_;
    ros::Subscriber back_left_close_sub_;
    ros::Subscriber front_right_open_sub_;
    ros::Subscriber front_right_close_sub_;
    ros::Subscriber front_left_open_sub_;
    ros::Subscriber front_left_close_sub_;
    ros::Subscriber all_open_sub_;
    ros::Subscriber all_close_sub_;
    
    // Servo angle parameters for each servo
    int back_right_open_angle_;
    int back_right_close_angle_;
    int back_right_half_angle_;   // ← 预松留缝（初始化时发，投放前开）
    int back_left_open_angle_;
    int back_left_close_angle_;
    int back_left_half_angle_;
    int front_right_open_angle_;
    int front_right_close_angle_;
    int front_right_half_angle_;
    int front_left_open_angle_;
    int front_left_close_angle_;
    int front_left_half_angle_;
    
    // Initialize connection with servos
    void initServos() {
        std_msgs::Empty msg;
        host_up_pub_.publish(msg);
        ROS_INFO("Servos initialized");
    }
    
    // Callback functions for open/close commands
    void backRightOpenCallback(const std_msgs::Empty::ConstPtr& msg) {
        std_msgs::Int32 angle_msg;
        angle_msg.data = back_right_open_angle_;
        back_right_pub_.publish(angle_msg);
        ROS_INFO("Back right servo opened to angle: %d", back_right_open_angle_);
    }
    
    void backRightCloseCallback(const std_msgs::Empty::ConstPtr& msg) {
        std_msgs::Int32 angle_msg;
        angle_msg.data = back_right_close_angle_;
        back_right_pub_.publish(angle_msg);
        ROS_INFO("Back right servo closed to angle: %d", back_right_close_angle_);
    }
    
    void backLeftOpenCallback(const std_msgs::Empty::ConstPtr& msg) {
        std_msgs::Int32 angle_msg;
        angle_msg.data = back_left_open_angle_;
        back_left_pub_.publish(angle_msg);
        ROS_INFO("Back left servo opened to angle: %d", back_left_open_angle_);
    }
    
    void backLeftCloseCallback(const std_msgs::Empty::ConstPtr& msg) {
        std_msgs::Int32 angle_msg;
        angle_msg.data = back_left_close_angle_;
        back_left_pub_.publish(angle_msg);
        ROS_INFO("Back left servo closed to angle: %d", back_left_close_angle_);
    }
    
    void frontRightOpenCallback(const std_msgs::Empty::ConstPtr& msg) {
        std_msgs::Int32 angle_msg;
        angle_msg.data = front_right_open_angle_;
        front_right_pub_.publish(angle_msg);
        ROS_INFO("Front right servo opened to angle: %d", front_right_open_angle_);
    }
    
    void frontRightCloseCallback(const std_msgs::Empty::ConstPtr& msg) {
        std_msgs::Int32 angle_msg;
        angle_msg.data = front_right_close_angle_;
        front_right_pub_.publish(angle_msg);
        ROS_INFO("Front right servo closed to angle: %d", front_right_close_angle_);
    }
    
    void frontLeftOpenCallback(const std_msgs::Empty::ConstPtr& msg) {
        std_msgs::Int32 angle_msg;
        angle_msg.data = front_left_open_angle_;
        front_left_pub_.publish(angle_msg);
        ROS_INFO("Front left servo opened to angle: %d", front_left_open_angle_);
    }
    
    void frontLeftCloseCallback(const std_msgs::Empty::ConstPtr& msg) {
        std_msgs::Int32 angle_msg;
        angle_msg.data = front_left_close_angle_;
        front_left_pub_.publish(angle_msg);
        ROS_INFO("Front left servo closed to angle: %d", front_left_close_angle_);
    }
    
    void allOpenCallback(const std_msgs::Empty::ConstPtr& msg) {
        std_msgs::Int32 angle_msg;
        
        angle_msg.data = back_right_open_angle_;
        back_right_pub_.publish(angle_msg);
        
        angle_msg.data = back_left_open_angle_;
        back_left_pub_.publish(angle_msg);
        
        angle_msg.data = front_right_open_angle_;
        front_right_pub_.publish(angle_msg);
        
        angle_msg.data = front_left_open_angle_;
        front_left_pub_.publish(angle_msg);
        
        ROS_INFO("All servos opened");
    }
    
    void allCloseCallback(const std_msgs::Empty::ConstPtr& msg) {
        std_msgs::Int32 angle_msg;
        
        angle_msg.data = back_right_close_angle_;
        back_right_pub_.publish(angle_msg);
        
        angle_msg.data = back_left_close_angle_;
        back_left_pub_.publish(angle_msg);
        
        angle_msg.data = front_right_close_angle_;
        front_right_pub_.publish(angle_msg);
        
        angle_msg.data = front_left_close_angle_;
        front_left_pub_.publish(angle_msg);
        
        ROS_INFO("All servos closed");
    }
    
public:
    CatapultDriver() {
        ros::NodeHandle param_nh("~");
        // Get parameters for each servo
        back_right_open_angle_ = param_nh.param("back_right_open_angle", 180);
        back_right_close_angle_ = param_nh.param("back_right_close_angle", 30);
        back_right_half_angle_  = param_nh.param("back_right_half_angle", 60);

        back_left_open_angle_ = param_nh.param("back_left_open_angle", 0);
        back_left_close_angle_ = param_nh.param("back_left_close_angle", 149);
        back_left_half_angle_  = param_nh.param("back_left_half_angle", 119);

        front_right_open_angle_ = param_nh.param("front_right_open_angle", 0);
        front_right_close_angle_ = param_nh.param("front_right_close_angle", 149);
        front_right_half_angle_  = param_nh.param("front_right_half_angle", 119);

        front_left_open_angle_ = param_nh.param("front_left_open_angle", 180);
        front_left_close_angle_ = param_nh.param("front_left_close_angle", 30);
        front_left_half_angle_  = param_nh.param("front_left_half_angle", 60);
        
        // Initialize publishers
        back_right_pub_ = nh.advertise<std_msgs::Int32>("servo/back_right", 1);
        back_left_pub_ = nh.advertise<std_msgs::Int32>("servo/back_left", 1);
        front_right_pub_ = nh.advertise<std_msgs::Int32>("servo/front_right", 1);
        front_left_pub_ = nh.advertise<std_msgs::Int32>("servo/front_left", 1);
        host_up_pub_ = nh.advertise<std_msgs::Empty>("host_up", 1);
        host_down_pub_ = nh.advertise<std_msgs::Empty>("host_down", 1);
        
        // Initialize subscribers
        back_right_open_sub_ = nh.subscribe<std_msgs::Empty>("servo/back_right/open", 1, 
                                            &CatapultDriver::backRightOpenCallback, this);
        back_right_close_sub_ = nh.subscribe<std_msgs::Empty>("servo/back_right/close", 1, 
                                             &CatapultDriver::backRightCloseCallback, this);
        back_left_open_sub_ = nh.subscribe<std_msgs::Empty>("servo/back_left/open", 1, 
                                           &CatapultDriver::backLeftOpenCallback, this);
        back_left_close_sub_ = nh.subscribe<std_msgs::Empty>("servo/back_left/close", 1, 
                                            &CatapultDriver::backLeftCloseCallback, this);
        front_right_open_sub_ = nh.subscribe<std_msgs::Empty>("servo/front_right/open", 1, 
                                             &CatapultDriver::frontRightOpenCallback, this);
        front_right_close_sub_ = nh.subscribe<std_msgs::Empty>("servo/front_right/close", 1, 
                                              &CatapultDriver::frontRightCloseCallback, this);
        front_left_open_sub_ = nh.subscribe<std_msgs::Empty>("servo/front_left/open", 1, 
                                            &CatapultDriver::frontLeftOpenCallback, this);
        front_left_close_sub_ = nh.subscribe<std_msgs::Empty>("servo/front_left/close", 1, 
                                             &CatapultDriver::frontLeftCloseCallback, this);
        all_open_sub_ = nh.subscribe<std_msgs::Empty>("servo/all/open", 1,
                                     &CatapultDriver::allOpenCallback, this);
        all_close_sub_ = nh.subscribe<std_msgs::Empty>("servo/all/close", 1,
                                      &CatapultDriver::allCloseCallback, this);
        
        // Initialize servos
        initServos();
        
        ROS_INFO("Catapult driver initialized.");
    }
    
    void run() {
        // Sleep for 5 seconds to allow time for other nodes to initialize
        ROS_INFO("Waiting 4 seconds before starting...");
        ros::Duration(4.0).sleep();
        host_up_pub_.publish(std_msgs::Empty());
        ROS_INFO("Catapult driver ready.");
        ros::Duration(0.01).sleep();
        // 初始化舵机到"半关预松"位置（留缝防卡，投放时再发 open 全开）
        std_msgs::Int32 angle_msg;
        
        angle_msg.data = back_right_half_angle_;
        back_right_pub_.publish(angle_msg);
        
        angle_msg.data = back_left_half_angle_;
        back_left_pub_.publish(angle_msg);
        
        angle_msg.data = front_right_half_angle_;
        front_right_pub_.publish(angle_msg);
        
        angle_msg.data = front_left_half_angle_;
        front_left_pub_.publish(angle_msg);
        ROS_INFO("All servos half-closed (pre-loosen).");
        ros::spin();
        host_down_pub_.publish(std_msgs::Empty());
        ROS_INFO("Catapult host down.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "catapult_driver");
    
    CatapultDriver driver;
    driver.run();
    
    return 0;
}
