#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

// 定义舵机引脚
const int BACK_RIGHT_PIN = 5;  // 1号舵机 - back_right
const int BACK_LEFT_PIN = 6;   // 2号舵机 - back_left
const int FRONT_RIGHT_PIN = 9; // 3号舵机 - front_right
const int FRONT_LEFT_PIN = 10; // 4号舵机 - front_left
const int SPARE_PIN = 3;       // 备用舵机

// 定义舵机连接状态
bool servos_attached = false;

// 创建ROS节点
ros::NodeHandle nh;

// 创建舵机对象
Servo back_right_servo;  // 1号舵机
Servo back_left_servo;   // 2号舵机
Servo front_right_servo; // 3号舵机
Servo front_left_servo;  // 4号舵机
Servo spare_servo;       // 备用舵机

// 舵机默认打开和关闭的角度
const int OPEN_ANGLE = 180;
const int CLOSE_ANGLE = 0;

// 处理舵机的回调函数
void servoCallback(const std_msgs::Int32& msg, Servo& servo) {
  if (servos_attached) {
    int angle = constrain(msg.data, 0, 180);
    servo.write(angle);
  }
}

// 处理back_right舵机的回调函数
void backRightCallback(const std_msgs::Int32& msg) {
  servoCallback(msg, back_right_servo);
}

// 处理back_left舵机的回调函数
void backLeftCallback(const std_msgs::Int32& msg) {
  servoCallback(msg, back_left_servo);
}

// 处理front_right舵机的回调函数
void frontRightCallback(const std_msgs::Int32& msg) {
  servoCallback(msg, front_right_servo);
}

// 处理front_left舵机的回调函数
void frontLeftCallback(const std_msgs::Int32& msg) {
  servoCallback(msg, front_left_servo);
}

// 处理host_up的回调函数
void hostUpCallback(const std_msgs::Empty& msg) {
  if (!servos_attached) {
    back_right_servo.attach(BACK_RIGHT_PIN);
    back_left_servo.attach(BACK_LEFT_PIN);
    front_right_servo.attach(FRONT_RIGHT_PIN);
    front_left_servo.attach(FRONT_LEFT_PIN);
    spare_servo.attach(SPARE_PIN);
    
    servos_attached = true;
  }
}

// 处理host_down的回调函数
void hostDownCallback(const std_msgs::Empty& msg) {
  if (servos_attached) {
    back_right_servo.detach();
    back_left_servo.detach();
    front_right_servo.detach();
    front_left_servo.detach();
    spare_servo.detach();
    
    servos_attached = false;
  }
}

// 创建订阅者
ros::Subscriber<std_msgs::Int32> back_right_sub("servo/back_right", &backRightCallback);
ros::Subscriber<std_msgs::Int32> back_left_sub("servo/back_left", &backLeftCallback);
ros::Subscriber<std_msgs::Int32> front_right_sub("servo/front_right", &frontRightCallback);
ros::Subscriber<std_msgs::Int32> front_left_sub("servo/front_left", &frontLeftCallback);
ros::Subscriber<std_msgs::Empty> host_up_sub("host_up", &hostUpCallback);
ros::Subscriber<std_msgs::Empty> host_down_sub("host_down", &hostDownCallback);

void setup() {
  // 初始化ROS节点
  nh.initNode();
  
  // 订阅话题
  nh.subscribe(back_right_sub);
  nh.subscribe(back_left_sub);
  nh.subscribe(front_right_sub);
  nh.subscribe(front_left_sub);
  nh.subscribe(host_up_sub);
  nh.subscribe(host_down_sub);
}

void loop() {
  // 处理ROS消息
  nh.spinOnce();
}
