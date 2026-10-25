#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

std::string frame_remap;

void pose_cb(const nav_msgs::Odometry::ConstPtr &msg,
             const ros::Publisher &forward_target) {
  geometry_msgs::PoseStamped ret;
  ret.header = msg->header;
  if (frame_remap.length() > 0)
    ret.header.frame_id = frame_remap;
  ret.pose = msg->pose.pose;
  forward_target.publish(ret);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_forward_node");
  ros::NodeHandle nh;
  ros::NodeHandle param_nh("~");
  auto odometry_topic =
      param_nh.param<std::string>("odometry_topic", "Odometry");
  frame_remap = param_nh.param<std::string>("frame_id", "");
  ros::Publisher vision_pose_pub =
      nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);
  ros::Subscriber px4_sub = nh.subscribe<nav_msgs::Odometry>(
      odometry_topic, 1, boost::bind(&pose_cb, _1, vision_pose_pub));
  ros::spin();
  return 0;
}
