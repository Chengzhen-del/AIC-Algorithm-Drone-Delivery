#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <algorithm>
#include <deque>

class RingCenterDetector
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher center_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher box_marker_pub_;
    
    // Parameter settings
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    
    // Store detection results for multiple frames
    std::deque<geometry_msgs::Point> detection_history_;
    int max_history_size_;
    
public:
    RingCenterDetector() : nh_("~")
    {
        // Initialize parameters
        nh_.param("x_min", x_min_, 5.0);
        nh_.param("x_max", x_max_, 7.0);
        nh_.param("y_min", y_min_, -1.7);
        nh_.param("y_max", y_max_, -1.5);
        nh_.param("z_min", z_min_, 1.15);
        nh_.param("z_max", z_max_, 1.35);
        nh_.param("max_history_size", max_history_size_, 10);
        
        // Subscribe to point cloud data
        cloud_sub_ = nh_.subscribe("/cloud_registered", 1, &RingCenterDetector::cloudCallback, this);
        
        // Publish ring center position
        center_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/ring_center", 1);
        
        // Publish visualization marker
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/ring_center_marker", 1);
        
        // 新增：初始化检测区域长方体标记发布器
        box_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/detection_box_marker", 1);
        
        ROS_INFO("Ring center detection node started!");
        ROS_INFO("Detection range: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]", 
                 x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
        ROS_INFO("Accumulating %d frames for stable detection", max_history_size_);
    }
    
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        // Use pass-through filter to extract region of interest
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        
        // X-axis filtering
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min_, x_max_);
        pass_x.filter(*cloud_filtered);
        
        // Y-axis filtering
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(cloud_filtered);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min_, y_max_);
        pass_y.filter(*cloud_filtered);
        
        // Z-axis filtering
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(cloud_filtered);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(z_min_, z_max_);
        pass_z.filter(*cloud_filtered);
        
	    //ROS_INFO("cloud_filtered->size:%zu", cloud_filtered->size());
        if (cloud_filtered->points.empty())
        {
            return;
        }
        
        // Create a collection of points for sorting
        std::vector<pcl::PointXYZ> all_points(cloud_filtered->points.begin(), cloud_filtered->points.end());
        
        // Sort by X coordinate
        std::sort(all_points.begin(), all_points.end(), 
                 [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) { return a.x < b.x; });
        
        // Get the 10 minimum and 10 maximum points
        std::vector<pcl::PointXYZ> min_points, max_points;
        
        // Ensure we have enough points
        int num_points = std::min(20, static_cast<int>(all_points.size()));
        //if (num_points < 10)
        //{
        //    ROS_WARN("Not enough points found, only %d points available", num_points);
        //}
        
        min_points.assign(all_points.begin(), all_points.begin() + num_points);
        max_points.assign(all_points.end() - num_points, all_points.end());
        
        // Calculate the average of these 20 points as the ring center
        double sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto& p : min_points)
        {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }
        for (const auto& p : max_points)
        {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }
        
        int total_points = min_points.size() + max_points.size();
        
        // Current frame detection result
        geometry_msgs::Point current_center;
        current_center.x = sum_x / total_points;
        current_center.y = sum_y / total_points;
        current_center.z = sum_z / total_points;
        
        // Add to history
        detection_history_.push_back(current_center);
        
        // Keep history size limited
        while (detection_history_.size() > max_history_size_)
        {
            detection_history_.pop_front();
        }
        
        // Calculate average from history
        geometry_msgs::Point avg_center;
        avg_center.x = avg_center.y = avg_center.z = 0;
        
        for (const auto& point : detection_history_)
        {
            avg_center.x += point.x;
            avg_center.y += point.y;
            avg_center.z += point.z;
        }
        
        avg_center.x /= detection_history_.size();
        avg_center.y /= detection_history_.size();
        avg_center.z /= detection_history_.size();
        
        // Create and publish ring center position message
        geometry_msgs::PoseStamped center_msg;
        center_msg.header.stamp = ros::Time::now();
        center_msg.header.frame_id = cloud_msg->header.frame_id;
        center_msg.pose.position = avg_center;
        center_msg.pose.orientation.w = 1.0;  // Default quaternion
        
        center_pub_.publish(center_msg);
        
        // Create and publish visualization marker
        publishCenterMarker(avg_center, cloud_msg->header.frame_id);

        publishDetectionAreaBox();
        
        //ROS_INFO("Current frame center: (%.3f, %.3f, %.3f)", 
        //         current_center.x, current_center.y, current_center.z);
        //ROS_INFO("Averaged center position (%lu frames): (%.3f, %.3f, %.3f)", 
        //         detection_history_.size(), avg_center.x, avg_center.y, avg_center.z);
    }
    
    void publishCenterMarker(const geometry_msgs::Point& center, const std::string& frame_id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "ring_center";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Set marker position
        marker.pose.position = center;
        marker.pose.orientation.w = 1.0;
        
        // Set marker scale
        marker.scale.x = 0.2;  // 20cm diameter sphere
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        
        // Set marker color - red
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;  // Fully opaque
        
        // Set marker lifetime
        marker.lifetime = ros::Duration(0.5);  // Half second
        
        marker_pub_.publish(marker);
    }
    
    // 新增：发布检测区域长方体的函数
    void publishDetectionAreaBox()
    {
        visualization_msgs::Marker box_marker;
        box_marker.header.frame_id = "map";  // 使用与点云相同的坐标系
        box_marker.header.stamp = ros::Time::now();
        box_marker.ns = "detection_box";
        box_marker.id = 0;
        box_marker.type = visualization_msgs::Marker::CUBE;
        box_marker.action = visualization_msgs::Marker::ADD;

        // 计算长方体的中心点
        box_marker.pose.position.x = (x_min_ + x_max_) / 2.0;
        box_marker.pose.position.y = (y_min_ + y_max_) / 2.0;
        box_marker.pose.position.z = (z_min_ + z_max_) / 2.0;
        box_marker.pose.orientation.w = 1.0;

        // 设置长方体的尺寸
        box_marker.scale.x = x_max_ - x_min_;
        box_marker.scale.y = y_max_ - y_min_;
        box_marker.scale.z = z_max_ - z_min_;

        // 设置长方体的颜色（半透明蓝色）
        box_marker.color.r = 0.0;
        box_marker.color.g = 0.0;
        box_marker.color.b = 1.0;
        box_marker.color.a = 0.5;  // 设置透明度

        // 持续发布
        box_marker.lifetime = ros::Duration(0.5);

        box_marker_pub_.publish(box_marker);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ring_center_detect_node");
    
    RingCenterDetector detector;
    
    ros::spin();
    
    return 0;
}
