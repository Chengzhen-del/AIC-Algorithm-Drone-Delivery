#!/usr/bin/env python3
import rospy
import struct
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np

class LivoxToPointCloud2:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('livox_to_pointcloud2', anonymous=True)
        
        # 获取参数
        self.publish_intensity = rospy.get_param('~publish_intensity', True)
        self.publish_tag = rospy.get_param('~publish_tag', False)
        self.publish_line = rospy.get_param('~publish_line', False)
        self.publish_time_offset = rospy.get_param('~publish_time_offset', False)
        self.is_dense = rospy.get_param('~is_dense', False)

        # 创建发布者
        self.pointcloud_pub = rospy.Publisher('livox/pointcloud2', PointCloud2, queue_size=10)
        
        # 创建订阅者
        rospy.Subscriber('livox/lidar', CustomMsg, self.livox_callback, queue_size=10)
        
        rospy.loginfo("Livox CustomMsg to PointCloud2 converter initialized")
        
    def livox_callback(self, custom_msg):
        """
        处理Livox CustomMsg消息的回调函数
        """
        # 创建PointCloud2消息
        pointcloud_msg = PointCloud2()
        
        # 设置header
        pointcloud_msg.header = custom_msg.header
        
        # 计算每个点的大小和字段
        fields = []
        byte_offset = 0
        
        # 添加x, y, z字段
        fields.append(PointField(name='x', offset=byte_offset, datatype=PointField.FLOAT32, count=1))
        byte_offset += 4
        fields.append(PointField(name='y', offset=byte_offset, datatype=PointField.FLOAT32, count=1))
        byte_offset += 4
        fields.append(PointField(name='z', offset=byte_offset, datatype=PointField.FLOAT32, count=1))
        byte_offset += 4
        
        # 可选字段：反射强度
        if self.publish_intensity:
            fields.append(PointField(name='intensity', offset=byte_offset, datatype=PointField.UINT8, count=1))
            byte_offset += 1
        
        # 可选字段：标签
        if self.publish_tag:
            fields.append(PointField(name='tag', offset=byte_offset, datatype=PointField.UINT8, count=1))
            byte_offset += 1
        
        # 可选字段：线号
        if self.publish_line:
            fields.append(PointField(name='line', offset=byte_offset, datatype=PointField.UINT8, count=1))
            byte_offset += 1
            
        # 可选字段：时间偏移
        if self.publish_time_offset:
            fields.append(PointField(name='time_offset', offset=byte_offset, datatype=PointField.UINT32, count=1))
            byte_offset += 4
        
        # 设置字段
        pointcloud_msg.fields = fields
        
        # 设置点云属性
        pointcloud_msg.height = 1  # 无序点云
        pointcloud_msg.width = custom_msg.point_num  # 点数
        pointcloud_msg.is_bigendian = False  # 小端字节序
        pointcloud_msg.point_step = byte_offset  # 每个点的字节数
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width  # 每行的字节数
        pointcloud_msg.is_dense = self.is_dense
        
        # 创建点云数据
        point_data = bytearray()
        
        for point in custom_msg.points:
            # 打包坐标
            point_data.extend(struct.pack('fff', point.x, point.y, point.z))
            
            # 可选：反射强度
            if self.publish_intensity:
                point_data.extend(struct.pack('B', point.reflectivity))
            
            # 可选：标签
            if self.publish_tag:
                point_data.extend(struct.pack('B', point.tag))
            
            # 可选：线号
            if self.publish_line:
                point_data.extend(struct.pack('B', point.line))
                
            # 可选：时间偏移
            if self.publish_time_offset:
                point_data.extend(struct.pack('I', point.offset_time))
        
        # 设置点云数据
        pointcloud_msg.data = bytes(point_data)
        
        # 发布转换后的点云
        self.pointcloud_pub.publish(pointcloud_msg)
        
    def run(self):
        """运行节点"""
        rospy.spin()

def main():
    try:
        converter = LivoxToPointCloud2()
        converter.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
