#!/usr/bin/env python3

import rospy
import numpy as np
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Vector3
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool

class PositionCommandToPositionTarget:
    def __init__(self):
        rospy.init_node('ego_planner_gate')
        self.coordinate_frame = rospy.get_param('~coordinate_frame', PositionTarget.FRAME_LOCAL_NED)
        self.type_mask = rospy.get_param('~type_mask', 0)
        self.frame_id = rospy.get_param('~frame_id', '')
        self.sub = rospy.Subscriber('ego_planner/pos_cmd', PositionCommand, self.position_cmd_callback, queue_size=10)
        self.enable_sub = rospy.Subscriber('ego_planner/enable', Bool, self.enable_callback, queue_size=10)
        self.pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.enabled = True
        self.use_ned_coordinates = (self.coordinate_frame == PositionTarget.FRAME_LOCAL_NED)

    def enable_callback(self, msg):
        self.enabled = msg.data

    def position_cmd_callback(self, msg):
        if not self.enabled:
            return
        try:
            target = PositionTarget()
            target.header = msg.header
            if len(self.frame_id) > 0:
                target.header.frame_id = self.frame_id
            target.coordinate_frame = self.coordinate_frame
            target.type_mask = self.type_mask
            if self.use_ned_coordinates:
                target.position.x = msg.position.y   # ENU的E对应NED的N
                target.position.y = msg.position.x   # ENU的N对应NED的E
                target.position.z = -msg.position.z  # ENU的U对应NED的D
                if hasattr(msg, 'velocity') and msg.velocity:
                    target.velocity.x = msg.velocity.y
                    target.velocity.y = msg.velocity.x
                    target.velocity.z = -msg.velocity.z
                if hasattr(msg, 'acceleration') and msg.acceleration:
                    target.acceleration_or_force.x = msg.acceleration.y
                    target.acceleration_or_force.y = msg.acceleration.x
                    target.acceleration_or_force.z = -msg.acceleration.z
                if hasattr(msg, 'yaw'):
                    yaw_ned = -msg.yaw + np.pi/2  # 转换到NED坐标系
                    target.yaw = yaw_ned
                    while target.yaw > np.pi:
                        target.yaw -= 2*np.pi
                    while target.yaw < -np.pi:
                        target.yaw += 2*np.pi
                if hasattr(msg, 'yaw_dot'):
                    target.yaw_rate = -msg.yaw_dot
            else:
                target.position = msg.position
                if hasattr(msg, 'velocity') and msg.velocity:
                    target.velocity = msg.velocity
                if hasattr(msg, 'acceleration') and msg.acceleration:
                    target.acceleration_or_force = msg.acceleration
                if hasattr(msg, 'yaw'):
                    target.yaw = msg.yaw
                if hasattr(msg, 'yaw_dot'):
                    target.yaw_rate = msg.yaw_dot
            self.pub.publish(target)
        except Exception as e:
            rospy.logerr(f"Failed forward ego-planner control message: {e}")

if __name__ == '__main__':
    try:
        converter = PositionCommandToPositionTarget()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
