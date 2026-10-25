#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机体安全气泡（反应式避障守卫）
================================
把雷达点云变成"还能不能往前飞"的实时信号，供任务节点做速度级矫正：

    输入:  MID360 原始点云 /livox/lidar (livox_ros_driver2/CustomMsg)
           或 FAST-LIO 机体系点云 /cloud_registered_body (sensor_msgs/PointCloud2)
    输出:  /safety/stop       (std_msgs/Bool)     净空不足 -> true（任务节点就地悬停）
           /safety/clearance  (geometry_msgs/Vector3)
                              x = 四周最小净空, y = 左侧净空, z = 右侧净空（m，已扣机体半径）

点云按"机体坐标系"使用（雷达装在机体上、无相对旋转，FAST-LIO 的 body 系点云同理），
过滤规则：
  * 屏蔽机体自身点（水平半径 < body_radius + self_margin）
  * 屏蔽地面（机体下方离地 < floor_margin 的点）——用 mavros 局部高度判断，
    低空投放时不会把地板当成障碍
  * 只保留机体上下 band_down~band_up、水平 range_max 以内的点
"""

import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool

HAS_LIVOX = True
try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:  # 没装 livox 驱动时退化为只支持 PointCloud2
    HAS_LIVOX = False


class ObstacleGuard(object):

    def __init__(self):
        rospy.init_node("obstacle_guard")

        self.cloud_type = str(rospy.get_param("~cloud_type", "livox")).lower()
        self.cloud_topic = rospy.get_param("~cloud_topic", "/livox/lidar")
        self.pose_topic = rospy.get_param("~pose_topic", "mavros/local_position/pose")

        self.body_radius = float(rospy.get_param("~body_radius", 0.30))
        self.self_margin = float(rospy.get_param("~self_margin", 0.15))
        self.range_max = float(rospy.get_param("~range_max", 1.6))
        self.band_up = float(rospy.get_param("~band_up", 0.35))
        self.band_down = float(rospy.get_param("~band_down", -0.45))
        self.floor_margin = float(rospy.get_param("~floor_margin", 0.25))
        self.side_width = float(rospy.get_param("~side_width", 0.12))  # |y|<该值不计入左/右

        self.stop_clearance = float(rospy.get_param("~stop_clearance", 0.25))
        self.release_clearance = float(rospy.get_param("~release_clearance", 0.35))  # 迟滞
        self.warn_clearance = float(rospy.get_param("~warn_clearance", 0.90))
        self.stale_timeout = float(rospy.get_param("~stale_timeout", 0.5))

        self.altitude = 0.0
        self.have_pose = False
        self.clearance = float("inf")
        self.left = float("inf")
        self.right = float("inf")
        self.last_cloud_time = rospy.Time(0)
        self.stop = False
        self.last_log_time = rospy.Time.now()

        self.pub_stop = rospy.Publisher("/safety/stop", Bool, queue_size=5)
        self.pub_clear = rospy.Publisher("/safety/clearance", Vector3, queue_size=5)
        rospy.Subscriber(self.pose_topic, PoseStamped, self._cb_pose, queue_size=1)

        if self.cloud_type == "livox":
            if not HAS_LIVOX:
                rospy.logerr("~cloud_type=livox 但没装 livox_ros_driver2，请改用 pc2")
                raise SystemExit(1)
            rospy.Subscriber(self.cloud_topic, CustomMsg, self._cb_livox, queue_size=1)
        else:
            rospy.Subscriber(self.cloud_topic, PointCloud2, self._cb_pc2, queue_size=1)

        rospy.loginfo("避障守卫就绪: %s (%s) | 机体半径 %.2f 气泡 %.2f m 上下限 %.2f/%.2f "
                      "| 停 %.2f 松 %.2f m",
                      self.cloud_topic, self.cloud_type, self.body_radius, self.range_max,
                      self.band_down, self.band_up, self.stop_clearance, self.release_clearance)
        rospy.Timer(rospy.Duration(0.05), self._tick)

    # ------------------------------------------------------------------ 回调
    def _cb_pose(self, msg):
        self.altitude = msg.pose.position.z
        self.have_pose = True

    def _cb_livox(self, msg):
        pts = [(p.x, p.y, p.z) for p in msg.points]
        self._update(pts)

    def _cb_pc2(self, msg):
        """PointCloud2 -> (N,3) numpy，支持任意字段顺序"""
        names = [f.name for f in msg.fields]
        if not all(k in names for k in ("x", "y", "z")):
            rospy.logwarn_throttle(5.0, "点云缺少 x/y/z 字段: %s", names)
            return
        dtype = np.dtype([(f.name, np.float32 if f.datatype == 7 else np.uint8,
                           f.count if f.count else 1) for f in msg.fields])
        if msg.is_bigendian:
            dtype = dtype.newbyteorder(">")
        arr = np.frombuffer(msg.data, dtype=dtype)
        pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
        self._update(pts)

    # ------------------------------------------------------------------ 核心
    def _update(self, pts):
        if isinstance(pts, np.ndarray):
            p = pts[np.isfinite(pts).all(axis=1)]
            x = p[:, 0]; y = p[:, 1]; z = p[:, 2]
        else:
            if not pts:
                return
            a = np.asarray(pts, dtype=np.float64)
            x, y, z = a[:, 0], a[:, 1], a[:, 2]

        r = np.hypot(x, y)
        keep = (r <= self.range_max) & (r >= self.body_radius + self.self_margin) \
            & (z <= self.band_up) & (z >= self.band_down)
        if self.have_pose:  # 去掉地面：离地 floor_margin 以内不算障碍
            keep &= z >= -(max(self.altitude - self.floor_margin, 0.0))
        x, y, r = x[keep], y[keep], r[keep]
        self.last_cloud_time = rospy.Time.now()
        if r.size == 0:
            self.clearance = self.left = self.right = float("inf")
            return
        self.clearance = float(r.min()) - self.body_radius
        left = y > self.side_width
        right = y < -self.side_width
        self.left = float(r[left].min()) - self.body_radius if np.any(left) else float("inf")
        self.right = float(r[right].min()) - self.body_radius if np.any(right) else float("inf")

    # ------------------------------------------------------------------ 周期
    def _tick(self, _evt):
        stale = (rospy.Time.now() - self.last_cloud_time).to_sec() > self.stale_timeout
        if stale:  # 点云断了：不新增 stop，但也别把历史净空当成"空"
            self.clearance = self.left = self.right = float("inf")

        if not stale:
            if self.clearance < self.stop_clearance:
                self.stop = True
            elif self.clearance > self.release_clearance:
                self.stop = False

        if (rospy.Time.now() - self.last_log_time).to_sec() > 2.0:
            self.last_log_time = rospy.Time.now()
            if self.stop or self.clearance < self.warn_clearance:
                rospy.logwarn("净空 %.2f m (左 %.2f / 右 %.2f) alt=%.2f -> %s%s",
                              self.clearance, self.left, self.right, self.altitude,
                              "STOP" if self.stop else "slow",
                              "  [点云超时, 按无数据放行]" if stale else "")

        self.pub_stop.publish(Bool(self.stop))
        v = Vector3()
        v.x = self.clearance if self.clearance < 1e6 else 99.0
        v.y = self.left if self.left < 1e6 else 99.0
        v.z = self.right if self.right < 1e6 else 99.0
        self.pub_clear.publish(v)


def main():
    ObstacleGuard()
    rospy.spin()


if __name__ == "__main__":
    main()
