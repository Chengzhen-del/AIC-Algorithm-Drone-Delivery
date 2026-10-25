#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阶段3：视觉识别节点（仿真）
识别下视相机图像中的投放靶（同心圆）和 H 降落点（H字+外圈）。

订阅：/camera/image_raw  (sensor_msgs/Image)
发布：
  /aic/vision/target_center   (geometry_msgs/PointStamped)  靶心在图像中的像素坐标+置信度
  /aic/vision/h_center        (geometry_msgs/PointStamped)  H字中心在图像中的像素坐标+置信度
  /aic/vision/debug           (sensor_msgs/Image)  调试图像（可选）

坐标系：像素坐标原点在左上角，x向右，y向下。
置信度：0~1，基于轮廓面积与几何一致性。
"""

import rospy
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped


class VisionDetector:
    def __init__(self):
        rospy.init_node("vision_detector")

        self.bridge = CvBridge()
        self.debug = rospy.get_param("~debug", True)

        # 相机内参（从 SDF 读取）
        self.fx = 1004.603218
        self.fy = 1004.603218
        self.cx = 640.0
        self.cy = 360.0

        # 靶标颜色阈值（HSV）——绿色环
        self.target_lower = np.array([40, 80, 80])
        self.target_upper = np.array([90, 255, 255])
        # 靶心红点阈值
        self.red_lower1 = np.array([0, 120, 120])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 120, 120])
        self.red_upper2 = np.array([180, 255, 255])

        # H 字阈值（黑色）
        self.h_black_lower = np.array([0, 0, 0])
        self.h_black_upper = np.array([180, 255, 80])

        # H 检测滑动窗口（多帧平均，抑制霍夫圆跳变）
        from collections import deque
        self._h_window = deque(maxlen=8)

        self.pub_target = rospy.Publisher("/aic/vision/target_center", PointStamped, queue_size=1)
        self.pub_h = rospy.Publisher("/aic/vision/h_center", PointStamped, queue_size=1)
        self.pub_debug = rospy.Publisher("/aic/vision/debug", Image, queue_size=1)

        # 显示窗口（需要 GUI；无显示环境时自动禁用，防 SSH/无桌面终端下启动即崩）
        import os
        self.gui = bool(os.environ.get("DISPLAY"))
        if self.gui:
            try:
                cv2.namedWindow("vision_debug", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("vision_debug", 640, 360)
            except Exception as e:
                rospy.logwarn("调试窗口创建失败，转纯话题模式: %s", e)
                self.gui = False
        else:
            rospy.loginfo("无 DISPLAY，调试窗口禁用（检测结果仍通过话题发布）")

        rospy.Subscriber("/camera/image_raw", Image, self.cb_image, queue_size=1)
        rospy.loginfo("vision_detector 就绪，等待 /camera/image_raw ...")

    def cb_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge 转换失败: %s", e)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # --- 识别投放靶（同心圆） ---
        target_found, target_center, target_conf = self.detect_target(frame, hsv)

        # --- 识别 H 降落点 ---
        h_found, h_center, h_conf = self.detect_h(frame, hsv)

        # H 检测多帧滑动平均（抑制霍夫圆在内外圆间跳变）
        now_h = rospy.Time.now().to_sec()
        if h_found and h_conf > 0.1:
            self._h_window.append((h_center[0], h_center[1], h_conf))
            self._last_h_found = now_h
        # 防陈旧窗口Bug：检测丢失>0.5s立即清空窗口并停止发布，
        # 否则旧中值会以新时间戳续发，对准逻辑会把旧像素当实时数据修正（实测污染精对准）
        if now_h - getattr(self, '_last_h_found', 0.0) > 0.5:
            self._h_window.clear()
        # 窗口未满时继续等待
        if len(self._h_window) >= 3:
            smooth_u = np.median([w[0] for w in self._h_window])
            smooth_v = np.median([w[1] for w in self._h_window])
            smooth_conf = np.mean([w[2] for w in self._h_window])
            h_found = True
            h_center = (int(smooth_u), int(smooth_v))
            h_conf = float(smooth_conf)
        else:
            h_found = False

        # --- 打印识别状态（每2秒一次） ---
        now = rospy.Time.now().to_sec()
        if not hasattr(self, '_last_log') or now - self._last_log > 2.0:
            self._last_log = now
            if target_found:
                rospy.loginfo("[视觉] 靶标: (%d,%d) conf=%.2f", target_center[0], target_center[1], target_conf)
            else:
                rospy.loginfo("[视觉] 靶标: 未识别")
            if h_found:
                rospy.loginfo("[视觉] H点: (%d,%d) conf=%.2f", h_center[0], h_center[1], h_conf)
            else:
                rospy.loginfo("[视觉] H点: 未识别")

        # --- 发布结果 ---
        stamp = msg.header.stamp
        if target_found:
            p = PointStamped()
            p.header.stamp = stamp
            p.header.frame_id = "camera"
            p.point.x = float(target_center[0])
            p.point.y = float(target_center[1])
            p.point.z = float(target_conf)
            self.pub_target.publish(p)

        if h_found:
            p = PointStamped()
            p.header.stamp = stamp
            p.header.frame_id = "camera"
            p.point.x = float(h_center[0])
            p.point.y = float(h_center[1])
            p.point.z = float(h_conf)
            self.pub_h.publish(p)

        # --- 调试图像：显示原始图像 + 识别结果 ---
        if self.debug:
            dbg = frame.copy()
            # 画黑色掩码轮廓（H点黑环）
            mask_black = cv2.inRange(hsv, self.h_black_lower, self.h_black_upper)
            bc, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(dbg, bc, -1, (200, 200, 200), 1)
            # 画绿色掩码的轮廓
            mask_green = cv2.inRange(hsv, self.target_lower, self.target_upper)
            contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(dbg, contours, -1, (0, 255, 255), 1)
            # 画红色掩码轮廓（靶心）
            mr1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mr2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mr = cv2.bitwise_or(mr1, mr2)
            rc, _ = cv2.findContours(mr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(dbg, rc, -1, (0, 0, 255), 2)
            if target_found:
                cv2.circle(dbg, target_center, 25, (0, 255, 0), 3)
                cv2.putText(dbg, "TARGET %.2f" % target_conf, (target_center[0]+30, target_center[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            if h_found:
                cv2.circle(dbg, h_center, 30, (255, 0, 0), 3)
                cv2.line(dbg, (h_center[0]-15, h_center[1]), (h_center[0]+15, h_center[1]), (255,0,0), 2)
                cv2.line(dbg, (h_center[0], h_center[1]-15), (h_center[0], h_center[1]+15), (255,0,0), 2)
                cv2.putText(dbg, "H %.2f" % h_conf, (h_center[0]+30, h_center[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)
            # 画图像中心十字
            icx, icy = int(self.cx), int(self.cy)
            cv2.line(dbg, (icx-20, icy), (icx+20, icy), (0, 255, 255), 1)
            cv2.line(dbg, (icx, icy-20), (icx, icy+20), (0, 255, 255), 1)
            # 显示窗口（无 GUI 时跳过，只发 /aic/vision/debug 话题）
            if self.gui:
                cv2.imshow("vision_debug", dbg)
                cv2.waitKey(1)
            try:
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8"))
            except Exception:
                pass

    def detect_target(self, frame, hsv):
        """识别同心圆靶标，返回 (found, center, confidence)。
        优先用靶心小红点定位（真实圆心、贴近正上方时也不被画面边缘裁切影响）；
        找不到红点时退回绿色环拟合圆心。"""
        # 绿色环
        mask_green = cv2.inRange(hsv, self.target_lower, self.target_upper)
        # 红色靶心
        mask_red1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask_red2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        # 红点很小，轻微闭运算去噪
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE,
                                    np.ones((3, 3), np.uint8))

        # 找绿色环的轮廓（作为靶标存在的判据 + 粗圆心）
        contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, (0, 0), 0.0

        # 取最大绿色轮廓
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 500:  # 太小不可靠
            return False, (0, 0), 0.0

        (gx, gy), radius = cv2.minEnclosingCircle(c)
        green_center = (int(gx), int(gy))

        # 优先：在绿环内部区域找红色靶心点（真实圆心）
        red_contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
        best_red = None
        best_d = 1e9
        for rc in red_contours:
            ra = cv2.contourArea(rc)
            if ra < 2.0 or ra > radius * radius * 0.25:
                continue  # 红点应很小；排除大块红色噪声
            M = cv2.moments(rc)
            if M["m00"] == 0:
                continue
            rx = M["m10"] / M["m00"]
            ry = M["m01"] / M["m00"]
            d = math.hypot(rx - gx, ry - gy)
            # 红点必须在绿环中心的 0.55 半径内（避免把场景里其他红色当靶心）
            if d < radius * 0.55 and d < best_d:
                best_d = d
                best_red = (int(rx), int(ry), ra)

        if best_red is not None:
            center = (best_red[0], best_red[1])
            conf = 0.95
            return True, center, float(conf)

        # 退回绿环圆心（远距离红点看不清时）
        conf = min(0.7, area / 20000.0)
        return True, green_center, float(conf)

    def detect_h(self, frame, hsv):
        """识别 H 降落点。找最大的黑色轮廓（H 点黑环+H字是画面里最大的黑色物体），
        拟合最小外接圆，圆心即 H 中心。"""
        mask_black = cv2.inRange(hsv, self.h_black_lower, self.h_black_upper)

        contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, (0, 0), 0.0

        H, W = frame.shape[:2]
        # 按面积排序，取最大的几个候选
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:5]
        best_cx, best_cy, best_score = 0, 0, 0.0

        for c in contours:
            area = cv2.contourArea(c)
            if area < 1000:
                continue

            (ox, oy), r = cv2.minEnclosingCircle(c)
            if r < 30 or r > 450:
                continue

            # 圆形度
            circle_area = math.pi * r * r
            if circle_area == 0:
                continue
            roundness = area / circle_area
            # H 点黑环是环形，roundness 约 0.3-0.7；纯 H 字 roundness 更低
            if roundness < 0.15:
                continue

            # 验证中心区域有黑白混合（H 字）
            r_in = int(r * 0.5)
            inner = mask_black[max(0, int(oy)-r_in):int(oy)+r_in,
                               max(0, int(ox)-r_in):int(ox)+r_in]
            if inner.size == 0:
                continue
            black_ratio = float(np.sum(inner > 0)) / inner.size
            # H 字在白底上，黑色占比 10%-50%
            if black_ratio < 0.05 or black_ratio > 0.55:
                continue

            # 分数：圆形度 + 大小（优先大圆）
            size_bonus = min(1.0, r / 150.0)
            score = roundness * (0.5 + 0.5 * size_bonus)

            if score > best_score:
                best_score = score
                best_cx, best_cy = int(ox), int(oy)

        if best_score < 0.08:
            return False, (0, 0), 0.0
        conf = min(1.0, best_score * 2.5)
        return True, (best_cx, best_cy), float(conf)


if __name__ == "__main__":
    try:
        VisionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
