#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
白盘靶心检测节点（AIC2025 实物靶标：黑环 + 灰环(+绿环) + 中心白盘，无红点）

输出接口与 center_red_dot_distance_node.py 完全一致，可直接替换：
    topic : target_center_diff  (geometry_msgs/Vector3)
            x =  dy  =  (H/2 - cy)   靶心相对画面中心偏上为正, 单位像素
            y = -dx  = -(cx - W/2)   靶心相对画面中心偏右时为负, 单位像素
            z =  sqrt(dx^2+dy^2)  像素距离;  未检出时 z = -1
run_aic2025.py 的 get_visual_adjustment() 直接使用 x/y，无需任何改动。

检测思路：
    1. HSV 白色掩码 -> 找外轮廓
    2. 面积 / 圆度 / 半径范围过滤（排除墙面、地面等大片白色）
    3. 环带校验：白盘外侧一圈必须"明显更暗"（灰环/黑环），可选要求该环带为绿色
       —— 这样即使现场还有别的白色物体也不会误锁
"""

import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image


class WhiteCenterTargetDetector(object):

    def __init__(self):
        rospy.init_node("white_center_target_detector")
        self.bridge = CvBridge()
        self.load_params()

        # 形态学核
        self.close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        self.last_log_time = rospy.Time.now()

        self.diff_pub = rospy.Publisher("target_center_diff", Vector3, queue_size=1)
        self.debug_pub = (rospy.Publisher("target_center_diff_raw", Vector3, queue_size=1)
                          if self.pub_debug else None)
        self.image_pub = (rospy.Publisher("target_center_diff_img", Image, queue_size=1)
                          if self.pub_img else None)
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback, queue_size=1)

        rospy.loginfo("[white_center_target_detector] 订阅 %s -> 发布 target_center_diff", self.camera_topic)
        rospy.loginfo("  白盘: S<=%d V>=max(%d, %.2f*V99.5) | 面积 %.0f~%.0f px | 半径 %.0f~%.0f px "
                      "| 圆度>=%.2f 填充度>=%.2f",
                      self.white_sat_max, self.white_val_min, self.white_val_ratio,
                      self.min_area, self.max_area, self.min_radius, self.max_radius,
                      self.min_circularity, self.min_circle_fill)
        rospy.loginfo("  环带校验: %s (环带 V < 白盘 V * %.2f, 绿占比>=%.2f%s)",
                      self.use_ring_validation, self.ring_dark_ratio,
                      self.ring_green_ratio_min,
                      " [mode=%s]" % self.ring_mode)

    # ------------------------------------------------------------------ params
    def load_params(self):
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/image_raw")
        self.show_image = rospy.get_param("~show_image", False)
        self.pub_img = rospy.get_param("~pubImg", False)
        self.pub_debug = rospy.get_param("~pubDebug", False)

        # 白色阈值（HSV）
        #   阈值 = max(white_val_min, white_val_ratio * 画面 V 的 99.5 分位)
        #   自适应是为了"同一个参数既能跑仿真(白盘 V≈138)又能跑实物(白盘 V≈230)"，
        #   绝对下限 white_val_min 防止全黑画面里把噪声当白盘。
        self.white_sat_max = rospy.get_param("~white_sat_max", 100)
        self.white_val_min = rospy.get_param("~white_val_min", 60)
        self.white_val_ratio = rospy.get_param("~white_val_ratio", 0.70)
        self._white_val_thr = float(self.white_val_min)

        # 尺寸 / 形状过滤：白盘 Ø0.2 m，0.8 m 高度约 250 px 直径（焦距≈1000 px 时）
        self.min_area = rospy.get_param("~min_area", 300.0)
        self.max_area = rospy.get_param("~max_area", 0.0)          # 0 = 不限制
        self.min_radius = rospy.get_param("~min_radius", 8.0)
        self.max_radius = rospy.get_param("~max_radius", 0.0)      # 0 = 不限制
        self.min_circularity = rospy.get_param("~min_circularity", 0.60)
        # 轮廓面积 / 最小外接圆面积：真正的圆盘≈1.0，方块≈0.64，用来挡掉白墙
        self.min_circle_fill = rospy.get_param("~min_circle_fill", 0.75)
        self.gaussian_blur_size = rospy.get_param("~gaussian_blur_size", 5)

        # 环带校验：白盘外 1.15r~1.90r 的圆环
        self.use_ring_validation = rospy.get_param("~use_ring_validation", True)
        self.ring_inner_scale = rospy.get_param("~ring_inner_scale", 1.15)
        self.ring_outer_scale = rospy.get_param("~ring_outer_scale", 1.90)
        self.ring_dark_ratio = rospy.get_param("~ring_dark_ratio", 0.80)   # 环带平均亮度 <= 白盘 * 0.8
        self.ring_green_ratio_min = rospy.get_param("~ring_green_ratio_min", 0.20)
        # or   = 环带"更暗"或"是绿色"任一满足即可（灰环靶标 / 绿环靶标都适用，默认）
        # dark = 只认"更暗"（实物为灰环/黑环时用）
        # green= 只认绿色环（仿真贴图那种纯绿环）
        self.ring_mode = rospy.get_param("~ring_mode", "or")
        self.require_green_ring = rospy.get_param("~require_green_ring", False)
        self.green_hue_low = rospy.get_param("~green_hue_low", 35)
        self.green_hue_high = rospy.get_param("~green_hue_high", 90)
        self.green_sat_min = rospy.get_param("~green_sat_min", 60)
        self.green_val_min = rospy.get_param("~green_val_min", 60)

        # 选择策略：best=环带最暗/最绿的那一个, largest=白盘面积最大的那一个
        self.select = rospy.get_param("~select", "best")

    # ------------------------------------------------------------------ helper
    def white_mask(self, hsv):
        """HSV 白色掩码（自适应亮度阈值，与 detection 使用同一套阈值）"""
        v = hsv[:, :, 2]
        v_ref = float(np.percentile(v, 99.5))
        self._white_val_thr = max(float(self.white_val_min), self.white_val_ratio * v_ref)
        return cv2.inRange(
            hsv,
            np.array([0, 0, int(self._white_val_thr)]),
            np.array([179, self.white_sat_max, 255]),
        )

    def preprocess(self, frame):
        blur = max(1, int(self.gaussian_blur_size))
        if blur % 2 == 0:
            blur += 1
        return cv2.GaussianBlur(frame, (blur, blur), 0)

    @staticmethod
    def _ring_mask(shape, center, r_in, r_out):
        mask = np.zeros(shape[:2], dtype=np.uint8)
        cv2.circle(mask, (int(round(center[0])), int(round(center[1]))), int(round(r_out)), 255, -1)
        cv2.circle(mask, (int(round(center[0])), int(round(center[1]))), int(round(r_in)), 0, -1)
        return mask

    def ring_evidence(self, frame, hsv, center, radius):
        """返回 (环带/白盘 平均亮度比, 环带绿色像素占比, 环带白色像素占比)"""
        r_in = radius * self.ring_inner_scale
        r_out = radius * self.ring_outer_scale
        ring = self._ring_mask(frame.shape, center, r_in, r_out)
        ring_pixels = ring > 0
        if not np.any(ring_pixels):
            return 1.0, 0.0, 0.0

        disc = self._ring_mask(frame.shape, center, 0, radius * 0.9)
        disc_pixels = disc > 0

        v = hsv[:, :, 2]
        disc_v = float(np.mean(v[disc_pixels])) if np.any(disc_pixels) else 255.0
        ring_v = float(np.mean(v[ring_pixels]))
        dark_ratio = ring_v / max(disc_v, 1e-6)

        green_mask = cv2.inRange(
            hsv,
            np.array([self.green_hue_low, self.green_sat_min, self.green_val_min]),
            np.array([self.green_hue_high, 255, 255]),
        )
        ring_count = float(np.count_nonzero(ring_pixels))
        green_ratio = float(np.count_nonzero(green_mask[ring_pixels])) / ring_count
        ring_white = self.white_mask(hsv)[ring_pixels]
        white_ratio = float(np.count_nonzero(ring_white)) / ring_count
        return dark_ratio, green_ratio, white_ratio

    # ------------------------------------------------------------------ detect
    def detect(self, frame):
        """返回 (cx, cy, radius, area, circularity, dark_ratio, green_ratio) 或 None"""
        enhanced = self.preprocess(frame)
        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        white_mask = self.white_mask(hsv)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, self.close_kernel)

        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            if self.max_area > 0 and area > self.max_area:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter ** 2)
            if circularity < self.min_circularity:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            if radius < self.min_radius:
                continue
            if self.max_radius > 0 and radius > self.max_radius:
                continue
            circle_fill = area / max(math.pi * radius * radius, 1e-6)
            if circle_fill < self.min_circle_fill:
                continue

            dark_ratio, green_ratio, white_ratio = (1.0, 0.0, 0.0)
            if self.use_ring_validation:
                dark_ratio, green_ratio, white_ratio = self.ring_evidence(frame, hsv, (cx, cy), radius)
                if white_ratio > 0.5:      # 环带本身还是白的 -> 说明是墙面/地面的一部分
                    continue
                is_dark = dark_ratio <= self.ring_dark_ratio
                is_green = green_ratio >= self.ring_green_ratio_min
                if self.ring_mode == "dark":
                    ok = is_dark
                elif self.ring_mode == "green":
                    ok = is_green
                else:
                    ok = is_dark or is_green
                if self.require_green_ring:
                    ok = ok and is_green
                if not ok:
                    continue
            candidates.append({
                "cx": cx, "cy": cy, "radius": radius, "area": area,
                "circularity": circularity, "dark_ratio": dark_ratio,
                "green_ratio": green_ratio, "white_ratio": white_ratio,
                "circle_fill": circle_fill,
            })

        if not candidates:
            return None, white_mask, hsv

        if self.select == "largest":
            best = max(candidates, key=lambda c: c["area"])
        else:
            best = min(candidates, key=lambda c: c["dark_ratio"])
        return best, white_mask, hsv

    # ------------------------------------------------------------------ ros cb
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge error: %s", e)
            return

        height, width = frame.shape[:2]
        center = (width // 2, height // 2)
        best, white_mask, hsv = self.detect(frame)

        if best is None:
            # 未检出：发零向量，z=-1（与红点节点约定一致）
            self.diff_pub.publish(Vector3(0.0, 0.0, -1.0))
            if self.show_image:
                self.draw(frame.copy(), None, center, white_mask, "no target")
            return

        dx = best["cx"] - center[0]          # 靶心在右为正
        dy = center[1] - best["cy"]          # 靶心在上为正（图像 y 向下）
        dist = math.sqrt(dx * dx + dy * dy)
        self.diff_pub.publish(Vector3(float(dy), float(-dx), float(dist)))
        if self.debug_pub is not None:
            self.debug_pub.publish(Vector3(float(best["cx"]), float(best["cy"]), float(best["radius"])))

        now = rospy.Time.now()
        if (now - self.last_log_time).to_sec() > 1.0:
            self.last_log_time = now
            rospy.loginfo("靶心 (%.0f,%.0f) r=%.0f px 误差=(%.1f,%.1f) %.1f px | 环带亮度比 %.2f 绿占比 %.2f",
                          best["cx"], best["cy"], best["radius"], dx, dy, dist,
                          best["dark_ratio"], best["green_ratio"])

        if self.image_pub is not None or self.show_image:
            self.draw(frame.copy(), best, center, white_mask, "target")

    # ------------------------------------------------------------------ debug
    def draw(self, frame, best, image_center, white_mask, tag):
        cv2.line(frame, (0, image_center[1]), (frame.shape[1] - 1, image_center[1]), (0, 255, 0), 1)
        cv2.line(frame, (image_center[0], 0), (image_center[0], frame.shape[0] - 1), (0, 255, 0), 1)
        if best is not None:
            c = (int(best["cx"]), int(best["cy"]))
            r = int(best["radius"])
            cv2.circle(frame, c, r, (0, 0, 255), 2)
            cv2.circle(frame, c, 3, (0, 0, 255), -1)
            cv2.line(frame, image_center, c, (255, 0, 0), 2)
            cv2.putText(frame, "err=(%.1f,%.1f)px r=%.0f" % (best["cx"] - image_center[0],
                                                             image_center[1] - best["cy"], best["radius"]),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(frame, "no target", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 255), 2)

        if self.image_pub is not None:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError:
                pass
        if self.show_image:
            cv2.imshow("White Center Target", frame)
            cv2.imshow("White Mask", white_mask)
            cv2.waitKey(1)

    def spin(self):
        rospy.spin()
        if self.show_image:
            cv2.destroyAllWindows()


def main():
    try:
        node = WhiteCenterTargetDetector()
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
