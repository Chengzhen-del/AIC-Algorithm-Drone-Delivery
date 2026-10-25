#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError


class RedDotDetectorNode:
    """ROS节点：红点检测并发布与图像中心的偏差向量"""
    
    def __init__(self):
        rospy.init_node('center_red_dot_distance_node', anonymous=True)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 获取ROS参数
        self.load_parameters()
        
        # 形态学操作核
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        
        # 帧率跟踪
        self.last_frame_time = None
        self.frame_rate = 0.0
        self.fps_alpha = 0.1  # 用于平滑帧率计算的因子
        
        # ROS Publishers
        self.target_diff_pub = rospy.Publisher('target_center_diff', Vector3, queue_size=1)
        
        # ROS Subscribers
        self.image_sub = rospy.Subscriber(
            self.camera_topic, Image, self.image_callback, queue_size=1
        )
            
        rospy.loginfo("Red Dot Detector Node initialized")
        rospy.loginfo(f"Subscribed to: {self.camera_topic}")
        rospy.loginfo(f"Publishing to: target_center_diff")
        rospy.loginfo(f"Show image: {self.show_image}")
        rospy.loginfo(f"Detection mode: {'validated' if self.use_white_validation else 'basic'}")

    def load_parameters(self):
        """加载ROS参数"""
        # 相机话题
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        
        # 显示参数
        self.show_image = rospy.get_param('~show_image', False)
        
        # 红色检测参数
        self.red_hue_low1 = rospy.get_param('~red_hue_low1', 0)
        self.red_hue_high1 = rospy.get_param('~red_hue_high1', 10)
        self.red_hue_low2 = rospy.get_param('~red_hue_low2', 170)
        self.red_hue_high2 = rospy.get_param('~red_hue_high2', 180)
        self.red_sat_low = rospy.get_param('~red_sat_low', 50)
        self.red_val_low = rospy.get_param('~red_val_low', 50)
        
        # 白色检测参数
        self.white_sat_max = rospy.get_param('~white_sat_max', 50)
        self.white_val_min = rospy.get_param('~white_val_min', 150)
        self.white_threshold = rospy.get_param('~white_threshold', 0.3)
        
        # 检测参数
        self.min_contour_area = rospy.get_param('~min_contour_area', 10)
        self.use_white_validation = rospy.get_param('~use_white_validation', True)
        self.auto_fallback = rospy.get_param('~auto_fallback', True)
        
        # 图像处理参数
        self.gaussian_blur_size = rospy.get_param('~gaussian_blur_size', 5)
        
        
        # 构建HSV范围
        self.red_lower1 = np.array([self.red_hue_low1, self.red_sat_low, self.red_val_low])
        self.red_upper1 = np.array([self.red_hue_high1, 255, 255])
        self.red_lower2 = np.array([self.red_hue_low2, self.red_sat_low, self.red_val_low])
        self.red_upper2 = np.array([self.red_hue_high2, 255, 255])
        self.white_lower = np.array([0, 0, self.white_val_min])
        self.white_upper = np.array([255, self.white_sat_max, 255])
        

    def preprocess_image(self, frame):
        """图像预处理"""
        blur_size = max(1, self.gaussian_blur_size)
        if blur_size % 2 == 0:
            blur_size += 1  # 确保为奇数
        return cv2.GaussianBlur(frame, (blur_size, blur_size), 0)

    def detect_white_regions(self, frame):
        """检测白色区域"""
        enhanced = self.preprocess_image(frame)
        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, self.dilate_kernel)
        return white_mask

    def detect_red_regions_with_white_context(self, frame):
        """检测红色区域，利用白色底色信息进行筛选"""
        enhanced = self.preprocess_image(frame)
        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        
        # 创建红色掩码
        mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # 检测白色区域
        white_mask = self.detect_white_regions(frame)
        white_expanded = cv2.dilate(white_mask, self.dilate_kernel, iterations=1)
        
        # 只保留在白色区域附近的红色像素
        red_mask_filtered = cv2.bitwise_and(red_mask, white_expanded)
        red_mask_filtered = cv2.morphologyEx(red_mask_filtered, cv2.MORPH_CLOSE, self.kernel)
        red_mask_filtered = cv2.morphologyEx(red_mask_filtered, cv2.MORPH_OPEN, self.kernel)
        
        return red_mask_filtered, white_mask

    def detect_red_regions_basic(self, frame):
        """基础红色检测"""
        enhanced = self.preprocess_image(frame)
        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, self.kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, self.kernel)
        
        return red_mask

    def verify_red_dot_with_white_surround(self, frame, center, radius):
        """验证红点周围确实是白色"""
        if center[0] is None or center[1] is None:
            return False, 0
        
        cx, cy = center
        height, width = frame.shape[:2]
        mask = np.zeros((height, width), dtype=np.uint8)
        
        outer_radius = max(int(radius * 5), 50)
        inner_radius = max(int(radius * 1.5), 10)
        
        if (cx - outer_radius >= 0 and cx + outer_radius < width and 
            cy - outer_radius >= 0 and cy + outer_radius < height):
            
            cv2.circle(mask, (cx, cy), outer_radius, 255, -1)
            cv2.circle(mask, (cx, cy), inner_radius, 0, -1)
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            ring_pixels = hsv[mask > 0]
            
            if len(ring_pixels) > 0:
                ring_pixels_reshaped = ring_pixels.reshape(-1, 1, 3)
                white_mask_ring = cv2.inRange(ring_pixels_reshaped, self.white_lower, self.white_upper)
                white_ratio = np.sum(white_mask_ring > 0) / len(ring_pixels)
                return white_ratio > self.white_threshold, white_ratio
        
        return False, 0

    def find_red_dot_center_basic(self, red_mask):
        """基础红点检测"""
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None, 0
        
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_contour_area]
        if not valid_contours:
            return None, None, 0
        
        largest_contour = max(valid_contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy, area
        
        return None, None, 0

    def find_red_dot_center_with_validation(self, red_mask, white_mask, frame):
        """找到红点的中心位置，并验证周围的白色区域"""
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, None, 0, 0
        
        valid_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.min_contour_area:
                (cx, cy), radius = cv2.minEnclosingCircle(contour)
                cx, cy = int(cx), int(cy)
                
                is_valid, white_ratio = self.verify_red_dot_with_white_surround(
                    frame, (cx, cy), radius)
                
                if is_valid:
                    valid_contours.append((contour, area, white_ratio))
        
        if not valid_contours:
            return None, None, 0, 0
        
        best_contour, area, white_ratio = max(valid_contours, key=lambda x: x[2])
        
        M = cv2.moments(best_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return cx, cy, area, white_ratio
        
        return None, None, 0, 0

    def calculate_deviation_vector(self, red_center, image_center):
        """计算从图像中心到红点的向量"""
        if red_center[0] is None or red_center[1] is None:
            return None
        
        # 计算像素偏差
        dx = red_center[0] - image_center[0]
        dy = image_center[1] - red_center[1]  # Y轴向上为正
        
        # 距离
        distance_px = math.sqrt(dx**2 + dy**2)
        
        return dx, dy, distance_px

    def draw_debug_info(self, frame, red_center, image_center, dx, dy, distance_px, detection_mode, frame_rate):
        """绘制调试信息"""
        height, width = frame.shape[:2]
        
        # 绘制图像中心十字线
        cv2.line(frame, (0, image_center[1]), (width-1, image_center[1]), (0, 255, 0), 1)
        cv2.line(frame, (image_center[0], 0), (image_center[0], height-1), (0, 255, 0), 1)
        cv2.circle(frame, image_center, 3, (0, 255, 0), -1)
        
        if red_center[0] is not None and red_center[1] is not None:
            # 绘制红点中心
            color = (0, 255, 0) if detection_mode == "validated" else (0, 255, 255)
            cv2.circle(frame, red_center, 5, color, -1)
            cv2.circle(frame, red_center, 10, color, 2)
            
            # 绘制连接线
            cv2.line(frame, image_center, red_center, color, 2)
            
            # 显示偏差信息
            info_lines = [
                f"Resolution: {width}x{height} ({frame_rate:.1f}fps)",
                f"Red Center: ({red_center[0]}, {red_center[1]})",
                f"Vector: ({dx:.1f}, {dy:.1f})",
                f"Distance: {distance_px:.1f} px",
                f"Mode: {detection_mode}"
            ]
            
            for i, line in enumerate(info_lines):
                cv2.putText(frame, line, (10, 30 + i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            # 即使没有检测到红点，也显示帧率和分辨率信息
            info_lines = [
                f"Resolution: {width}x{height} ({frame_rate:.1f}fps)",
                "No red dot detected"
            ]
            for i, line in enumerate(info_lines):
                color = (255, 255, 255) if i == 0 else (0, 0, 255)
                cv2.putText(frame, line, (10, 30 + i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.6 if i == 0 else 0.8, color, 2)

    def image_callback(self, msg):
        """图像回调函数"""
        # 计算帧率
        current_time = time.time()
        if self.last_frame_time is not None:
            frame_time = current_time - self.last_frame_time
            if frame_time > 0:
                current_fps = 1.0 / frame_time
                # 使用指数移动平均平滑帧率
                self.frame_rate = self.fps_alpha * current_fps + (1 - self.fps_alpha) * self.frame_rate
        self.last_frame_time = current_time
        
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return
        
        # 计算图像中心
        height, width = cv_image.shape[:2]
        image_center = (width // 2, height // 2)
        
        # 红点检测
        red_center_x = None
        red_center_y = None
        detection_mode = "failed"
        red_mask = None
        white_mask = None
        
        if self.use_white_validation:
            # 先尝试带白色校验的检测
            red_mask, white_mask = self.detect_red_regions_with_white_context(cv_image)
            red_center_x, red_center_y, area, white_ratio = self.find_red_dot_center_with_validation(
                red_mask, white_mask, cv_image)
            
            if red_center_x is not None and red_center_y is not None:
                detection_mode = "validated"
            elif self.auto_fallback:
                # 降级到基础检测
                red_mask = self.detect_red_regions_basic(cv_image)
                red_center_x, red_center_y, area = self.find_red_dot_center_basic(red_mask)
                if red_center_x is not None and red_center_y is not None:
                    detection_mode = "basic"
                # 如果使用基础检测，需要重新生成白色掩码用于显示
                if white_mask is None:
                    white_mask = self.detect_white_regions(cv_image)
        else:
            # 只使用基础检测
            red_mask = self.detect_red_regions_basic(cv_image)
            red_center_x, red_center_y, area = self.find_red_dot_center_basic(red_mask)
            if red_center_x is not None and red_center_y is not None:
                detection_mode = "basic"
            # 生成白色掩码用于显示
            white_mask = self.detect_white_regions(cv_image)
        
        # 确保掩码存在（用于显示）
        if red_mask is None:
            red_mask = np.zeros((height, width), dtype=np.uint8)
        if white_mask is None:
            white_mask = np.zeros((height, width), dtype=np.uint8)
        
        # 计算并发布向量
        if red_center_x is not None and red_center_y is not None:
            red_center = (red_center_x, red_center_y)
            result = self.calculate_deviation_vector(red_center, image_center)
            
            if result is not None:
                dx, dy, distance_px = result
                
                # 发布Vector3消息
                vector_msg = Vector3()
                vector_msg.x = float(dy)
                vector_msg.y = float(-dx)
                vector_msg.z = float(distance_px)
                
                self.target_diff_pub.publish(vector_msg)
                
                rospy.logdebug(f"Published vector: ({dx:.1f}, {dy:.1f}, {distance_px:.1f}), mode: {detection_mode}")
        else:
            # 没有检测到红点，发布零向量
            vector_msg = Vector3()
            vector_msg.x = 0.0
            vector_msg.y = 0.0
            vector_msg.z = -1.0  # 使用负值表示未检测到
            
            self.target_diff_pub.publish(vector_msg)
            rospy.logdebug("No red dot detected, published zero vector")
        
        # 图像显示
        if self.show_image:
            debug_image = cv_image.copy()
            red_center = (red_center_x, red_center_y)
            
            if red_center_x is not None and red_center_y is not None:
                dx, dy, distance_px = self.calculate_deviation_vector(red_center, image_center)
            else:
                dx = dy = distance_px = 0
            
            self.draw_debug_info(debug_image, red_center, image_center, dx, dy, distance_px, detection_mode, self.frame_rate)
            
            cv2.imshow('Red Dot Detection', debug_image)
            cv2.imshow('Red Mask', red_mask)
            cv2.imshow('White Mask', white_mask)
            
            cv2.waitKey(1)

    def run(self):
        """运行节点"""
        rospy.loginfo("Red Dot Detector Node is running...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down Red Dot Detector Node")
        finally:
            if self.show_image:
                cv2.destroyAllWindows()


def main():
    try:
        node = RedDotDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
