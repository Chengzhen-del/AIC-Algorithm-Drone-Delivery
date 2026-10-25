#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import csv
import os
import threading
import sys
from bisect import bisect_right
from typing import List, Sequence, Tuple
from enum import Enum
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest

# Type definitions
Point2D = Tuple[float, float]

def load_path_csv(file_path: str) -> List[Point2D]:
    """
    读取 CSV 路径文件，支持列名包含 'x', 'y'，
    或者按 [index, x, y] 的无表头三列格式。
    """
    points: List[Point2D] = []
    with open(file_path, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        first_row = next(reader, None)
        if first_row is None:
            return points

        def try_parse_float(value: str) -> float:
            try:
                return float(value)
            except Exception:
                return math.nan

        # 判断是否为表头
        lower_first = [c.strip().lower() for c in first_row]
        header_like = ("x" in lower_first) and ("y" in lower_first)

        if header_like:
            x_idx = lower_first.index("x")
            y_idx = lower_first.index("y")
            for row in reader:
                if len(row) <= max(x_idx, y_idx):
                    continue
                x = try_parse_float(row[x_idx])
                y = try_parse_float(row[y_idx])
                if not (math.isnan(x) or math.isnan(y)):
                    points.append((x, y))
        else:
            # 将第一行也按数据读取
            if len(first_row) >= 3:
                x = try_parse_float(first_row[-2])
                y = try_parse_float(first_row[-1])
                if not (math.isnan(x) or math.isnan(y)):
                    points.append((x, y))
            for row in reader:
                if len(row) < 2:
                    continue
                # 允许 [x,y] 或 [idx,x,y]
                if len(row) == 2:
                    x = try_parse_float(row[0])
                    y = try_parse_float(row[1])
                else:
                    x = try_parse_float(row[-2])
                    y = try_parse_float(row[-1])
                if not (math.isnan(x) or math.isnan(y)):
                    points.append((x, y))

    return points

def euclidean_distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])

def cumulative_lengths(points: Sequence[Point2D]) -> List[float]:
    """返回累计弧长数组，长度与点数相同，起点为 0。"""
    if not points:
        return []
    cum: List[float] = [0.0]
    for i in range(len(points) - 1):
        cum.append(cum[-1] + euclidean_distance(points[i], points[i + 1]))
    return cum

def interpolate_along(points: Sequence[Point2D], cum: Sequence[float], s: float) -> Point2D:
    """
    在折线上按弧长参数 s 线性插值，s 将在 [0, total) 上取模。
    要求 len(points) == len(cum) 且非空。
    """
    if not points:
        raise ValueError("points 为空")
    if len(points) == 1:
        return points[0]
    total = cum[-1]
    if total <= 0:
        return points[0]
    s = s % total
    # 找到 s 所在段，使得 cum[idx-1] <= s < cum[idx]
    idx = bisect_right(cum, s)
    idx = min(max(1, idx), len(points) - 1)
    s0 = cum[idx - 1]
    s1 = cum[idx]
    if s1 == s0:
        return points[idx]
    t = (s - s0) / (s1 - s0)
    ax, ay = points[idx - 1]
    bx, by = points[idx]
    return (ax + t * (bx - ax), ay + t * (by - ay))

class FlightState(Enum):
    TAKEOFF = 1       # 起飞
    CIRCLE_FLIGHT = 2 # 圆形飞行
    GOTO_PATH_START = 3 # 前往路径起始点
    PATH_FOLLOWING = 4 # 路径跟随
    GOTO_HOVER_POSITIONS = 5 # 前往悬停位置
    HOVER_PHASE = 6   # 悬停阶段
    FORMATION_FLIGHT = 7 # 编队飞行
    RETURN_TO_HOVER = 8 # 返回悬停位置
    GOTO_LANDING = 9  # 前往降落点
    LANDING = 10      # 降落
    EMERGENCY_LANDING = 11  # 紧急降落

def keyboard_listener(controller):
    """键盘监听线程函数，监听紧急降落指令"""
    try:
        rospy.loginfo("Emergency landing keyboard listener started.")
        rospy.loginfo("Commands: Type 'e' + Enter for emergency landing, 'q' + Enter to quit.")
        
        while not rospy.is_shutdown() and not controller.emergency_landing_requested:
            try:
                # 使用input()等待用户输入，这不会干扰终端显示
                user_input = input().strip().lower()
                
                if user_input == 'e':
                    rospy.logwarn("EMERGENCY LANDING REQUESTED BY USER!")
                    controller.emergency_landing_requested = True
                    break
                elif user_input == 'q':
                    rospy.loginfo("User requested shutdown.")
                    rospy.signal_shutdown("User requested shutdown")
                    break
                elif user_input:  # 如果输入了其他内容
                    rospy.loginfo("Invalid command. Use 'e' for emergency landing, 'q' to quit.")
                    
            except EOFError:
                # 当stdin被关闭时（比如在后台运行）
                break
            except KeyboardInterrupt:
                # Ctrl+C被按下
                rospy.loginfo("Keyboard listener interrupted.")
                break
                    
    except Exception as e:
        rospy.logerr(f"Keyboard listener error: {e}")
    
    rospy.loginfo("Keyboard listener thread terminated.")

class Drone:
    def __init__(self, drone_name):
        self.name = f'/{drone_name}' if len(drone_name) > 0 else ''
        
        # 状态信息
        self.current_state = State()
        self.current_state.connected = False
        self.current_state.armed = False
        self.current_state.mode = ""
        
        # 位置信息
        self.current_pose = PoseStamped()
        self.vision_pose = PoseStamped()  # 视觉定位位置
        self.target_pose = PoseStamped()
        self.takeoff_pos = PoseStamped()
        self.landing_pos = PoseStamped()
        
        # 位置监控相关
        self.vision_pose_received = False
        self.last_position_check_time = rospy.Time.now()
        self.position_discrepancy_detected = False
        
        # ROS通信 - 自动初始化
        self.setup_ros_communication()
    
    def setup_ros_communication(self):
        """根据无人机名称自动设置ROS通信"""
        # 构建话题名称
        state_topic = f"{self.name}/mavros/state"
        pose_topic = f"{self.name}/mavros/local_position/pose"
        vision_pose_topic = f"{self.name}/mavros/vision_pose/pose"
        setpoint_topic = f"{self.name}/mavros/setpoint_position/local"
        arming_service = f"{self.name}/mavros/cmd/arming"
        setmode_service = f"{self.name}/mavros/set_mode"
        land_service = f"{self.name}/mavros/cmd/land"
        
        # 设置订阅者
        self.state_sub = rospy.Subscriber(
            state_topic, State, self.state_callback, queue_size=10)
        self.pose_sub = rospy.Subscriber(
            pose_topic, PoseStamped, self.pose_callback, queue_size=10)
        self.vision_pose_sub = rospy.Subscriber(
            vision_pose_topic, PoseStamped, self.vision_pose_callback, queue_size=10)
        
        # 设置发布者
        self.local_pos_pub = rospy.Publisher(
            setpoint_topic, PoseStamped, queue_size=10)
        
        # 设置服务客户端
        self.arming_client = rospy.ServiceProxy(arming_service, CommandBool)
        self.set_mode_client = rospy.ServiceProxy(setmode_service, SetMode)
        self.land_client = rospy.ServiceProxy(land_service, CommandTOL)
        
        rospy.loginfo(f"{self.name}: ROS communication setup completed")
    
    def state_callback(self, msg):
        """状态回调函数"""
        self.current_state = msg
    
    def pose_callback(self, msg):
        """位置回调函数"""
        self.current_pose = msg
    
    def vision_pose_callback(self, msg):
        """视觉定位位置回调函数"""
        self.vision_pose = msg
        self.vision_pose_received = True
    
    def publish_target_pose(self):
        """发布目标位置"""
        self.local_pos_pub.publish(self.target_pose)
    
    def set_offboard_mode(self):
        """设置OFFBOARD模式"""
        try:
            offb_set_mode = SetModeRequest()
            offb_set_mode.custom_mode = "OFFBOARD"
            response = self.set_mode_client(offb_set_mode)
            if response.mode_sent:
                rospy.loginfo(f"{self.name}: Offboard mode enabled")
                return True
        except rospy.ServiceException as e:
            rospy.logwarn(f"{self.name}: SetMode service call failed: {e}")
        return False
    
    def arm_vehicle(self):
        """解锁无人机"""
        try:
            arm_cmd = CommandBoolRequest()
            arm_cmd.value = True
            response = self.arming_client(arm_cmd)
            if response.success:
                rospy.loginfo(f"{self.name}: Vehicle armed")
                return True
        except rospy.ServiceException as e:
            rospy.logwarn(f"{self.name}: Arming service call failed: {e}")
        return False
    
    def land_vehicle(self):
        """降落无人机"""
        try:
            land_cmd = CommandTOLRequest()
            land_cmd.yaw = 0
            land_cmd.latitude = 0
            land_cmd.longitude = 0
            land_cmd.altitude = 0
            response = self.land_client(land_cmd)
            if response.success:
                rospy.loginfo(f"{self.name}: Landing command sent")
                return True
        except rospy.ServiceException as e:
            rospy.logwarn(f"{self.name}: Landing service call failed: {e}")
        return False
    
    def is_ready_for_takeoff(self):
        """检查是否准备好起飞"""
        return (self.current_state.connected and 
                self.current_state.armed and 
                self.current_state.mode == "OFFBOARD")
    
    def is_connected_and_armed(self):
        """检查是否连接并解锁"""
        return self.current_state.connected and self.current_state.armed
    
    def calculate_pose_distance(self, pose1, pose2):
        """计算两个位置之间的3D距离"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def check_position_discrepancy(self, threshold=2.0):
        """检查本地位置和视觉定位位置之间的差异"""
        if not self.vision_pose_received:
            return False
        
        distance = self.calculate_pose_distance(self.current_pose, self.vision_pose)
        
        if distance > threshold:
            if not self.position_discrepancy_detected:
                rospy.logerr(f"{self.name}: CRITICAL POSITION DISCREPANCY DETECTED!")
                rospy.logerr(f"{self.name}: Local position: ({self.current_pose.pose.position.x:.2f}, "
                           f"{self.current_pose.pose.position.y:.2f}, {self.current_pose.pose.position.z:.2f})")
                rospy.logerr(f"{self.name}: Vision position: ({self.vision_pose.pose.position.x:.2f}, "
                           f"{self.vision_pose.pose.position.y:.2f}, {self.vision_pose.pose.position.z:.2f})")
                rospy.logerr(f"{self.name}: Distance difference: {distance:.2f}m (threshold: {threshold:.2f}m)")
                self.position_discrepancy_detected = True
            return True
        else:
            if self.position_discrepancy_detected:
                rospy.loginfo(f"{self.name}: Position discrepancy resolved. Distance now: {distance:.2f}m")
                self.position_discrepancy_detected = False
            return False

class MultiDroneController:
    def __init__(self):
        rospy.init_node('multi_drone_control', anonymous=True)
        
        # 飞行参数 - 从ROS参数获取
        self.circle_radius = rospy.get_param('~circle_radius', 1.5)  # 圆形飞行半径
        self.flight_height = rospy.get_param('~flight_height', 1.3)  # 飞行高度
        self.circle_speed = rospy.get_param('~circle_speed', 0.5)    # 圆形飞行角速度 (rad/s)
        self.circle_flight_laps = rospy.get_param('~circle_flight_laps', 3)  # 圆形飞行圈数
        
        # 状态变量
        self.flight_state = FlightState.TAKEOFF
        self.flight_start_time = None
        
        # 紧急降落标志
        self.emergency_landing_requested = False
        self.emergency_landing_start_time = None
        
        # 圆形飞行状态跟踪（稍后根据实际无人机数量初始化）
        self.circle_initial_angles = []
        self.circle_completed_laps = []
        self.circle_previous_angles = []
        
        # 从ROS参数获取参与飞行的无人机列表
        # active_drones_param = rospy.get_param('~active_drones', ['drone001', 'drone003', 'drone008'])
        active_drones_param = rospy.get_param('~active_drones', [])
        if isinstance(active_drones_param, str):
            # 如果是字符串，按逗号分割
            self.active_drone_names = [name.strip() for name in active_drones_param.split(',')]
        else:
            # 如果是列表
            self.active_drone_names = active_drones_param
        
        # 初始化参与飞行的无人机
        self.drones = []
        if len(self.active_drone_names) > 0:
            for drone_name in self.active_drone_names:
                self.drones.append(Drone(drone_name))
        else:
            self.drones.append(Drone(''))

        # 根据无人机数量动态计算角度偏移和状态跟踪
        num_drones = len(self.drones)
        if num_drones > 0:
            self.angle_offset = [i * 2 * math.pi / num_drones for i in range(num_drones)]
            # 初始化圆形飞行状态跟踪
            self.circle_initial_angles = [i * 2 * math.pi / num_drones for i in range(num_drones)]
            self.circle_completed_laps = [0.0] * num_drones
            self.circle_previous_angles = [0.0] * num_drones
        else:
            self.angle_offset = []
            rospy.logerr("No active drones configured!")
            return
        
        # 路径跟随相关
        self.paths = []  # 存储三个路径
        self.cum_lengths = []  # 存储累积长度
        self.path_totals = []  # 存储路径总长度
        self.path_start_positions = []  # 存储路径起始点位置
        
        # 从ROS参数获取路径配置
        self.path_cycle_time = rospy.get_param('~path_cycle_time', 20.0)  # 一个周期的时间（秒）
        self.max_path_cycles = rospy.get_param('~max_path_cycles', 5)  # 飞行周期数
        self.path_start_hover_time = rospy.get_param('~path_start_hover_time', 3.0)  # 在路径起始点悬停时间（秒）
        
        # 相位差配置 - 根据无人机数量动态分配
        start_phase = rospy.get_param('~start_phase', 1.0/4.0)  # 起始相位
        
        # 可以通过参数自定义相位延迟，如果没有则平均分配
        custom_phases = rospy.get_param('~custom_phases', [])
        
        if custom_phases and len(custom_phases) >= num_drones:
            # 使用自定义相位
            self.path_phases = custom_phases[:num_drones]
        else:
            # 根据无人机数量平均分配相位延迟
            phase_interval = rospy.get_param('~phase_interval', 1.0/6.0)  # 相位间隔
            self.path_phases = []
            for i in range(num_drones):
                phase = start_phase - i * phase_interval
                self.path_phases.append(phase)
        
        self.path_start_time = None
        self.path_cycles_completed = 0
        self.drone_cycles_completed = []  # 记录每个无人机完成的周期数
        self.drone_last_cycle_time = []   # 记录每个无人机最后一次完成周期的时间
        
        # 前往路径起始点相关
        self.goto_path_start_time = None
        self.path_start_hover_start_time = None
        
        # 悬停位置相关
        self.hover_positions = []  # 存储悬停位置
        self.goto_hover_start_time = None
        self.hover_start_time = None
        self.hover_duration = 3.0  # 悬停3秒
        
        # 编队飞行相关
        self.formation_waypoints = [
            (1.8, 1.0, 1.6),
            (1.8, -1.0, 1.6),
            (-0.5, 1.0, 1.0),
            (-0.5, -1.0, 1.0),
            (1, 0, 1.4)  # 最后回到起始悬停位置
        ]
        self.current_waypoint_index = 0
        self.formation_start_time = None
        self.waypoint_reach_time = None
        self.formation_waypoint_hover_duration = 2.0  # 在每个航点悬停2秒
        self.return_to_hover_start_time = None
        
        rospy.loginfo(f"Active drones: {self.active_drone_names} (total: {num_drones})")
        rospy.loginfo(f"Flight parameters - Circle radius: {self.circle_radius}m, Height: {self.flight_height}m, Speed: {self.circle_speed} rad/s, Circle laps: {self.circle_flight_laps}")
        rospy.loginfo(f"Path cycle time: {self.path_cycle_time}s")
        rospy.loginfo(f"Max path cycles: {self.max_path_cycles}")
        rospy.loginfo(f"Path start hover time: {self.path_start_hover_time}s")
        rospy.loginfo(f"Path phases: {self.path_phases}")
        
        # 控制频率
        self.rate = rospy.Rate(60.0)
        
        # 安全监控时间
        self.last_safety_check = rospy.Time.now()
        
        # 初始化悬停位置
        self.setup_hover_positions()
        
        rospy.loginfo("Multi-drone controller initialized with encapsulated communication and parameter-based configuration")
    
    def setup_hover_positions(self):
        """设置悬停位置"""
        # 定义固定的悬停位置：(1,0,1.4), (-0.5,0.7,1), (-0.5,-0.7,1)
        predefined_positions = [
            (1.0, 0.0, 1.4),
            (-0.5, 0.7, 1.4),
            (-0.5, -0.7, 1.4)
        ]
        
        num_drones = len(self.drones)
        self.hover_positions = []
        
        for i in range(num_drones):
            if i < len(predefined_positions):
                # 使用预定义位置
                x, y, z = predefined_positions[i]
            else:
                # 如果无人机数量超过预定义位置，使用默认位置
                angle = i * 2 * math.pi / num_drones
                x = 1.5 * math.cos(angle)
                y = 1.5 * math.sin(angle)
                z = 1.2
            
            hover_pos = PoseStamped()
            hover_pos.pose.position.x = x
            hover_pos.pose.position.y = y
            hover_pos.pose.position.z = z
            self.hover_positions.append(hover_pos)
            
            rospy.loginfo(f"Drone {i} ({self.drones[i].name}) hover position: ({x:.2f}, {y:.2f}, {z:.2f})")
    
    def load_paths(self):
        """加载路径文件"""
        # 从ROS参数中获取路径文件目录
        path_file_dir = rospy.get_param('~path_file_dir', '.')
        rospy.loginfo(f"Path file directory: {path_file_dir}")
        
        # 根据实际参与飞行的无人机生成路径文件列表
        path_files = []
        for drone_index, drone in enumerate(self.drones):
            path_files.append(os.path.join(path_file_dir, f"path_{drone_index+1}.csv"))
        
        self.paths = []
        self.cum_lengths = []
        self.path_totals = []
        self.path_start_positions = []
        
        for i, path_file in enumerate(path_files):
            if os.path.exists(path_file):
                try:
                    points = load_path_csv(path_file)
                    if len(points) >= 2:
                        cum_length = cumulative_lengths(points)
                        total_length = cum_length[-1] if cum_length else 0.0
                        
                        if total_length > 0:
                            self.paths.append(points)
                            self.cum_lengths.append(cum_length)
                            self.path_totals.append(total_length)
                            # 根据起始相位计算路径起始点
                            phase = self.path_phases[0]
                            phase_fraction = phase % 1.0  # 确保相位在0-1范围内
                            s = phase_fraction * total_length
                            px, py = interpolate_along(points, cum_length, s)
                            start_pos = PoseStamped()
                            start_pos.pose.position.x = px
                            start_pos.pose.position.y = py
                            start_pos.pose.position.z = self.flight_height
                            self.path_start_positions.append(start_pos)
                            rospy.loginfo(f"Loaded path {i}: {path_file} with {len(points)} points, total length: {total_length:.2f}")
                            rospy.loginfo(f"Path {i} start position with phase {phase:.3f}: ({px:.2f}, {py:.2f}, {self.flight_height:.2f})")
                        else:
                            rospy.logwarn(f"Path {i} has zero length: {path_file}")
                            self.paths.append([])
                            self.cum_lengths.append([])
                            self.path_totals.append(0.0)
                            # 添加默认起始位置
                            default_start_pos = PoseStamped()
                            default_start_pos.pose.position.x = 0.0
                            default_start_pos.pose.position.y = 0.0
                            default_start_pos.pose.position.z = self.flight_height
                            self.path_start_positions.append(default_start_pos)
                    else:
                        rospy.logwarn(f"Path {i} has insufficient points: {path_file}")
                        self.paths.append([])
                        self.cum_lengths.append([])
                        self.path_totals.append(0.0)
                        # 添加默认起始位置
                        default_start_pos = PoseStamped()
                        default_start_pos.pose.position.x = 0.0
                        default_start_pos.pose.position.y = 0.0
                        default_start_pos.pose.position.z = self.flight_height
                        self.path_start_positions.append(default_start_pos)
                except Exception as e:
                    rospy.logerr(f"Failed to load path {i}: {path_file}, error: {e}")
                    self.paths.append([])
                    self.cum_lengths.append([])
                    self.path_totals.append(0.0)
                    # 添加默认起始位置
                    default_start_pos = PoseStamped()
                    default_start_pos.pose.position.x = 0.0
                    default_start_pos.pose.position.y = 0.0
                    default_start_pos.pose.position.z = self.flight_height
                    self.path_start_positions.append(default_start_pos)
            else:
                rospy.logwarn(f"Path file not found: {path_file}, using default circular path")
                # 创建默认圆形路径
                default_points = []
                for j in range(36):  # 36个点，每10度一个
                    angle = j * math.pi / 18  # 10度 = pi/18
                    x = 2.0 * math.cos(angle)
                    y = 2.0 * math.sin(angle)
                    default_points.append((x, y))
                
                cum_length = cumulative_lengths(default_points)
                total_length = cum_length[-1] if cum_length else 0.0
                
                self.paths.append(default_points)
                self.cum_lengths.append(cum_length)
                self.path_totals.append(total_length)
                # 根据起始相位计算默认路径起始点
                phase = self.path_phases[i] if i < len(self.path_phases) else 0.0
                phase_fraction = phase % 1.0  # 确保相位在0-1范围内
                s = phase_fraction * total_length
                px, py = interpolate_along(default_points, cum_length, s)
                start_pos = PoseStamped()
                start_pos.pose.position.x = px
                start_pos.pose.position.y = py
                start_pos.pose.position.z = self.flight_height
                self.path_start_positions.append(start_pos)
                rospy.loginfo(f"Created default circular path {i} with {len(default_points)} points, total length: {total_length:.2f}")
                rospy.loginfo(f"Default path {i} start position with phase {phase:.3f}: ({px:.2f}, {py:.2f}, {self.flight_height:.2f})")
        
        # 确保有足够数量的路径（与活跃无人机数量一致）
        while len(self.paths) < len(self.drones):
            self.paths.append([])
            self.cum_lengths.append([])
            self.path_totals.append(0.0)
            # 添加默认起始位置
            default_start_pos = PoseStamped()
            default_start_pos.pose.position.x = 0.0
            default_start_pos.pose.position.y = 0.0
            default_start_pos.pose.position.z = self.flight_height
            self.path_start_positions.append(default_start_pos)
    
    def calculate_distance(self, drone1, drone2):
        """计算两个无人机之间的距离"""
        dx = drone1.current_pose.pose.position.x - drone2.current_pose.pose.position.x
        dy = drone1.current_pose.pose.position.y - drone2.current_pose.pose.position.y
        dz = drone1.current_pose.pose.position.z - drone2.current_pose.pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def calculate_2d_distance(self, pos1, pos2):
        """计算2D距离"""
        dx = pos1.pose.position.x - pos2.pose.position.x
        dy = pos1.pose.position.y - pos2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def reached_target(self, drone, tolerance=0.3):
        """检查是否到达目标位置"""
        dx = drone.current_pose.pose.position.x - drone.target_pose.pose.position.x
        dy = drone.current_pose.pose.position.y - drone.target_pose.pose.position.y
        dz = drone.current_pose.pose.position.z - drone.target_pose.pose.position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        return distance < tolerance
    
    def check_collision_risk(self, drone_index, safety_distance=0.6):
        """检查是否存在碰撞风险"""
        for i, drone in enumerate(self.drones):
            if i != drone_index:
                distance = self.calculate_distance(self.drones[drone_index], drone)
                if distance < safety_distance:
                    return True
        return False
    
    def calculate_safe_position(self, drone_index, desired_pos):
        """计算避碰后的安全位置"""
        safe_pos = PoseStamped()
        safe_pos.pose.position.x = desired_pos.pose.position.x
        safe_pos.pose.position.y = desired_pos.pose.position.y
        safe_pos.pose.position.z = desired_pos.pose.position.z
        
        # 检查与其他无人机的距离
        for i, drone in enumerate(self.drones):
            if i != drone_index:
                dx = desired_pos.pose.position.x - drone.current_pose.pose.position.x
                dy = desired_pos.pose.position.y - drone.current_pose.pose.position.y
                distance_2d = math.sqrt(dx*dx + dy*dy)
                
                # 如果距离太近，调整位置
                if distance_2d < 0.75:  # 安全距离1.2米
                    angle = math.atan2(dy, dx)
                    safe_pos.pose.position.x = drone.current_pose.pose.position.x + 0.85 * math.cos(angle)
                    safe_pos.pose.position.y = drone.current_pose.pose.position.y + 0.85 * math.sin(angle)
                    rospy.logwarn(f"Collision risk detected between {self.drones[drone_index].name} and {drone.name}, distance: {distance_2d:.2f}m")
        
        return safe_pos
    
    def all_drones_ready(self):
        """检查所有无人机是否都连接并解锁"""
        return all(drone.is_connected_and_armed() for drone in self.drones)
    
    def all_drones_ready_for_takeoff(self):
        """检查所有无人机是否都连接并在offboard模式"""
        return all(drone.is_ready_for_takeoff() for drone in self.drones)
    
    def all_drones_connected(self):
        """检查所有无人机是否都连接"""
        return all(drone.current_state.connected for drone in self.drones)
    
    def check_all_drones_position_discrepancy(self):
        """检查所有无人机的位置差异，如果有任何一架超过阈值则触发紧急降落"""
        for drone in self.drones:
            if drone.check_position_discrepancy(threshold=1.0):
                rospy.logerr(f"EMERGENCY TRIGGERED: {drone.name} position discrepancy exceeds 1.0m threshold!")
                return True
        return False
    
    def calculate_formation_positions(self, leader_pos):
        """根据1号机（leader）位置计算其他无人机的编队位置"""
        formation_positions = []
        
        if len(self.drones) == 0:
            return formation_positions
        
        # 计算相对于悬停位置的偏移量
        if len(self.hover_positions) > 0:
            leader_hover_pos = self.hover_positions[0]  # 1号机的悬停位置
            
            for i, drone in enumerate(self.drones):
                if i == 0:
                    # 1号机（leader）使用指定位置
                    pos = PoseStamped()
                    pos.pose.position.x = leader_pos[0]
                    pos.pose.position.y = leader_pos[1]
                    pos.pose.position.z = leader_pos[2]
                    formation_positions.append(pos)
                else:
                    # 其他无人机保持相对位置
                    if i < len(self.hover_positions):
                        hover_offset_x = self.hover_positions[i].pose.position.x - leader_hover_pos.pose.position.x
                        hover_offset_y = self.hover_positions[i].pose.position.y - leader_hover_pos.pose.position.y
                        hover_offset_z = self.hover_positions[i].pose.position.z - leader_hover_pos.pose.position.z
                        
                        pos = PoseStamped()
                        pos.pose.position.x = leader_pos[0] + hover_offset_x
                        pos.pose.position.y = leader_pos[1] + hover_offset_y
                        pos.pose.position.z = leader_pos[2] + hover_offset_z
                        formation_positions.append(pos)
                    else:
                        # 如果没有对应的悬停位置，保持在leader附近
                        pos = PoseStamped()
                        pos.pose.position.x = leader_pos[0] + (i-1) * 0.5
                        pos.pose.position.y = leader_pos[1]
                        pos.pose.position.z = leader_pos[2]
                        formation_positions.append(pos)
        
        return formation_positions
    
    def all_drones_back_to_path_start(self, tolerance=0.4):
        """检查所有无人机是否都回到了各自的路径起始点"""
        for i, drone in enumerate(self.drones):
            if i < len(self.path_start_positions):
                # 计算当前位置与路径起始点的距离
                dx = drone.current_pose.pose.position.x - self.path_start_positions[i].pose.position.x
                dy = drone.current_pose.pose.position.y - self.path_start_positions[i].pose.position.y
                dz = drone.current_pose.pose.position.z - self.path_start_positions[i].pose.position.z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if distance > tolerance:
                    return False
            else:
                # 如果没有对应的起始位置，则认为已到达
                continue
        return True
    
    def all_drones_reached_target(self):
        """检查所有无人机是否都到达目标位置"""
        for drone in self.drones:
            if not self.reached_target(drone):
                return False
        return True
    
    def calculate_takeoff_positions(self):
        """计算安全的起飞位置"""
        takeoff_radius = 1.0  # 起飞圆形半径，确保足够的分离距离
        num_drones = len(self.drones)
        
        # 根据无人机数量动态分配起飞位置
        for i, drone in enumerate(self.drones):
            angle = i * 2 * math.pi / num_drones  # 平均分布在圆周上
            drone.takeoff_pos.pose.position.x = takeoff_radius * math.cos(angle)
            drone.takeoff_pos.pose.position.y = takeoff_radius * math.sin(angle)
            drone.takeoff_pos.pose.position.z = self.flight_height
            
            rospy.loginfo(f"{drone.name} takeoff position: ({drone.takeoff_pos.pose.position.x:.2f}, "
                         f"{drone.takeoff_pos.pose.position.y:.2f}, {drone.takeoff_pos.pose.position.z:.2f})")
    
    def update_circle_laps(self, drone_index, current_angle):
        """更新圆形飞行圈数"""
        previous_angle = self.circle_previous_angles[drone_index]
        
        # 检测是否跨越了0度（完成一圈）
        if previous_angle > 5.0 and current_angle < 1.0:  # 从接近2π跳到接近0
            self.circle_completed_laps[drone_index] += 1.0
            rospy.loginfo(f"{self.drones[drone_index].name} completed lap {self.circle_completed_laps[drone_index]}")
        
        self.circle_previous_angles[drone_index] = current_angle
    
    def all_drones_completed_laps(self):
        """检查是否所有无人机都完成了指定圈数"""
        return all(laps >= self.circle_flight_laps for laps in self.circle_completed_laps)
    
    def all_drones_near_initial_position(self, tolerance=0.3):
        """检查是否所有无人机都接近初始位置"""
        for i, drone in enumerate(self.drones):
            # 计算当前角度
            current_angle = math.atan2(
                drone.current_pose.pose.position.y,
                drone.current_pose.pose.position.x
            )
            if current_angle < 0:
                current_angle += 2 * math.pi
            
            # 计算与初始角度的差异
            initial_angle = self.circle_initial_angles[i]
            angle_diff = abs(current_angle - initial_angle)
            # 处理角度跨越0度的情况
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # 如果任何一架无人机距离初始位置太远，返回False
            if angle_diff > tolerance:
                return False
        
        return True

    def assign_nearest_landing_positions(self):
        """根据当前位置分配最近的降落位置"""
        landing_radius = 1.5  # 降落圆形半径
        num_drones = len(self.drones)
        
        # 根据无人机数量创建可能的降落位置
        landing_candidates = []
        for i in range(num_drones):
            angle = i * 2 * math.pi / num_drones  # 平均分布在圆周上
            pos = PoseStamped()
            pos.pose.position.x = landing_radius * math.cos(angle)
            pos.pose.position.y = landing_radius * math.sin(angle)
            pos.pose.position.z = 0
            landing_candidates.append(pos)
        
        # 跟踪哪些位置已被分配
        position_assigned = [False] * num_drones
        
        # 为每架无人机分配最近的未被占用的降落位置
        for drone_idx, drone in enumerate(self.drones):
            min_distance = float('inf')
            best_position = -1
            
            # 找到距离当前位置最近的未分配位置
            for pos_idx, candidate in enumerate(landing_candidates):
                if not position_assigned[pos_idx]:
                    distance = self.calculate_2d_distance(drone.current_pose, candidate)
                    if distance < min_distance:
                        min_distance = distance
                        best_position = pos_idx
            
            # 分配最近的位置
            if best_position != -1:
                self.drones[drone_idx].landing_pos = landing_candidates[best_position]
                position_assigned[best_position] = True
                
                rospy.loginfo(f"{drone.name} assigned to nearest landing position {best_position}: "
                             f"({drone.landing_pos.pose.position.x:.2f}, {drone.landing_pos.pose.position.y:.2f}, "
                             f"{drone.landing_pos.pose.position.z:.2f}), distance: {min_distance:.2f} m")
    
    def wait_for_connection(self):
        """等待所有飞控连接"""
        rospy.loginfo("Waiting for all drones to connect...")
        while not rospy.is_shutdown():
            if self.all_drones_connected():
                rospy.loginfo("All drones connected!")
                break
            self.rate.sleep()
    
    def pre_publish_targets(self):
        """预发布目标位置"""
        for i in range(100):
            if rospy.is_shutdown():
                break
            for drone in self.drones:
                drone.publish_target_pose()
            self.rate.sleep()
    
    def setup_flight_mode(self):
        """设置飞行模式和解锁"""
        last_request = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            # 对每架无人机进行模式设置和解锁
            for drone in self.drones:
                if (drone.current_state.mode != "OFFBOARD" and 
                    (current_time - last_request).to_sec() > 5.0):
                    drone.set_offboard_mode()
                    last_request = current_time
                    
                elif (not drone.current_state.armed and 
                      (current_time - last_request).to_sec() > 5.0):
                    drone.arm_vehicle()
                    last_request = current_time
            
            # 如果所有无人机都准备好了，就退出设置循环
            if self.all_drones_ready_for_takeoff():
                break
            
            # 发布目标位置
            for drone in self.drones:
                drone.publish_target_pose()
            
            self.rate.sleep()
    
    def execute_flight_state_machine(self):
        """执行飞行状态机"""
        last_ready_check = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            # 检查紧急降落请求（包括位置差异触发）
            position_discrepancy_emergency = self.check_all_drones_position_discrepancy()
            
            if (self.emergency_landing_requested or position_discrepancy_emergency) and self.flight_state != FlightState.EMERGENCY_LANDING:
                if position_discrepancy_emergency:
                    rospy.logerr("EMERGENCY LANDING TRIGGERED BY POSITION DISCREPANCY! Switching all drones to emergency landing mode.")
                else:
                    rospy.logwarn("EMERGENCY LANDING INITIATED BY USER! Switching all drones to emergency landing mode.")
                
                self.flight_state = FlightState.EMERGENCY_LANDING
                self.emergency_landing_start_time = current_time
                
                # 设置所有无人机的紧急降落位置（当前位置）
                for drone in self.drones:
                    drone.landing_pos.pose.position.x = drone.current_pose.pose.position.x
                    drone.landing_pos.pose.position.y = drone.current_pose.pose.position.y
                    drone.landing_pos.pose.position.z = 0  # 降落到地面
                    rospy.logwarn(f"EMERGENCY: {drone.name} emergency landing at current position: "
                                f"({drone.landing_pos.pose.position.x:.2f}, {drone.landing_pos.pose.position.y:.2f}, 0)")
            
            # 飞行状态机
            if self.flight_state == FlightState.TAKEOFF:
                # 确保所有无人机都准备好后再同时开始起飞
                if self.all_drones_ready_for_takeoff():
                    # 所有无人机同时起飞到各自的指定位置，使用碰撞检测
                    for i, drone in enumerate(self.drones):
                        # 使用预定义的起飞位置
                        desired_pos = drone.takeoff_pos
                        
                        # 检查碰撞风险并计算安全位置
                        drone.target_pose = self.calculate_safe_position(i, desired_pos)
                        
                        # 检查碰撞风险并记录
                        # if self.check_collision_risk(i, 0.6):
                        #     rospy.logwarn(f"{drone.name}: Collision risk detected during takeoff, adjusting position")
                    
                    # 检查是否所有无人机都到达起飞位置
                    if self.all_drones_reached_target():
                        rospy.loginfo("All drones reached their takeoff positions simultaneously, starting circle flight...")
                        self.flight_state = FlightState.CIRCLE_FLIGHT
                        self.flight_start_time = current_time
                        
                        # 初始化圆形飞行状态
                        num_drones = len(self.drones)
                        self.circle_completed_laps = [0.0] * num_drones
                        self.circle_previous_angles = [angle for angle in self.circle_initial_angles]
                        rospy.loginfo(f"Circle flight initialized - target laps: {self.circle_flight_laps}")
                else:
                    # 等待所有无人机准备就绪
                    if (current_time - last_ready_check).to_sec() > 2.0:
                        rospy.loginfo("Waiting for all drones to be ready for simultaneous takeoff...")
                        last_ready_check = current_time
            
            elif self.flight_state == FlightState.CIRCLE_FLIGHT:
                elapsed_time = (current_time - self.flight_start_time).to_sec()
                
                # 计算每架无人机的圆形飞行位置，并应用碰撞避免
                for i, drone in enumerate(self.drones):
                    # 计算期望位置
                    angle = self.circle_speed * elapsed_time + self.angle_offset[i]
                    # 确保角度在0-2π范围内
                    angle = angle % (2 * math.pi)
                    
                    desired_pos = PoseStamped()
                    desired_pos.pose.position.x = self.circle_radius * math.cos(angle)
                    desired_pos.pose.position.y = self.circle_radius * math.sin(angle)
                    desired_pos.pose.position.z = self.flight_height
                    
                    # 更新圈数计算
                    self.update_circle_laps(i, angle)
                    
                    # 检查碰撞风险并计算安全位置
                    drone.target_pose = self.calculate_safe_position(i, desired_pos)
                    
                    # 监控距离并报告
                    # for j in range(i + 1, len(self.drones)):
                    #     distance = self.calculate_distance(drone, self.drones[j])
                    #     if distance < 1.5:
                    #         rospy.loginfo(f"Distance between {drone.name} and {self.drones[j].name}: {distance:.2f} m")
                
                # 检查圆形飞行是否完毕
                if self.all_drones_completed_laps():
                    # 所有无人机都完成了指定圈数
                    if self.all_drones_near_initial_position(tolerance=0.5):
                        # 都接近初始位置，可以退出
                        rospy.loginfo("All drones completed required laps and returned to initial positions, starting to go to path start positions...")
                        self.flight_state = FlightState.GOTO_PATH_START
                        self.goto_path_start_time = current_time
                    else:
                        # 继续转圈直到回到初始位置附近
                        rospy.loginfo_throttle(2.0, "All drones completed required laps, waiting to return to initial positions...")
            
            elif self.flight_state == FlightState.GOTO_PATH_START:
                # 前往路径起始点阶段
                elapsed_time = (current_time - self.goto_path_start_time).to_sec()
                
                # 计算每架无人机前往对应路径起始点的位置
                for i, drone in enumerate(self.drones):
                    if i < len(self.path_start_positions):
                        # 设置目标为路径起始点
                        desired_pos = self.path_start_positions[i]
                        
                        # 检查碰撞风险并计算安全位置
                        drone.target_pose = self.calculate_safe_position(i, desired_pos)
                        
                        # 监控距离并报告
                        for j in range(i + 1, len(self.drones)):
                            distance = self.calculate_distance(drone, self.drones[j])
                            if distance < 1.5:
                                rospy.loginfo(f"GOTO_PATH_START: Distance between {drone.name} and {self.drones[j].name}: {distance:.2f} m")
                    else:
                        # 如果没有对应的起始位置，保持当前位置
                        rospy.logwarn(f"{drone.name}: No path start position available, maintaining current position")
                
                # 检查是否所有无人机都到达路径起始点
                if self.all_drones_reached_target() and self.path_start_hover_start_time is None:
                    rospy.loginfo("All drones reached path start positions, starting hover...")
                    self.path_start_hover_start_time = current_time
                
                # 如果已经开始悬停，检查悬停时间
                if self.path_start_hover_start_time is not None:
                    hover_elapsed = (current_time - self.path_start_hover_start_time).to_sec()
                    rospy.loginfo_throttle(1.0, f"Hovering at path start positions... {hover_elapsed:.1f}/{self.path_start_hover_time:.1f}s")
                    
                    if hover_elapsed >= self.path_start_hover_time:
                        rospy.loginfo("Hover complete, starting path following...")
                        self.flight_state = FlightState.PATH_FOLLOWING
                        self.path_start_time = current_time
                        
                        # 重置状态变量
                        self.path_cycles_completed = 0
                        self.drone_cycles_completed = [0] * len(self.drones)
                        self.drone_last_cycle_time = [current_time] * len(self.drones)
            
            elif self.flight_state == FlightState.PATH_FOLLOWING:
                # 路径跟随阶段
                if self.path_start_time is not None:
                    # 检查是否所有无人机都完成了指定周期且回到起始点
                    all_completed = True
                    for i, drone in enumerate(self.drones):
                        if self.drone_cycles_completed[i] < self.max_path_cycles:
                            all_completed = False
                            break
                    
                    if all_completed and self.all_drones_back_to_path_start():
                        rospy.loginfo("All drones completed path cycles and returned to start positions, transitioning to hover positions...")
                        self.flight_state = FlightState.GOTO_HOVER_POSITIONS
                        self.goto_hover_start_time = current_time
                    else:
                        # 定期报告进度
                        if (current_time.to_sec() % 5.0) < 0.1:  # 每5秒报告一次
                            for i, drone in enumerate(self.drones):
                                cycles_info = f"{self.drone_cycles_completed[i]}/{self.max_path_cycles}"
                                if i < len(self.path_start_positions):
                                    dx = drone.current_pose.pose.position.x - self.path_start_positions[i].pose.position.x
                                    dy = drone.current_pose.pose.position.y - self.path_start_positions[i].pose.position.y
                                    dz = drone.current_pose.pose.position.z - self.path_start_positions[i].pose.position.z
                                    dist_to_start = math.sqrt(dx*dx + dy*dy + dz*dz)
                                    rospy.loginfo_throttle(5.0, f"{drone.name}: Cycles {cycles_info}, Distance to start: {dist_to_start:.2f}m")
                        
                        # 计算每个无人机在路径上的位置
                        for i, drone in enumerate(self.drones):
                            if i < len(self.paths) and self.path_totals[i] > 0:
                                # 计算该无人机的路径跟随时间
                                drone_elapsed_time = (current_time - self.drone_last_cycle_time[i]).to_sec()
                                
                                # 如果该无人机还需要继续路径跟随
                                if self.drone_cycles_completed[i] < self.max_path_cycles:
                                    # 计算当前周期进度
                                    cycle_progress = drone_elapsed_time / self.path_cycle_time
                                    current_cycle = int(cycle_progress)
                                    
                                    # 检查是否完成了新的周期
                                    if current_cycle > 0 and cycle_progress >= 1.0:
                                        self.drone_cycles_completed[i] += 1
                                        self.drone_last_cycle_time[i] = current_time
                                        rospy.loginfo(f"{drone.name}: Completed cycle {self.drone_cycles_completed[i]}/{self.max_path_cycles}")
                                        drone_elapsed_time = 0  # 重置时间
                                        cycle_progress = 0
                                    
                                    # 计算路径位置
                                    if self.drone_cycles_completed[i] < self.max_path_cycles:
                                        # 计算当前周期内的进度 (0-1)
                                        cycle_fraction = cycle_progress
                                        
                                        # 应用相位差
                                        phase_fraction = (cycle_fraction + self.path_phases[i]) % 1.0
                                        
                                        # 计算路径上的位置
                                        s = phase_fraction * self.path_totals[i]
                                        px, py = interpolate_along(self.paths[i], self.cum_lengths[i], s)
                                        
                                        # 设置目标位置
                                        desired_pos = PoseStamped()
                                        desired_pos.pose.position.x = px
                                        desired_pos.pose.position.y = py
                                        desired_pos.pose.position.z = self.flight_height
                                        
                                        # 应用碰撞避免
                                        drone.target_pose = self.calculate_safe_position(i, desired_pos)
                                    else:
                                        # 已完成所有周期，前往起始点
                                        if i < len(self.path_start_positions):
                                            desired_pos = self.path_start_positions[i]
                                            drone.target_pose = self.calculate_safe_position(i, desired_pos)
                                else:
                                    # 已完成所有周期，前往并保持在起始点
                                    if i < len(self.path_start_positions):
                                        desired_pos = self.path_start_positions[i]
                                        drone.target_pose = self.calculate_safe_position(i, desired_pos)
                            else:
                                # 如果没有路径，保持当前位置
                                rospy.logwarn_throttle(5.0, f"{drone.name}: No valid path, maintaining current position")
            
            elif self.flight_state == FlightState.GOTO_HOVER_POSITIONS:
                # 前往悬停位置阶段
                for i, drone in enumerate(self.drones):
                    if i < len(self.hover_positions):
                        # 设置目标为悬停位置
                        desired_pos = self.hover_positions[i]
                        
                        # 检查碰撞风险并计算安全位置
                        drone.target_pose = self.calculate_safe_position(i, desired_pos)
                
                # 检查是否所有无人机都到达悬停位置
                if self.all_drones_reached_target():
                    rospy.loginfo("All drones reached hover positions, starting 3-second hover...")
                    self.flight_state = FlightState.HOVER_PHASE
                    self.hover_start_time = current_time
            
            elif self.flight_state == FlightState.HOVER_PHASE:
                # 悬停阶段
                if self.hover_start_time is not None:
                    hover_elapsed = (current_time - self.hover_start_time).to_sec()
                    rospy.loginfo_throttle(1.0, f"Hovering... {hover_elapsed:.1f}/{self.hover_duration:.1f}s")
                    
                    # 保持在悬停位置
                    for i, drone in enumerate(self.drones):
                        if i < len(self.hover_positions):
                            desired_pos = self.hover_positions[i]
                            drone.target_pose = self.calculate_safe_position(i, desired_pos)
                    
                    # 检查悬停时间是否完成
                    if hover_elapsed >= self.hover_duration:
                        rospy.loginfo("Hover phase complete, starting formation flight...")
                        self.flight_state = FlightState.GOTO_LANDING
                        self.formation_start_time = current_time
                        self.current_waypoint_index = 0
            
            elif self.flight_state == FlightState.FORMATION_FLIGHT:
                # 编队飞行阶段
                if self.current_waypoint_index < len(self.formation_waypoints):
                    current_waypoint = self.formation_waypoints[self.current_waypoint_index]
                    
                    # 计算编队位置
                    formation_positions = self.calculate_formation_positions(current_waypoint)
                    
                    # 设置每架无人机的目标位置
                    for i, drone in enumerate(self.drones):
                        if i < len(formation_positions):
                            desired_pos = formation_positions[i]
                            drone.target_pose = self.calculate_safe_position(i, desired_pos)
                    
                    # 检查是否所有无人机都到达当前航点
                    if self.all_drones_reached_target():
                        if self.waypoint_reach_time is None:
                            self.waypoint_reach_time = current_time
                            rospy.loginfo(f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.formation_waypoints)}: {current_waypoint}")
                        
                        # 在航点悬停指定时间
                        hover_elapsed = (current_time - self.waypoint_reach_time).to_sec()
                        rospy.loginfo_throttle(1.0, f"Hovering at waypoint {self.current_waypoint_index + 1}... {hover_elapsed:.1f}/{self.formation_waypoint_hover_duration:.1f}s")
                        
                        if hover_elapsed >= self.formation_waypoint_hover_duration:
                            # 前往下一个航点
                            self.current_waypoint_index += 1
                            self.waypoint_reach_time = None
                            
                            if self.current_waypoint_index < len(self.formation_waypoints):
                                rospy.loginfo(f"Moving to waypoint {self.current_waypoint_index + 1}/{len(self.formation_waypoints)}")
                            else:
                                rospy.loginfo("Formation flight complete, preparing for landing...")
                                self.assign_nearest_landing_positions()  # 动态分配最近的降落位置
                                self.flight_state = FlightState.GOTO_LANDING
                
            elif self.flight_state == FlightState.GOTO_LANDING:
                # 原地降落 - 保持当前水平位置，逐渐降低高度
                rospy.loginfo("Starting in-place landing - maintaining horizontal position, reducing altitude...")
                
                
                for i, drone in enumerate(self.drones):
                    # 保持当前的x,y位置，设置目标高度稍低一些作为过渡
                    drone.target_pose.pose.position.x = drone.landing_pos.pose.position.x
                    drone.target_pose.pose.position.y = drone.landing_pos.pose.position.y
                    drone.target_pose.pose.position.z = 0.5  # 先降到0.5米高度作为准备
                
                hover_start_time = current_time
                
                while not self.all_drones_reached_target() or (current_time - hover_start_time).to_sec() < 10:
                    pass
                
                # 由于是原地降落，直接进入降落阶段
                rospy.loginfo("Transitioning to final landing phase...")
                self.flight_state = FlightState.LANDING
            
            elif self.flight_state == FlightState.LANDING:
                # 执行降落，继续监控距离
                for i, drone in enumerate(self.drones):
                    # 监控降落过程中的距离
                    for j in range(i + 1, len(self.drones)):
                        distance = self.calculate_distance(drone, self.drones[j])
                        if distance < 1.0:
                            rospy.logwarn(f"LANDING: Close distance between {drone.name} and {self.drones[j].name}: {distance:.2f} m")
                    
                    # 使用封装的降落方法
                    drone.land_vehicle()
                
                # 等待一段时间后退出
                if not hasattr(self, 'landing_start_time'):
                    self.landing_start_time = current_time
                
                if (current_time - self.landing_start_time).to_sec() > 10.0:
                    rospy.loginfo("Mission complete!")
                    return
            
            elif self.flight_state == FlightState.EMERGENCY_LANDING:
                # 紧急降落状态处理
                rospy.logwarn_throttle(5.0, "EMERGENCY LANDING IN PROGRESS...")
                
                for i, drone in enumerate(self.drones):
                    # 紧急降落：直接使用mavros降落服务
                    drone.land_vehicle()
                    
                    # 监控降落过程中的距离（安全起见）
                    for j in range(i + 1, len(self.drones)):
                        distance = self.calculate_distance(drone, self.drones[j])
                        if distance < 1.0:
                            rospy.logwarn_throttle(2.0, f"EMERGENCY LANDING: Close distance between {drone.name} and {self.drones[j].name}: {distance:.2f} m")
                
                # 等待一段时间后退出
                if self.emergency_landing_start_time is not None:
                    if (current_time - self.emergency_landing_start_time).to_sec() > 15.0:
                        rospy.logwarn("Emergency landing procedure complete!")
                        return
            
            # 安全监控：定期检查所有无人机之间的距离
            # if (current_time - self.last_safety_check).to_sec() > 2.0:
            #     rospy.loginfo("=== Safety Monitoring ===")
            #     for i, drone in enumerate(self.drones):
            #         rospy.loginfo(f"{drone.name} position: ({drone.current_pose.pose.position.x:.2f}, "
            #                      f"{drone.current_pose.pose.position.y:.2f}, {drone.current_pose.pose.position.z:.2f})")
                    
            #         for j in range(i + 1, len(self.drones)):
            #             distance = self.calculate_distance(drone, self.drones[j])
            #             rospy.loginfo(f"Distance {drone.name} - {self.drones[j].name}: {distance:.2f} m")
                        
            #             if distance < 0.8:
            #                 rospy.logerr(f"CRITICAL: Very close distance detected! {drone.name} - {self.drones[j].name}: {distance:.2f} m")
                
            #     self.last_safety_check = current_time
            
            # 发布所有无人机的目标位置
            for drone in self.drones:
                drone.publish_target_pose()
            
            self.rate.sleep()
    
    def run(self):
        """运行主程序"""
        # 启动键盘监听线程
        keyboard_thread = threading.Thread(target=keyboard_listener, args=(self,))
        keyboard_thread.daemon = True  # 设置为守护线程
        keyboard_thread.start()
        rospy.loginfo("Keyboard listener thread started. Type 'e' + Enter for emergency landing, 'q' + Enter to quit.")
        
        # 等待所有飞控连接
        self.wait_for_connection()
        
        # 加载路径文件
        self.load_paths()
        
        # 计算起飞位置
        self.calculate_takeoff_positions()
        
        # 初始化目标位置为各自的起飞位置
        for drone in self.drones:
            drone.target_pose = drone.takeoff_pos
        
        # 预发布目标位置
        self.pre_publish_targets()
        
        # 设置飞行模式和解锁
        rospy.loginfo("Starting multi-drone mission...")
        self.setup_flight_mode()
        
        # 执行飞行任务
        self.execute_flight_state_machine()

if __name__ == '__main__':
    try:
        controller = MultiDroneController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Multi-drone control terminated.")
