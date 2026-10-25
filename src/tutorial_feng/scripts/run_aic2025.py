#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ROS节点：AIC2025无人机任务控制程序
# 功能：控制无人机起飞、飞行路径规划、物品投放、降落等任务

import rospy  # ROS Python客户端库
import smach  # 状态机库
import smach_ros  # SMACH的ROS集成
import math  # 数学计算库
import tf.transformations  # TF坐标变换库

# 导入ROS消息类型
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3  # 几何消息：速度、位姿、向量
from mavros_msgs.msg import State  # MAVROS状态消息
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest  # MAVROS服务：解锁、模式设置
from std_msgs.msg import Bool, Empty  # 空消息类型 / 安全层布尔


class SafetyFilteredVelocityPublisher(object):
    """速度出口安全层：run_aic2025 里所有速度指令都发到 ctx.vel_pub，
    这里统一过一遍 obstacle_guard 的净空信息后再发给飞控。

    * 净空 < stop_clearance     -> 水平速度清零（就地悬停，垂直速度保留，可继续爬升/下降）
    * 净空 < slow_clearance     -> 水平速度按比例缩小，并朝更空的一侧横移让开
    * 没有安全信息（节点没起/点云断） -> 放行（保持原有行为），可用 ~safety_fail_safe 改成悬停
    """

    def __init__(self, ctx, topic="mavros/setpoint_velocity/cmd_vel"):
        self._ctx = ctx
        self._pub = rospy.Publisher(topic, TwistStamped, queue_size=1)

    def get_num_connections(self):
        return self._pub.get_num_connections()

    def publish(self, msg):
        vx, vy = self._ctx.apply_safety(msg.twist.linear.x, msg.twist.linear.y)
        out = TwistStamped()
        out.header.stamp = rospy.Time.now()
        out.twist.linear.x = vx
        out.twist.linear.y = vy
        out.twist.linear.z = msg.twist.linear.z
        out.twist.angular.x = msg.twist.angular.x
        out.twist.angular.y = msg.twist.angular.y
        out.twist.angular.z = msg.twist.angular.z
        self._pub.publish(out)


class NodeContext(object):
    """节点上下文类：存储节点的状态信息和提供控制方法"""
    
    def __init__(self):
        """初始化节点上下文，包括状态变量、发布者、订阅者和服务客户端"""
        # 飞控状态和位置信息
        self.current_state = State()  # 当前飞控状态
        self.current_pose = PoseStamped()  # 当前位姿（位置+姿态）
        self.current_rpy = [0.0, 0.0, 0.0]  # 当前姿态角：横滚、俯仰、偏航
        self.last_srv_request = rospy.Time.now()  # 上次服务请求时间（用于节流）
        self.rate = rospy.Rate(30.0)  # 控制循环频率：30Hz

        # PID控制变量
        self.last_err = [0.0, 0.0, 0.0]  # 上次位置误差 [x, y, z]
        self.err_sum = [0.0, 0.0, 0.0]  # 位置误差积分 [x, y, z]
        self.last_yaw_err = 0.0  # 上次偏航角误差
        self.yaw_err_sum = 0.0  # 偏航角误差积分
        self.last_pid_control_time = rospy.Time.now()  # 上次PID控制时间
        
        # 视觉跟踪变量
        self.target_center_diff = Vector3()  # 目标中心偏差（像素）
        self.target_center_diff_received = False  # 是否接收到视觉反馈
        self.last_visual_feedback_time = rospy.Time.now()  # 上次视觉反馈时间
        
        # 发布者（Publishers）
        # 安全层参数（obstacle_guard 提供 /safety/stop 与 /safety/clearance）
        # 前馈避障：已知障碍（柱子）做成 keep-out 圆，航段直线会穿过去时自动绕行
        self.keepout_circles = rospy.get_param('~keepout_circles', [])
        self.keepout_body_radius = rospy.get_param('~keepout_body_radius', 0.30)
        self.keepout_margin = rospy.get_param('~keepout_margin', 0.35)
        self.keepout_margin_floor = rospy.get_param('~keepout_margin_floor', 0.12)
        self._margin_scale = 1.0
        # 场地边界（外墙/围网）：绕行点会被收进这个方框内
        self.field_enabled = rospy.get_param('~field_enabled', True)
        self.field = [rospy.get_param('~field/x_min', -4.6), rospy.get_param('~field/x_max', 4.6),
                      rospy.get_param('~field/y_min', -3.1), rospy.get_param('~field/y_max', 3.1)]
        self.wall_margin = rospy.get_param('~wall_margin', 0.35)
        self.safety_enable = rospy.get_param('~safety_enable', True)
        self.safety_fail_safe = rospy.get_param('~safety_fail_safe', False)   # 无安全信息时: false=放行, true=悬停
        self.safety_stale_timeout = rospy.get_param('~safety_stale_timeout', 1.0)
        self.safety_stop_clearance = rospy.get_param('~safety_stop_clearance', 0.25)
        self.safety_slow_clearance = rospy.get_param('~safety_slow_clearance', 0.90)
        self.safety_escape_speed = rospy.get_param('~safety_escape_speed', 0.25)  # 横移让开速度 m/s
        self.safety_stop = False
        self.safety_clearance = float('inf')
        self.safety_left = float('inf')
        self.safety_right = float('inf')
        self.safety_last_time = rospy.Time(0)
        self.safety_last_log_time = rospy.Time.now()
        self.safety_block_count = 0

        # 速度控制发布者：发送速度指令给飞控（已经过安全层过滤）
        self.vel_pub = SafetyFilteredVelocityPublisher(self)

        # 位置设定点发布者：发送位置指令给飞控
        self.setpoint_position_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )

        # 舵机发布者：用于物品投放
        self.front_left_open_pub = rospy.Publisher("servo/front_left/open", Empty, queue_size=1)  # 左前舵机打开
        self.front_right_open_pub = rospy.Publisher("servo/front_right/open", Empty, queue_size=1)  # 右前舵机打开
        self.back_left_open_pub = rospy.Publisher("servo/back_left/open", Empty, queue_size=1)  # 左后舵机打开
        self.back_right_open_pub = rospy.Publisher("servo/back_right/open", Empty, queue_size=1)  # 右后舵机打开
        self.all_servo_close_pub = rospy.Publisher("servo/all/close", Empty, queue_size=1)  # 关闭所有舵机

        # 订阅者（Subscribers）
        # 飞控状态订阅：接收飞控当前状态（连接、解锁、模式等）
        self.state_sub = rospy.Subscriber(
            "mavros/state", State, self._state_cb, queue_size=1
        )
        # 位置订阅：接收无人机当前位姿
        self.pose_sub = rospy.Subscriber(
            "mavros/local_position/pose", PoseStamped, self._pose_cb, queue_size=1
        )
        # 视觉反馈订阅：接收目标中心偏差信息
        self.target_center_diff_sub = rospy.Subscriber(
            "target_center_diff", Vector3, self._target_center_diff_cb, queue_size=1
        )
        # 安全层订阅：净空/急停信号（obstacle_guard 发布）
        self.safety_stop_sub = rospy.Subscriber(
            "/safety/stop", Bool, self._safety_stop_cb, queue_size=1
        )
        self.safety_clear_sub = rospy.Subscriber(
            "/safety/clearance", Vector3, self._safety_clear_cb, queue_size=1
        )

        # 服务客户端（Service Clients）
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)  # 解锁/上锁服务
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)  # 飞行模式设置服务

    def _state_cb(self, msg):
        """飞控状态回调函数：更新当前飞控状态"""
        self.current_state = msg

    def _pose_cb(self, msg):
        """位姿回调函数：更新当前位置和姿态"""
        self.current_pose = msg
        # 将四元数转换为欧拉角（横滚、俯仰、偏航）
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        self.current_rpy = tf.transformations.euler_from_quaternion(quaternion)

    def _target_center_diff_cb(self, msg):
        """视觉目标中心偏差回调函数：接收视觉系统反馈的目标偏差"""
        # 约定：z < 0 表示"本帧没有检测到靶标"（红点/白盘节点都是这么发的）
        # 此时绝不能更新反馈时间/标记，否则会被状态机当成"误差=0 的完美对齐"
        if msg.z < 0.0:
            self.target_center_diff_received = False
            return
        self.target_center_diff = msg
        self.target_center_diff_received = True  # 标记已接收到视觉反馈
        self.last_visual_feedback_time = rospy.Time.now()  # 更新最后接收时间

    def _safety_stop_cb(self, msg):
        """净空不足急停信号（obstacle_guard）"""
        self.safety_stop = msg.data
        self.safety_last_time = rospy.Time.now()

    def _safety_clear_cb(self, msg):
        """净空信息：x=四周最小, y=左, z=右（单位 m）"""
        self.safety_clearance = msg.x if msg.x < 90.0 else float('inf')
        self.safety_left = msg.y if msg.y < 90.0 else float('inf')
        self.safety_right = msg.z if msg.z < 90.0 else float('inf')
        self.safety_last_time = rospy.Time.now()

    def apply_safety(self, vx, vy):
        """安全层：对水平速度做实时矫正（就地悬停 / 减速 / 朝空侧让开）

        净空是"机体系"（y=左），而任务发出的速度是"世界系"，
        所以侧移让开的速度必须用当前偏航角从机体系旋到世界系。
        """
        if not self.safety_enable:
            return vx, vy

        fresh = (rospy.Time.now() - self.safety_last_time).to_sec() <= self.safety_stale_timeout
        if not fresh:
            # 没有安全信息：默认放行（保持旧行为），可用 ~safety_fail_safe 改成悬停
            return (0.0, 0.0) if self.safety_fail_safe else (vx, vy)

        def escape_side():
            """让开方向：+1=左, -1=右, 0=信息不足不许乱动"""
            l, r = self.safety_left, self.safety_right
            if math.isinf(l) and math.isinf(r):
                return 0.0          # 两侧都没有点（例如障碍正对机头被罩住）-> 原地不动
            if l - r > 0.25:
                return 1.0          # 左侧明显更空
            if r - l > 0.25:
                return -1.0         # 右侧明显更空
            return 0.0

        def escape_velocity(magnitude):
            """朝更空的一侧让开（机体系 +y = 左），返回世界系 (vx, vy)"""
            side = escape_side()
            if side == 0.0:
                return 0.0, 0.0
            v_body = side * magnitude
            yaw = self.current_rpy[2]
            return -math.sin(yaw) * v_body, math.cos(yaw) * v_body

        slow = self.safety_slow_clearance
        if self.safety_stop or self.safety_clearance < self.safety_stop_clearance:
            self.safety_block_count += 1
            # 前进/后退停住，但左右净空差异明显时缓慢侧移让开（避免被卡死）
            evx, evy = escape_velocity(self.safety_escape_speed)
            if (rospy.Time.now() - self.safety_last_log_time).to_sec() > 1.0:
                self.safety_last_log_time = rospy.Time.now()
                rospy.logwarn("安全层: 净空 %.2f m (左 %.2f / 右 %.2f) -> %s",
                              self.safety_clearance, self.safety_left, self.safety_right,
                              "悬停并向%s侧移让开" % ("左" if escape_side() > 0 else "右")
                              if (evx or evy) else "就地悬停")
            return evx, evy

        if self.safety_clearance < slow and slow > self.safety_stop_clearance:
            scale = (self.safety_clearance - self.safety_stop_clearance) / \
                    (slow - self.safety_stop_clearance)
            scale = max(0.0, min(1.0, scale))
            vx *= scale
            vy *= scale
            # 朝更空的一侧横移让开
            evx, evy = escape_velocity(self.safety_escape_speed * (1.0 - scale))
            vx += evx
            vy += evy
            if (rospy.Time.now() - self.safety_last_log_time).to_sec() > 1.0:
                self.safety_last_log_time = rospy.Time.now()
                rospy.logwarn("安全层: 净空 %.2f m -> 限速到 %.0f%% 并向%s让开",
                              self.safety_clearance, scale * 100.0,
                              "左" if escape_side() > 0 else "右")
        return vx, vy

    def publish_velocity(self, vx=0.0, vy=0.0, vz=0.0, vyaw=0.0):
        """发布速度指令
        Args:
            vx: x方向速度 (m/s)
            vy: y方向速度 (m/s)
            vz: z方向速度 (m/s)
            vyaw: 偏航角速度 (rad/s)
        """
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        twist.twist.angular.z = vyaw
        self.vel_pub.publish(twist)

    # ------------------------------------------------------------ 前馈避障
    @staticmethod
    def _seg_point_dist(px, py, a, b):
        """点到线段的最短距离"""
        dx, dy = b[0] - a[0], b[1] - a[1]
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 <= 1e-12 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / l2))
        return math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy))

    def _detour_pair(self, a, b, circle, r_eff, circles=None):
        """在航线两侧采样若干横向偏移，返回 [(该候选的全局最小净空, 两个中间点), ...]。

        偏移量从 -r_eff 到 +r_eff 采样（含两者）。之所以要采一圈而不是"只往反方向
        平移 r_eff"：柱子可能成对夹出一条窄走廊（本题矩形下边就是从 (2.5,1.3) 和
        (2.5,-1.3) 中间穿过），平移满 r_eff 会撞上另一根，取走廊中线反而最安全。
        """
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            return []
        ux, uy = dx / seg_len, dy / seg_len
        nx, ny = -uy, ux
        cx, cy = circle[0], circle[1]
        t = max(0.0, min(seg_len, (cx - a[0]) * ux + (cy - a[1]) * uy))
        cands = []
        n_steps = 6
        offsets = [r_eff * (2.0 * k / n_steps - 1.0) for k in range(n_steps + 1)]
        offsets.append(0.0)
        for off in offsets:
            pts = []
            for tt in (max(0.0, t - r_eff), min(seg_len, t + r_eff)):
                pts.append((a[0] + tt * ux + nx * off,
                            a[1] + tt * uy + ny * off))
            poly = [a] + pts + [b]
            worst = min(self._seg_point_dist(c[0], c[1], poly[k], poly[k + 1])
                        for k in range(len(poly) - 1)
                        for c in (circles if circles is not None else self.keepout_circles))
            cands.append((worst, pts))
        return cands

    def keepout_effective_radius(self, circle):
        # _margin_scale<1 表示本航段通道太窄，已自动收紧绕行余量
        return circle[2] + self.keepout_body_radius + self.keepout_margin * self._margin_scale

    def _transit_circles(self, goal):
        """本次航段真正要躲的圆。
        transit_only=True 的圆（例如地面投放靶）只在"路过"时躲；
        如果目标点本身就在该圆内（就是要飞进去投放），本航段不禁它。"""
        active = []
        for c in self.keepout_circles:
            if len(c) < 3:
                continue
            x, y, r = float(c[0]), float(c[1]), float(c[2])
            transit_only = bool(c[3]) if len(c) > 3 else False
            if transit_only and math.hypot(goal[0] - x, goal[1] - y) < r:
                continue
            active.append((x, y, r))
        return active

    def clamp_to_field(self, x, y):
        """把点收进场地边界内（留 wall_margin + 机体半径），避免绕行点贴到外墙上"""
        if not self.field_enabled:
            return x, y
        m = self.wall_margin + self.keepout_body_radius
        cx = min(max(x, self.field[0] + m), self.field[1] - m)
        cy = min(max(y, self.field[2] + m), self.field[3] - m)
        return cx, cy

    def _plan_once(self, p0, goal, circles):
        """按给定 keep-out 圆规划一次，返回 (path, inserted, 是否仍有侵入)"""
        path = [p0, goal]
        inserted = []
        for _ in range(6):
            violation = None    # (缺的净空, 段号, 圆)
            for i in range(len(path) - 1):
                for circle in circles:
                    r_eff = self.keepout_effective_radius(circle)
                    d = self._seg_point_dist(circle[0], circle[1], path[i], path[i + 1])
                    if d < r_eff and (violation is None or d - r_eff < violation[0]):
                        violation = (d - r_eff, i, circle)
            if violation is None:
                return path, inserted, False
            _, idx, circle = violation
            r_eff = self.keepout_effective_radius(circle)
            cands = self._detour_pair(path[idx], path[idx + 1], circle, r_eff, circles)
            if not cands:
                break
            _, pts = max(cands, key=lambda it: it[0])                # 选更安全的一侧
            pts = [self.clamp_to_field(p[0], p[1]) for p in pts]     # 绕行点也收进场地
            if any(math.hypot(p[0] - q[0], p[1] - q[1]) < 0.15 for p in pts for q in inserted):
                break                                                # 同一对点不重复插
            path = path[:idx + 1] + pts + path[idx + 1:]
            inserted.extend(pts)
        return path, inserted, True

    def plan_leg(self, goal_x, goal_y):
        """前馈避障：航段直线穿过 keep-out 圆时自动插入绕行点，迭代到不再侵入为止。

        keep-out 圆由 ~keepout_circles 给（每项 [x, y, 半径]，半径取障碍外接半径），
        实际绕行半径 = r + 机体半径 + ~keepout_margin。
        返回 [(x, y), ...]；空列表表示直线本来就安全（此时行为与原来完全一致）。
        """
        if not self.keepout_circles:
            return []
        p0 = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        goal = (goal_x, goal_y)
        if math.hypot(goal[0] - p0[0], goal[1] - p0[1]) < 1e-3:
            return []

        circles = self._transit_circles(goal)
        if not circles:
            return []

        # 先按配置的余量规划；通道太窄（例如夹在两根柱子中间的走廊还要躲投放靶）就
        # 自动逐级收紧余量，直到不再侵入，或收到 ~keepout_margin_floor 下限
        path, inserted, violated = [], [], True
        scale_used = 1.0
        for scale in (1.0, 0.7, 0.5, 0.35, 0.0):
            self._margin_scale = max(scale, self.keepout_margin_floor /
                                     max(self.keepout_margin, 1e-6))
            path, inserted, violated = self._plan_once(p0, goal, circles)
            scale_used = self._margin_scale
            if not violated:
                break
        self._margin_scale = 1.0
        if scale_used < 0.999:
            rospy.logwarn("前馈避障: 通道太窄, 绕行余量自动收紧到 %.2f m (配置 %.2f m)",
                          self.keepout_margin * scale_used, self.keepout_margin)

        # 合并过近的相邻点，避免无意义的"到位"等待
        cleaned = [path[0]]
        for pt in path[1:]:
            if math.hypot(pt[0] - cleaned[-1][0], pt[1] - cleaned[-1][1]) > 0.15:
                cleaned.append(pt)
        path = cleaned

        if inserted:
            rospy.loginfo("前馈避障: 直线被挡, 插入 %d 个绕行点 %s",
                          len(inserted), ["(%.2f, %.2f)" % p for p in inserted])
        return path[1:-1]

    def fly_leg(self, goal_x, goal_y, goal_z, tolerance=0.05, name="航段",
                via_tolerance=0.20):
        """按前馈避障规划的中间点依次飞到目标点（PID 速度控制，30 Hz）。"""
        via = self.plan_leg(goal_x, goal_y)
        targets = list(via) + [(goal_x, goal_y)]
        for idx, (tx, ty) in enumerate(targets):
            is_last = (idx == len(targets) - 1)
            tol = tolerance if is_last else via_tolerance
            if not is_last:
                rospy.loginfo("%s: 绕行点 %d/%d -> (%.2f, %.2f)", name, idx + 1, len(via), tx, ty)
            while not rospy.is_shutdown():
                distance, _, _, _ = self.get_length_between_points(tx, ty, goal_z)
                if distance < tol:
                    break
                vx, vy, vz, vyaw = self.get_pid_vel(tx, ty, goal_z)
                self.publish_velocity(vx, vy, vz, vyaw)
                self.rate.sleep()
        return True

    def slew_altitude(self, target_alt, max_speed, accel, tolerance=0.05, timeout_sec=30.0):
        """缓起缓降：限速 + 限加速的高度斜坡

        速度按梯形曲线给出：起步从 0 平滑加速；接近目标时按 v=sqrt(2*a*d) 提前减速，
        所以到位是"软"的，不会像原来那样一路 1.2 m/s 冲上去再刹。

        Args:
            target_alt: 目标高度（m）
            max_speed:  最大垂直速度（m/s）
            accel:      最大垂直加速度（m/s²），越小越柔和
            tolerance:  到位判定阈值（m）
            timeout_sec: 超时时间（s），超时返回 False（例如没进 OFFBOARD）
        Returns:
            True = 到位；False = 超时
        """
        rate = self.rate
        start_time = rospy.Time.now()
        last_time = start_time
        last_log_time = start_time
        v_cmd = 0.0
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max((now - last_time).to_sec(), 1e-3)
            last_time = now

            z = self.current_pose.pose.position.z
            remaining = target_alt - z

            if abs(remaining) <= tolerance:
                self.publish_velocity(0.0, 0.0, 0.0)
                rospy.loginfo("高度到位: z=%.2f m (目标 %.2f m)，用时 %.1f s",
                              z, target_alt, (now - start_time).to_sec())
                return True
            if (now - start_time).to_sec() > timeout_sec:
                self.publish_velocity(0.0, 0.0, 0.0)
                rospy.logwarn("高度斜坡超时 %.0f s，当前 z=%.2f m，目标 %.2f m",
                              timeout_sec, z, target_alt)
                return False

            # 越接近目标允许的速度越低 -> 自动减速到位
            v_allow = min(max_speed, math.sqrt(max(2.0 * accel * abs(remaining), 0.0)))
            v_des = v_allow if remaining > 0 else -v_allow
            # 加速度限幅（缓起 + 缓停）
            dv = accel * dt
            v_cmd = min(max(v_cmd, v_des - dv), v_des + dv)
            self.publish_velocity(0.0, 0.0, v_cmd)

            if (now - last_log_time).to_sec() >= 1.0:
                last_log_time = now
                rospy.loginfo("高度斜坡: z=%.2f m -> %.2f m, vz=%.2f m/s", z, target_alt, v_cmd)
            rate.sleep()
        return False

    def try_set_mode(self, custom_mode, throttle_sec):
        """尝试设置飞行模式（带节流限制）
        Args:
            custom_mode: 目标飞行模式（如"OFFBOARD", "AUTO.LAND"等）
            throttle_sec: 两次请求之间的最小时间间隔（秒）
        """
        now = rospy.Time.now()
        # 节流控制：只有超过指定时间间隔才发送请求
        if (now - self.last_srv_request).to_sec() > throttle_sec:
            try:
                req = SetModeRequest()
                req.custom_mode = custom_mode
                resp = self.set_mode_client(req)
                if resp.mode_sent:
                    rospy.loginfo("设置模式已发送: %s", custom_mode)
                else:
                    rospy.logwarn("发送模式失败: %s", custom_mode)
            except rospy.ServiceException as exc:
                rospy.logwarn("设置模式服务失败: %s", str(exc))
            finally:
                self.last_srv_request = now

    def try_arm(self, arm_value=True, throttle_sec=1.0):
        """尝试解锁/上锁飞控（带节流限制）
        Args:
            arm_value: True为解锁，False为上锁
            throttle_sec: 两次请求之间的最小时间间隔（秒）
        """
        now = rospy.Time.now()
        # 节流控制：只有超过指定时间间隔才发送请求
        if (now - self.last_srv_request).to_sec() > throttle_sec:
            try:
                req = CommandBoolRequest()
                req.value = arm_value
                resp = self.arming_client(req)
                if resp.success:
                    rospy.loginfo("飞控已解锁" if arm_value else "飞控已上锁")
                else:
                    rospy.logwarn("改变解锁状态失败: %s", arm_value)
            except rospy.ServiceException as exc:
                rospy.logwarn("解锁服务失败: %s", str(exc))
            finally:
                self.last_srv_request = now

    def get_length_between_points(self, target_x, target_y, target_z):
        """计算当前位置与目标位置之间的距离和误差
        Args:
            target_x, target_y, target_z: 目标位置坐标
        Returns:
            distance: 欧氏距离
            err_x, err_y, err_z: 各轴的位置误差
        """
        err_x = target_x - self.current_pose.pose.position.x
        err_y = target_y - self.current_pose.pose.position.y
        err_z = target_z - self.current_pose.pose.position.z
        distance = math.sqrt(err_x*err_x + err_y*err_y + err_z*err_z)
        return distance, err_x, err_y, err_z

    def get_pid_vel(self, target_x, target_y, target_z):
        """PID速度控制器：根据目标位置计算控制速度（从C++代码翻译）
        Args:
            target_x, target_y, target_z: 目标位置坐标
        Returns:
            linear_x, linear_y, linear_z: 线性速度控制量
            angular_z: 角速度控制量（偏航）
        """
        # PID控制参数
        VEL_P = 2.028  # 位置比例系数
        VEL_I = 0.0  # 位置积分系数
        VEL_D = 0.255  # 位置微分系数
        
        VEL_Pz = 2.333  # 位置比例系数
        VEL_Iz = 0.0  # 位置积分系数
        VEL_Dz = 0.343  # 位置微分系数
        YAW_VEL_P = 1.0  # 偏航比例系数
        YAW_VEL_I = 0.0  # 偏航积分系数
        YAW_VEL_D = 0.0  # 偏航微分系数
        
        # 获取当前时间戳
        current_time = self.current_pose.header.stamp
        if current_time.to_sec() == 0:
            current_time = rospy.Time.now()
            
        # 计算时间间隔
        dt = (current_time - self.last_pid_control_time).to_sec()

        # 计算位置误差和距离
        distance, err_x, err_y, err_z = self.get_length_between_points(target_x, target_y, target_z)
        
        # 偏航角控制（保持偏航角为0）
        y_err = 0.0 - self.current_rpy[2]  # 偏航角误差
        dy_err = (y_err - self.last_yaw_err) / dt if dt > 0 else 0.0  # 偏航角误差微分
        
        # 偏航角速度PID控制
        angular_z = YAW_VEL_P * y_err + YAW_VEL_I * self.yaw_err_sum + YAW_VEL_D * dy_err
        
        if distance > 1.0:
            # 距离目标较远（>0.8m）- 使用归一化速度（恒定速度2.6m/s朝向目标）
            linear_x = err_x * 2.1 / distance
            linear_y = err_y * 2.1 / distance
            linear_z = err_z * 0.8 / distance
            
            # 重置积分项
            self.err_sum = [0.0, 0.0, 0.0]
            
        
        else:
            # 接近目标（<=0.8m）- 使用完整PID控制
            # 计算误差微分
            d_err_x = (err_x - self.last_err[0]) / dt if dt > 0 else 0.0
            d_err_y = (err_y - self.last_err[1]) / dt if dt > 0 else 0.0
            d_err_z = (err_z - self.last_err[2]) / dt if dt > 0 else 0.0
            
            # PID控制律：u = Kp*e + Ki*∫e*dt + Kd*de/dt
            linear_x = VEL_P * err_x + VEL_I * self.err_sum[0] + VEL_D * d_err_x
            linear_y = VEL_P * err_y + VEL_I * self.err_sum[1] + VEL_D * d_err_y
            linear_z = VEL_Pz * err_z + VEL_Iz * self.err_sum[2] + VEL_Dz * d_err_z
            
            # 更新积分项
            self.err_sum[0] += err_x * dt
            self.err_sum[1] += err_y * dt
            self.err_sum[2] += err_z * dt
            rospy.loginfo("err_x: %f, err_y: %f, err_z: %f", err_x, err_y, err_z)

        # 保存当前误差用于下次微分计算
        self.last_err = [err_x, err_y, err_z]
        self.last_yaw_err = y_err
        self.yaw_err_sum += y_err * dt
        self.last_pid_control_time = current_time
        rospy.loginfo("linear_x: %f, linear_y: %f, linear_z: %f, angular_z: %f", linear_x, linear_y, linear_z, angular_z)
        
        return linear_x, linear_y, linear_z, angular_z
    
    def drop_item(self, item_name):
        """使用指定舵机投放物品
        Args:
            item_name: 舵机名称（'front_left', 'front_right', 'back_left', 'back_right'）
        """
        empty_msg = Empty()
        
        if item_name == 'front_left':
            self.front_left_open_pub.publish(empty_msg)
            rospy.loginfo("使用左前舵机投放物品")
        elif item_name == 'front_right':
            self.front_right_open_pub.publish(empty_msg)
            rospy.loginfo("使用右前舵机投放物品")
        elif item_name == 'back_left':
            self.back_left_open_pub.publish(empty_msg)
            rospy.loginfo("使用左后舵机投放物品")
        elif item_name == 'back_right':
            self.back_right_open_pub.publish(empty_msg)
            rospy.loginfo("使用右后舵机投放物品")
        else:
            rospy.logwarn("未知的舵机名称: %s", item_name)

    def close_all_servos(self):
        """关闭所有舵机门"""
        self.all_servo_close_pub.publish(Empty())
        rospy.loginfo("已关闭所有舵机门")

    def get_visual_adjustment(self, timeout_sec=3.0):
        """获取视觉反馈调整量用于投放时的精确定位（偏航角补偿）
        Args:
            timeout_sec: 视觉反馈超时时间（秒）
        Returns:
            adj_x, adj_y: 世界坐标系下的位置调整量（米）
        """
        # 检查是否有最近的视觉反馈
        time_since_feedback = (rospy.Time.now() - self.last_visual_feedback_time).to_sec()

        if not self.target_center_diff_received or time_since_feedback > timeout_sec:
            # 没有视觉反馈或反馈过时，返回零调整
            return 0.0, 0.0

        # 视觉反馈缩放因子（根据相机和期望灵敏度调整）
        visual_scale_x = 0.0001  # 米/像素：x方向（前后）
        visual_scale_y = 0.0001  # 米/像素：y方向（左右）

        # 获取机体坐标系下的x/y调整量
        local_x = self.target_center_diff.x * visual_scale_x
        local_y = self.target_center_diff.y * visual_scale_y

        # 获取当前姿态四元数
        orientation = (
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        )

        # 提取偏航角
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)


        # 将调整量从机体坐标系旋转到世界坐标系（ENU坐标系）
        # [world_x]   [cos(yaw) -sin(yaw)] [local_x]
        # [world_y] = [sin(yaw)  cos(yaw)] [local_y]
        adj_x =  math.cos(yaw)*local_x - math.sin(yaw)*local_y
        adj_y =  math.sin(yaw)*local_x + math.cos(yaw)*local_y

        # 限制调整量在合理范围内
        max_adj = 0.2  # 每个方向最大调整20厘米
        adj_x = max(-max_adj, min(max_adj, adj_x))
        adj_y = max(-max_adj, min(max_adj, adj_y))

        return adj_x, adj_y



class BeforeOffboard(smach.State):
    """切换到OFFBOARD模式前的状态"""
    
    def __init__(self, ctx):
        """初始化状态
        Args:
            ctx: 节点上下文对象
        """
        smach.State.__init__(
            self, 
            outcomes=["offboard"],  # 成功切换到OFFBOARD模式
        ) 
        self.ctx = ctx

    def execute(self, userdata):
        """执行状态逻辑：等待并切换到OFFBOARD模式"""
        rate = self.ctx.rate
        while not rospy.is_shutdown():
            # 检查是否已连接飞控
            if self.ctx.current_state.connected: 
                # 检查是否已经是OFFBOARD模式
                if self.ctx.current_state.mode == "OFFBOARD":
                    return "offboard"
                # 尝试设置为OFFBOARD模式
                self.ctx.try_set_mode("OFFBOARD", throttle_sec=1.0)
            # 持续发布零速度指令（OFFBOARD模式要求）
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            self.ctx.vel_pub.publish(twist)
            rate.sleep()


class BeforeArmed(smach.State):
    """解锁飞控前的状态"""
    
    def __init__(self, ctx):
        """初始化状态
        Args:
            ctx: 节点上下文对象
        """
        smach.State.__init__(
            self, 
            outcomes=["armed"],  # 成功解锁
            output_keys=["takeoff_start_pose"]  # 输出起飞起始位置
        ) 
        self.ctx = ctx

    def execute(self, userdata):
        """执行状态逻辑：等待飞控解锁"""
        rate = self.ctx.rate
        while not rospy.is_shutdown():
            # 检查飞控是否已解锁
            if self.ctx.current_state.armed:
                # 记录解锁时的位置作为起飞起始点
                userdata.takeoff_start_pose = self.ctx.current_pose
                rospy.loginfo("飞控已解锁，位置: x=%.2f, y=%.2f, z=%.2f", 
                             self.ctx.current_pose.pose.position.x,
                             self.ctx.current_pose.pose.position.y,
                             self.ctx.current_pose.pose.position.z)
                return "armed"
            # 尝试解锁飞控
            self.ctx.try_arm(True, throttle_sec=1.0)
            # 持续发布零速度指令
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            self.ctx.vel_pub.publish(twist)
            rate.sleep()


class Takeoff(smach.State):
    """起飞状态：控制无人机上升到目标高度"""
    
    def __init__(self, ctx, target_alt):
        """初始化起飞状态
        Args:
            ctx: 节点上下文对象
            target_alt: 目标高度（米）
        """
        smach.State.__init__(
            self, 
            outcomes=["at_altitude"],  # 到达目标高度
            input_keys=["takeoff_start_pose"],  # 输入：起飞起始位置
            output_keys=["takeoff_altitude"]  # 输出：最终到达的高度
        ) 
        self.ctx = ctx
        self.target_alt = target_alt
        # 缓起参数（原来是恒定 1.2 m/s 硬爬升）
        self.max_speed = rospy.get_param('~takeoff_speed', 1.0)   # m/s
        self.accel = rospy.get_param('~takeoff_accel', 0.6)       # m/s²
        self.tolerance = rospy.get_param('~takeoff_tolerance', 0.05)

    def execute(self, userdata):
        """执行起飞逻辑：限速+限加速缓起到目标高度"""
        start_pose = getattr(userdata, 'takeoff_start_pose', None)
        if start_pose:
            rospy.loginfo("开始缓起，从 z=%.2f 到 z=%.2f (限速 %.2f m/s, 限加速 %.2f m/s²)", 
                         start_pose.pose.position.z, 
                         self.target_alt, self.max_speed, self.accel)

        self.ctx.slew_altitude(self.target_alt, self.max_speed, self.accel, self.tolerance)

        takeoff_altitude = self.ctx.current_pose.pose.position.z
        userdata.takeoff_altitude = takeoff_altitude
        rospy.loginfo("到达目标高度: %.2f", takeoff_altitude)
        return "at_altitude"


class FlyToWaypoint(smach.State):
    """飞往航点状态：控制无人机飞往指定航点，支持投放点配置"""
    
    def __init__(self, ctx, target_x, target_y, target_z, waypoint_name, tolerance=0.05,
                 drop_point_id=None, drop_config=None,target_drop_altitude_dt=0.0):
        """初始化飞往航点状态
        Args:
            ctx: 节点上下文对象
            target_x, target_y, target_z: 目标航点坐标
            waypoint_name: 航点名称（用于日志）
            tolerance: 到达判定容差（米）
            drop_point_id: 投放点ID（1, 2, 3, 或 4），None表示非投放点
            drop_config: 投放配置字典
            target_drop_altitude_dt:投放高度补偿
        """
        smach.State.__init__(
            self, 
            outcomes=["reached", "drop_required"],  # reached:到达航点, drop_required:需要投放
            input_keys=["cycle_count"],  # 输入：当前圈数
            output_keys=["drop_target_x", "drop_target_y", "drop_altitude", "drop_item", 
                        "drop_item_offset_x", "drop_item_offset_y", "drop_waypoint_name"]  # 输出：投放相关参数
        ) 
        self.ctx = ctx
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        self.target_drop_altitude_dt = target_drop_altitude_dt
        self.waypoint_name = waypoint_name
        self.tolerance = tolerance
        self.drop_point_id = drop_point_id  # 投放点ID：1, 2, 3, 或 4
        self.drop_config = drop_config  # 投放配置字典

    def execute(self, userdata):
        """执行飞往航点逻辑：使用PID控制飞往目标点，到达后判断是否需要投放"""
        rospy.loginfo("飞往航点 %s: (%.2f, %.2f, %.2f)", 
                     self.waypoint_name, self.target_x, self.target_y, self.target_z)

        # 前馈避障：需要时自动插入绕柱中间点（直线安全时与原来完全一致）
        self.ctx.fly_leg(self.target_x, self.target_y, self.target_z,
                         tolerance=self.tolerance, name="飞往 " + self.waypoint_name)

        rospy.loginfo("到达航点 %s", self.waypoint_name)

        # 检查是否是投放点且应该在当前圈投放
        if self.drop_point_id and self.drop_config:
            current_cycle = getattr(userdata, 'cycle_count', 0) + 1  # +1 因为我们在当前圈中
            drop_circle_key = f'drop_point{self.drop_point_id}_circle'

            # 判断该投放点是否应该在当前圈投放
            if drop_circle_key in self.drop_config and self.drop_config[drop_circle_key] == current_cycle:
                rospy.loginfo("在 %s 需要投放（第 %d 圈）", self.waypoint_name, current_cycle)

                # 将投放上下文传递给userdata
                userdata.drop_target_x = self.target_x
                userdata.drop_target_y = self.target_y
                userdata.drop_altitude = self.target_drop_altitude_dt + self.drop_config['drop_altitude']
                userdata.drop_item = self.drop_config[f'drop_point{self.drop_point_id}_item']
                userdata.drop_item_offset_x = self.drop_config[f'drop_point{self.drop_point_id}_item_offset_x']
                userdata.drop_item_offset_y = self.drop_config[f'drop_point{self.drop_point_id}_item_offset_y']
                userdata.drop_waypoint_name = self.waypoint_name

                return "drop_required"

        return "reached"


class WaypointState(smach.State):
    """简单航点状态：导航到指定航点（不支持投放功能）"""
    
    def __init__(self, ctx, target_x, target_y, altitude, waypoint_name, tolerance=0.05):
        """初始化航点状态
        Args:
            ctx: 节点上下文对象
            target_x, target_y: 目标x、y坐标
            altitude: 目标高度
            waypoint_name: 航点名称
            tolerance: 到达判定容差（米）
        """
        smach.State.__init__(
            self,
            outcomes=["reached"]  # 到达航点
        )
        self.ctx = ctx
        self.target_x = target_x
        self.target_y = target_y
        self.altitude = altitude
        self.waypoint_name = waypoint_name
        self.tolerance = tolerance

    def execute(self, userdata):
        """执行导航逻辑：使用PID控制飞往航点"""
        rospy.loginfo("导航到 %s: (%.2f, %.2f, %.2f)", 
                     self.waypoint_name, self.target_x, self.target_y, self.altitude)

        # 前馈避障：需要时自动插入绕柱中间点（直线安全时与原来完全一致）
        self.ctx.fly_leg(self.target_x, self.target_y, self.altitude,
                         tolerance=self.tolerance, name="导航 " + self.waypoint_name)

        rospy.loginfo("到达 %s，位置 (%.2f, %.2f)", self.waypoint_name, self.target_x, self.target_y)
        return "reached"


class RectangularPath(smach.StateMachine):
    """矩形路径状态机：绕障碍物飞行的矩形路径"""
    
    def __init__(self, ctx, altitude):
        """初始化矩形路径状态机
        Args:
            ctx: 节点上下文对象
            altitude: 飞行高度
        """
        smach.StateMachine.__init__(
            self,
            outcomes=["path_complete"]  # 路径完成
        )
        
        self.ctx = ctx
        self.altitude = altitude
        
        with self:
            # 定义绕障碍物的矩形路径航点
            # 航点1：右上角
            smach.StateMachine.add(
                "WAYPOINT_TOP_LEFT",
                WaypointState(ctx, 3.5, 2.3, altitude, "右上角"),
                transitions={"reached": "WAYPOINT_BOTTOM_LEFT"}
            )
            
            # 航点2：左上角
            smach.StateMachine.add(
                "WAYPOINT_BOTTOM_LEFT",
                WaypointState(ctx, 1.5, 2.3, altitude, "左上角"),
                transitions={"reached": "WAYPOINT_BOTTOM_RIGHT"},
            )
            
            # 航点3：左下角
            smach.StateMachine.add(
                "WAYPOINT_BOTTOM_RIGHT",
                WaypointState(ctx, 1.5, 0.3, altitude, "左下角"),
                transitions={"reached": "WAYPOINT_TOP_RIGHT"},
            )
            
            # 航点4：右下角
            smach.StateMachine.add(
                "WAYPOINT_TOP_RIGHT",
                WaypointState(ctx, 3.5, 0.3, altitude, "右下角"),
                transitions={"reached": "WAYPOINT_COMPLETE_RECTANGLE"},
            )
            
            # 航点5：返回右上角（完成矩形）
            smach.StateMachine.add(
                "WAYPOINT_COMPLETE_RECTANGLE",
                WaypointState(ctx, 3.5, 2.3, altitude, "完成矩形"),
                transitions={"reached": "path_complete"},
            )


class NavigateToDropPosition(smach.State):
    """导航到投放位置状态：使用视觉反馈导航到投放点上方"""
    
    def __init__(self, ctx):
        """初始化导航到投放位置状态
        Args:
            ctx: 节点上下文对象
        """
        smach.State.__init__(
            self,
            outcomes=["close_enough"],  # 足够接近投放目标
            input_keys=["drop_target_x", "drop_target_y", "drop_altitude"]  # 输入：投放目标位置
        )
        self.ctx = ctx

    def execute(self, userdata):
        """执行导航逻辑：结合视觉反馈导航到投放位置"""
        target_x = userdata.drop_target_x
        target_y = userdata.drop_target_y
        drop_altitude = userdata.drop_altitude
        
        rate = self.ctx.rate
        
        while not rospy.is_shutdown():
            
            # 获取视觉调整量用于精确定位
            visual_adj_x, visual_adj_y = self.ctx.get_visual_adjustment()
            adjusted_target_x = target_x + visual_adj_x
            adjusted_target_y = target_y + visual_adj_y
            
            # 计算到调整后目标的距离（高度+0.5m用于安全裕度）
            distance = self.ctx.get_length_between_points(adjusted_target_x, adjusted_target_y, drop_altitude+0.5)[0]
            
            # 检查是否足够接近（距离<0.1m）
            if distance < 0.1:
                rospy.loginfo("足够接近投放目标 (%.3f m)", distance)
                return "close_enough"
            
            # 导航到投放位置
            vx, vy, vz, vyaw = self.ctx.get_pid_vel(adjusted_target_x, adjusted_target_y, drop_altitude+0.5)
            
            # 发布速度指令
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x = vx/2.0
            twist.twist.linear.y = vy/2.0
            twist.twist.linear.z = vz
            twist.twist.angular.z = vyaw
            self.ctx.vel_pub.publish(twist)
            
            rate.sleep()


class VisualAlignment(smach.State):
    """视觉对齐状态：使用视觉反馈进行精确对齐"""
    
    def __init__(self, ctx):
        """初始化视觉对齐状态
        Args:
            ctx: 节点上下文对象
        """
        smach.State.__init__(
            self,
            outcomes=["aligned", "timeout"],  # aligned:对齐成功, timeout:超时
            input_keys=["drop_target_x", "drop_target_y", "drop_item_offset_x", 
                       "drop_item_offset_y", "drop_altitude", "drop_waypoint_name"],  # 输入参数
            output_keys=["drop_target_x_real", "drop_target_y_real"]  # 输出：实际投放位置
        )
        self.ctx = ctx
        self.alignment_start_time = None  # 对齐开始时间
        self.stable_alignment_start_time = None  # 稳定对齐开始时间
        # 视觉参数（可用 ROS 参数覆盖，方便现场调）
        self.visual_alignment_timeout = rospy.get_param('~visual_alignment_timeout', 3.0)  # 对齐超时（秒）
        self.stable_alignment_duration = rospy.get_param('~visual_stable_duration', 0.5)   # 稳定持续时间（秒）
        self.visual_error_threshold = rospy.get_param('~visual_error_threshold', 80.0)    # 误差阈值（像素）
        # True: 只有"真的检测到靶标"并稳定在阈值内才算对齐成功（推荐）
        #       检测不到就一直找，找到超时才按当前位置投放
        # False: 旧行为——收不到视觉反馈时直接判定"对齐"，会静默地不做视觉修正
        self.require_vision_align = rospy.get_param('~require_vision_align', True)
        self.last_lost_log_time = rospy.Time.now()

    def execute(self, userdata):
        """执行视觉对齐逻辑：通过视觉反馈进行精确对齐，记录最佳位置"""
        target_x = userdata.drop_target_x
        target_y = userdata.drop_target_y
        drop_altitude = userdata.drop_altitude
        waypoint_name = userdata.drop_waypoint_name
        
        rate = self.ctx.rate

        # 记录最小误差时的位置（用于最终投放）
        min_pose_x = self.ctx.current_pose.pose.position.x
        min_pose_y = self.ctx.current_pose.pose.position.y
        min_visual_error = 0xffff  # 初始化为最大值
        
        # 初始化对齐计时器
        if self.alignment_start_time is None:
            self.alignment_start_time = rospy.Time.now()
            rospy.loginfo("开始视觉对齐阶段：%s", waypoint_name)
        
        while not rospy.is_shutdown():
            
            # 获取视觉调整量用于精确定位
            visual_adj_x, visual_adj_y = self.ctx.get_visual_adjustment()
            adjusted_target_x = target_x + visual_adj_x
            adjusted_target_y = target_y + visual_adj_y
            
            # 检查视觉误差（回调已把"未检出(z<0)"过滤掉，received=False 表示失锁）
            visual_error = 0.0
            if self.ctx.target_center_diff_received:
                # 计算欧氏距离作为视觉误差
                visual_error = (self.ctx.target_center_diff.x**2 + self.ctx.target_center_diff.y**2)**0.5
                # 记录最小误差时的位置
                if visual_error < min_visual_error:
                    min_pose_x = self.ctx.current_pose.pose.position.x
                    min_pose_y = self.ctx.current_pose.pose.position.y
                    min_visual_error = visual_error
            
            # 检查超时
            alignment_elapsed = (rospy.Time.now() - self.alignment_start_time).to_sec()
            if alignment_elapsed > self.visual_alignment_timeout:
                rospy.logwarn("视觉对齐超时 (%.1fs)，继续投放", alignment_elapsed)
                self._reset_timers()
                # 超时则使用当前位置（加上舵机偏移）
                userdata.drop_target_x_real = self.ctx.current_pose.pose.position.x + userdata.drop_item_offset_x
                userdata.drop_target_y_real = self.ctx.current_pose.pose.position.y + userdata.drop_item_offset_y
                return "timeout"
            
            # 跟踪稳定对齐
            current_time = rospy.Time.now()
            # 判断对齐是否良好
            if self.require_vision_align:
                # 必须有真实检出 + 误差小于阈值
                alignment_is_good = (self.ctx.target_center_diff_received
                                     and visual_error < self.visual_error_threshold)
            else:
                alignment_is_good = (not self.ctx.target_center_diff_received
                                     or visual_error < self.visual_error_threshold)

            if not self.ctx.target_center_diff_received and \
                    (current_time - self.last_lost_log_time).to_sec() > 1.0:
                self.last_lost_log_time = current_time
                rospy.logwarn("未检测到靶标(视觉失锁)，继续搜索，剩余 %.1f s 后按当前位置投放",
                              max(0.0, self.visual_alignment_timeout - alignment_elapsed))
            
            if alignment_is_good:
                # 对齐良好，开始或继续稳定计时
                if self.stable_alignment_start_time is None:
                    self.stable_alignment_start_time = current_time
                    rospy.loginfo("达到良好视觉对齐 (%.1f px)，开始稳定性计时器", visual_error)
            else:
                # 对齐不良，重置稳定计时器
                if self.stable_alignment_start_time is not None:
                    rospy.loginfo("失去视觉对齐 (%.1f px)，重置稳定性计时器", visual_error)
                self.stable_alignment_start_time = None
            
            # 检查是否达到稳定对齐
            stable_elapsed = 0.0
            if self.stable_alignment_start_time is not None:
                stable_elapsed = (current_time - self.stable_alignment_start_time).to_sec()
                # 稳定对齐持续足够长时间，返回成功
                if stable_elapsed >= self.stable_alignment_duration:
                    rospy.loginfo("达到稳定对齐: %.1f px 持续 %.1fs", visual_error, stable_elapsed)
                    self._reset_timers()
                    # 使用最小误差时的位置（加上舵机偏移）
                    userdata.drop_target_x_real = min_pose_x + userdata.drop_item_offset_x
                    userdata.drop_target_y_real = min_pose_y + userdata.drop_item_offset_y
                    return "aligned"
            
            # 记录当前状态
            if self.ctx.target_center_diff_received:
                if alignment_is_good and self.stable_alignment_start_time is not None:
                    rospy.loginfo("稳定视觉对齐中: 误差=%.1f px, 稳定时间 %.1fs/%.1fs (总耗时: %.1fs)", 
                                 visual_error, stable_elapsed, self.stable_alignment_duration, alignment_elapsed)
                elif visual_error >= self.visual_error_threshold:
                    rospy.loginfo("视觉对齐中: 误差=%.1f px (目标 <%.0f px, 耗时: %.1fs)", 
                                 visual_error, self.visual_error_threshold, alignment_elapsed)

            # 使用PID控制飞向调整后的目标（高度0.8m）
            vx, vy, vz, vyaw = self.ctx.get_pid_vel(adjusted_target_x, adjusted_target_y, 0.8)
            
            # 发布速度指令
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x = vx/2.0
            twist.twist.linear.y = vy/2.0 
            twist.twist.linear.z = vz
            twist.twist.angular.z = vyaw
            self.ctx.vel_pub.publish(twist)
            
            rate.sleep()
    
    def _reset_timers(self):
        """重置计时器"""
        self.alignment_start_time = None
        self.stable_alignment_start_time = None


class DropAndHover(smach.State):
    """投放并悬停状态：执行物品投放并悬停指定时间"""
    
    def __init__(self, ctx, hover_duration=3.2, tolerance=0.05):
        """初始化投放并悬停状态
        Args:
            ctx: 节点上下文对象
            hover_duration: 悬停持续时间（秒）
            tolerance: 位置容差（米）
        """
        smach.State.__init__(
            self,
            outcomes=["dropped"],  # 投放完成
            input_keys=["drop_target_x", "drop_target_y", "drop_altitude", "drop_item", "drop_waypoint_name"],
        )
        self.ctx = ctx
        self.start_time = None  # 投放开始时间
        self.hover_duration = hover_duration  # 悬停时长
        self.tolerance = tolerance  # 位置容差

    def execute(self, userdata):
        """执行投放逻辑：到达位置后投放物品，然后悬停"""
        target_x = userdata.drop_target_x
        target_y = userdata.drop_target_y
        drop_altitude = userdata.drop_altitude
        item_name = userdata.drop_item
        waypoint_name = userdata.drop_waypoint_name
        
        rate = self.ctx.rate
        self.start_time = None
        rospy.loginfo("开始投放和悬停序列：%s", waypoint_name)
        
        while not rospy.is_shutdown():
            # 计算到目标位置的距离
            distance, _, _, _ = self.ctx.get_length_between_points(target_x, target_y, drop_altitude)

            # 到达目标位置后执行投放（仅执行一次）
            if distance < self.tolerance and self.start_time is None:
                #self.ctx.drop_item(item_name)
                self.start_time = rospy.Time.now()
                rospy.loginfo("即将投放物品 %s 在 %s", item_name, waypoint_name)
            
            # 检查悬停时间是否已足够(悬停一半时间投放)
            if self.start_time is not None and rospy.Time.now() - self.start_time > rospy.Duration(self.hover_duration)/2:
                self.ctx.drop_item(item_name)
                if self.start_time is not None and rospy.Time.now() - self.start_time > rospy.Duration(self.hover_duration):
                    rospy.loginfo("悬停结束，物品 %s 已成功投放", item_name)
                    return "dropped"

            # 使用PID控制保持位置
            vx, vy, vz, vyaw = self.ctx.get_pid_vel(target_x, target_y, drop_altitude)
            
            # 发布速度指令
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x = vx/2
            twist.twist.linear.y = vy/2
            twist.twist.linear.z = vz
            twist.twist.angular.z = vyaw
            self.ctx.vel_pub.publish(twist)
            rate.sleep()
    
    def _reset_timers(self):
        """重置计时器（供下次使用）"""
        self.drop_start_time = None
        self.hover_start_time = None
        self.item_dropped = False


class Drop(smach.StateMachine):
    """投放状态机：包含导航、视觉对齐、投放并悬停三个阶段"""
    
    def __init__(self, ctx):
        """初始化投放状态机
        Args:
            ctx: 节点上下文对象
        """
        smach.StateMachine.__init__(
            self,
            outcomes=["dropped"],  # 投放完成
            input_keys=["drop_target_x", "drop_target_y", "drop_item_offset_x", "drop_item_offset_y", 
                       "drop_altitude", "drop_item", "drop_waypoint_name"],
        )
        
        with self:
            # 阶段1：导航到投放位置上方
            smach.StateMachine.add(
                "NAVIGATE_TO_DROP",
                NavigateToDropPosition(ctx),
                transitions={"close_enough": "VISUAL_ALIGNMENT"},
                remapping={
                    "drop_target_x": "drop_target_x",
                    "drop_target_y": "drop_target_y",
                    "drop_altitude": "drop_altitude"
                }
            )
            
            # 阶段2：使用视觉反馈进行精确对齐
            smach.StateMachine.add(
                "VISUAL_ALIGNMENT",
                VisualAlignment(ctx),
                transitions={
                    "aligned": "DROP_AND_HOVER",  # 对齐成功
                    "timeout": "DROP_AND_HOVER"   # 超时也继续投放
                },
                remapping={
                    "drop_target_x": "drop_target_x",
                    "drop_target_y": "drop_target_y",
                    "drop_item_offset_x": "drop_item_offset_x",
                    "drop_item_offset_y": "drop_item_offset_y",
                    "drop_target_x_real": "drop_target_x_real",  # 对齐后的实际目标位置
                    "drop_target_y_real": "drop_target_y_real",
                    "drop_altitude": "drop_altitude",
                    "drop_waypoint_name": "drop_waypoint_name"
                }
            )
            
            # 阶段3：执行投放并悬停
            smach.StateMachine.add(
                "DROP_AND_HOVER",
                DropAndHover(ctx),
                transitions={"dropped": "dropped"},
                remapping={
                    "drop_target_x": "drop_target_x_real",  # 使用对齐后的位置
                    "drop_target_y": "drop_target_y_real",
                    "drop_altitude": "drop_altitude",
                    "drop_item": "drop_item",
                    "drop_waypoint_name": "drop_waypoint_name"
                }
            )


class DropComplete(smach.State):
    """投放完成状态：根据投放点决定下一步去向"""
    
    def __init__(self):
        """初始化投放完成状态"""
        smach.State.__init__(
            self, 
            outcomes=["to_rectangular_path", "to_waypoint1", "to_waypoint2", "to_waypoint4"],  # 可能的下一步
            input_keys=["drop_waypoint_name"],  # 输入：投放航点名称
            output_keys=[]
        )

    def execute(self, userdata):
        """根据投放点决定下一个目标"""
        waypoint_name = userdata.drop_waypoint_name
        rospy.loginfo("投放完成于 %s，确定下一个目的地", waypoint_name)
        
        # 根据投放点名称决定下一步
        if "Drop Point 1" in waypoint_name:
            return "to_rectangular_path"  # 投放点1后去矩形路径
        elif "Drop Point 2" in waypoint_name:
            return "to_waypoint1"  # 投放点2后去航点1
        elif "Drop Point 3" in waypoint_name:
            return "to_waypoint2"  # 投放点3后去航点2
        elif "Drop Point 4" in waypoint_name:
            return "to_waypoint4"  # 投放点4后去航点4
        else:
            rospy.logwarn("未知航点名称: %s，默认去矩形路径", waypoint_name)
            return "to_rectangular_path"


class Hover(smach.State):
    """悬停状态：在指定位置悬停一段时间"""
    
    def __init__(self, ctx, target_x, target_y, target_z, duration_sec=2.0):
        """初始化悬停状态
        Args:
            ctx: 节点上下文对象
            target_x, target_y, target_z: 目标位置（None表示使用当前位置）
            duration_sec: 悬停持续时间（秒）
        """
        smach.State.__init__(
            self, 
            outcomes=["hover_complete"]  # 悬停完成
        ) 
        self.ctx = ctx
        # 如果目标位置为None，使用当前位置
        if target_x is None:
            self.target_x = ctx.current_pose.pose.position.x
        else:
            self.target_x = target_x
        if target_y is None:
            self.target_y = ctx.current_pose.pose.position.y
        else:
            self.target_y = target_y
        if target_z is None:
            self.target_z = ctx.current_pose.pose.position.z
        else:
            self.target_z = target_z
        self.duration_sec = duration_sec

    def execute(self, userdata):
        """执行悬停逻辑：保持在目标位置并计时"""
        rospy.loginfo("在位置 (%.2f, %.2f, %.2f) 悬停 %.1f 秒", 
                     self.target_x, self.target_y, self.target_z, self.duration_sec)
        
        rate = self.ctx.rate
        hover_start_time = None
        
        while not rospy.is_shutdown():
            # 使用PID控制保持目标位置
            vx, vy, vz, vyaw = self.ctx.get_pid_vel(self.target_x, self.target_y, self.target_z)

            # 计算到目标位置的距离
            current_pose = self.ctx.current_pose.pose.position
            dx = self.target_x - current_pose.x
            dy = self.target_y - current_pose.y
            dz = self.target_z - current_pose.z
            dist = (dx**2 + dy**2 + dz**2) ** 0.5

            # 如果在目标位置0.1m范围内，认为到达
            target_reached = dist < 0.1

            # 到达目标后开始计时
            if target_reached and hover_start_time is None:
                hover_start_time = rospy.Time.now()
                rospy.loginfo("到达悬停目标，开始悬停计时")

            # 检查悬停时间是否已到
            if hover_start_time is not None:
                elapsed = (rospy.Time.now() - hover_start_time).to_sec()
                if elapsed >= self.duration_sec:
                    rospy.loginfo("悬停完成，已悬停 %.1f 秒", elapsed)
                    return "hover_complete"

            # 创建并发布速度指令
            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x = vx
            twist.twist.linear.y = vy
            twist.twist.linear.z = vz
            twist.twist.angular.z = vyaw
            self.ctx.vel_pub.publish(twist)
            
            rate.sleep()


class MissionCounter(smach.State):
    """任务计数器状态：跟踪任务圈数，判断是否完成所有圈"""
    
    def __init__(self, max_cycles):
        """初始化任务计数器
        Args:
            max_cycles: 最大圈数
        """
        smach.State.__init__(
            self, 
            outcomes=["continue", "mission_complete"],  # continue:继续下一圈, mission_complete:任务完成
            input_keys=["cycle_count"],  # 输入：当前圈数
            output_keys=["cycle_count"]  # 输出：更新后的圈数
        )
        self.max_cycles = max_cycles

    def execute(self, userdata):
        """执行计数逻辑：增加圈数并检查是否完成"""
        current_count = getattr(userdata, 'cycle_count', 0)
        current_count += 1  # 圈数加1
        userdata.cycle_count = current_count
        
        rospy.loginfo("完成第 %d/%d 圈任务", current_count, self.max_cycles)
        
        # 检查是否完成所有圈
        if current_count >= self.max_cycles:
            rospy.loginfo("所有任务圈已完成！")
            return "mission_complete"
        else:
            return "continue"


class Land(smach.State):
    """降落状态：先限速缓降到交接高度，再切 AUTO.LAND 收尾"""
    
    def __init__(self, ctx):
        """初始化降落状态
        Args:
            ctx: 节点上下文对象
        """
        smach.State.__init__(
            self, 
            outcomes=["landed"],  # 降落完成
            output_keys=["mission_summary"]  # 输出：任务摘要
        ) 
        self.ctx = ctx
        # 缓降参数（原来是直接从当前高度切 AUTO.LAND）
        self.descent_speed = rospy.get_param('~land_speed', 0.4)        # m/s
        self.descent_accel = rospy.get_param('~land_accel', 0.3)        # m/s²
        self.handover_alt = rospy.get_param('~land_handover_alt', 0.25) # 交飞控的高度（m）

    def execute(self, userdata):
        """执行降落逻辑：缓降到交接高度后切 AUTO.LAND"""
        rospy.loginfo("开始缓降: z=%.2f m -> %.2f m (限速 %.2f m/s, 限加速 %.2f m/s²), 之后交 AUTO.LAND",
                      self.ctx.current_pose.pose.position.z, self.handover_alt,
                      self.descent_speed, self.descent_accel)
        self.ctx.slew_altitude(self.handover_alt, self.descent_speed, self.descent_accel, 0.05)

        rate = self.ctx.rate
        while not rospy.is_shutdown():
            mode = self.ctx.current_state.mode
            z = self.ctx.current_pose.pose.position.z
            
            # 检查是否已切换到自动降落模式
            if mode == "AUTO.LAND":
                # 记录任务摘要
                userdata.mission_summary = {
                    'final_position': {
                        'x': self.ctx.current_pose.pose.position.x,
                        'y': self.ctx.current_pose.pose.position.y,
                        'z': self.ctx.current_pose.pose.position.z
                    },
                    'mission_completed': True
                }
                rospy.loginfo("任务完成！降落于 x=%.2f, y=%.2f, z=%.2f", 
                             self.ctx.current_pose.pose.position.x,
                             self.ctx.current_pose.pose.position.y,
                             self.ctx.current_pose.pose.position.z)
                return "landed"
            # 尝试设置为自动降落模式
            self.ctx.try_set_mode("AUTO.LAND", throttle_sec=0.5)
            # 发布零速度指令（接近地面时）
            self.ctx.publish_velocity(0.0, 0.0, 0.0)
            rate.sleep()


def main():
    """主函数：程序入口，初始化节点并构建状态机"""
    # 初始化ROS节点
    rospy.init_node("run_aic2025")

    # 获取任务参数
    fly_height = rospy.get_param('~fly_height', 1.3)  # 飞行高度（米）
    circle_count = rospy.get_param('~circle_count', 2)  # 任务圈数
    
    # 投放配置参数：每个投放点在第几圈投放
    drop_point1_circle = rospy.get_param('~drop_point1_circle', 1)  # 投放点1在第几圈投放
    drop_point2_circle = rospy.get_param('~drop_point2_circle', 1)  # 投放点2在第几圈投放
    drop_point3_circle = rospy.get_param('~drop_point3_circle', 2)  # 投放点3在第几圈投放
    drop_point4_circle = rospy.get_param('~drop_point4_circle', 2)  # 投放点4在第几圈投放
    
    # 投放物品配置：每个投放点使用哪个舵机
    drop_point1_item = rospy.get_param('~drop_point1_item', 'front_left')    # 可选：front_left, front_right, back_left, back_right
    drop_point2_item = rospy.get_param('~drop_point2_item', 'front_right')
    drop_point3_item = rospy.get_param('~drop_point3_item', 'back_left')
    drop_point4_item = rospy.get_param('~drop_point4_item', 'back_right')

    # 每个舵机的位置偏移（相对于无人机中心）
    drop_point1_item_offset_x = rospy.get_param(f'~{drop_point1_item}_offset_x', 0.0)
    drop_point1_item_offset_y = rospy.get_param(f'~{drop_point1_item}_offset_y', 0.0)
    drop_point2_item_offset_x = rospy.get_param(f'~{drop_point2_item}_offset_x', 0.0)
    drop_point2_item_offset_y = rospy.get_param(f'~{drop_point2_item}_offset_y', 0.0)
    drop_point3_item_offset_x = rospy.get_param(f'~{drop_point3_item}_offset_x', 0.0)
    drop_point3_item_offset_y = rospy.get_param(f'~{drop_point3_item}_offset_y', 0.0)
    drop_point4_item_offset_x = rospy.get_param(f'~{drop_point4_item}_offset_x', 0.0)
    drop_point4_item_offset_y = rospy.get_param(f'~{drop_point4_item}_offset_y', 0.0)
    
    # 投放高度
    drop_altitude = rospy.get_param('~drop_altitude', 0.4)
    
    # 打印任务参数
    rospy.loginfo("AIC2025 任务参数：")
    rospy.loginfo("- 飞行高度: %.2f m", fly_height)
    rospy.loginfo("- 任务圈数: %d", circle_count)
    rospy.loginfo("- 投放高度: %.2f m", drop_altitude)
    rospy.loginfo("- 投放点1: 第%d圈, 使用 %s", drop_point1_circle, drop_point1_item)
    rospy.loginfo("- 投放点2: 第%d圈, 使用 %s", drop_point2_circle, drop_point2_item)
    rospy.loginfo("- 投放点3: 第%d圈, 使用 %s", drop_point3_circle, drop_point3_item)
    rospy.loginfo("- 投放点4: 第%d圈, 使用 %s", drop_point4_circle, drop_point4_item)

    # 创建节点上下文
    ctx = NodeContext()

    # 创建投放配置字典
    drop_config = {
        'drop_point1_circle': drop_point1_circle,
        'drop_point2_circle': drop_point2_circle,
        'drop_point3_circle': drop_point3_circle,
        'drop_point4_circle': drop_point4_circle,
        'drop_point1_item': drop_point1_item,
        'drop_point2_item': drop_point2_item,
        'drop_point3_item': drop_point3_item,
        'drop_point4_item': drop_point4_item,
        'drop_point1_item_offset_x': drop_point1_item_offset_x,
        'drop_point1_item_offset_y': drop_point1_item_offset_y,
        'drop_point2_item_offset_x': drop_point2_item_offset_x,
        'drop_point2_item_offset_y': drop_point2_item_offset_y,
        'drop_point3_item_offset_x': drop_point3_item_offset_x,
        'drop_point3_item_offset_y': drop_point3_item_offset_y,
        'drop_point4_item_offset_x': drop_point4_item_offset_x,
        'drop_point4_item_offset_y': drop_point4_item_offset_y,
        'drop_altitude': drop_altitude
    }

    # 启动时关闭所有舵机
    ctx.close_all_servos()

    # 创建主状态机
    sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    sm.userdata.cycle_count = 0  # 初始化圈数为0

    with sm:
        # ========== 启动序列 ==========
        # 状态1：等待切换到OFFBOARD模式
        smach.StateMachine.add(
            "BEFORE_OFFBOARD",
            BeforeOffboard(ctx),
            transitions={"offboard": "BEFORE_ARMED"},
        )

        # 状态2：等待飞控解锁
        smach.StateMachine.add(
            "BEFORE_ARMED",
            BeforeArmed(ctx),
            transitions={"armed": "TAKEOFF"},
        )

        test_mode = False  # 测试模式：不执行任务航点序列(调试PID)
        if(test_mode):
            # 状态3：起飞到指定高度
            smach.StateMachine.add(
                "TAKEOFF",
                Takeoff(ctx, target_alt=fly_height),
                transitions={"at_altitude": "TPOINT1"},
                remapping={
                    "takeoff_start_pose": "takeoff_start_pose",
                    "takeoff_altitude": "takeoff_altitude"
                }
            )

            # ========== 任务航点序列 ==========
            smach.StateMachine.add(
                "TPOINT1",
                FlyToWaypoint(ctx, 1.0, 1.0, fly_height, "TPOINT1"),
                transitions={"reached": "TPOINT2", "drop_required": "failed"},
                remapping={
                    "cycle_count": "cycle_count"
                }
            )
            smach.StateMachine.add(
                "TPOINT2",
                FlyToWaypoint(ctx, -1.0, 1.0, fly_height, "TPOINT2"),
                transitions={"reached": "TPOINT3", "drop_required": "failed"},
                remapping={
                    "cycle_count": "cycle_count"
                }
            )
            smach.StateMachine.add(
                "TPOINT3",
                FlyToWaypoint(ctx, -1.0, -1.0, fly_height, "TPOINT3"),
                transitions={"reached": "TPOINT1", "drop_required": "failed"},
                remapping={
                    "cycle_count": "cycle_count"
                }
            )
        else:
            # 状态3：起飞到指定高度
            smach.StateMachine.add(
                "TAKEOFF",
                Takeoff(ctx, target_alt=fly_height),
                transitions={"at_altitude": "DROP_POINT1"},
                remapping={
                    "takeoff_start_pose": "takeoff_start_pose",
                    "takeoff_altitude": "takeoff_altitude"
                }
            )




        # 投放点1（坐标：3.3, 0.0）
        smach.StateMachine.add(
            "DROP_POINT1",
            FlyToWaypoint(ctx, 3.3, 0.0, fly_height, "Drop Point 1", 
                         drop_point_id=1, drop_config=drop_config,target_drop_altitude_dt=-0.1),
            transitions={
                "reached": "RECTANGULAR_PATH",  # 不投放则继续矩形路径
                "drop_required": "DROP"  # 需要投放则进入投放状态机
            },
            remapping={
                "cycle_count": "cycle_count",
                "drop_target_x": "drop_target_x",
                "drop_target_y": "drop_target_y",
                "drop_item_offset_x": "drop_item_offset_x",
                "drop_item_offset_y": "drop_item_offset_y",
                "drop_altitude": "drop_altitude",
                "drop_item": "drop_item",
                "drop_waypoint_name": "drop_waypoint_name"
            }
        )

        # 投放状态机（所有投放点共用）
        smach.StateMachine.add(
            "DROP",
            Drop(ctx),
            transitions={"dropped": "DROP_COMPLETE"},  # 投放完成后检查下一步
            remapping={
                "drop_target_x": "drop_target_x",
                "drop_target_y": "drop_target_y", 
                "drop_item_offset_x": "drop_item_offset_x",
                "drop_item_offset_y": "drop_item_offset_y",
                "drop_altitude": "drop_altitude",
                "drop_item": "drop_item",
                "drop_waypoint_name": "drop_waypoint_name"
            }
        )

        # 投放完成后的路由判断
        smach.StateMachine.add(
            "DROP_COMPLETE",
            DropComplete(),
            transitions={
                "to_rectangular_path": "RECTANGULAR_PATH",  # 投放点1后→矩形路径
                "to_waypoint1": "WAYPOINT1",  # 投放点2后→航点1
                "to_waypoint2": "BEFORE_WAYPOINT2",  # 投放点3后→航点2前
                "to_waypoint4": "WAYPOINT4"  # 投放点4后→航点4
            },
            remapping={"drop_waypoint_name": "drop_waypoint_name"}
        )

        # 矩形路径（绕障碍物）
        smach.StateMachine.add(
            "RECTANGULAR_PATH",
            RectangularPath(ctx, fly_height),
            transitions={"path_complete": "DROP_POINT2"},
        )

        # 投放点2（坐标：0.0, 2.2）
        smach.StateMachine.add(
            "DROP_POINT2",
            FlyToWaypoint(ctx, 0.0, 2.2, fly_height, "Drop Point 2", 
                         drop_point_id=2, drop_config=drop_config,target_drop_altitude_dt=0.05),
            transitions={
                "reached": "WAYPOINT1",  # 不投放则继续
                "drop_required": "DROP"  # 需要投放
            },
            remapping={
                "cycle_count": "cycle_count",
                "drop_target_x": "drop_target_x",
                "drop_target_y": "drop_target_y",
                "drop_item_offset_x": "drop_item_offset_x",
                "drop_item_offset_y": "drop_item_offset_y",
                "drop_altitude": "drop_altitude",
                "drop_item": "drop_item",
                "drop_waypoint_name": "drop_waypoint_name"
            }
        )

        # 航点1（坐标：-3.3, 2.2）
        smach.StateMachine.add(
            "WAYPOINT1",
            FlyToWaypoint(ctx, -3.55, 2.2, fly_height, "Waypoint 1"),
            transitions={"reached": "DROP_POINT3", "drop_required": "failed"},
            remapping={
                "cycle_count": "cycle_count"
            }
        )

        # 投放点3（坐标：-3.3, 0.0）
        smach.StateMachine.add(
            "DROP_POINT3",
            FlyToWaypoint(ctx, -3.3, 0.0, fly_height, "Drop Point 3", 
                         drop_point_id=3, drop_config=drop_config,target_drop_altitude_dt=0.1),
            transitions={
                "reached": "BEFORE_WAYPOINT2",  # 不投放则继续
                "drop_required": "DROP"  # 需要投放
            },
            remapping={
                "cycle_count": "cycle_count",
                "drop_target_x": "drop_target_x",
                "drop_target_y": "drop_target_y",
                "drop_item_offset_x": "drop_item_offset_x",
                "drop_item_offset_y": "drop_item_offset_y",
                "drop_altitude": "drop_altitude",
                "drop_item": "drop_item",
                "drop_waypoint_name": "drop_waypoint_name"
            }
        )

        # 航点1.5（坐标：-3.3, -0.4, 高度1.45m）- 过渡航点
        smach.StateMachine.add(
            "BEFORE_WAYPOINT2",
            FlyToWaypoint(ctx, -3.3, -0.4, 1.45, "Waypoint 1.5",tolerance=0.05),
            transitions={"reached": "WAYPOINT3", "drop_required": "failed"},
            remapping={
                "cycle_count": "cycle_count"
            }
        )

        # 航点2（坐标：-3.3, -1.3, 高度1.45m）
        """
        smach.StateMachine.add(
            "WAYPOINT2",
            FlyToWaypoint(ctx, -3.3, -1.3, 1.45, "Waypoint 2",tolerance=0.15),
            transitions={"reached": "WAYPOINT3", "drop_required": "failed"},
            remapping={
                "cycle_count": "cycle_count"
            }
        )
        """

        # 航点3（坐标：-3.3, -2.2）
        smach.StateMachine.add(
            "WAYPOINT3",
            FlyToWaypoint(ctx, -3.3, -2.3, 1.45, "Waypoint 3",tolerance=0.05),
            transitions={"reached": "DROP_POINT4", "drop_required": "failed"},
            remapping={
                "cycle_count": "cycle_count"
            }
        )

        # 投放点4（坐标：0, -2.2）
        smach.StateMachine.add(
            "DROP_POINT4",
            FlyToWaypoint(ctx, 0, -2.2, fly_height, "Drop Point 4", 
                         drop_point_id=4, drop_config=drop_config,target_drop_altitude_dt=0.05),
            transitions={
                "reached": "WAYPOINT4",  # 不投放则继续
                "drop_required": "DROP"  # 需要投放
            },
            remapping={
                "cycle_count": "cycle_count",
                "drop_target_x": "drop_target_x",
                "drop_target_y": "drop_target_y",
                "drop_item_offset_x": "drop_item_offset_x",
                "drop_item_offset_y": "drop_item_offset_y",
                "drop_altitude": "drop_altitude",
                "drop_item": "drop_item",
                "drop_waypoint_name": "drop_waypoint_name"
            }
        )

        # 航点4（坐标：3.3, -2.2）
        smach.StateMachine.add(
            "WAYPOINT4",
            FlyToWaypoint(ctx, 3.45, -2.2, fly_height, "Waypoint 4"),
            transitions={"reached": "DROP_POINT1_END", "drop_required": "failed"},
            remapping={
                "cycle_count": "cycle_count"
            }
        )

        # 返回投放点1结束位置（完成一圈）
        smach.StateMachine.add(
            "DROP_POINT1_END",
            FlyToWaypoint(ctx, 3.3, 0, fly_height, "Drop Point 1 End"),
            transitions={"reached": "CHECK_MISSION_COMPLETE", "drop_required": "failed"},
            remapping={
                "cycle_count": "cycle_count"
            }
        )

        # ========== 任务完成检查 ==========
        # 检查是否完成所有圈数
        smach.StateMachine.add(
            "CHECK_MISSION_COMPLETE",
            MissionCounter(circle_count),
            transitions={
                "continue": "DROP_POINT1",  # 继续下一圈
                "mission_complete": "LAND_POINT"  # 所有圈完成，去降落点
            },
            remapping={"cycle_count": "cycle_count"}
        )

        # 飞往降落点（坐标：0, 0）
        smach.StateMachine.add(
            "LAND_POINT",
            FlyToWaypoint(ctx, 0, 0, fly_height, "Land Point"),
            transitions={
                "reached": "LAND_POINT1",
                "drop_required": "failed"
            },
            remapping={
                "cycle_count": "cycle_count"
            }
        )

        # 降落前悬停（高度0.1m，）
        smach.StateMachine.add(
            "LAND_POINT1",
            FlyToWaypoint(ctx, 0, 0, 0.1, "Land Point1",tolerance=0.05),
            transitions={
                "reached": "LAND_POINT2",
                "drop_required": "failed"
            },
            remapping={
                "cycle_count": "cycle_count"
            }
        )
        smach.StateMachine.add(
            "LAND_POINT2",
            FlyToWaypoint(ctx, 0, 0, -0.3, "Land Point2",tolerance=0.1),
            transitions={
                "reached": "HOVER_BEFORE_LAND",
                "drop_required": "failed"
            },
            remapping={
                "cycle_count": "cycle_count"
            }
        )
        smach.StateMachine.add(
            "HOVER_BEFORE_LAND",
            Hover(ctx, 0.0, 0.0, -0.3, 3.0),
            transitions={"hover_complete": "LAND"},
        )

        # ========== 降落 ==========
        smach.StateMachine.add(
            "LAND",
            Land(ctx),
            transitions={"landed": "succeeded"},  # 降落成功，任务完成
            remapping={
                "mission_summary": "mission_summary"
            }
        )

    # 启动SMACH状态机可视化服务器
    sis = smach_ros.IntrospectionServer("smach_server", sm, "/run_aic2025")
    sis.start()
    rospy.loginfo("AIC2025任务已启动，SMACH可视化服务器已开启")

    # 执行状态机
    outcome = sm.execute()
    rospy.loginfo("AIC2025任务完成，结果: %s", outcome)
    sis.stop()


if __name__ == "__main__":
    """程序入口点"""
    main()
