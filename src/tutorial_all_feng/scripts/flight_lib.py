#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flight_lib.py —— MAVROS OFFBOARD 飞行控制基础库
任务节点、测试脚本共用。所有关键指令均“重复发送 + 状态回读确认”。
"""
import math
import time
import threading
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


def safe_rate_sleep(rate):
    """容忍 Gazebo /clock 回跳的 Rate.sleep。
    RTF 低/卡顿会导致仿真时间短暂倒退，普通 rate.sleep() 会抛
    ROSTimeMovedBackwardsException 进而杀死控制/setpoint 线程，这里吞掉并短等。"""
    try:
        rate.sleep()
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        time.sleep(0.02)
    except rospy.exceptions.ROSInterruptException:
        raise


def safe_sleep(t):
    """rospy.sleep 的时间回跳安全版。"""
    try:
        rospy.sleep(t)
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        time.sleep(min(t, 0.1))


def quat_from_yaw(yaw):
    p = PoseStamped().pose.orientation
    p.x, p.y = 0.0, 0.0
    p.z = math.sin(yaw / 2.0)
    p.w = math.cos(yaw / 2.0)
    return p


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FlightController:
    def __init__(self):
        self.state = State()
        self.pose = PoseStamped()
        self._have_pose = False
        self._sp = PoseStamped()
        self._sp.pose.orientation.w = 1.0
        self._stream_started = False
        # 安全中止标志：由 Z 看门狗等监控置位，各飞行原语循环检测到即返回 False
        self.abort_requested = False

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._pose_cb, queue_size=1)
        self.sp_pub = rospy.Publisher("/mavros/setpoint_position/local",
                                      PoseStamped, queue_size=10)
        # 速度控制（2025式穿环用）：_vel_mode=True 时流线程改发 cmd_vel
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",
                                       TwistStamped, queue_size=10)
        self._vel_mode = False
        self._vel_cmd = TwistStamped()
        self._vel_cmd.twist.linear.x = 0.0
        # 无限等待 MAVROS 服务（仿真启动慢，15s超时太紧，启动顺序反了就炸）
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service("/mavros/cmd/arming", timeout=5)
                rospy.wait_for_service("/mavros/set_mode", timeout=5)
                break
            except rospy.ROSException:
                rospy.logwarn("等待 MAVROS 服务...（仿真是否已启动？）")
        self.arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self._rate = rospy.Rate(20)
        self._lock = threading.Lock()
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

    # ---------- 基础回调 ----------
    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg
        self._have_pose = True

    def _stream_loop(self):
        """OFFBOARD 要求 setpoint 流不能中断（>2Hz），独立线程持续发送。
        绝不允许因 /clock 回跳等异常退出（否则 OFFBOARD 断流坠机）。"""
        while not rospy.is_shutdown():
            try:
                with self._lock:
                    if self._vel_mode:
                        self._vel_cmd.header.stamp = rospy.Time.now()
                        self._vel_cmd.header.frame_id = "map"
                        self.vel_pub.publish(self._vel_cmd)
                    else:
                        self._sp.header.stamp = rospy.Time.now()
                        self._sp.header.frame_id = "map"
                        self.sp_pub.publish(self._sp)
            except rospy.exceptions.ROSInterruptException:
                break
            except Exception:
                time.sleep(0.02)
                continue
            safe_rate_sleep(self._rate)

    def _set_target(self, x, y, z, yaw=None):
        with self._lock:
            self._sp.pose.position.x = x
            self._sp.pose.position.y = y
            self._sp.pose.position.z = z
            if yaw is None:
                self._sp.pose.orientation.w = 1.0
                self._sp.pose.orientation.x = 0.0
                self._sp.pose.orientation.y = 0.0
                self._sp.pose.orientation.z = 0.0
            else:
                q = quat_from_yaw(yaw)
                self._sp.pose.orientation = q

    # ---------- 状态查询 ----------
    @property
    def position(self):
        return self.pose.pose.position

    @property
    def yaw(self):
        return yaw_from_quat(self.pose.pose.orientation)

    def distance_to(self, x, y, z=None):
        p = self.position
        d = math.hypot(p.x - x, p.y - y)
        if z is not None:
            d = math.hypot(d, p.z - z)
        return d

    def wait_connected(self, timeout=30):
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.state.connected:
                return True
            if rospy.Time.now().to_sec() - t0 > timeout:
                return False
            safe_sleep(0.2)
        return False

    def wait_pose(self, timeout=15):
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and not self._have_pose:
            if rospy.Time.now().to_sec() - t0 > timeout:
                return False
            safe_sleep(0.1)
        return self._have_pose

    def request_abort(self, reason=""):
        """请求中止所有进行中的飞行原语（安全监控触发，如 Z 看门狗）。
        goto/fly_to/goto_vel/orbit 循环检测到后立即返回 False，
        主任务流程据此走 fc.land() 兜底降落。"""
        if not self.abort_requested:
            rospy.logerr("[安全] 请求中止当前动作：%s", reason)
        self.abort_requested = True

    # ---------- 模式 / 解锁 ----------
    def set_mode(self, mode, timeout=8):
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.state.mode == mode:
                return True
            try:
                self.mode_srv(custom_mode=mode)
            except rospy.ServiceException:
                pass
            if rospy.Time.now().to_sec() - t0 > timeout:
                return False
            safe_sleep(0.3)
        return False

    def arm(self, value=True, timeout=8):
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self.state.armed == value:
                return True
            try:
                self.arming_srv(value)
            except rospy.ServiceException:
                pass
            if rospy.Time.now().to_sec() - t0 > timeout:
                return False
            safe_sleep(0.3)
        return False

    def arm_and_offboard(self):
        """先以当前位置为期望持续发 setpoint，再切 OFFBOARD，最后解锁（带回读确认）"""
        if not self.wait_connected():
            return False, "FCU 未连接"
        if not self.wait_pose():
            return False, "无有效位置估计（检查 vision_pose）"

        p = self.position
        self._set_target(p.x, p.y, max(p.z, 0.3))
        safe_sleep(0.8)  # 让 setpoint 流先被 PX4 接收（原1.5s砍为0.8s）

        if not self.set_mode("OFFBOARD"):
            return False, "切换 OFFBOARD 失败"
        if not self.arm(True):
            return False, "解锁失败"
        rospy.loginfo("OFFBOARD 已进入，飞机已解锁")
        return True, "ok"

    # ---------- 航点飞行 ----------
    def goto(self, x, y, z, yaw=None, xy_tol=0.15, z_tol=0.10,
             timeout=25, arrive_h=0.3, max_jump=None):
        """飞向目标点；到达判定需稳定 arrive_h 秒。
        默认容差收紧到 xy_tol=0.15, z_tol=0.10，提升定位精度。
        max_jump: 相邻两帧位置跳变阈值（米），超过则判定定位异常立即中止。
                  正常飞行 20Hz 下每帧位移<0.05m；穿环等关键段设 0.6 兜底。"""
        self._set_target(x, y, z, yaw)
        t0 = rospy.Time.now().to_sec()
        stable_since = None
        last_p = None
        while not rospy.is_shutdown():
            if self.abort_requested:
                return False
            p = self.position
            if max_jump is not None and last_p is not None:
                jmp = math.hypot(p.x - last_p.x, p.y - last_p.y, p.z - last_p.z)
                if jmp > max_jump:
                    rospy.logerr("位置单帧跳变 %.2fm（定位异常/碰撞），中止 goto 目标(%.2f,%.2f,%.2f)",
                                 jmp, x, y, z)
                    return False
            last_p = p
            xy = math.hypot(p.x - x, p.y - y)
            dz = abs(p.z - z)
            if xy < xy_tol and dz < z_tol:
                if stable_since is None:
                    stable_since = rospy.Time.now().to_sec()
                elif rospy.Time.now().to_sec() - stable_since >= arrive_h:
                    return True
            else:
                stable_since = None
            if rospy.Time.now().to_sec() - t0 > timeout:
                rospy.logwarn("goto(%.2f,%.2f,%.2f) 超时，当前(%.2f,%.2f,%.2f) 剩余%.2fm",
                              x, y, z, p.x, p.y, p.z,
                              math.hypot(p.x - x, p.y - y))
                return False
            safe_rate_sleep(self._rate)
        return False

    # ---------- 速度PID控制（2025方案移植，穿环用） ----------
    def _set_vel(self, vx, vy, vz, vyaw=0.0):
        """进入速度模式并下发速度指令（流线程持续发 cmd_vel）。"""
        with self._lock:
            self._vel_mode = True
            t = self._vel_cmd.twist
            t.linear.x = vx
            t.linear.y = vy
            t.linear.z = vz
            t.angular.z = vyaw

    def _exit_vel_mode(self):
        """退出速度模式：位置setpoint锁定当前点，无缝切回位置悬停。"""
        p = self.position
        self._set_target(p.x, p.y, p.z)
        with self._lock:
            self._vel_mode = False

    def goto_vel(self, x, y, z, tol=0.12, timeout=15, speed=1.5, kp=1.0, hold=0.15):
        """2025式速度PID直飞（移植 ring_pass_through_node.cpp get_pid_vel）：
        位置误差→cmd_vel，纯比例 P=kp，>0.8m 处按 speed m/s 限速逼近。
        与 goto(位置setpoint)相比：无 max_jump 中止、无精细稳定判定，
        瞬时定位跳变不会中断动作——穿环等需要快速直穿的场景用。
        退出时自动切回位置模式悬停。返回是否到达。"""
        rate = rospy.Rate(20)
        t0 = rospy.Time.now().to_sec()
        stable_since = None
        try:
            while not rospy.is_shutdown():
                if self.abort_requested:
                    return False
                p = self.position
                ex, ey, ez = x - p.x, y - p.y, z - p.z
                dist = math.sqrt(ex * ex + ey * ey + ez * ez)
                if dist < tol:
                    if stable_since is None:
                        stable_since = rospy.Time.now().to_sec()
                    elif rospy.Time.now().to_sec() - stable_since >= hold:
                        return True
                else:
                    stable_since = None
                if dist > 0.8:
                    vx, vy, vz = ex * speed / dist, ey * speed / dist, ez * speed / dist
                else:
                    vx, vy, vz = kp * ex, kp * ey, kp * ez
                self._set_vel(vx, vy, vz, vyaw=-1.0 * self.yaw)  # 锁航向0
                if rospy.Time.now().to_sec() - t0 > timeout:
                    rospy.logwarn("goto_vel(%.2f,%.2f,%.2f) 超时，当前(%.2f,%.2f,%.2f) 剩余%.2fm",
                                  x, y, z, p.x, p.y, p.z, dist)
                    return False
                safe_rate_sleep(rate)
            return False
        finally:
            self._exit_vel_mode()

    def fly_to(self, x, y, z, tol=0.08, timeout=20, hold=0.15, xy_scale=1.0,
               vz_max=None, xy_max=None):
        """2025底层速度PID（完整移植 run_aic2025.py get_pid_vel，实机验证过）：
          远(>1m): xy 按 3.0m/s（可调 xy_max 参数降速）、z 按 1.0m/s 归一化限速直趋；
          近(<=1m): 全PID —— xy: P=2.028 D=0.255，z: P=2.333 Dz=0.343；
          航向锁定 yaw=0（P=1.0）。
        xy_scale<1 用于投放逼近等需慢速平移的场景（2025投放时 vx/2）。
        xy_max 硬限巡航段速度（m/s），首圈建议 2.5 观察定位，2圈后放 3.0。
        vz_max: z向速度限幅（m/s），低空贴地投放下降时防下冲过急。
        到达判定: 3D距离<tol 持续 hold 秒。退出后自动切回位置悬停。"""
        VEL_P, VEL_D = 2.028, 0.255
        VEL_Pz, VEL_Dz = 2.333, 0.343
        last_err = None
        last_t = None
        stable_since = None
        t0 = rospy.Time.now().to_sec()
        rate = rospy.Rate(20)
        if xy_max is None:
            xy_max = 2.0   # 默认 2.0 m/s（从 2.5 再降：P1 到东墙仅 1.2m，
                           # 2.5 刹停 1.56m 蹭墙；2.0 刹停 1.0m 刚好安全）
        try:
            while not rospy.is_shutdown():
                if self.abort_requested:
                    return False
                p = self.position
                ex, ey, ez = x - p.x, y - p.y, z - p.z
                dist = math.sqrt(ex * ex + ey * ey + ez * ez)
                # 微分项用位姿消息时间戳（仿真 RTF<1 时仍正确）
                stamp = self.pose.header.stamp.to_sec()
                if stamp == 0:
                    stamp = rospy.Time.now().to_sec()
                dt = stamp - last_t if last_t is not None else 0.0
                last_t = stamp

                if dist > 1.0:
                    vx, vy, vz = ex * xy_max / dist, ey * xy_max / dist, ez * 1.0 / dist
                    last_err = None  # 远处清微分，防接近时微分踢
                else:
                    dex = dey = dez = 0.0
                    if last_err is not None and 0 < dt < 0.2:
                        dex = (ex - last_err[0]) / dt
                        dey = (ey - last_err[1]) / dt
                        dez = (ez - last_err[2]) / dt
                    vx = VEL_P * ex + VEL_D * dex
                    vy = VEL_P * ey + VEL_D * dey
                    vz = VEL_Pz * ez + VEL_Dz * dez
                    last_err = (ex, ey, ez)

                vx *= xy_scale
                vy *= xy_scale
                if vz_max is not None:
                    vz = max(-vz_max, min(vz_max, vz))
                self._set_vel(vx, vy, vz, vyaw=-1.0 * self.yaw)

                if dist < tol:
                    if stable_since is None:
                        stable_since = rospy.Time.now().to_sec()
                    elif rospy.Time.now().to_sec() - stable_since >= hold:
                        return True
                else:
                    stable_since = None
                if rospy.Time.now().to_sec() - t0 > timeout:
                    rospy.logwarn("fly_to(%.2f,%.2f,%.2f) 超时，当前(%.2f,%.2f,%.2f) 剩余%.2fm",
                                  x, y, z, p.x, p.y, p.z, dist)
                    return False
                safe_rate_sleep(rate)
            return False
        finally:
            self._exit_vel_mode()

    def servo_by_vision(self, get_vision, height,
                        fx=1004.603218, fy=1004.603218,
                        cx=640.0, cy=360.0,
                        tol_pixel=28, hold_time=0.4, timeout=30.0,
                        rate_hz=8.0, kp=0.4, max_step=0.07,
                        min_conf=0.5, max_age=0.6):
        """视觉伺服闭环：实时读像素误差，微调飞机直到靶心红点居中。
        get_vision() -> (u, v, conf, age_仿真秒) 或 None。

        相机机腹 pitch=90° 朝下（几何确定符号）：
          靶在前方→画面上方→dv<0；闭环令  err_x = -dv·(h/fy)
          靶在右方→du>0；闭环令          err_y = -du·(h/fx)

        比例控制 + 小限幅：远时限幅快速接近，进入 ~0.17m 内步长线性减小，
        末段温和，避免低帧率(RTF≈0.2)下两帧之间冲过靶心。

        返回 (ok, x, y)；成功时靶心在相机光心正下方。"""
        rate = rospy.Rate(rate_hz)
        t0 = rospy.Time.now().to_sec()
        centered_since = None
        last_du = last_dv = 0

        while not rospy.is_shutdown():
            vis = get_vision()
            p = self.position
            if vis is None:
                rospy.logwarn_throttle(1.0, "[servo] 无视觉数据，保持悬停")
                self._set_target(p.x, p.y, height)
                centered_since = None
            else:
                u, v, conf, age = vis
                if conf < min_conf or age > max_age:
                    rospy.logwarn_throttle(1.0,
                        "[servo] 数据不可用 conf=%.2f age=%.2f，保持悬停", conf, age)
                    self._set_target(p.x, p.y, height)
                    centered_since = None
                else:
                    du = u - cx
                    dv = v - cy
                    # 像素误差 → 世界米制误差（pitch=90° 几何符号）
                    err_x = -dv * height / fy
                    err_y = -du * height / fx
                    # 比例步长 + 限幅
                    step_x = max(-max_step, min(max_step, err_x * kp))
                    step_y = max(-max_step, min(max_step, err_y * kp))
                    self._set_target(p.x + step_x, p.y + step_y, height)
                    rospy.loginfo_throttle(0.4,
                        "[servo] du=%4d dv=%4d err=(%.2f,%.2f) step=(%.3f,%.3f) pos=(%.2f,%.2f)",
                        du, dv, err_x, err_y, step_x, step_y, p.x, p.y)
                    last_du, last_dv = du, dv
                    if abs(du) < tol_pixel and abs(dv) < tol_pixel:
                        if centered_since is None:
                            centered_since = rospy.Time.now().to_sec()
                        elif rospy.Time.now().to_sec() - centered_since >= hold_time:
                            rospy.loginfo("[servo] 靶心已居中并保持 %.1fs，相机位置(%.2f,%.2f)",
                                          hold_time, p.x, p.y)
                            return True, p.x, p.y
                    else:
                        centered_since = None

            if rospy.Time.now().to_sec() - t0 > timeout:
                rospy.logwarn("[servo] 超时（末次 du=%d dv=%d）", last_du, last_dv)
                return False, p.x, p.y
            safe_rate_sleep(rate)
        return False, self.position.x, self.position.y

    def goto_visual_target(self, get_vision, height,
                           fx=1004.603218, fy=1004.603218,
                           cx=640.0, cy=360.0,
                           timeout=10.0, min_conf=0.15):
        """视觉一步对准：读取目标像素，直接计算目标世界坐标并 goto。
        比增量伺服快得多。get_vision() -> (u, v, conf, age) or None。
        返回 (ok, x, y)。"""
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            vis = get_vision()
            if vis is not None:
                u, v, conf, age = vis
                if conf >= min_conf and age < 0.6:
                    break
            if rospy.Time.now().to_sec() - t0 > timeout:
                rospy.logwarn("[goto_visual] 无视觉数据")
                return False, self.position.x, self.position.y
            safe_sleep(0.1)

        p = self.position
        # 像素偏差 → 世界系位移（相机下视，符号由实飞标定：v+→x-, u+→y-）
        dx = -(v - cy) * height / fy
        dy = -(u - cx) * height / fx
        tx = p.x + dx
        ty = p.y + dy
        rospy.loginfo("[goto_visual] H像素(%d,%d) → 目标(%.3f,%.3f) 偏移(%.3f,%.3f)",
                      int(u), int(v), tx, ty, dx, dy)
        ok = self.goto(tx, ty, height, xy_tol=0.08, z_tol=0.1, timeout=8)
        return ok, tx, ty

    def align_visual(self, get_vision, height,
                     fx=1004.603218, fy=1004.603218,
                     cx=640.0, cy=360.0,
                     tol_pixel=45, max_iters=6, settle=0.8,
                     gain=0.5, max_step=0.18,
                     bounds=(-4.2, 4.2, -2.8, 2.8),
                     min_conf=0.3, max_age=0.6,
                     frame_gap=0.15, timeout=40.0):
        """统一视觉精对准（投放/H 共用），专治"运动中采帧+单帧绝对 goto"发散：
          1) 每次先原地悬停 settle 秒，消除残余俯仰/平移后再采帧；
          2) 连续采 3 帧新鲜像素取中值，滤掉倾斜瞬帧/单帧跳变；
          3) 残差步进：只走 gain(0.6) 比例的位移，逐步逼近不冲过；
          4) 双边界钳制：单步 ≤max_step，目标点不得超出 bounds=(xmin,xmax,ymin,ymax)，
             视觉异常时飞机物理上不可能飞向墙/场外。
        get_vision() -> (u,v,conf,age) 或 None。
        返回 (是否收敛, x, y)。"""
        t0 = rospy.Time.now().to_sec()
        p = self.position
        self._set_target(p.x, p.y, height)

        for it in range(max_iters):
            if rospy.Time.now().to_sec() - t0 > timeout:
                rospy.logwarn("[align] 超时，末次像素未收敛")
                return False, p.x, p.y

            # 1) 悬停稳帧（抑制俯仰倾斜造成的像素投影误差）
            self._set_target(p.x, p.y, height)
            safe_sleep(settle)

            # 2) 采 3 帧新鲜数据取中值
            us, vs = [], []
            ft = rospy.Time.now().to_sec()
            while len(us) < 3 and not rospy.is_shutdown():
                vis = get_vision()
                if vis is not None:
                    u, v, conf, age = vis
                    if conf >= min_conf and age <= max_age:
                        us.append(u); vs.append(v)
                if rospy.Time.now().to_sec() - ft > 3.0:
                    break
                safe_sleep(frame_gap)
            if len(us) < 2:
                rospy.logwarn("[align] iter%d 有效帧不足(%d)，悬停等待", it, len(us))
                continue
            u = float(np.median(us)); v = float(np.median(vs))
            du = u - cx; dv = v - cy
            rospy.loginfo("[align] iter%d 采%d帧 像素(%d,%d) du=%d dv=%d pos=(%.2f,%.2f)",
                          it, len(us), int(u), int(v), int(du), int(dv), p.x, p.y)

            if abs(du) < tol_pixel and abs(dv) < tol_pixel:
                self._set_target(p.x, p.y, height)
                safe_sleep(0.2)
                rospy.loginfo("[align] 收敛，位置(%.3f,%.3f)", p.x, p.y)
                return True, p.x, p.y

            # 3) 残差比例步进（相机下视安装，符号由实飞数据标定：
            #    飞机x+→靶v+，故 dx 与 dv 反号；飞机y+→靶u+，故 dy 与 du 反号）
            dx = -dv * height / fy * gain
            dy = -du * height / fx * gain
            dx = max(-max_step, min(max_step, dx))
            dy = max(-max_step, min(max_step, dy))
            tx, ty = p.x + dx, p.y + dy

            # 4) 安全边界钳制
            if bounds is not None:
                xmin, xmax, ymin, ymax = bounds
                tx = max(xmin, min(xmax, tx))
                ty = max(ymin, min(ymax, ty))
            rospy.loginfo("[align] iter%d 步进(%.3f,%.3f) →(%.3f,%.3f)", it, tx - p.x, ty - p.y, tx, ty)
            self.fly_to(tx, ty, height, tol=0.08, timeout=8, xy_scale=0.5)
            p = self.position
        rospy.logwarn("[align] %d 次迭代未收敛", max_iters)
        return False, p.x, p.y

    def align_visual_2025(self, get_vision, base_x, base_y, height,
                          offset_x=0.0, offset_y=0.0,
                          tol_pixel=80, stable_time=0.5, timeout=5.0,
                          max_vel=0.35, gain=0.5):
        """2025式视觉对准：持续速度指令边飞边对，记录最佳位置。
        与旧迭代式区别：不停下来采帧，而是始终向"base+视觉修正"发速度指令；
        误差<tol_pixel 持续 stable_time 即成功；超时返回最佳记录位置。
        返回 (是否收敛, best_x, best_y)。"""
        rate = rospy.Rate(10)
        t0 = rospy.Time.now().to_sec()
        stable_start = None
        best_err = float('inf')
        best_x, best_y = base_x, base_y

        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now().to_sec()
                if now - t0 > timeout:
                    rospy.logwarn("[align2025] 超时，用最佳位置 (%.3f,%.3f) err=%.0fpx",
                                  best_x, best_y, best_err)
                    return False, best_x, best_y

                vis = get_vision()
                if vis is None:
                    safe_sleep(0.1)
                    continue
                u, v, conf, age = vis
                if conf < 0.15 or age > 0.8:
                    safe_sleep(0.1)
                    continue

                du = u - 640.0
                dv = v - 360.0
                err = math.hypot(du, dv)

                # 记录最佳位置
                p = self.position
                if err < best_err:
                    best_err = err
                    best_x, best_y = p.x, p.y

                # 计算目标点：基准点 + 视觉偏移 + 舱位偏移
                # 视觉偏移：像素误差 → 世界位移（下视相机，符号已标定）
                adj_x = -dv * height / 1004.0 * gain
                adj_y = -du * height / 1004.0 * gain
                tx = base_x + adj_x + offset_x
                ty = base_y + adj_y + offset_y

                # 稳定判定
                if err < tol_pixel:
                    if stable_start is None:
                        stable_start = now
                    elif now - stable_start >= stable_time:
                        rospy.loginfo("[align2025] 收敛 err=%.0fpx，最佳位置 (%.3f,%.3f)",
                                      err, best_x, best_y)
                        return True, best_x, best_y
                else:
                    stable_start = None

                # 速度指令飞向目标（限速，避免过冲）
                ex, ey, ez = tx - p.x, ty - p.y, height - p.z
                dist = math.sqrt(ex*ex + ey*ey + ez*ez)
                if dist < 0.03:
                    self._set_vel(0, 0, 0)
                else:
                    scale = min(1.0, dist / 0.5)
                    vx = ex * max_vel * scale / dist
                    vy = ey * max_vel * scale / dist
                    vz = ez * max_vel * scale / dist
                    self._set_vel(vx, vy, vz, vyaw=-1.0 * self.yaw)

                safe_rate_sleep(rate)
            return False, best_x, best_y
        finally:
            self._exit_vel_mode()

    def servo_to_pixel(self, get_vision, height,
                       target_du=0.0, target_dv=0.0,
                       fx=1004.603218, fy=1004.603218,
                       cx=640.0, cy=360.0,
                       tol_pixel=28, hold_time=0.4, timeout=30.0,
                       rate_hz=8.0, kp=0.4, max_step=0.07,
                       min_conf=0.5, max_age=0.6):
        """视觉伺服到【指定目标像素偏移】（靶心应出现在 cx+target_du, cy+target_dv）。
        用于投放：让靶心出现在对应舱位的像素位置，使舱位正压靶心。
        get_vision() -> (u, v, conf, age_仿真秒) 或 None。
        返回 (ok, x, y)。"""
        rate = rospy.Rate(rate_hz)
        t0 = rospy.Time.now().to_sec()
        centered_since = None
        last_du = last_dv = 0

        while not rospy.is_shutdown():
            vis = get_vision()
            p = self.position
            if vis is None:
                rospy.logwarn_throttle(1.0, "[servo] 无视觉数据，保持悬停")
                self._set_target(p.x, p.y, height)
                centered_since = None
            else:
                u, v, conf, age = vis
                if conf < min_conf or age > max_age:
                    rospy.logwarn_throttle(1.0,
                        "[servo] 数据不可用 conf=%.2f age=%.2f，保持悬停", conf, age)
                    self._set_target(p.x, p.y, height)
                    centered_since = None
                else:
                    du = u - cx - target_du
                    dv = v - cy - target_dv
                    err_x = -dv * height / fy
                    err_y = -du * height / fx
                    step_x = max(-max_step, min(max_step, err_x * kp))
                    step_y = max(-max_step, min(max_step, err_y * kp))
                    self._set_target(p.x + step_x, p.y + step_y, height)
                    rospy.loginfo_throttle(0.4,
                        "[servo] du=%4d dv=%4d err=(%.2f,%.2f) step=(%.3f,%.3f) pos=(%.2f,%.2f)",
                        du, dv, err_x, err_y, step_x, step_y, p.x, p.y)
                    last_du, last_dv = du, dv
                    if abs(du) < tol_pixel and abs(dv) < tol_pixel:
                        if centered_since is None:
                            centered_since = rospy.Time.now().to_sec()
                        elif rospy.Time.now().to_sec() - centered_since >= hold_time:
                            rospy.loginfo("[servo] 已对准目标像素并保持 %.1fs，位置(%.2f,%.2f)",
                                          hold_time, p.x, p.y)
                            return True, p.x, p.y
                    else:
                        centered_since = None

            if rospy.Time.now().to_sec() - t0 > timeout:
                rospy.logwarn("[servo] 超时（末次 du=%d dv=%d）", last_du, last_dv)
                return False, p.x, p.y
            safe_rate_sleep(rate)
        return False, self.position.x, self.position.y

    def goto_avoid(self, tx, ty, z, avoider,
                   goal_tol=0.30, approach_dist=0.50,
                   step_far=0.45, step_near=0.20,
                   rate_hz=8.0, timeout=45.0, blocked_timeout=4.0):
        """雷达避障飞向世界点 (tx,ty)。位置增量控制 + 扇区 VFH 选航向。
        - 接近目标 approach_dist 内信任目标点、直接精进入；
        - 雷达数据丢失：立即悬停（fail-fast，禁止盲飞）；
        - 四周堵死：悬停，超过 blocked_timeout 判失败。"""
        rate = rospy.Rate(rate_hz)
        t0 = rospy.Time.now().to_sec()
        blocked_since = None
        avoider.clear_memory()  # 新航段从干净栅格开始，避免上一机动(如绕桩)拖影

        while not rospy.is_shutdown():
            p = self.position
            yaw = self.yaw
            dxw, dyw = tx - p.x, ty - p.y
            dist = math.hypot(dxw, dyw)
            if dist < goal_tol:
                self._set_target(p.x, p.y, z)
                rospy.loginfo("[avoid] 到达目标(%.2f,%.2f)，剩余%.2fm", tx, ty, dist)
                return True

            # 接近目标：直接朝目标（目标点已知安全），但仍受硬停保护
            if dist < approach_dist:
                desired = math.atan2(dyw, dxw) - yaw
                heading = desired
                forced = True
            else:
                # 传感器丢失 → 悬停 fail-fast
                if not avoider.fresh:
                    self._set_target(p.x, p.y, z)
                    rospy.logwarn_throttle(1.0, "[avoid] 雷达数据丢失(age=%.1f)，悬停", avoider.age)
                    blocked_since = rospy.Time.now().to_sec()
                    if rospy.Time.now().to_sec() - t0 > timeout:
                        return False
                    safe_rate_sleep(rate)
                    continue
                desired = math.atan2(dyw, dxw) - yaw
                heading, blocked = avoider.choose_heading(desired)
                forced = False
                if blocked:
                    self._set_target(p.x, p.y, z)
                    now = rospy.Time.now().to_sec()
                    if blocked_since is None:
                        blocked_since = now
                    rospy.logwarn_throttle(1.0, "[avoid] 前方受阻，悬停等待")
                    if now - blocked_since > blocked_timeout:
                        rospy.logwarn("[avoid] 持续受阻超时")
                        return False
                    safe_rate_sleep(rate)
                    continue
                blocked_since = None

            # 硬安全：选定航向正前方小于 stop 距离则不前进（悬停）
            free = avoider.free_distance(math.atan2(math.sin(heading), math.cos(heading)))
            if (not forced) and free < avoider.stop_dist:
                self._set_target(p.x, p.y, z)
                safe_rate_sleep(rate)
                continue

            # 步长：近障减速，且不跨过目标
            step = step_near if free < avoider.slow_dist else step_far
            step = min(step, dist)
            ang = yaw + heading
            nx = p.x + step * math.cos(ang)
            ny = p.y + step * math.sin(ang)
            self._set_target(nx, ny, z)
            if not forced:
                des_free = avoider.free_distance(desired)
                rospy.loginfo_throttle(0.5,
                    "[avoid] 目标(%.1f,%.1f) dist=%.2f 期望%.0f°(空%.1f) 选向%.0f°(空%.1f) yaw%.0f pos=(%.2f,%.2f)",
                    tx, ty, dist, math.degrees(desired), des_free,
                    math.degrees(heading), free, math.degrees(yaw), p.x, p.y)

            if rospy.Time.now().to_sec() - t0 > timeout:
                rospy.logwarn("[avoid] goto_avoid 超时，当前(%.2f,%.2f) 剩%.2fm", p.x, p.y, dist)
                return False
            safe_rate_sleep(rate)
        return False

    def orbit(self, cx, cy, z, radius, ccw=True, lead=0.8,
              tol_rad=0.08, rate_hz=8.0, timeout=40.0,
              entry_ang_deg=None, entry_tol=0.30, entry_hold=0.0):
        """以(cx,cy)为圆心、radius为半径绕飞一整圈(2π)。
        用飞机相对圆心的真实方位角累计进度（不依赖时间，RTF 低也准），
        目标点始终超前 lead 弧度牵引。返回是否完成整圈。
        入轨：默认按当前方位角切入；指定 entry_ang_deg（世界系角度，
        逆时针 0°=正东）可选用切向与来向一致的点，实现不停顿平滑入圈
        （fly_to 飞越式入轨，不再 goto 停车+悬停0.4s）。"""
        direction = 1.0 if ccw else -1.0

        # 1) 飞到入轨点（飞越式，残余速度小，绕圈循环无缝续接）
        p = self.position
        if entry_ang_deg is None:
            a0 = math.atan2(p.y - cy, p.x - cx)
        else:
            a0 = math.radians(entry_ang_deg)
        sx, sy = cx + radius * math.cos(a0), cy + radius * math.sin(a0)
        if not self.fly_to(sx, sy, z, tol=entry_tol, timeout=15, hold=entry_hold):
            rospy.logwarn("[orbit] 进入圆周起点失败")
            return False

        rate = rospy.Rate(rate_hz)
        t0 = rospy.Time.now().to_sec()
        start_ang = a0
        last_log = 0.0

        # 2) traveled 初始对齐：把飞机当前方位角 cur 到 start_ang 的差算入 traveled，
        #    消除"入轨时偏了角度→先飞到 target 再绕回来"的折返
        p = self.position
        cur = math.atan2(p.y - cy, p.x - cx)
        d0 = direction * self._wrap_pi(cur - start_ang)
        if -0.5 < d0 < 0.5:
            traveled = max(0.0, d0)
        else:
            traveled = 0.0
        prev = cur

        while not rospy.is_shutdown() and traveled < 2 * math.pi - 0.05:
            if self.abort_requested:
                return False
            p = self.position
            cur = math.atan2(p.y - cy, p.x - cx)
            d = direction * self._wrap_pi(cur - prev)
            if 0 < d < 0.6:          # 合理增量才累计，抗跳变
                traveled += d
                prev = cur
            tgt_ang = start_ang + direction * (traveled + lead)
            tx = cx + radius * math.cos(tgt_ang)
            ty = cy + radius * math.sin(tgt_ang)
            self._set_target(tx, ty, z)

            now = rospy.Time.now().to_sec()
            if now - last_log > 1.0:
                last_log = now
                rospy.loginfo("[orbit] 已绕 %.0f/360° r=%.2f pos=(%.2f,%.2f)",
                              math.degrees(traveled),
                              math.hypot(p.x - cx, p.y - cy), p.x, p.y)
            if now - t0 > timeout:
                rospy.logwarn("[orbit] 绕圈超时(%.0f°)", math.degrees(traveled))
                return False
            safe_rate_sleep(rate)
        rospy.loginfo("[orbit] 完成整圈 360°")
        return True

    @staticmethod
    def _wrap_pi(a):
        return math.atan2(math.sin(a), math.cos(a))

    def land(self, timeout=30):
        """2025式降落：持续发零速度，同时尝试切 AUTO.LAND，直到上锁"""
        rate = rospy.Rate(10)
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            # 持续发零速度（保持 OFFBOARD 活跃）
            self._set_vel(0, 0, 0)
            # 尝试切 AUTO.LAND
            if self.state.mode != "AUTO.LAND":
                self.set_mode("AUTO.LAND", timeout=2)
            # 检查是否已上锁
            if not self.state.armed:
                rospy.loginfo("已降落上锁，位置 (%.2f, %.2f, %.2f)",
                              self.position.x, self.position.y, self.position.z)
                return True
            if rospy.Time.now().to_sec() - t0 > timeout:
                rospy.logwarn("降落超时，armed=%s mode=%s", self.state.armed, self.state.mode)
                return not self.state.armed
            safe_rate_sleep(rate)
        return not self.state.armed
