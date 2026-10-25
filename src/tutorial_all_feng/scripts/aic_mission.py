#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
全流程任务状态机（七圈制）：起飞 → 第1~6圈绕场穿环不投放 → 第7圈4点纯坐标投放 → 回H降落。

航点（2026-10-23 新场地实测校准）：
  起飞点(0,0)  投放点1(3.40,-0.18)  投放点2(0.10,2.19)  投放点3(-3.29,0.13)  投放点4(-0.07,-2.24)
  障碍物1(2.5,-1.3)  障碍物2(2.5,1.3)  障碍物3(-2.5,1.3)  圆环(-3.28,-1.19)

场地坐标历史：
  2026-10-17 场地：P1(3.27,-0.09) P2(0.26,2.24) P3(-3.38,0.10) P4(-0.16,-2.19)
                  BOX2(2.4,1.3) RING(-3.3,-1.3)
  2026-10-23 新场地：P1(3.40,-0.18) P2(0.10,2.19) P3(-3.29,0.13) P4(-0.07,-2.24)
                  BOX2(2.5,1.3) RING(-3.28,-1.19)

投放策略（仿真阶段简化版）：
  飞到实测靶心正上方 → 限速0.5m/s下降到DROP_Z(0.10m)贴地 → 投放 → 升回1.4m（纯坐标，不用视觉）
  不做视觉精细对准（仿真舱位偏移与实机不同，留待实机适配阶段处理）。
  实机阶段可在此处加入视觉伺服投放。
"""
import rospy
import math
import threading
import numpy as np
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import Altitude
from flight_lib import FlightController, safe_sleep
from avoid_lib import LidarAvoider

CRUISE_Z = 1.4
DROP_Z = 0.10          # 投放高度（由0.02抬高：减少贴地时地效晃动耦合到水平落点）
RING_Z = 1.6

# ===== 安全监控参数 =====
# 解锁前原点检查：飞机必须已摆在起飞点、静止且 FAST-LIO 原点正常
# （防 2026-10-18 事故：FAST-LIO 在错误状态初始化 → 起飞冲高/坐标系整体漂移）
GATE_XY_TOL = 0.5            # 原点水平偏差上限（m）
GATE_Z_RANGE = (-0.5, 0.8)   # 起飞前 z 合理区间（m，地面≈0，雷达离地≈0.1~0.3）
GATE_STILL_TOL = 0.05        # 静止采样期间最大漂移（m）
GATE_SAMPLE_T = 3.0          # 静止采样时长（s）
# 飞行中 Z 看门狗：z 估计越界并持续超时 → 判定定位发散，强制 AUTO.LAND
Z_WATCH_MIN = -0.6           # 正常飞行 z 下界（低于此=估计塌陷，飞机会追着爬升）
Z_WATCH_MAX = 2.4            # 正常飞行 z 上界（高于此=估计冲高，飞机会追着下潜）
Z_WATCH_T = 1.0              # 越界持续时长阈值（s）
# 气压交叉校验（方案B）：视觉 z 增量 与 气压高度增量 之差超阈 → 判定视觉 z 发散
Z_WATCH_BARO_TOL = 1.0       # 视觉z与气压高度增量差上限（m）
Z_WATCH_BARO_T = 1.5         # 差值持续时长阈值（s）

# 4个投放点坐标（2026-10-23 新场地里程计实测）
# 旧场地 2026-10-18: 1:(3.27,-0.09) 2:(0.26,2.24) 3:(-3.38,0.10) 4:(-0.16,-2.19)
DROP_POINTS = {
    1: (3.40, -0.18),
    2: (0.10, 2.19),
    3: (-3.29, 0.13),
    4: (-0.07, -2.24),
}
# 4个物块对应舱位
DROP_SLOTS = ["back_right", "back_left", "front_right", "front_left"]
# 舱位偏移（相对机体中心，实机标定后修改；仿真用0）
SLOT_OFFSETS = {
    "back_right":  (0.0, 0.0),
    "back_left":   (0.0, 0.0),
    "front_right": (0.0, 0.0),
    "front_left":  (0.0, 0.0),
}

# 障碍物2 绕圈（新场地柱心按 world 标准 (2.5,1.3)；实测绕柱半径 1.15）
# 旧场地 2026-10-17: BOX2=(2.4,1.3)（实测反推）
BOX2 = (2.5, 1.3)
ORBIT_R = 1.15

# 漂移补偿开关：实测绕场路线用标称坐标已很准（说明真实漂移≈0），
# 若环心采样有偏差会算出假 drift，反而把投放/降落推偏。
# False = 投放/降落直接用标称坐标（drift 仍照常估计并打日志，便于判断真实漂移量）
USE_DRIFT = False

# 圆环标称环心（新场地 2026-10-23 环前(-3.28,-0.75) 环后(-3.28,-1.62) 中点推算 (-3.28,-1.19)）
# z 暂用 world 标准 1.6（用户未实测环心高度）；旧场地: (-3.3,-1.3,1.6)
RING_EXPECT = (-3.28, -1.19, 1.6)

# 关键航点（2026-10-23 新场地校准）
# 旧场地 2026-10-17: P_BOX2_EAST=(3.7,1.3) P_BOX2_NORTH=(3.5,2.15)
#                     P_CORNER_DN=(-3.37,2.5) P_CORNER_UP=(3.4,-2.45)
# 绕完障碍物2后走外侧（东侧北上→北侧走廊西移）到投放点2，避免直线切柱西南角
P_BOX2_EAST = (3.7, 1.3, CRUISE_Z)    # BOX2(2.5,1.3)正东1.2m，东墙4.6留0.9m
P_BOX2_NORTH = (3.5, 2.20, CRUISE_Z)  # 沿东界北上到北侧走廊
P_CORNER_DN_UP = (0.0, 2.2, CRUISE_Z)
P_CORNER_DN = (-3.29, 2.5, CRUISE_Z)  # 北侧走廊（对齐新 P3 x=-3.29）
# 圆环穿环参数（速度PID直穿，穿环前后 z 统一 1.4）
RING_PASS_Z = 1.40
P_RING_ALIGN = (-3.28, -0.40, 1.40)   # 环前对轴点 y=-0.4（离环心0.79m，足够减速缓冲；原实测-0.75太近冲过）
P_RING_IN = (-3.28, -1.19, RING_Z)    # = RING_EXPECT 环心
P_RING_OUT = (-3.28, -2.24, 1.40)      # 穿环后直接接投放点4(y=-2.24)，z=1.4与穿环前统一
P_CORNER_UP_DN = (0.0, -2.2, CRUISE_Z)
P_CORNER_UP = (3.40, -2.40, CRUISE_Z)  # 南侧走廊（对齐新 P1 x=3.40）

_ring_center = None
_ring_time = 0.0
def _ring_cb(msg):
    global _ring_center, _ring_time
    _ring_center = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
    _ring_time = msg.header.stamp.to_sec()


class AICMission:
    def __init__(self):
        rospy.init_node("aic_mission")
        self.fc = FlightController()
        self.av = LidarAvoider()

        # 投放舵机话题（与实机 catapult_driver 接口一致）
        self.drop_pubs = {
            s: rospy.Publisher("/servo/%s/open" % s, Empty, queue_size=1)
            for s in DROP_SLOTS
        }

        # H 点视觉识别（降落用）
        self.h_center = None
        self.h_stamp = 0.0
        rospy.Subscriber("/aic/vision/h_center", PointStamped, self._cb_h)

        # 靶标视觉识别（投放对准用）
        self.target_center = None
        self.target_stamp = 0.0
        rospy.Subscriber("/aic/vision/target_center", PointStamped, self._cb_target)

        # FAST-LIO 里程计就绪标志（定位源，代替真值）
        self.odom_ready = False
        self.loc_source = "unknown"
        rospy.Subscriber("/Odometry", Odometry, self._cb_odom)

        # 雷达环心（穿环前现场采样）
        rospy.Subscriber("/ring_center", PoseStamped, _ring_cb)

        rospy.sleep(0.5)
        self.blocks_dropped = 0

        # 全场漂移估计：每圈穿环时由环心实测与标称的差更新，
        # 用于投放点/降落进场的坐标补偿（实测5圈重复性±1.3cm,偏差稳定6~9cm）
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_n_obs = 0        # 累计地标漂移观测次数（多观测融合用）

        # 气压高度（方案B：作为独立基准与视觉 z 交叉校验，判断视觉 z 是否发散）
        self._baro_alt = None
        rospy.Subscriber("/mavros/altitude", Altitude, self._cb_altitude)

        # Z 看门狗线程停止标志
        self._watch_stop = threading.Event()

    def _cb_target(self, msg):
        self.target_center = (msg.point.x, msg.point.y, msg.point.z)
        self.target_stamp = msg.header.stamp.to_sec()

    def get_target(self):
        if self.target_center is None:
            return None
        u, v, conf = self.target_center
        age = rospy.Time.now().to_sec() - self.target_stamp
        return (u, v, conf, age)

    def _cb_odom(self, msg):
        self.odom_ready = True

    def _cb_altitude(self, msg):
        # monotonic = 气压原始高度，不参与 EKF 融合（EKF2_BARO_CTRL=0），
        # 因此可作为判断视觉 z 是否发散的独立基准
        self._baro_alt = msg.monotonic

    def wait_odometry(self, timeout=30):
        """等定位源就绪：优先 FAST-LIO /Odometry（无坐标版），
        超时则降级 GT 真值（坐标版，full_sim）。"""
        rospy.loginfo("等待定位源 (FAST-LIO /Odometry) ...")
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and not self.odom_ready:
            if rospy.Time.now().to_sec() - t0 > timeout:
                # FAST-LIO 超时，检查是否有 GT 桥接可用
                topics = [t[0] for t in rospy.get_published_topics()]
                if "/mavros/vision_pose/pose" in topics:
                    rospy.logwarn("FAST-LIO 未启动，降级使用 GT 真值定位（坐标版）")
                    self.loc_source = "gt"
                    safe_sleep(1.0)
                    return True
                rospy.logerr("无定位源：FAST-LIO 和 GT 都未启动，检查仿真是否启动")
                return False
            safe_sleep(0.2)
        self.loc_source = "fastlio"
        # 再等1秒让 EKF 融合到 vision pose
        safe_sleep(1.0)
        rospy.loginfo("定位源: FAST-LIO 里程计（无坐标版）")
        return True

    def _cb_h(self, msg):
        self.h_center = (msg.point.x, msg.point.y, msg.point.z)
        self.h_stamp = msg.header.stamp.to_sec()

    def get_h(self):
        if self.h_center is None:
            return None
        u, v, conf = self.h_center
        age = rospy.Time.now().to_sec() - self.h_stamp
        return (u, v, conf, age)

    def wait_h(self, timeout=6.0, min_conf=0.2):
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t0 < timeout:
            h = self.get_h()
            if h is not None and h[2] >= min_conf and h[3] < 0.6:
                return True
            safe_sleep(0.1)
        return False

    def wait_radar(self, timeout=10):
        rospy.loginfo("等待雷达数据 ...")
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and not self.av.fresh:
            if rospy.Time.now().to_sec() - t0 > timeout:
                rospy.logerr("雷达数据超时")
                return False
            safe_sleep(0.2)
        rospy.loginfo("雷达就绪，前距 %.2fm", self.av.front_distance())
        return True

    def sanity_gate(self):
        """解锁前原点/高度/静止合理性检查（防 FAST-LIO 在错误状态下初始化导致的
        起飞冲高/全场航点整体偏移 —— 2026-10-18 实机事故根因）。
        要求：位姿有效、原点附近(|x|,|y|≤GATE_XY_TOL)、高度合理(z∈GATE_Z_RANGE)、
        静止(采样 GATE_SAMPLE_T 秒内最大漂移<GATE_STILL_TOL)。返回 (ok, msg)。"""
        fc = self.fc
        if not fc.wait_pose(timeout=5):
            return False, "无有效位置估计（检查 FAST-LIO / vision_pose）"
        xs, ys, zs = [], [], []
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t0 < GATE_SAMPLE_T:
            p = fc.position
            xs.append(p.x); ys.append(p.y); zs.append(p.z)
            safe_sleep(0.1)
        if len(xs) < 5:
            return False, "位姿采样不足，定位未就绪"
        n = float(len(xs))
        x0, y0, z0 = sum(xs) / n, sum(ys) / n, sum(zs) / n
        still = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        rospy.loginfo("[解锁前检查] 原点(%.3f,%.3f) z=%.3f 静止漂移%.3fm",
                      x0, y0, z0, still)
        if abs(x0) > GATE_XY_TOL or abs(y0) > GATE_XY_TOL:
            return False, ("原点偏差过大 (%.2f,%.2f)：FAST-LIO 可能未在起飞点初始化"
                           % (x0, y0))
        if not (GATE_Z_RANGE[0] <= z0 <= GATE_Z_RANGE[1]):
            return False, "起飞前 z=%.2fm 不合理：FAST-LIO z 通道异常" % z0
        if still > GATE_STILL_TOL:
            return False, ("静止漂移 %.3fm 过大：定位未收敛，请静置飞机后重启 FAST-LIO"
                           % still)
        return True, "ok"

    def _z_watchdog(self):
        """飞行中 Z 看门狗（独立线程），两层判定，防 2026-10-18 那种
        "追着错误 z 估计无限爬升"的失控。触发后的动作由 ROS 参数
        ~z_watchdog_action 决定：warn=仅告警不干预（首飞观察，默认）；
        abort=中止当前动作并强制 AUTO.LAND：

        ① 气压交叉校验（方案B）：视觉 z 增量 与 气压高度增量 之差超
           Z_WATCH_BARO_TOL 并持续 Z_WATCH_BARO_T 秒 → 视觉 z 发散。
           气压不参与 EKF 融合（EKF2_BARO_CTRL=0），是独立基准，能提前捕捉崩坏。
        ② 绝对边界兜底：z 越界 [Z_WATCH_MIN, Z_WATCH_MAX] 持续 Z_WATCH_T 秒。
        """
        # 等首帧气压数据，并同步取基线（z0 与 baro0 同一时刻，增量差才有效）
        t0 = rospy.Time.now().to_sec()
        while (not rospy.is_shutdown() and self._baro_alt is None
               and rospy.Time.now().to_sec() - t0 < 3.0):
            safe_sleep(0.1)
        baro0 = self._baro_alt
        z0 = self.fc.position.z
        if baro0 is None:
            rospy.logwarn("[Z看门狗] 无气压数据，仅用绝对边界兜底")

        # 触发动作：warn=仅告警（首飞观察数据用，默认）；abort=中止动作并强制降落
        action = rospy.get_param("~z_watchdog_action", "warn")
        rospy.loginfo("[Z看门狗] 动作模式=%s（warn=仅告警不干预 / abort=强制降落）", action)

        baro_live = None    # None=待判定 True/False=气压读数是否随高度变化
        bad_since = None
        diff_since = None
        last_warn = 0.0
        while not rospy.is_shutdown() and not self._watch_stop.is_set():
            now = rospy.Time.now().to_sec()
            z = self.fc.position.z
            b = self._baro_alt
            reason = None

            # 气压有效性判定：视觉已爬升>0.8m 而气压几乎不动 → 气压无效（读数冻结）
            if baro_live is None and baro0 is not None and b is not None:
                if abs(z - z0) > 0.8:
                    baro_live = abs(b - baro0) >= 0.3
                    if baro_live:
                        rospy.loginfo("[Z看门狗] 气压交叉校验已生效（基线 z=%.2f baro=%.2f）",
                                      z0, baro0)
                    else:
                        rospy.logwarn("[Z看门狗] 气压读数不随高度变化，判定气压无效，"
                                      "仅用绝对边界兜底")

            # ① 气压交叉校验
            if baro_live and b is not None:
                diff = (z - z0) - (b - baro0)
                if abs(diff) > Z_WATCH_BARO_TOL:
                    if diff_since is None:
                        diff_since = now
                    elif now - diff_since > Z_WATCH_BARO_T:
                        reason = ("视觉z增量%+.2fm 与 气压增量%+.2fm 差%+.2fm(>%.1fm)"
                                  % (z - z0, b - baro0, diff, Z_WATCH_BARO_TOL))
                else:
                    diff_since = None

            # ② 绝对边界兜底
            if reason is None:
                if z < Z_WATCH_MIN or z > Z_WATCH_MAX:
                    if bad_since is None:
                        bad_since = now
                    elif now - bad_since > Z_WATCH_T:
                        reason = ("z=%.2fm 越界(%.1f,%.1f)持续>%.1fs"
                                  % (z, Z_WATCH_MIN, Z_WATCH_MAX, Z_WATCH_T))
                else:
                    bad_since = None

            if reason is not None:
                if action == "abort":
                    if not self.fc.abort_requested:
                        self.fc.request_abort("Z看门狗: %s，判定高度估计发散" % reason)
                        self.fc.land()
                elif now - last_warn > 3.0:     # 仅告警模式：限频打印，不干预飞行
                    rospy.logwarn("[Z看门狗-仅告警] %s（当前不干预，仅记录；"
                                  "如需强制降落请设 ~z_watchdog_action:=abort）", reason)
                    last_warn = now
            safe_sleep(0.1)

    def land_with_vision(self):
        """先纯坐标飞到 H 点上空 → 悬停 1s → 看到 H 才修正，没看到就原地坐标降落。
        不再做网格搜索（视觉异常时四处飞会把降落带偏）。"""
        fc = self.fc
        # 降落回 H：默认直接用标称 (0,0)（USE_DRIFT 控制是否叠加漂移补偿）
        dx, dy = self._drift_xy()
        rospy.loginfo(">>> 返回 H 点（标称 + 补偿 %+.3f,%+.3f；估计漂移 %+.3f,%+.3f）",
                      dx, dy, self.drift_x, self.drift_y)
        if not fc.fly_to(dx, dy, CRUISE_Z, tol=0.25, timeout=30):
            rospy.logwarn(">>> 飞回 H 点超时，按当前位置降落")

        # 停稳 1s 再采帧（抑制残余俯仰/平移造成的像素误差）
        rospy.loginfo(">>> 悬停 1.0s 等相机稳定")
        rospy.sleep(1.0)

        hx, hy = self._drift_xy()
        found = False
        g = self.get_h()
        if g is not None and g[2] >= 0.15 and g[3] < 0.6:
            mx, my = self._landmark_odom(g, CRUISE_Z)
            off = math.hypot(mx - hx, my - hy)
            if off <= 0.9:
                hx, hy = mx, my
                found = True
                rospy.loginfo(">>> 看到 H，视觉修正 %.2fm → (%.2f,%.2f)", off, hx, hy)
                self._update_drift(hx, hy, 0.0, 0.0, "H点")
            else:
                rospy.logwarn(">>> H 换算偏移 %.2fm 超视野，判为野值，按坐标降落", off)
        else:
            rospy.logwarn(">>> 1.4m 没找到 H，按坐标降落 → (%.2f,%.2f)", hx, hy)

        if not found:
            # 坐标已由前几圈环心校准，收紧容差精确停到标称点上再降
            fc.fly_to(hx, hy, CRUISE_Z, tol=0.10, timeout=8)

        # 降到 0.8m
        rospy.loginfo(">>> 下降到 0.8m")
        fc.fly_to(hx, hy, 0.8, tol=0.1, timeout=15, vz_max=0.5)

        # 仅当高空看到了 H，才在低空做一次小范围精修
        if found:
            ok, hx2, hy2 = fc.goto_visual_target(self.get_h, 0.8, timeout=3.0)
            off2 = math.hypot(hx2 - hx, hy2 - hy) if ok else 99.0
            if ok and off2 <= 0.45:
                rospy.loginfo(">>> 低空精修 → (%.2f,%.2f)", hx2, hy2)
            elif ok:
                rospy.logwarn(">>> 低空视觉偏移 %.2fm 异常，忽略", off2)

        rospy.loginfo(">>> 悬停稳定 1.0s 后切 AUTO.LAND")
        rospy.sleep(1.0)
        fc.land()

    def drop_at(self, point_id):
        """纯坐标三段投放：飞到靶心上方→限速降到DROP_Z→悬停投放。
        四个靶心均里程计实测，弃用视觉对准（实飞漂移大、耗时）。
        漂移补偿：加上最近一圈穿环时环心实测偏差（≈FAST-LIO累计漂移）。"""
        dx, dy = self._drift_xy()
        tx = DROP_POINTS[point_id][0] + dx
        ty = DROP_POINTS[point_id][1] + dy
        slot = DROP_SLOTS[self.blocks_dropped]
        rospy.loginfo(">>> [投放点%d] 飞到 (%.2f,%.2f) 上方（补偿 %+.3f,%+.3f）",
                      point_id, tx, ty, dx, dy)
        if not self.fc.fly_to(tx, ty, CRUISE_Z, tol=0.15, timeout=30):
            rospy.logerr("未到投放点%d", point_id)
            return False

        # 靶心上空的视觉机会：直接测得靶心在里程计系中的位置 → 更新漂移估计，
        # 并据此重算下降坐标（纯坐标之上再加一层"投放前实时校准"）。
        # 视觉缺失/低置信 → 跳过，仍按漂移补偿坐标下降（不阻塞、不影响"只降一次"）。
        vis = self.get_target()
        if vis is not None and vis[2] >= 0.15 and vis[3] < 0.6:
            mx, my = self._landmark_odom(vis, self.fc.position.z)
            if self._update_drift(mx, my, DROP_POINTS[point_id][0],
                                  DROP_POINTS[point_id][1], "投放点%d靶心" % point_id):
                dx, dy = self._drift_xy()
                tx = DROP_POINTS[point_id][0] + dx
                ty = DROP_POINTS[point_id][1] + dy
                rospy.loginfo(">>> [投放点%d] 靶心视觉校准 → 下降坐标重算 (%.3f,%.3f)",
                              point_id, tx, ty)
        else:
            rospy.loginfo(">>> [投放点%d] 未见靶心（或无置信），按漂移补偿坐标下降",
                          point_id)

        # 低空下降限速 0.5m/s（原 fly_to <1m 段 PD 初始约2m/s，砸地易弹起）
        # 只下降一次、无论是否判定"到达"都投放：tol=0.10 收得偏紧，落点若有
        # 水平稳态偏差会让 3D 距离判不达标而误判"失败"，但此时高度通常已到位，
        # 投放没问题（历史上忽略返回值照投，4/4 全中）。故仅告警、不重试不跳过。
        rospy.loginfo(">>> [投放点%d] 限速下降到 %.2fm 投放", point_id, DROP_Z)
        if not self.fc.fly_to(tx, ty, DROP_Z, tol=0.10, timeout=15, vz_max=0.5):
            rospy.logwarn(">>> [投放点%d] 下降未达 tol=0.10（多为水平稳态偏差，"
                          "高度通常已到位），当前 z=%.2f，仍按计划投放",
                          point_id, self.fc.position.z)

        rospy.loginfo(">>> [投放点%d] 悬停0.8s，0.2s时投放 %s", point_id, slot)
        rospy.sleep(0.2)
        self.drop_pubs[slot].publish(Empty())
        self.blocks_dropped += 1
        rospy.sleep(0.6)

        rospy.loginfo(">>> [投放点%d] 升回巡航高度 %.1fm", point_id, CRUISE_Z)
        self.fc.fly_to(tx, ty, CRUISE_Z, tol=0.2, timeout=15)
        return True

    def _pass_or_drop(self, point_id, dropping):
        """两圈制共用入口：投放圈→纯坐标投放；计分圈→飞越靶心不停车。
        飞越用 tol=0.5 + hold=0：3D距离一进0.5m即返回，近段PD来不及收敛
        刹停就被下一航点接管，速度保持连续，轨迹在靶心处切角约0.5m。"""
        if dropping:
            return self.drop_at(point_id)
        tx, ty = DROP_POINTS[point_id]
        rospy.loginfo(">>> [计分圈] 飞越投放点%d (%.2f,%.2f) 不停车", point_id, tx, ty)
        return self.fc.fly_to(tx, ty, CRUISE_Z, tol=0.5, hold=0.0, timeout=20)

    # ---------- 里程计漂移：多观测地标校准 ----------
    def _landmark_odom(self, vis, height,
                       fx=1004.603218, fy=1004.603218,
                       cx=640.0, cy=360.0):
        """把一次视觉检测换算成「地标在里程计系下的坐标」。
        与 flight_lib.goto_visual_target 同一套换算（相机下视，height=当前离地高）。"""
        u, v = vis[0], vis[1]
        p = self.fc.position
        return p.x - (v - cy) * height / fy, p.y - (u - cx) * height / fx

    def _drift_xy(self):
        """当前生效的漂移补偿量。USE_DRIFT=False 时恒为 (0,0)，
        drift 仍照常估计并打日志（只观察不施加）。"""
        return (self.drift_x, self.drift_y) if USE_DRIFT else (0.0, 0.0)

    def _update_drift(self, meas_x, meas_y, nom_x, nom_y, tag,
                      cap=0.8, alpha=0.5):
        """用地标实测与标称之差更新全场漂移估计。
        ① 偏离当前估计超过 cap → 判野值丢弃，防单次误检带偏全场航点；
        ② 首次观测直接采纳，之后指数平滑融合，压掉单点 ±1.3cm 的观测噪声。"""
        ox, oy = meas_x - nom_x, meas_y - nom_y
        if abs(ox - self.drift_x) > cap or abs(oy - self.drift_y) > cap:
            rospy.logwarn("[漂移校准] %s 观测(%+.3f,%+.3f) 偏离当前估计(%+.3f,%+.3f)"
                          " 超 %.1fm，判为野值丢弃", tag, ox, oy,
                          self.drift_x, self.drift_y, cap)
            return False
        if self.drift_n_obs == 0:
            self.drift_x, self.drift_y = ox, oy
        else:
            self.drift_x = (1.0 - alpha) * self.drift_x + alpha * ox
            self.drift_y = (1.0 - alpha) * self.drift_y + alpha * oy
        self.drift_n_obs += 1
        rospy.loginfo("[漂移校准] %s 观测(%+.3f,%+.3f) → 估计(%+.3f,%+.3f) [第%d次]",
                      tag, ox, oy, self.drift_x, self.drift_y, self.drift_n_obs)
        return True

    def _near_ring(self, t=0.6, max_age=0.6):
        """近环悬停采样 t 秒，按消息时间戳去重，返回 xyz 中值。
        距环面 0.9m 正对时采（实测环前点），几何较准且距穿越仅几秒，漂移来不及变化。"""
        seen = -1.0
        samples = []
        t0 = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - t0 < t:
            now = rospy.Time.now().to_sec()
            if (_ring_center is not None and _ring_time != seen
                    and now - _ring_time < max_age):
                seen = _ring_time
                samples.append(_ring_center)
            safe_sleep(0.1)
        if len(samples) < 2:
            return None, len(samples)
        med = np.median(np.array(samples), axis=0)
        return tuple(med), len(samples)

    def pass_ring(self):
        """穿圆环（简化+漂移校准版）：环前对轴→轻量采样环心更新漂移→一步直穿到环后。
        穿环不分步（实飞验证直接飞准），但保留 0.3s 环心采样做 _update_drift——
        这是全场唯一的漂移估计更新源（除了降落那次太晚），砍掉会导致 7 圈跑完
        drift 始终为 0，降落飞回标称 (0,0) 但 FAST-LIO 已漂几十厘米，H 不在视野。"""
        cx = RING_EXPECT[0]
        rospy.loginfo(">>> [穿环] 对轴到环前 %s", P_RING_ALIGN)
        if not self.fc.fly_to(cx, P_RING_ALIGN[1], RING_PASS_Z,
                              tol=0.15, timeout=30):
            rospy.logerr("穿环对轴失败"); return False

        # 轻量采样环心更新漂移（0.15s 够采 1~2帧，再少会丢 drift 校准）
        near, nn = self._near_ring(t=0.15)
        if near is not None:
            self._update_drift(near[0], near[1],
                               RING_EXPECT[0], RING_EXPECT[1], "环心")
            rospy.loginfo("[穿环] 漂移更新 n=%d → drift(%+.3f,%+.3f)",
                          nn, self.drift_x, self.drift_y)
        else:
            rospy.logwarn("[穿环] 环心采样不足(n=%d)，跳过漂移更新", nn)

        # 直接速度PID穿到环后离场点（一步直穿，hold=0.0 不停省 0.15s）
        rospy.loginfo(">>> [穿环] 速度PID直穿 (%.2f,%.2f)→(%.2f,%.2f) speed=1.2",
                      cx, P_RING_ALIGN[1], cx, P_RING_OUT[1])
        if not self.fc.goto_vel(cx, P_RING_OUT[1], RING_PASS_Z,
                                tol=0.12, timeout=15, speed=1.2, hold=0.0):
            rospy.logerr("[穿环] 直穿失败，当前(%.2f,%.2f,%.2f)",
                         self.fc.position.x, self.fc.position.y, self.fc.position.z)
            return False

        self.av.clear_memory()
        return True

    def run(self):
        fc = self.fc
        av = self.av
        rospy.loginfo("========== AIC 全流程任务开始 ==========")

        if not self.wait_radar():
            return
        if not self.wait_odometry():
            return
        ok, msg = self.sanity_gate()
        if not ok:
            rospy.logerr("解锁前检查未通过：%s", msg)
            rospy.logerr("请确认：①飞机已摆在起飞点且静止 ②FAST-LIO 在飞机就位后启动 "
                         "③/Odometry 数值正常")
            return
        ok, msg = fc.arm_and_offboard()
        if not ok:
            rospy.logerr("解锁失败: %s", msg); return

        # 启动 Z 看门狗（安全监控：z 估计越界自动中止并降落）
        threading.Thread(target=self._z_watchdog, daemon=True).start()

        # 起飞
        # hold=0.5：到位后稳定悬停0.5s再动（0.8砍→0.5，起飞段短够稳，省0.3s/圈）
        if not fc.fly_to(0, 0, CRUISE_Z, tol=0.15, timeout=20, hold=0.5):
            rospy.logerr("起飞失败"); fc.land(); return
        rospy.loginfo("=== 起飞 %.1fm，悬停稳定完成 ===", CRUISE_Z)

        # ===== N 圈制主循环：第1~N-1圈计分不投放，第N圈投放 =====
        # 圈数由 rosparam ~n_laps 控制（默认 8），launch 覆盖示例：
        #   roslaunch tutorial_all_feng aic2026_task.launch aic_mission/n_laps:=1
        #   → 1圈即投放圈（直接 drop_at 4点），跳计分
        n_laps = rospy.get_param("~n_laps", 9)
        rospy.loginfo("===== 共 %d 圈：1~%d 计分不投放，第 %d 圈投放 =====",
                      n_laps, n_laps - 1, n_laps)
        for lap in range(1, n_laps + 1):
            dropping = (lap == n_laps)
            rospy.loginfo("========== 第 %d/%d 圈（%s） ==========",
                          lap, n_laps, "投放圈" if dropping else "计分圈不投放")

            # --- 投放点1 ---
            if not self._pass_or_drop(1, dropping): fc.land(); return

            # --- 绕障碍物2 一圈 ---
            rospy.loginfo(">>> 绕障碍物2(%s) 一圈 r=%.1f", BOX2, ORBIT_R)
            if not fc.orbit(BOX2[0], BOX2[1], CRUISE_Z, ORBIT_R, ccw=True,
                            timeout=60, entry_ang_deg=0.0, lead=0.8, rate_hz=15,
                            entry_tol=0.30, entry_hold=0.0):
                rospy.logerr("绕圈失败"); fc.land(); return
            av.clear_memory()

            # --- 投放点2 ---
            # orbit 完直接飞走廊入口（原经 P_BOX2_NORTH 冗余，砍）
            if dropping:
                if not self.drop_at(2): fc.land(); return
                if not fc.fly_to(*P_CORNER_DN_UP, tol=0.2, timeout=40):
                    rospy.logerr("受阻"); fc.land(); return
            else:
                if not fc.fly_to(*P_CORNER_DN_UP, tol=0.5, hold=0.0, timeout=40):
                    rospy.logerr("受阻"); fc.land(); return

            # --- 北侧走廊穿障碍物3外侧（hold=0 飞越不停，tol=0.4放大到 0.4m 即过即走）---
            # xy_scale=0.8：≈1.6m/s 刹停距离≈0.64m（PX4横向加速度≈2m/s²），安全
            if not fc.fly_to(*P_CORNER_DN, tol=0.4, hold=0.0, timeout=40, xy_scale=0.8):
                rospy.logerr("受阻"); fc.land(); return
            av.clear_memory()

            # --- 投放点3 ---
            if not self._pass_or_drop(3, dropping): fc.land(); return

            # --- 穿圆环（两圈都穿，穿环稳定且每圈计分） ---
            if not self.pass_ring():
                rospy.logerr("穿环失败"); fc.land(); return

            # --- 投放点4 ---
            # 计分圈：P4(-0.16,-2.19)与走廊入口(0,-2.2)几乎同点，合并为
            # 单个飞绝点(hold=0)，直接穿过去不停
            if dropping:
                if not self.drop_at(4): fc.land(); return
                if not fc.fly_to(*P_CORNER_UP_DN, tol=0.2, timeout=40):
                    rospy.logerr("受阻"); fc.land(); return
            else:
                if not fc.fly_to(*P_CORNER_UP_DN, tol=0.5, hold=0.0, timeout=40):
                    rospy.logerr("受阻"); fc.land(); return

            # --- 南侧走廊穿障碍物1外侧 ---
            # xy_scale=0.8：同上保守值
            # 南侧走廊 hold=0 飞越不停 + tol 放大
            if not fc.fly_to(*P_CORNER_UP, tol=0.4, hold=0.0, timeout=40, xy_scale=0.8):
                rospy.logerr("受阻"); fc.land(); return

            # --- 回投放点1（过+X轴计1圈）：直接飞 P1，砍中间 (3.5,-1.3) 冗余 ---
            # 计分圈飞越不停(hold=0)；投放圈保留停车为P1对准铺垫
            if dropping:
                if not fc.fly_to(DROP_POINTS[1][0], DROP_POINTS[1][1],
                                 CRUISE_Z, tol=0.25, timeout=30):
                    rospy.logerr("返回失败"); fc.land(); return
            else:
                if not fc.fly_to(DROP_POINTS[1][0], DROP_POINTS[1][1],
                                 CRUISE_Z, tol=0.5, hold=0.0, timeout=30):
                    rospy.logerr("返回失败"); fc.land(); return
            rospy.loginfo("===== 第 %d 圈完成（累计投放 %d/4） =====", lap, self.blocks_dropped)

        rospy.loginfo("==================================================")
        rospy.loginfo("  完成绕场 %d 圈，已投放 %d/4 个物块", n_laps, self.blocks_dropped)
        rospy.loginfo("==================================================")

        self.land_with_vision()
        self._watch_stop.set()
        rospy.loginfo("========== 全流程任务完成 ==========")


if __name__ == "__main__":
    try:
        AICMission().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("异常: %s", e)
        import traceback
        traceback.print_exc()
