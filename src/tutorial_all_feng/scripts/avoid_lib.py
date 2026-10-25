#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
avoid_lib.py —— 基于 Livox mid360 点云的反应式避障（简化 VFH 扇区直方图）

输入：/livox/lidar  (livox_ros_driver2/CustomMsg)，frame=laser_livox（机体系 x前 y左 z上）
输出：LidarAvoider 提供
  - fresh / age          数据是否新鲜（丢失时上层必须悬停，禁止盲飞）
  - free_distance(theta) 机体系某航向(弧度)前方的最近通行距离
  - choose_heading(desired)  给定期望航向，返回 (航向, 是否受阻)
  - front_distance()     正前方最近障碍距离（调试/硬停用）

设计：
  3D 点云按高度层过滤（滤地面/天花板）→ 水平投影 → 90 个 4° 扇区取最近距离；
  扇区按障碍距离动态膨胀（给机体半径留裕度）；用 0.5s 滑动最小值填补 mid360
  非重复扫描的瞬时空洞，同时避免长期拖影。
"""
import math
import threading
from collections import deque
import numpy as np
import rospy
from livox_ros_driver2.msg import CustomMsg


class LidarAvoider:
    def __init__(self, topic="/livox/lidar"):
        # ---- 可调参数 ----
        self.n_sec = 90                     # 扇区数（每扇区 4°）
        self.z_min, self.z_max = -0.7, 0.6  # 机体高度层（米），滤地面/天花板
        self.r_min, self.r_max = 0.20, 6.0  # 有效水平距离
        self.robot_radius = 0.35            # 机体等效半径（桨盘+裕度）
        self.stop_dist = 0.60               # 小于此距离视为危险
        self.slow_dist = 0.80               # 小于此距离开始绕行/减速
        self.memory_sec = 0.3               # 滑动帧缓冲窗口（仿真秒）
        self.min_pts = 2                    # 窗口内每扇区至少点数，滤偶发单点(桨叶等)
        self.search_half = math.radians(70) # 优先在期望航向 ±70° 内找通道
        self.max_turn = math.radians(40)    # 单周期航向最大变化（抗抖）
        self.data_timeout = 0.8             # 数据超过此时长视为丢失

        self.sec_w = 2.0 * math.pi / self.n_sec
        self._lock = threading.Lock()
        # 帧缓冲：每元素 (t, 每扇区最小距离[inf表示无点], 每扇区点数)
        # 旧帧整体淘汰，杜绝单扇区在 minimum 累积 + 时间戳抖动下残留幽灵障碍
        self._frames = deque()
        self._last_stamp = 0.0
        self._have = False
        self._last_heading = None

        rospy.Subscriber(topic, CustomMsg, self._cb, queue_size=1)

    # ---------- 点云回调 ----------
    def _cb(self, msg):
        n = len(msg.points)
        if n == 0:
            return
        pts = np.empty((n, 3), dtype=np.float32)
        for i, p in enumerate(msg.points):
            pts[i, 0] = p.x
            pts[i, 1] = p.y
            pts[i, 2] = p.z

        # 高度层过滤
        m = (pts[:, 2] > self.z_min) & (pts[:, 2] < self.z_max)
        pts = pts[m]
        if len(pts) == 0:
            self._mark_stamp(msg.header.stamp.to_sec())
            return
        r = np.hypot(pts[:, 0], pts[:, 1])
        m = (r > self.r_min) & (r < self.r_max)
        pts, r = pts[m], r[m]
        if len(pts) == 0:
            self._mark_stamp(msg.header.stamp.to_sec())
            return

        ang = np.arctan2(pts[:, 1], pts[:, 0])         # [-pi, pi]
        sec = np.clip(((ang + math.pi) / self.sec_w).astype(int),
                      0, self.n_sec - 1)

        gmin = np.full(self.n_sec, np.inf, dtype=np.float64)
        gcnt = np.zeros(self.n_sec, dtype=np.int32)
        np.minimum.at(gmin, sec, r)
        np.add.at(gcnt, sec, 1)

        t = msg.header.stamp.to_sec()
        with self._lock:
            self._frames.append((t, gmin, gcnt))
            # 整体淘汰窗口外旧帧（以最新帧时间为基准，抗时间戳轻微抖动）
            t_new = t
            while self._frames and (t_new - self._frames[0][0]) > self.memory_sec:
                self._frames.popleft()
            self._last_stamp = t
            self._have = True

    def _mark_stamp(self, t):
        with self._lock:
            self._last_stamp = t
            self._have = True

    def _aggregate(self):
        """聚合滑动窗口：逐扇区取最小距离，点数不足 min_pts 视为无点。"""
        with self._lock:
            frames = list(self._frames)
        if not frames:
            return np.full(self.n_sec, np.inf)
        grid = np.full(self.n_sec, np.inf, dtype=np.float64)
        cnt = np.zeros(self.n_sec, dtype=np.int32)
        for _, gmin, gcnt in frames:
            better = np.isfinite(gmin) & (gmin < grid)
            grid[better] = gmin[better]
            cnt += gcnt
        grid[cnt < self.min_pts] = np.inf
        return grid

    # ---------- 状态查询 ----------
    @property
    def fresh(self):
        if not self._have:
            return False
        with self._lock:
            return (rospy.Time.now().to_sec() - self._last_stamp) < self.data_timeout

    @property
    def age(self):
        with self._lock:
            return rospy.Time.now().to_sec() - self._last_stamp if self._have else 9e9

    def _blocked_grid(self):
        """返回膨胀后的障碍距离栅格。"""
        grid = self._aggregate()
        # 动态膨胀：越近的障碍，需要两侧留越大的角度裕度
        infl = np.zeros(self.n_sec, dtype=int)
        for i in range(self.n_sec):
            if np.isfinite(grid[i]):
                ang = math.atan2(self.robot_radius, max(grid[i], 0.05))
                infl[i] = int(math.ceil(ang / self.sec_w))
        blocked = grid.copy()
        if infl.max() > 0:
            k = infl.max()
            padded = np.concatenate([grid[-k:], grid, grid[:k]]) if k > 0 else grid
            out = np.full(self.n_sec, np.inf)
            for i in range(self.n_sec):
                seg = padded[i:i + 2 * k + 1]
                out[i] = seg.min()
            blocked = out
        return blocked

    def free_distance(self, theta_body):
        """机体系航向 theta_body(弧度，0=正前方) 的最近通行距离。"""
        idx = int(((theta_body + math.pi) / self.sec_w)) % self.n_sec
        return float(self._blocked_grid()[idx])

    def front_distance(self):
        return self.free_distance(0.0)

    def choose_heading(self, desired):
        """给定期望机体系航向 desired，选一个可通行航向。
        返回 (heading, blocked)：
          blocked=False 时 heading 为建议航向；
          blocked=True  表示四周都被堵死（上层应悬停/后退）。"""
        desired = math.atan2(math.sin(desired), math.cos(desired))
        g = self._blocked_grid()

        def sector_free(theta):
            idx = int(((theta + math.pi) / self.sec_w)) % self.n_sec
            return g[idx]

        # 期望方向若有足够净空，直奔目标
        des_free = sector_free(desired)
        if des_free > self.slow_dist + 0.1:
            return self._smooth(desired), False

        # 期望方向受阻：在 ±search_half 内选可通行航向。
        # 策略：只要净空 > stop_dist（安全通过），就优先选最接近期望的方向，
        # 避免为了"大净空"而大幅偏离目标走弯路。只有危险方向(cl<=stop)才跳过。
        best, best_cost = None, 1e9
        last_side = 0.0  # 上一帧相对期望的偏差符号
        if self._last_heading is not None:
            last_side = math.atan2(
                math.sin(self._last_heading - desired),
                math.cos(self._last_heading - desired))
        steps = int(2 * self.search_half / self.sec_w)
        for k in range(steps + 1):
            theta = -self.search_half + k * self.sec_w
            cand = desired + theta
            cand = math.atan2(math.sin(cand), math.cos(cand))
            cl = sector_free(cand)
            if cl <= self.stop_dist:
                continue  # 危险方向不考虑
            dev = math.atan2(math.sin(cand - desired), math.cos(cand - desired))
            # 代价=偏差（净空够大就不惩罚，只选最接近期望的）
            cost = abs(dev)
            # 净空特别小(刚过stop线)时轻微惩罚，避免贴障太近
            if cl < self.stop_dist + 0.3:
                cost += 0.3 * (self.stop_dist + 0.3 - cl)
            # 绕侧惯性惩罚（只有当上一帧已明显偏离时才生效）
            if abs(last_side) > 0.2 and dev * last_side < 0:
                cost += 0.6
            if cost < best_cost:
                best_cost, best = cost, cand
        if best is not None:
            return self._smooth(best), False

        # 全向找最远通道
        best, best_d = None, -1.0
        for i in range(self.n_sec):
            if g[i] > best_d:
                best_d, best = g[i], -math.pi + (i + 0.5) * self.sec_w
        if best_d > self.stop_dist:
            return self._smooth(best), False

        # 全堵死
        return 0.0, True

    def _smooth(self, heading):
        """限制单周期航向跳变，抑制抖动。输出归一化到 [-π, π]。"""
        if self._last_heading is None:
            self._last_heading = heading
            return heading
        diff = math.atan2(math.sin(heading - self._last_heading),
                          math.cos(heading - self._last_heading))
        diff = max(-self.max_turn, min(self.max_turn, diff))
        out = self._last_heading + diff
        out = math.atan2(math.sin(out), math.cos(out))  # 归一化
        self._last_heading = out
        return out

    def reset_hysteresis(self):
        self._last_heading = None

    def clear_memory(self):
        """清空滑动帧缓冲（绕桩等机动后调用，避免旧方位障碍点拖影误导下一段）。"""
        with self._lock:
            self._frames.clear()
        self.reset_hysteresis()
