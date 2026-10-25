#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
起降专项测试节点（不跑任务，只练降落）
=========================================
每轮流程：原地起飞到1.4m → 悬停1.5s → 1.4m粗对准 → 0.8m精对准 → AUTO.LAND
全程 10Hz 记录：里程计(x,y,z)、H像素(u,v,conf,age)、阶段
每轮结束输出：
  - 粗/精对准偏移量（= 里程计漂移的视觉实测值）
  - 预测落地误差（精对准后 H 剩余像素 → 换算地面距离，唯一不依赖里程计的真值）
N 轮后汇总：均值/最大/标准差，用来判断降落方案是否收敛。

用法：
  先起底层：roslaunch tutorial_all_feng aic2026_task.launch  （会带任务，别用）
  正确做法：单独起底层 + 本测试节点，见下方命令汇总
    终端1: roslaunch tutorial_basic mavros.launch
    终端2: roslaunch tutorial_navigation mid360.launch
    终端3: rosrun tutorial_vision simple_camera_driver.launch（或对应launch）
           rosrun tutorial_all_feng vision_detector.py
    终端4: rosrun tutorial_all_feng test_landing.py _rounds:=5

输出：tutorial_all_feng/results/landing_test_<时间戳>.csv
"""
import csv
import math
import os
import time

import rospy
import rospkg
from geometry_msgs.msg import PointStamped

from flight_lib import FlightController, safe_sleep

CRUISE_Z = 1.4
FINE_Z = 0.8
FX = FY = 1004.603218
CX, CY = 640.0, 360.0


class LandingTest(object):
    def __init__(self):
        rospy.init_node("test_landing")
        self.rounds = int(rospy.get_param("~rounds", 5))
        self.fc = FlightController()

        self.h = None          # (u, v, conf)
        self.h_t = 0.0
        rospy.Subscriber("/aic/vision/h_center", PointStamped, self._cb_h)

        pkg = rospkg.RosPack().get_path("tutorial_all_feng")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.out = os.path.join(pkg, "results", "landing_test_%s.csv" % stamp)
        os.makedirs(os.path.dirname(self.out), exist_ok=True)
        self.f = open(self.out, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["wall_t", "round", "phase", "x", "y", "z",
                         "u", "v", "conf", "age",
                         "h_pix_err", "pred_gnd_err_m"])
        self.phase = "INIT"
        self.cur_round = 0
        rospy.loginfo("记录文件：%s", self.out)

    # ------------------------------------------------- vision
    def _cb_h(self, msg):
        self.h = (msg.point.x, msg.point.y, msg.point.z)
        self.h_t = msg.header.stamp.to_sec()

    def get_h(self):
        if self.h is None:
            return None
        u, v, conf = self.h
        age = rospy.Time.now().to_sec() - self.h_t
        return (u, v, conf, age)

    def pred_gnd_err(self, z):
        """当前H像素偏差 → 预测落地水平误差（米）。无视觉返回 None。"""
        g = self.get_h()
        if g is None or g[2] < 0.1 or g[3] > 0.6:
            return None
        u, v = g[0], g[1]
        dx = -(v - CY) * z / FY
        dy = -(u - CX) * z / FX
        return math.hypot(dx, dy)

    # ------------------------------------------------- logging
    def logger_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            p = self.fc.position
            g = self.get_h()
            if g is not None:
                u, v, conf, age = g
                pix = math.hypot(u - CX, v - CY)
                pred = self.pred_gnd_err(p.z)
            else:
                u = v = conf = age = -1
                pix = -1
                pred = None
            self.w.writerow([round(time.time(), 3), self.cur_round, self.phase,
                             round(p.x, 4), round(p.y, 4), round(p.z, 4),
                             round(u, 1), round(v, 1), round(conf, 3), round(age, 3),
                             round(pix, 1), (round(pred, 4) if pred is not None else "")])
            self.f.flush()
            rate.sleep()

    # ------------------------------------------------- one round
    def do_round(self, r):
        fc = self.fc
        self.cur_round = r
        rospy.loginfo("============ 第 %d/%d 轮 ============", r, self.rounds)

        self.phase = "ARM"
        if not fc.arm_and_offboard():
            rospy.logerr("解锁/OFFBOARD 失败，终止")
            return None

        self.phase = "TAKEOFF"
        p = fc.position
        if not fc.fly_to(p.x, p.y, CRUISE_Z, tol=0.15, timeout=25, hold=1.5):
            rospy.logerr("起飞失败"); fc.land(); return None

        # 粗对准（1.4m，视野±0.9m）
        self.phase = "COARSE"
        ok_c, hx, hy = fc.goto_visual_target(self.get_h, CRUISE_Z, timeout=5.0)
        d_coarse = None
        if ok_c:
            hx = max(-1.2, min(1.2, hx)); hy = max(-1.2, min(1.2, hy))
            pp = fc.position
            d_coarse = math.hypot(hx - pp.x, hy - pp.y)
        else:
            hx, hy = fc.position.x, fc.position.y

        # 下降
        self.phase = "DESCEND"
        fc.fly_to(hx, hy, FINE_Z, tol=0.1, timeout=15, vz_max=0.5)

        # 精对准（0.8m）
        self.phase = "FINE"
        ok_f, hx2, hy2 = fc.goto_visual_target(self.get_h, FINE_Z, timeout=5.0)
        if ok_f:
            hx2 = max(-1.2, min(1.2, hx2)); hy2 = max(-1.2, min(1.2, hy2))

        # 精对准后静止采 1s，取预测落地误差（视觉真值）
        safe_sleep(1.0)
        preds = []
        t0 = time.time()
        while time.time() - t0 < 1.0:
            e = self.pred_gnd_err(FINE_Z)
            if e is not None:
                preds.append(e)
            safe_sleep(0.1)
        pred_final = sorted(preds)[len(preds) // 2] if preds else None

        # 降落
        self.phase = "LAND"
        fc.land()
        t0 = time.time()
        while time.time() - t0 < 20:
            if not fc.state.armed:
                break
            safe_sleep(0.2)
        p_end = fc.position

        self.phase = "LANDED"
        res = dict(round=r,
                   coarse_ok=ok_c, coarse_d=(round(d_coarse, 3) if d_coarse else None),
                   fine_ok=ok_f,
                   pred_err=(round(pred_final, 4) if pred_final is not None else None),
                   land_x=round(p_end.x, 3), land_y=round(p_end.y, 3))
        rospy.loginfo("第%d轮: 粗对准%s(偏移%s m) 精对准%s 预测落地误差%s m 落点odom(%.2f,%.2f)",
                      r, "OK" if ok_c else "失败", res["coarse_d"],
                      "OK" if ok_f else "失败",
                      res["pred_err"], p_end.x, p_end.y)
        safe_sleep(3.0)   # 落地后静置再上锁下一次
        return res

    # ------------------------------------------------- main
    def run(self):
        import threading
        t = threading.Thread(target=self.logger_loop)
        t.daemon = True
        t.start()

        if not self.fc.wait_connected():
            rospy.logerr("FCU 未连接"); return
        if not self.fc.wait_pose():
            rospy.logerr("无位姿"); return
        rospy.loginfo("底层就绪，3秒后开始第1轮（确保人员撤离）")
        safe_sleep(3.0)

        results = []
        for r in range(1, self.rounds + 1):
            if rospy.is_shutdown():
                break
            res = self.do_round(r)
            if res is None:
                break
            results.append(res)

        # ---------- 汇总 ----------
        preds = [x["pred_err"] for x in results if x["pred_err"] is not None]
        coarse_ok = sum(1 for x in results if x["coarse_ok"])
        fine_ok = sum(1 for x in results if x["fine_ok"])
        rospy.loginfo("================ 起降测试汇总 ================")
        rospy.loginfo("有效轮次 %d，粗对准成功 %d，精对准成功 %d",
                      len(results), coarse_ok, fine_ok)
        if preds:
            mean = sum(preds) / len(preds)
            var = sum((e - mean) ** 2 for e in preds) / len(preds)
            rospy.loginfo("预测落地误差: 均值 %.3f m  最大 %.3f m  标准差 %.3f m",
                          mean, max(preds), math.sqrt(var))
        rospy.loginfo("详细数据：%s", self.out)
        self.phase = "DONE"


if __name__ == "__main__":
    try:
        LandingTest().run()
    except rospy.ROSInterruptException:
        pass
