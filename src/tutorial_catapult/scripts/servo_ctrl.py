#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机控制脚本（手动测试 + 任务前初始化）

舵机角度配置（来自 catapult_driver.launch，可改 launch 微调）：
  back_right  open=180  close=30    半关=105
  back_left   open=0    close=149   半关=75
  front_right open=0    close=149   半关=75
  front_left  open=180  close=30    半关=105

用法：
  ⭐ 起飞前初始化（半关预松，防卡死）：
    rosrun tutorial_catapult servo_ctrl.py init

  全部开 / 全部关 / 全部半关：
    rosrun tutorial_catapult servo_ctrl.py open_all
    rosrun tutorial_catapult servo_ctrl.py close_all
    rosrun tutorial_catapult servo_ctrl.py half_close_all

  单个仓位：
    rosrun tutorial_catapult servo_ctrl.py open  back_right
    rosrun tutorial_catapult servo_ctrl.py close back_left
    rosrun tutorial_catapult servo_ctrl.py half_close front_right

  依次测试 4 个仓位（开→停0.8s→半关）：
    rosrun tutorial_catapult servo_ctrl.py test

仓位名称：back_right / back_left / front_right / front_left
"""
import sys
import time
import rospy
from std_msgs.msg import Empty, Int32

SLOTS = ["back_right", "back_left", "front_right", "front_left"]

# open / close / half_close 角度（半关 = 中间值，留缝防卡）
ANGLES = {
    "back_right":  {"open": 180, "close": 30,  "half": 60},    # 原105 → 往close挪45°
    "back_left":   {"open": 0,   "close": 149, "half": 119},   # 原75 → 往close挪44°
    "front_right": {"open": 0,   "close": 149, "half": 119},   # 原75 → 往close挪44°
    "front_left":  {"open": 180, "close": 30,  "half": 60},    # 原105 → 往close挪45°
}

pubs_angle = {}   # 直接发角度的 publisher /servo/<slot>
pubs_open = {}    # 发 Empty 给 catapult_driver 的 /servo/<slot>/open
pubs_close = {}   # /servo/<slot>/close


def init_pubs():
    for s in SLOTS:
        pubs_angle[s] = rospy.Publisher("/servo/%s" % s, Int32, queue_size=1)
        pubs_open[s] = rospy.Publisher("/servo/%s/open" % s, Empty, queue_size=1)
        pubs_close[s] = rospy.Publisher("/servo/%s/close" % s, Empty, queue_size=1)
    rospy.sleep(0.5)


def _send_angle(slot, angle):
    pubs_angle[slot].publish(Int32(data=int(angle)))
    rospy.loginfo("  %s → %d°", slot, angle)


def open_all():
    rospy.loginfo("=== 全部开舱 ===")
    for s in SLOTS:
        _send_angle(s, ANGLES[s]["open"])
        rospy.sleep(0.12)


def close_all():
    rospy.loginfo("=== 全部关舱（全关） ===")
    for s in SLOTS:
        _send_angle(s, ANGLES[s]["close"])
        rospy.sleep(0.12)


def half_close_all():
    """半关预松（留缝防卡）——起飞前 init 和任务结束收尾推荐"""
    rospy.loginfo("=== 全部半关（预松防卡） ===")
    for s in SLOTS:
        _send_angle(s, ANGLES[s]["half"])
        rospy.sleep(0.12)


def open_one(slot):
    if slot not in pubs_angle:
        rospy.logerr("未知仓位: %s（可选 %s）", slot, SLOTS); return
    _send_angle(slot, ANGLES[slot]["open"])


def close_one(slot):
    if slot not in pubs_angle:
        rospy.logerr("未知仓位: %s（可选 %s）", slot, SLOTS); return
    _send_angle(slot, ANGLES[slot]["close"])


def half_close_one(slot):
    if slot not in pubs_angle:
        rospy.logerr("未知仓位: %s（可选 %s）", slot, SLOTS); return
    _send_angle(slot, ANGLES[slot]["half"])


def init_servos():
    """起飞前初始化：半关预松（留缝防卡）。
    直接发 half，投放时发 open 即可，不走 close 方向。"""
    rospy.loginfo("=== 舵机初始化：半关预松（留缝防卡） ===")
    half_close_all()
    rospy.sleep(0.5)
    rospy.loginfo("=== 初始化完成。飞行投放时发 open 直接全开 ===")


def test_sequence():
    """依次半关→开→停0.8s→半关，验证每个舵机"""
    rospy.loginfo("=== 舵机测试序列开始 ===")
    for s in SLOTS:
        rospy.loginfo("--- %s ---", s)
        _send_angle(s, ANGLES[s]["half"])
        rospy.sleep(0.3)
        _send_angle(s, ANGLES[s]["open"])
        rospy.sleep(0.8)
        _send_angle(s, ANGLES[s]["half"])
        rospy.sleep(0.3)
    rospy.loginfo("=== 测试序列结束，全部已半关 ===")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    rospy.init_node("servo_ctrl", anonymous=True)
    init_pubs()

    cmd = sys.argv[1]
    if cmd == "init":
        init_servos()
    elif cmd == "open_all":
        open_all()
    elif cmd == "close_all":
        close_all()
    elif cmd == "half_close_all":
        half_close_all()
    elif cmd == "test":
        test_sequence()
    elif cmd == "open":
        if len(sys.argv) < 3:
            print("用法: servo_ctrl.py open <slot>"); return
        open_one(sys.argv[2])
    elif cmd == "close":
        if len(sys.argv) < 3:
            print("用法: servo_ctrl.py close <slot>"); return
        close_one(sys.argv[2])
    elif cmd == "half_close":
        if len(sys.argv) < 3:
            print("用法: servo_ctrl.py half_close <slot>"); return
        half_close_one(sys.argv[2])
    else:
        print("未知命令:", cmd)
        print(__doc__)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
