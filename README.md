# 2026 AIC 无人机配送 —— 优化版代码（feng_ws）

本工作空间为 2026 AIC 无人机配送赛题的**优化版**实机代码，基于 ROS1 Noetic + MAVROS OFFBOARD + Livox Mid-360 / FAST-LIO 定位。

> 冻结基线版在 `~/catkin_ws/src/tutorials/tutorial_all`（只做坐标同步，不再改逻辑）。

---

## 1. 目录结构

```
feng_ws/src/
├── tutorial_all_feng/          # 主任务包
│   ├── scripts/
│   │   ├── aic_mission.py      # 全流程任务状态机（主程序）
│   │   ├── flight_lib.py       # 飞行控制基础库（fly_to / goto / goto_vel / orbit）
│   │   └── avoid_lib.py        # 激光避障
│   └── launch/
│       └── aic2026_task.launch # 任务启动（含环心检测）
├── tutorial_basic_feng/        # 基础：mavros PX4 启动
├── tutorial_navigation_feng/   # 定位：MID360 → FAST-LIO · 环心检测
├── tutorial_vision_feng/       # 视觉：下视相机驱动
├── tutorial_catapult/          # 投放舵机包
│   ├── src/catapult_driver.cpp # 舵机驱动（启动自动发 half 预松）
│   ├── scripts/servo_ctrl.py   # 手动舵机测试工具
│   └── launch/catapult_driver.launch
└── tutorial_gazebo/ tutorial_feng/   # 教程/仿真辅助
```

## 2. 任务流程（9 圈制）

起飞 → 第 1~8 圈绕场穿环（不投放）→ **第 9 圈 4 点投放** → 回 H 降落。

单圈路径：
起飞点(0,0) → 投放点1 → 障碍物1 → 障碍物2（orbit 绕柱，R=1.15）→ 东界北上 → 北侧走廊 → 投放点2 → 投放点3 → 环前对轴 → 穿环（goto_vel 直穿）→ 环后 → 投放点4 → 南侧走廊回起点。

## 3. 启动命令

```bash
# 1. 舵机驱动（启动即自动 half 预松舱门，防卡死）
roslaunch tutorial_catapult catapult_driver.launch

# 2. 主任务（默认 8 个绕场圈 + 1 投放圈 = 9 圈制）
rosrun tutorial_all_feng aic_mission.py

# 指定圈数（如只飞 1 圈直投，用于调试）
rosrun tutorial_all_feng aic_mission.py aic_mission/n_laps:=1
```

更多命令见 `~/feng_ws_commands.txt`（飞行/舵机/监控/快照速查表）。

## 4. 舵机控制

舱门策略：**全程不走 close 方向**（易卡）。
- 起飞前：`half` 半关预松（60°/119°）
- 投放时：`open` 全开（0°/180°）
- 降落结束：回 `half`

手动测试：

```bash
rosrun tutorial_catapult servo_ctrl.py init           # 半关初始化
rosrun tutorial_catapult servo_ctrl.py open back_right # 开指定舱
rosrun tutorial_catapult servo_ctrl.py test            # 全舱遍历测试
```

舱位对应：投放点1→back_right，2→back_left，3→front_right，4→front_left。

## 5. 场地坐标（2026-10-23 新场地）

| 点 | 坐标 (x, y) |
|---|---|
| 投放点1 | (3.40, -0.18) |
| 投放点2 | (0.10, 2.19) |
| 投放点3 | (-3.29, 0.13) |
| 投放点4 | (-0.07, -2.24) |
| 障碍物2（绕柱） | (2.5, 1.3)，绕柱半径 1.15m |
| 环心 | (-3.28, -1.19, z≈1.6) |

旧场地坐标在 `aic_mission.py` 中以注释保留。

## 6. 关键参数（aic_mission.py / flight_lib.py）

| 参数 | 值 | 说明 |
|---|---|---|
| `USE_DRIFT` | **False** | 漂移补偿只估计打日志、不施加（实测标称坐标已很准，假 drift 会推偏落点） |
| `xy_max` | 2.0 m/s | 水平限速（刹停≈1.0m，P1 到东墙余量 1.2m，防蹭墙） |
| `RING_PASS_Z` | 1.40 | 穿环前后高度统一 1.4m |
| `P_RING_ALIGN` | (-3.28, -0.40) | 环前对轴点（y=-0.4 留减速缓冲） |
| 投放悬停 | 0.8s | sleep(0.2)+sleep(0.6) |
| orbit lead / rate | 0.8 / 15Hz | 绕柱牵引角与发布频率 |
| `~n_laps` | 8 | 绕场圈数（第 9 圈投放），rosparam 可覆盖 |

## 7. 安全机制

- **解锁前 sanity_gate**：原点偏差 ≤0.5m、z∈(-0.5,0.8)、3s 静止漂移 <0.05m，否则拒绝起飞（防 FAST-LIO 错误初始化）
- **Z 看门狗**：飞行中视觉 z 与气压增量交叉校验 + 绝对边界 [-0.6, 2.4] 兜底，发散则告警/强制 AUTO.LAND
- **降落**：坐标飞到 H 上空 1.4m → 悬停 1s → 采一帧 H 视觉（>0.9m 判野值丢弃）→ 看到才精确修正 → 降 0.8m → AUTO.LAND（坐标为主，视觉为辅）

## 8. 快照

```bash
# 当前基线（drift 修复版）
~/tutorial_all_feng_snapshot_v6_driftfix_20261023.tar.gz
```

## 9. 常见问题

- **roslaunch 刷屏 error / 全节点被 kill**：旧版 serial_node `required="true"` 连锁崩溃，已删除该属性；确认当前 launch 文件无 required
- **舵机首包丢失**：catapult_driver 启动会自动发 half，无需在任务脚本里内嵌初始化
- **圈数参数不生效**：ROS1 语法是 `aic_mission/n_laps:=1`，不是 `_n_laps:=1`
- **绕圈起点折返**：已修（orbit 入轨时把初始角度偏差计入 traveled），若复现检查 fly_to 入轨容差
