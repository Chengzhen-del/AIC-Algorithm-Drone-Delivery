# 🛩️ 2026 AIC 无人机配送 —— 优化版实机代码（feng_ws）

> **ROS 1 · MAVROS OFFBOARD · Livox Mid-360 + FAST-LIO 视觉惯性定位 · 无人机精准配送**

本工作空间是 **2026 全国大学生智能汽车竞赛 / AIC 无人机配送航程挑战赛** 的**优化版实机工程**。
在真实 PX4 无人机（FMU v6C + Jetson 机载电脑）上，于室内无 GNSS 环境下完成 **起飞 → 8 圈绕场穿环（计分）→ 第 9 圈 4 点定距投放 → 回 H 视觉辅助降落** 的全自主闭环任务。

冻结基线版在 `~/catkin_ws/src/tutorials/tutorial_all`（仅做坐标同步，不再改逻辑）。

---

## ✨ 项目亮点

- **无 GNSS 全自主**：立体重构定位 (FAST-LIO) 作为唯一定位源，出厂带「视觉 xyz 全融合」配置
- **9 圈制**：第 1~8 圈绕场穿环计分、第 9 圈 4 点投放，单圈全程速度分解最优化
- **三明治安全架构**：解锁前 `sanity_gate` + 飞行中 Z 看门狗（视觉/气压交叉校验）+ 视觉辅助降落
- **OFFBOARD setpoint 流永不中断**：独立守护线程以 20 Hz 持续发指令，兼容 `/clock` 时间回跳
- **多级掉电兜底**：从「定位源降级（FAST-LIO→GT 真值）」到「强制 AUTO.LAND」，层层熔断

| 指标 | 值 |
|---|---|
| 定位源 | Livox Mid-360 → FAST-LIO（无坐标版）|
| 巡航高度 | 1.40 m |
| 投放高度 | 0.10 m（贴地限速投放）|
| 水平限速 | 2.0 m/s（`xy_max`）|
| 绕柱半径 | 1.15 m |
| setpoint 流 | 20 Hz 独立线程 |

---

## 🏗️ 系统架构

```
┌─────────────────────────────── AICMission 状态机 ───────────────────────────────┐
│   waiting → 雷达就绪 → odom就绪 → sanity_gate → arm/offboard → 起飞 ──┐          │
│   每圈: 投放点1 → 绕障碍物2(orbit) → 北走廊 → 投放点3 → 穿环 → 投放点4           │
│   └─────────────── 第 N 圈切换为投放圈（4 点 drop_at）→ 回 H 视觉降落 ──────────│
└───────┬──────────────────────┬──────────────────────┬──────────────────────────┘
        │                      │                      │
   ┌────▼────┐           ┌─────▼─────┐          ┌─────▼─────┐
   │ 决策/规划 │           │  执行控制   │          │  安全监控   │
   │ 航点分解  │◄─────────►│  flight_lib│◄────────►│  Z 看门狗  │
   │ 圈数循环  │           │  OFFBOARD  │          │  sanity门  │
   │ 投放调度  │           │  orbit/vel │          │  定位降级   │
   └─────────┘           └───────────┘          └───────────┘
        │                      │                      │
        ▼                      ▼                      ▼
   定位层 FAST-LIO     执行层 MAVROS/PX4          感知层 相机+雷达
   /Odometry ──────►  OFFBOARD setpoint 流 ───►  /ring_center /h_center
```

| 层级 | 组件 | 职责 |
|---|---|---|
| 定位层 | `tutorial_navigation_feng`（FAST-LIO）| 视觉惯性里程计，提供 `/Odometry` 作为唯一定位源 |
| 感知层 | `tutorial_vision_feng`、激光点云 | 环心/靶心/H 点识别、激光前向避障 `avoid_lib` |
| 决策层 | `aic_mission.py` | 全流程状态机：圈数循环、穿环、投放、降落决策 |
| 执行层 | `flight_lib.py` | OFFBOARD 位置/速度控制原语，setpoint 流守护线程 |
| 安全层 | `sanity_gate` + `_z_watchdog` + 定位降级 | 起飞前 / 飞行中 / 降落三级熔断 |

---

## 📁 目录结构

```
feng_ws/
├── src/
│   ├── CMakeLists.txt
│   ├── tutorial_all_feng/            # ★ 主任务包（本仓库核心）
│   │   ├── scripts/
│   │   │   ├── aic_mission.py        #   全流程任务状态机（主程序，默认 9 圈制）
│   │   │   ├── flight_lib.py         #   OFFBOARD 控制库（fly_to/goto/orbit/land）
│   │   │   ├── avoid_lib.py          #   激光避障（前向安全气泡）
│   │   │   ├── vision_detector.py    #   视觉检测与距离解算
│   │   │   └── test_landing.py       #   单步降落测试
│   │   ├── launch/
│   │   │   └── aic2026_task.launch   #   任务总启动（含环心检测、全链路）
│   │   └── param/
│   │       ├── deliver_detect_params.yaml
│   │       └── parking_detect_params.yaml
│   ├── tutorial_basic_feng/          # 基础：mavros / PX4 启动、fly_path、vel_control、circle
│   ├── tutorial_navigation_feng/     # 定位：MID360→FAST-LIO、环心检测、move_base、ego_planner
│   ├── tutorial_vision_feng/         # 视觉：相机驱动、红点/圆环/H点/QR/落靶检测
│   ├── tutorial_catapult/            # 投放舵机：catapult_driver.cpp、servo_ctrl.py、固件
│   ├── tutorial_gazebo/              # 仿真：多套 world / magpie360 模型 / livox_mid360
│   ├── tutorial_feng/                # 历史样例（publish 兼容）
│   └── tutorial_all_feng_snapshot_*.tar.gz   # 快照基线
├── build/ / devel/                   # Catkin 编译产物（已 gitignore，不提交）
└── README.md
```

---

## 🚁 任务流程与状态机

### 9 圈制总览

```
[解锁前检查] → 起飞1.4m →  ┌── 第 1 ~ 8 圈（计分不投放）──┐ → 第 9 圈（4 点投放）→ 回 H 视觉降落
   sanity_gate       8 × ( 投放点1 → 绕柱 → 北走廊 → 穿环 → 投放点4 )               land_with_vision
                             └── 每圈穿环稳定计分 ──┘
```

### 单圈路径分解（2026-10-23 新场地）

```
起飞点 (0,0) → 投放点1(3.40,-0.18) → 障碍物2(2.5,1.3) orbit 绕柱 r=1.15
→ 东界北上 P_BOX2_EAST(3.7,1.3) → 北侧走廊 P_CORNER_DN(-3.29,2.5)
→ 投放点3(-3.29,0.13) → 环前对轴 P_RING_ALIGN(-3.28,-0.40) → 穿环(goto_vel 直穿)
→ 环后 → 投放点4(-0.07,-2.24) → 南侧走廊回起点
```

| 航点 | 作用 | 设计说明 |
|---|---|---|
| `P_BOX2_EAST (3.7,1.3)` | 绕柱后北行 | 正东 1.2 m，东墙 4.6 留 0.9 m 余量，防蹭墙 |
| `P_CORNER_DN (-3.29,2.5)` | 北侧走廊 | 对齐 P3 的 x = -3.29 |
| `P_RING_ALIGN (-3.28,-0.40)` | 环前对轴 | 离环心 0.79 m 减速缓冲（0.75 太近会冲过）|
| `P_RING_OUT (-3.28,-2.24)` | 穿环后 | 直接衔接投放点4，沿走廊回落 |

### 主流程时序（`aic_mission.py::run`）

```text
wait_radar() ──► wait_odometry() ──► sanity_gate() ──► arm_and_offboard()
──► [_z_watchdog] 线程启动 ──► fly_to(0,0,1.4) 起飞
──► for lap in 1..n_laps:
       _pass_or_drop(1, drop) → orbit(BOX2, r=1.15, ccw) → 投放点2
       → 北走廊(pass 障碍物3 外侧) → _pass_or_drop(3, drop) → pass_ring()
       → _pass_or_drop(4, drop)
──► land_with_vision()
```

> `n_laps` 默认 9：`rospy.get_param("~n_laps", 9)`，第 9 圈为投放圈。设 `1` 即单圈直投调试。

---

## ⚙️ 核心机制深度

### 1. OFFBOARD setpoint 流永不中断（`flight_lib.py`）

OFFBOARD 协议要求设置点流 **> 2 Hz 且不得中断**，否则失控。`FlightController` 用独立守护线程以 **20 Hz** 持续发布位置或速度 setpoint：

- 支持位置模式与速度模式（`nav_pos` / `nav_vel`）切换——穿环用速度直穿
- 所有控制指令「重复发送 + 状态回读确认」，避免丢包
- **`/clock` 时间回跳容错**：`safe_rate_sleep` 吞掉 `ROSTimeMovedBackwardsException`，防止低实时率下控制线程被杀
- 无限等待 MAVROS 服务（`wait_for_service` 不设死超时），串行启动顺序反了也不炸

### 2. 穿环（速度 PID 直穿）

```text
环前对轴 P_RING_ALIGN → 切换到速度模式 goto_vel 沿 x 直穿 → 环后 P_RING_OUT
穿环前后 z 统一 1.40 m，给速度 PID 留足收敛区间
```

### 3. 绕柱（orbit）

```text
orbit(cx, cy, z, R, ccw=True, entry_ang=0, lead=0.8, rate=15Hz, entry_tol=0.30)
牵引角 lead=0.8 超前量，入轨时计入初始角度偏差，避免“绕圈起点折返”
```

### 4. 投放（第 9 圈）

```text
drop_at(i)：飞到对应投放点 → 限速 0.5 m/s 下降至 DROP_Z=0.10 m（贴地）
→ 发布 /servo/<slot>/open 开舱 → 升回 1.4 m
舱位映射：投放点1→back_right, 2→back_left, 3→front_right, 4→front_left
```

> 投放策略为「纯坐标」版（仿真舱位偏移与实机不同，`SLOT_OFFSETS` 全 0）。实测落点误差 < 4cm；
> 实机阶段可在 `drop_at` 内接入视觉伺服对准。

### 5. 视觉辅助降落（`land_with_vision`）

```text
飞到 H 上空 1.4 m → 悬停 1 s → 采集一帧 H 视觉(>0.9m 判野值丢弃)
→ 看到才精确修正 → 降 0.8 m → AUTO.LAND
策略：坐标为主、视觉为辅；视觉只做进近修正，不依赖视觉收敛
```

---

## 🛡️ 三明治安全架构

| 阶段 | 机制 | 参数 | 失败动作 |
|---|---|---|---|
| 起飞前 | **sanity_gate**：原点/高度/静止检查 | `GATE_XY_TOL=0.5` <br>`GATE_Z_RANGE=(-0.5,0.8)` <br>`GATE_SAMPLE_T=3.0`、`GATE_STILL_TOL=0.05` | 拒绝解锁 |
| 飞行中 | **Z 看门狗**：绝对边界 + 视觉/气压交叉校验 | `Z_WATCH_MIN=-0.6`、`Z_WATCH_MAX=2.4` <br>`Z_WATCH_BARO_TOL=1.0`（与气压增量差）<br>`Z_WATCH_T=1.0`、`Z_WATCH_BARO_T=1.5` | 告警→强制 `AUTO.LAND` |
| 全阶段 | 定位源降级 | FAST-LIO 超时 | 降级 GT 真值（坐标版） |
| 避障 | 激光前向气泡 | `avoid_lib` / 点云前向距离 | 悬停避让/等待 |

> 双高度独立基准：视觉 z（EKF 全融合，`EKF2_BARO_CTRL=0`）与气压 `monotonic`（不参与融合）交叉校验，防止单源假象。

---

## 🧮 关键参数表

| 常量 / 参数 | 值 | 说明 |
|---|---|---|
| `~n_laps` | **9** | 圈数（第 9 圈投放），rosparam 可覆盖 |
| `CRUISE_Z` | 1.4 m | 巡航高度 |
| `DROP_Z` | 0.10 m | 投放高度（由 0.02 抬高，减地效耦合）|
| `RING_Z` | 1.6 m | 环心高度 |
| `RING_PASS_Z` | 1.40 m | 穿环高度 |
| `xy_max` | 2.0 m/s | 水平限速（刹停≈1.0 m，防蹭墙）|
| `USE_DRIFT` | **False** | 漂移只打日志不施加，防假 drift 推偏落点 |
| `ORBIT_R` | 1.15 m | 绕柱半径 |
| `BOX2` | (2.5, 1.3) | 障碍物2 柱心 |
| `P_RING_ALIGN` | (-3.28,-0.40) | 环前对轴点 |
| `RING_EXPECT` | (-3.28,-1.19,1.6) | 环心标称 |
| `DROP_POINTS` | P1(3.40,-0.18) P2(0.10,2.19)<br>P3(-3.29,0.13) P4(-0.07,-2.24) | 4 投放点（新场地）|
| orbit lead / rate | 0.8 / 15 Hz | 绕柱牵引角与发布频率 |
| setpoint 流 | 20 Hz | 独立守护线程 |

---

## 🚀 启动命令

```bash
# 1. 舵机驱动（启动即自动 half 预松舱门，防卡死）
roslaunch tutorial_catapult catapult_driver.launch

# 2. 主任务（默认 9 圈：8 计分 + 1 投放）
rosrun tutorial_all_feng aic_mission.py

# 3. 一键任务启动（含环心检测等全链路）
roslaunch tutorial_all_feng aic2026_task.launch

# 指定圈数（调试：单圈直投）
rosrun tutorial_all_feng aic_mission.py aic_mission/n_laps:=1
```

> 更多命令见 `~/feng_ws_commands.txt`（飞行/舵机/监控/快照速查表）。

---

## 🎛️ 舵机控制

舱门策略：**全程不走 close 方向**（易卡死）。
- 起飞前：`half` 半关预松（60°/119°）
- 投放时：`open` 全开（0° / 180°）
- 降落结束：回 `half`

```bash
rosrun tutorial_catapult servo_ctrl.py init            # 半关初始化
rosrun tutorial_catapult servo_ctrl.py open back_right # 开指定舱
rosrun tutorial_catapult servo_ctrl.py test            # 全舱遍历测试
```

---

## 📍 场地坐标（2026-10-23 新场地）

| 点 | 坐标 (x, y) |
|---|---|
| 投放点1 | (3.40, -0.18) |
| 投放点2 | (0.10, 2.19) |
| 投放点3 | (-3.29, 0.13) |
| 投放点4 | (-0.07, -2.24) |
| 障碍物2（绕柱） | (2.5, 1.3)，绕柱半径 1.15 m |
| 环心 | (-3.28, -1.19, z≈1.6) |

> 旧场地坐标（2026-10-18 及更早）在 `aic_mission.py` 中以注释保留，`DROP_POINTS` 与 `RING_EXPECT` 已按新场地校准。

---

## 🎯 调参与扩展建议

- **投放精度**：将 `SLOT_OFFSETS` 从全 0 改为实机标定值；在 `drop_at` 内接入视觉靶心伺服
- **节拍优化**：`takeoff` 段 `hold 0.5`、北走廊 `hold=0`（飞越不停）、北走廊 `xy_scale=0.8`——按场地安全余量逐步收紧
- **漂移补偿**：`USE_DRIFT=True` 试跑观察落点，若真实漂移显著再开启闭环补偿
- **校准**：`MEASURE` 文档列出相机内参、舱位偏移、投放悬停高度等实机标定项

---

## 📦 快照

```bash
# 当前基线（drift 修复版）
~/tutorial_all_feng_snapshot_v6_driftfix_20261023.tar.gz
# 仓库内归档（7 圈 multi-drift 快照）
src/tutorial_all_feng_snapshot_7laps_multidrift_20261019.tar.gz
```

---

## 🧯 常见问题（FAQ）

- **roslaunch 刷屏 error / 全节点被 kill**：旧版 serial_node `required="true"` 连锁崩溃，已删该属性；确认当前 launch 无 `required`
- **舵机首包丢失**：catapult_driver 启动即自动发 `half`，无需脚本内嵌初始化
- **圈数参数不生效**：ROS1 语法是 `aic_mission/n_laps:=1`，不是 `_n_laps:=1`
- **绕圈起点折返**：已修（orbit 入轨计入初始角度偏差）；若复现检查 `fly_to` 入轨容差
- **起飞冲高/全场漂移**：多为 FAST-LIO 在错误状态下初始化，走 `sanity_gate` 前检查原点与静止

---

## 📚 文档索引

| 文档 | 位置 |
|---|---|
| 主程序与飞行库实现 | `src/tutorial_all_feng/scripts/aic_mission.py`、`flight_lib.py` |
| 任务启动配置 | `src/tutorial_all_feng/launch/aic2026_task.launch` |
| 定位/视觉/仿真 | `src/tutorial_navigation_feng`、`tutorial_vision_feng`、`tutorial_gazebo` |

---

## 📄 许可

本仓库为 2026 AIC 无人机配送挑战赛参赛工程，代码仅供比赛与研究使用。