# 快速上手 / 编译 / 调参手册

## 1. 目标读者

本文档面向三类读者：

- 首次接手该项目的开发者
- 需要在目标机上完成编译与首轮联调的人
- 需要现场调参和故障定位的调试人员

---

## 2. 工程结构

- `controller/`：MSI Claw 端控制面板、手柄输入、TCP 客户端
- `robot/`：树莓派端控制主程序、ODrive/IMU、EKF、tracking MPC、日志
- `common/`：controller 与 robot 共用的轻量工具（如 mini JSON）
- `tools/`：离线工具，例如日志 replay
- `Doc/`：文档

---

## 3. 编译前准备

### 3.1 Raspberry Pi / robot 侧

确保安装：

```bash
sudo apt install libusb-1.0-0-dev cmake g++ python3
```

### 3.2 MSI Claw / controller 侧

确保安装：

```bash
sudo apt install libwayland-dev libxkbcommon-dev cmake g++
```

---

## 4. 编译步骤

### 4.1 robot

```bash
cd robot
mkdir -p build
cd build
cmake ..
make -j4
ctest --output-on-failure
```

### 4.2 controller

```bash
cd controller
mkdir -p build
cd build
cmake ..
make -j4
```

---

## 5. 启动顺序

推荐顺序：

1. 上电并连接 ODrive 与 IMU
2. 启动 `robot`
3. 确认 `robot` 打印：设备在线、TCP listening、日志文件创建成功
4. 启动 `controller`
5. 确认手柄输入和 TCP 遥测正常

---

## 6. 首轮验证清单

在开始实车控制前，先只验证基础链路：

- `robot` 是否能稳定读 IMU
- `robot` 是否能读到 ODrive 位置与速度
- `controller` 是否能显示 telemetry
- 改一个参数并按 `Y`，确认 `robot` 侧收到更新
- 是否生成 `logs/run_*.csv`

---

## 7. 调参总原则

### 7.1 顺序不能乱

必须按下面顺序调：

1. **先稳**：只开基础状态反馈 / LQR
2. **再顺**：调参考生成层
3. **再准**：调 EKF
4. **最后聪明**：再开 tracking MPC

### 7.2 一次只改一类参数

不要同时改 LQR、EKF 和 MPC。

### 7.3 每轮只改少数几个参数

推荐每轮只改 1~3 个参数，并记录现象。

---

## 8. 推荐调参流程

### Phase A：只开基础控制

```text
ekf_enable = 0 or 1（先固定一套）
auto_lqr_enable = 1
nmpc_enabled = 0
```

优先调：
- `lqr_k_theta` 或自动 LQR 权重
- `lqr_k_theta_d`
- `lqr_k_v`
- `lqr_k_x`

观察：
- 原地扶正是否容易接住
- 有无高频抖动
- 停车是否拖或顶

### Phase B：调参考层

优先调：
- `speed_to_pitch_ff`
- `accel_to_pitch_ff`
- `target_speed_rate_limit`
- `target_pitch_rate_limit_dps`
- `reference_position_gain`

观察：
- 起步是否太冲/太肉
- 松手是否自然停下
- 小油门点动是否顺滑

### Phase C：调 EKF

优先调：
- `ekf_q_*`
- `ekf_r_*`

观察：
- 估计是否抖动
- 估计是否明显滞后
- 切到 EKF 后控制是否更稳

### Phase D：最后开 tracking MPC

```text
nmpc_enabled = 1
```

优先调：
- `nmpc_w_theta`
- `nmpc_w_v`
- `nmpc_w_x`
- `nmpc_w_du`
- `nmpc_w_terminal_*`

观察：
- 参考是否更平滑
- 停车收尾是否更好
- 大动作是否更前瞻

---

## 9. 推荐起步参数（保守）

```text
auto_lqr_enable = 1
ekf_enable = 1
nmpc_enabled = 0

lqr_k_theta = 1.6
lqr_k_theta_d = 0.08
lqr_k_v = 0.40
lqr_k_x = 0.25

speed_to_pitch_ff = 1.5
target_speed_rate_limit = 4.0
target_pitch_rate_limit_dps = 90.0

ekf_q_position = 1e-4
ekf_q_velocity = 2e-2
ekf_q_pitch = 4e-2
ekf_q_pitch_rate = 1.5
ekf_r_position = 2e-4
ekf_r_velocity = 4e-2
ekf_r_pitch = 0.8
ekf_r_pitch_rate = 2.0
```

在基础控制稳定后，再开启 MPC。

---

## 10. 日志与 replay

### 10.1 日志文件位置

默认位于：

```text
logs/run_YYYYMMDD_HHMMSS.csv
```

### 10.2 回放

```bash
python3 tools/replay_csv.py logs/你的日志.csv --plot
```

建议每次调参都保留一份日志，并记录使用的参数集。

---

## 11. 实车安全建议

- 初次上电必须有人扶车
- 初次调试降低 `max_velocity`
- 开始时关闭 MPC，只调基础稳定环
- 若估计器或通信异常，优先确认传感器和日志，不要先改增益

---

## 12. 常见现象与快速对应

### 12.1 站不住 / 发软
- 增 `lqr_k_theta`
- 若为自动 LQR，增 `lqr_q_theta` 或减 `lqr_r_u`

### 12.2 高频抖动
- 增 `lqr_k_theta_d`
- 或减 `lqr_k_theta`
- 检查 EKF / 速度估计是否过噪

### 12.3 停车拖泥带水
- 增 `lqr_k_v`
- 增 `reference_position_gain`（小步）
- 开 MPC 后增 `nmpc_w_terminal_x`

### 12.4 停车拉扯太强
- 降 `lqr_k_x`
- 降 `reference_position_gain`

### 12.5 参考跳变
- 增 `nmpc_w_du`
- 降 `nmpc_pitch_slew_dps`
- 检查 controller 是否频繁发送大幅参数修改
