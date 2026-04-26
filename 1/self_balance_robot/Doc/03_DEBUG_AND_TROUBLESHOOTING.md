# 调试与问题排查手册

## 1. 推荐调试顺序

始终按以下顺序排查：

1. **设备层**：IMU / ODrive / USB 是否在线
2. **通信层**：controller 与 robot 是否连通
3. **估计层**：pitch / speed / displacement 是否可信
4. **基础控制层**：关闭 MPC，仅验证 LQR / state feedback
5. **参考层**：起步、减速、停车逻辑是否合理
6. **MPC 层**：在前 5 层稳定后再打开

---

## 2. 最重要的调试纪律

- 一次只改一类参数
- 一次只改 1~3 个参数
- 每轮调试都保存日志
- 不要凭体感连续乱改十几个参数

---

## 3. 首先看什么信号

实车调试时，最应该优先看的信号是：

- `pitch`
- `tgt_pitch` / `theta_ref`
- `spd_est`
- `pos_disp` / `pos_target`
- `base_cmd`
- `nmpc_used` / `nmpc_cost`

如果这些信号都不合理，先别改复杂控制参数。

---

## 4. 典型故障树

### 4.1 编译阶段

#### controller 编不过
重点检查：
- `controller/src/main.cpp` 花括号是否完整
- `controller/src/config_persistence.cpp` 是否有损坏的字符串字面量
- Wayland 开发头是否安装

#### robot 编不过
重点检查：
- `libusb-1.0` 开发头
- `robot/CMakeLists.txt` 是否包含新增源文件（如 `ekf.cpp`、`logger.cpp`）

---

### 4.2 启动阶段

#### robot 启动但 ODrive / IMU 不在线
重点检查：
- USB 枚举
- 设备 VID/PID
- 是否被内核驱动占用

#### controller 能启动但没 telemetry
重点检查：
- `server_ip` 是否正确
- robot 是否真正监听 `tcp_port`
- 防火墙 / 网段

---

### 4.3 实车控制阶段

#### 一上电就冲轮
可能原因：
- 俯仰符号反
- balance point 不对
- `lqr_k_theta` 过大
- `speed_to_pitch_ff` 过大

#### 原地高频抖
优先看：
- `spd_est` 是否很噪
- `pitch_rate` 是否很噪
- `lqr_k_theta_d` 是否太小
- EKF `R` 是否太小导致过信测量噪声

#### 慢慢漂走
优先看：
- `lqr_k_x` 太小
- `reference_position_gain` 太小
- `target_displacement` 是否在松手后正确收敛

#### 松手停车时拉扯
优先看：
- `reference_position_gain` 太大
- `lqr_k_x` 太大
- `nmpc_w_terminal_x` 太大

#### 开 MPC 后比不开更差
先确认：
- 不开 MPC 时基础控制是否已经稳定
- `nmpc_used` 是否经常为 false（说明结果被丢弃）
- `nmpc_cost` 是否异常大
- `nmpc_w_du` 是否太小导致控制变化太跳

---

## 5. EKF 专项调试

### 5.1 现象：估计明显滞后
- 降 `ekf_r_pitch` / `ekf_r_velocity`
- 增 `ekf_q_pitch` / `ekf_q_velocity`

### 5.2 现象：估计很抖
- 增 `ekf_r_pitch_rate` / `ekf_r_velocity`
- 降 `ekf_q_*`

### 5.3 现象：切到 EKF 后更差
先用同一组日志对比：
- 原 observer 输出
- EKF 输出

若 EKF 更差，优先改协方差，不要直接怀疑控制律。

---

## 6. TCP / JSON / UI 专项排查

### 6.1 参数改了但 robot 没反应
检查链路：

1. UI 参数是否真的改到（界面数值变化）
2. 是否按了 `Y` 发送
3. `controller` TCP 是否在线
4. robot 是否进入参数回调
5. `Config::sanitize()/validate()` 是否把值截掉了

### 6.2 telemetry JSON 偶发损坏
当前版本已经对 `msg` 做了转义。若仍出问题，重点看：
- 是否有新的字符串字段未走 JSON builder
- 是否有超大 payload 被上层截断

---

## 7. 推荐现场调试流程

### 场景 A：新版本首次上机
1. 关 MPC
2. 降低最大速度
3. 只测原地平衡
4. 再测小油门点动
5. 再测松手停车
6. 最后再开 MPC

### 场景 B：某次改动后变差
1. 先回放上一份正常日志
2. 对比参数快照
3. 检查是否改了多类参数
4. 先回退基础控制，再判断是估计层还是 MPC 层问题

---

## 8. 推荐日志策略

每一轮实车测试都至少记录：
- 日期与环境
- 主要参数改动
- 主观现象
- 对应日志文件名

后续做 autotune 和系统辨识时，这些记录非常有价值。
