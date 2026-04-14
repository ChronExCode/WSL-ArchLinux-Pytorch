## 完整控制流程解析

### 系统架构总览

```
遥控器 (controller)                树莓派 (robot)
    │                                   │
    │  throttle/steering (TCP 20Hz)     │
    ├──────────────────────────────────►│
    │                                   │
    │  telemetry (TCP 20Hz)             │
    │◄──────────────────────────────────┤
    │                                   │
                                        │
                              ┌─────────┴─────────┐
                              │   control_tick     │
                              │   (200Hz主循环)     │
                              └─────────┬─────────┘
                                        │
                    ┌───────────────────┼───────────────────┐
                    │                   │                   │
              ┌─────┴─────┐    ┌───────┴───────┐   ┌──────┴──────┐
              │  IMU数据   │    │ balance_controller│   │  ODrive    │
              │(MSP 250Hz)│    │   .update()    │   │set_velocities│
              └───────────┘    └───────────────┘   └─────────────┘
```

### 数据流时序

**每 5ms（200Hz）control_tick 被调用一次：**

```
1. 检查 ODrive 连接、IMU 新鲜度、RC 超时
2. 读取 IMU 数据（imu_.get_imu_data()）
3. 读取遥控命令（tcp_.get_command()）
4. 读取编码器位移（enc_displacement_ atomic，由 telemetry_loop 20Hz 更新）
5. 调用 controller_.update(imu_data, cmd, dt)
6. 把输出发给 ODrive（set_velocities）
```

### controller_.update() 内部的两层 PID

```
                    throttle
                       │
                       ▼
              ┌─────────────────┐
              │ target_displacement │ ← throttle 积分
              │ += throttle × max_vel × dt │
              └────────┬────────┘
                       │
         target_displacement    displacement (编码器)
                  │                    │
                  ▼                    ▼
          ┌──────────────────────────────┐
          │     位置环 PID (pos_pid_)      │  ← 外环，慢
          │  error = target - actual     │
          │  output = P + I + D          │
          │  限幅: [pos_out_max_bwd,     │
          │         pos_out_max_fwd]     │
          └────────────┬─────────────────┘
                       │ pos_output（度）
                       ▼
          ┌──────────────────────────────┐
          │  target_pitch =              │
          │    balance_point             │  ← ARM 时记录的角度
          │  + pitch_offset              │  ← 手动微调
          │  + pos_output                │  ← 位置环修正
          │                              │
          │  限幅: [target_pitch_max_bwd,│
          │         target_pitch_max_fwd]│
          └────────────┬─────────────────┘
                       │ target_pitch（度）
                       │
    target_pitch     actual_pitch     gyro_y（陀螺仪角速度）
         │               │               │
         ▼               ▼               ▼
    ┌────────────────────────────────────────┐
    │    内环 PID (pitch_pid_)                │  ← 内环，快（200Hz）
    │                                        │
    │  error = target_pitch - actual_pitch   │
    │  P = Kp × error                        │
    │  I += error × dt  (anti-windup)        │
    │  D = -Kd × gyro_y (低通滤波)            │
    │                                        │
    │  output = -(P + I + D)  ← 取反！        │
    │  限幅: [-max_velocity, +max_velocity]  │
    └──────────────┬─────────────────────────┘
                   │ base_velocity（turns/s）
                   │
         ┌─────────┴─────────┐
         │                   │
    left_velocity       right_velocity
    = base_velocity     = -base_velocity
    (axis0: 正=前)      (axis1: 正=后，镜像)
```

### 逐环详解

------

#### 第 1 层：角度估计（不是 PID，是传感器融合）

cpp

```cpp
// 从加速度计算 pitch（静态角度，慢但无漂移）
acc_pitch = atan2(-acc_x, sqrt(acc_y² + acc_z²))

// 互补滤波器（融合 gyro 和 acc）
our_pitch = 0.98 × (our_pitch + gyro_y × dt) + 0.02 × acc_pitch

// 混合飞控姿态（70%）和本地滤波（30%）
actual_pitch = 0.7 × imu.pitch + 0.3 × our_pitch

// 低通滤波（去毛刺）
pitch_lpf = 0.5 × actual_pitch + 0.5 × pitch_lpf
actual_pitch = pitch_lpf
```

**问题分析：** 这里有三层滤波叠加（互补滤波 + 混合 + LPF），引入了延迟。对于 200Hz 控制环来说，PITCH_LPF_ALPHA = 0.5 意味着大约 2 帧的延迟。飞控自带的 `imu.pitch` 已经经过了 Betaflight 内部的 Mahony/Madgwick 滤波，再加我们自己的滤波可能过度。

**建议：** 如果飞控的 `imu.pitch` 稳定（你确认过 Betaflight Configurator 显示正常），可以考虑直接用 `imu.pitch`，去掉互补滤波和混合，只保留一层 LPF。这样减少延迟，提高响应速度。

------

#### 第 2 层：位置环（外环，通过 pos_pid_）

```
输入：target_displacement（目标位置）、displacement（实际位置，编码器）
输出：pos_output（度，对 target_pitch 的修正量）

error = target_displacement - displacement
P = pos_kp × error     → 偏离越远，修正角度越大
I = pos_ki × ∫error dt → 消除稳态误差（摩擦力等）
D = pos_kd × d(displacement)/dt → 速度阻尼
```

**具体数值举例（当前参数 pos_kp=1.5）：**

- robot 向后漂移了 0.5 圈 → displacement = -0.5
- target = 0 → error = 0 - (-0.5) = +0.5
- P = 1.5 × 0.5 = 0.75° → target_pitch 增加 0.75°（前倾来拉回）

**问题分析：**

1. **更新频率不匹配**：displacement 由 telemetry_loop 20Hz 更新（通过 `enc_displacement_` atomic），但 pos_pid 在 200Hz 的 control_tick 里运行。这意味着位置环的 D 项看到的 displacement 是阶梯状的（每 50ms 跳一次），中间 9 帧的 d(displacement)/dt = 0，导致 D 项几乎无效。
2. **I 项在移动时被清零**：`pos_pid_.reset_integral()` 清掉了积分。松开 throttle 后 I 项从零开始重新积分，如果有摩擦力偏差，需要一段时间才能消除稳态误差。

------

#### 第 3 层：内环 pitch PID（通过 pitch_pid_，使用 compute_with_gyro）

```
输入：target_pitch（度）、actual_pitch（度）、gyro_y（度/秒）
输出：base_velocity（turns/s，取反后的）

error = target_pitch - actual_pitch
P = pitch_kp × error
I += pitch_ki × error × dt  (有 anti-windup)
D = -pitch_kd × gyro_y      (低通滤波后)

raw_output = P + I + D
base_velocity = -raw_output  ← 取反
```

**为什么取反？**

- 前倾 → actual_pitch 增大 → error = target - actual = 负
- P = Kp × 负 = 负
- raw_output = 负
- base_velocity = -负 = 正 → 轮子前转追重心 ✓

**D 项的符号推导：**

```
前倾且在加速倾倒 → gyro_y > 0（pitch 增大的速率）
D_raw = -Kd × gyro_y = 负
D 项 = 负 → raw_output 更负 → base_velocity 更正 → 更大的前转力
```

等等，**这好像是正反馈——倾倒越快，轮子转得越快，这是对的！** D 项在这里起的是"预判"作用：不等 error 变大，看到角速度就提前加速。

但仔细想——D 项应该是**阻尼**，应该是**反对**运动方向的。在普通 PID 里，D = -Kd × d(measurement)/dt，measurement 增大时 D 为负，和 P 方向相反，起减速作用。

这里用 gyro_y 做 D 项：

```
robot 从平衡点前倾并加速 → gyro_y > 0
D_raw = -Kd × (+gyro_y) = 负
但 base_velocity = -raw_output = -(P + I + 负D)
```

如果 P 主导（P 为负），raw_output 为负，D 为负让 raw_output 更负，base_velocity = -更负 = 更正。

**所以 D 项在这里起的不是阻尼，而是加速追赶。** 这对于平衡车来说是对的——越快要倒，轮子应该越快地追。

但当 robot 接近平衡点在减速时：

```
robot 在回摆（从前倾回到竖直）→ gyro_y < 0（pitch 在减小）
D_raw = -Kd × (-gyro_y) = 正
D 项 = 正 → raw_output 加上正值 → raw_output 不那么负了（甚至可能变正）
base_velocity = -(不那么负) = 不那么正 → 轮子减速
```

**这就是阻尼效果！回摆时轮子减速，避免越过平衡点。** 符号是对的。

------

#### 第 4 层：电机输出

```
left_velocity  =  (base_velocity + yaw_diff)  → axis0（正=前）
right_velocity = -(base_velocity - yaw_diff)  → axis1（正=后，取反）
```

yaw_diff 当前为 0（yaw 控制未启用）。

------

### 发现的问题总结

**问题 1：位置环 displacement 更新太慢（20Hz vs 200Hz 控制环）**

pos_pid 的 D 项需要 displacement 的变化率来做速度阻尼，但 displacement 每 50ms 才跳一次。D 项在 10 帧中只有 1 帧看到变化，其余 9 帧 d_raw = 0，被低通滤波后 D 项几乎消失了。

解决方案：在 control_tick 里用 motor output 积分来估算实时 displacement，每帧都更新，用 20Hz 的编码器真值做校准。

**问题 2：角度估计有过多滤波层叠加**

三层滤波（互补 + 混合 + LPF）引入延迟，降低了 D 项的有效性。如果飞控的 pitch 数据本身稳定，可以简化。

**问题 3：位置环参数可能需要和内环频率匹配**

pos_pid 在 200Hz 跑但数据 20Hz 更新，Ki 的积分速率在 200Hz 下是 20Hz 的 10 倍，这意味着 pos_ki 的实际效果比预期强 10 倍。