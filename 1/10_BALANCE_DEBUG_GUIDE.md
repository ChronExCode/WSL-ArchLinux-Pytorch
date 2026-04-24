# 平衡控制详细调试说明

> 目标：这份文档不是泛泛而谈，而是给你一条**从“能跑起来”到“定位具体控制问题”**的调试路径。读完以后，你应该知道：
>
> - 先看哪几个线程、状态和日志
> - 进入 `BALANCING` 之前要满足什么条件
> - `controller_.update()` 前后应该看到哪些量
> - 遇到“起不来 / 一 arm 就倒 / 抖动 / 慢漂 / 跑飞 / 退出卡住”时，应该按什么顺序定位

---

## 1. 先建立一条最重要的认识

这个工程的控制链不是“一个 PID 黑盒”，而是：

```text
遥控命令
  → 输入整形 / 参考生成
  → 状态估计（互补滤波 / EKF）
  → NMPC 参考修正（可选）
  → LQR 风格状态反馈
  → 左右轮速度命令
  → ODrive / 机器人本体
  → IMU + 编码器回读
```

所以调试时一定要分层。不要一看到“站不住”就直接改 `kp`。

正确顺序是：

1. **设备层**是否正常  
2. **状态机**是否正常  
3. **传感器符号/单位**是否正常  
4. **估计状态**是否正常  
5. **参考生成**是否合理  
6. **稳定控制输出**是否方向正确  
7. **执行器响应**是否跟上  
8. 最后才是**调参数**

---

## 2. 先认清关键源码入口

最关键的文件和入口：

- `robot/src/robot.cpp`
  - `Robot::run()`
  - `Robot::handle_state_command()`
  - `Robot::control_tick(double dt)`
  - `Robot::enter_fault(...)`
- `robot/src/balance_controller.cpp`
  - `BalanceController::update(...)`
  - `estimate_state(...)`
  - `generate_reference(...)`
  - `compute_stabilizing_control(...)`
- `robot/src/ekf.cpp`
  - EKF 预测与观测更新
- `robot/src/nmpc_controller.cpp`
  - 后台线程求解参考修正
- `robot/src/odrive_usb.cpp`
  - 速度命令、编码器读取、停机路径
- `robot/src/msp_imu_usb.cpp`
  - IMU polling / freshness / USB 收包

你真正应该单步调试的“主链路”只有两条：

```text
Robot::control_tick()
BalanceController::update()
```

---

## 3. 状态机调试：先确认系统有没有真正进入闭环

工程里你会看到这些状态：

- `INITIALIZING`
- `WAITING_USB`
- `CALIBRATING`
- `IDLE`
- `ARMED`
- `BALANCING`
- `FAULT`
- `SHUTDOWN`

### 3.1 正常路径应该是什么

```text
INITIALIZING
  → WAITING_USB
  → IDLE
  → CALIBRATING
  → BALANCING
```

### 3.2 一 arm 以后如果卡在 `CALIBRATING`

先查这几类问题：

1. IMU 是否持续新鲜
2. ODrive 是否连接正常
3. 零点 / 平衡点校准是否完成
4. arm 时姿态是否已经超出可接受窗口
5. 某个等待条件是否被 stale 数据阻塞

### 3.3 必看日志

你应该优先盯住这些日志：

- `[Robot] State: A -> B`
- `IMU data stale`
- `ODrive disconnected during balancing`
- `Remote emergency stop`
- `Tilt exceeded safety limit`
- `RC timeout (...) auto disarm`
- cleanup 阶段的
  - `stop motors`
  - `stop IMU polling`
  - `join telemetry`
  - `join reconnect`
  - `stop logger`
  - `stop tcp`
  - `detach odrive`
  - `detach imu`

如果状态流都不对，先不要碰控制参数。

---

## 4. control_tick 逐步调试法

`Robot::control_tick(dt)` 是最值得打断点的函数。

推荐断点顺序：

1. `Robot::control_tick(double dt)` 入口
2. `controller_.update(imu_data, cmd, dt)`
3. `controller_.is_tilt_safe(...)`
4. `odrive_.set_velocities(...)`
5. `enter_fault(...)`

### 4.1 你在 tick 里要检查的量

每次单步时看下面这些值：

- `dt`
- `state`
- `imu_.is_data_fresh(50)` 是否为 true
- `tcp_.has_client()`
- `tcp_.is_command_fresh(cfg_.rc_timeout_ms)`
- `imu_data.pitch`
- `imu_data.gyro_y`
- `cmd.throttle`
- `cmd.steering`
- `enc_displacement_`
- `enc_velocity_`
- `output.left_velocity`
- `output.right_velocity`

### 4.2 正常范围的直觉

不是绝对值，只是调试参考：

- `dt`：接近 `1 / control_rate_hz`
  - 例如 200Hz 时约 `0.005s`
- 静止时
  - `imu_data.pitch` 应接近平衡点
  - `imu_data.gyro_y` 应接近 0
  - `enc_velocity_` 应接近 0
- 没推杆时
  - `cmd.throttle ≈ 0`
  - `cmd.steering ≈ 0`
- 站稳时
  - 左右轮命令不应长时间大幅饱和
  - 左右轮速度平均值应围绕 0 或很小的补偿值波动

---

## 5. 先做“符号检查”，这是最容易把系统搞反的地方

对平衡机器人来说，**符号方向错一个，整个闭环就变正反馈**。

### 5.1 必查的四组符号

#### A. 车头前倾时，`pitch_deg` 的符号是什么

手动扶住车体，缓慢前倾 / 后仰，观察：

- `imu_data.pitch`
- `filtered_pitch_deg_`
- `EstimatedState.pitch_deg`

要求：三个量的方向必须一致。

#### B. 车向前滚动时，编码器平均位移和速度的符号是什么

观察：

- `raw_l`, `raw_r`
- `disp = (raw_l + raw_r) * 0.5`
- `enc_velocity_`

要求：前进时位移和速度符号一致，不允许一个正一个负。

#### C. 正的 `left/right velocity command` 会不会让车朝预期方向动

在安全条件下小幅测试：

- `odrive_.set_velocities(+v, +v)`  
  应对应车整体向前或向后，但必须和你定义的正方向一致。

#### D. 前倾误差出现时，控制输出是否推动轮子去“追身子”

这是最关键的物理检查。

如果车体前倾，正确控制应该让轮子朝前加速去把底盘送到重心下方。  
如果输出方向相反，必倒。

---

## 6. `BalanceController::update()` 应该怎么拆开看

建议在这个函数里做分段打印或断点。

它内部其实就三段：

```text
estimate_state()
generate_reference()
compute_stabilizing_control()
```

### 6.1 第一步：先看 estimate_state()

重点变量：

- `actual_pitch`
- `filtered_pitch_deg_`
- `filtered_pitch_rate_dps_`
- `observed_displacement_turns_`
- `filtered_velocity_tps_`
- `adaptive_trim_deg_`

#### 你要回答的问题

1. 静止时 pitch 会不会乱漂  
2. `gyro_y` 的方向和 `pitch_rate_dps` 是否一致  
3. 编码器速度和估计速度是否同向  
4. EKF 打开后是否比原始量更平滑，而不是更发散

#### 快速判断法

- **静止放在地上不动**
  - `pitch_deg` 抖动应小
  - `velocity_tps` 应接近 0
- **轻轻前后推**
  - `displacement_turns` 应连续变化
  - `velocity_tps` 应先变化，再回到 0
  - `pitch_rate_dps` 应在动作瞬间有明显响应

如果这里就错，后面看参考和 LQR 都没意义。

---

## 7. 第二步：看 generate_reference()

这一步决定“系统希望自己去哪”。

重点变量：

- `throttle_filtered_`
- `steering_filtered_`
- `target_speed_tps_`
- `target_displacement_turns_`
- `target_pitch_ff_deg`
- `ref.nominal_pitch_deg`
- `ref.constrained_pitch_deg`

### 7.1 没推杆时应该看到什么

- `throttle_filtered_ ≈ 0`
- `target_speed_tps_ ≈ 0`
- `target_displacement_turns_` 锁住当前附近位置
- `ref.nominal_pitch_deg` 不应持续大幅漂移
- `ref.constrained_pitch_deg` 应在一个较小范围内

### 7.2 推一点前进杆时应该看到什么

- `target_speed_tps_` 平滑上升，不是瞬间跳满
- `target_displacement_turns_` 开始向前推进
- `target_pitch_ff_deg` 随速度 / 加速度给出前倾参考
- `ref.constrained_pitch_deg` 在设定的 pitch 限幅内变化

### 7.3 常见异常

#### 异常 1：没推杆，参考位移自己慢慢跑
可能原因：

- 编码器零点或速度偏置没压住
- `target_displacement_catchup_rate` 逻辑过强
- 速度估计有偏置，导致系统以为自己没停住

#### 异常 2：推杆后参考 pitch 跳变很大
可能原因：

- rate limit 没生效
- `speed_to_pitch_ff`
- `accel_to_pitch_ff`
- pitch 约束前后的逻辑不一致

---

## 8. 第三步：看 compute_stabilizing_control()

这是最后把误差变成左右轮速度命令的地方。

你需要重点盯：

- `theta_error`
- `theta_d_error`
- `x_error`
- `v_error`
- `base_velocity_cmd`
- `steering_cmd`
- `left_velocity`
- `right_velocity`

### 8.1 先只看 base velocity，不看 steering

调试平衡时，先把 steering 影响降到最低。  
否则你会分不清楚车是“平衡问题”还是“转向叠加问题”。

### 8.2 四个误差项的物理意思

- `theta_error`：车身离目标倾角差多少
- `theta_d_error`：倾角速度偏差多少
- `x_error`：离目标位置差多少
- `v_error`：离目标速度差多少

系统本质上在做：

```text
u = -Kx * x_error
    -Kv * v_error
    -Kθ * theta_error
    -Kω * theta_d_error
    (+ integral)
```

### 8.3 正常控制输出的现象

- 小角度扰动时，`base_velocity_cmd` 应快速反应
- 但不会长期贴着饱和
- 扰动结束后，`base_velocity_cmd` 应回落
- 左右轮速度命令在无 steering 时应接近对称

### 8.4 常见异常与指向

#### A. 一启动马上冲飞
优先怀疑：

1. pitch 或 wheel velocity 符号反了
2. 平衡点 offset 错了
3. LQR / 手动增益量级过大
4. ODrive 方向定义和控制器假设不一致

#### B. 能站住，但持续高频抖动
优先怀疑：

1. `pitch_rate` 太噪
2. `wheel_velocity` 太噪
3. `lqr_k_theta_d` / `pitch_kd` 类阻尼过大或太敏感
4. 控制周期抖动 / USB 读写抖动
5. ODrive 速度环本身过硬或延迟大

#### C. 能站，但慢慢向前/后漂
优先怀疑：

1. `balance_point_deg_` 不对
2. `adaptive_trim_deg_` 逻辑在慢慢推偏
3. 编码器零偏 / 速度偏置
4. `target_displacement_turns_` 没锁住当前位置

#### D. 扰动后拉不回来，像“软脚”
优先怀疑：

1. `lqr_k_theta` / `theta` 权重偏小
2. `lqr_k_v` 偏小
3. 轮速执行跟不上
4. ODrive 实际速度没有达到命令值

---

## 9. NMPC 怎么调试，别把它当第一嫌疑人

这个项目的 NMPC 是后台线程，主要做**参考修正**，不是直接绕过主控制器去输出电机指令。

所以你调试顺序应该是：

1. **先在关闭 NMPC 的情况下让基础闭环能站**
2. 再打开 NMPC 看它有没有改善参考质量

### 9.1 NMPC 结果什么时候才会被采纳

你要检查：

- 结果是否 `valid`
- 结果是否足够新鲜
- `source_tick` 是否和当前控制上下文接近
- 状态 mismatch 是否超阈值

这意味着：

- NMPC 线程慢一点，不会直接把系统搞死
- 但如果条件判断错，可能会导致“结果永远不用”

### 9.2 NMPC 调试重点

先看这些量：

- `published_tick_`
- result 的 `source_tick`
- `stale_result_max_age_s`
- `nmpc_state_mismatch_pitch_deg`
- `nmpc_state_mismatch_vel_tps`
- `nmpc_state_mismatch_disp_turns`

### 9.3 典型现象

#### 现象 1：打开 NMPC 和关闭几乎一模一样
可能是：

- 结果一直没被采纳
- 成本函数设置让修正量太小
- 候选数 / horizon 太小，求出来接近 nominal

#### 现象 2：打开后变钝
可能是：

- `nmpc_w_du` 太大，动作过保守
- `nmpc_pitch_slew_dps` 太小
- preview 修正把参考变得太平

#### 现象 3：打开后低频来回晃
可能是：

- 参考修正和底层状态反馈在互相打架
- 状态 mismatch 门槛过宽，用到了不匹配结果
- 模型参数和真实系统偏差太大

---

## 10. ODrive / 执行器层怎么查

控制算法没问题，但执行层不跟，也一样会倒。

### 10.1 必看三个事实

1. 发出去的 `left_velocity/right_velocity`
2. 读回来的 `enc_vel_l/enc_vel_r`
3. 车体实际运动方向

三者必须一致。

### 10.2 你要确认的事情

- 命令方向和编码器方向一致
- 两个轮子方向约定一致
- 左右轮都能跟上，不是一边弱一边强
- 速度命令不是被底层 silently clamp 掉
- `emergency_stop()`、`set_velocities(0,0)` 能及时生效

### 10.3 一个很有效的离线检查

先不真正进平衡，只做低速手工命令测试：

- 给小的正速度
- 给小的负速度
- 给左右相反速度
- 看 telemetry 回来的左右轮速度和车体方向

这一步做不对，别进平衡。

---

## 11. 传感器层怎么查

### 11.1 IMU

检查：

- pitch/gyro 是否连续
- 时间戳是否持续更新
- `is_data_fresh(50)` 是否长期成立
- 激烈运动时 pitch 是否突跳
- USB polling 停止/退出时是否能干净结束

### 11.2 编码器

检查：

- 静止时速度是否接近 0
- 缓慢推行时速度是否连续
- 位置是否单调积累
- 左右轮平均后是否符合车体前后运动

### 11.3 最常见的隐藏坑

- IMU 的 pitch 是“机体前倾为正”，而你的控制公式默认“前倾为负”
- ODrive 某一轴在内部做了符号翻转，但上层又翻一次
- 编码器位置是 turns，速度是 turns/s，而你主观上按 rad/s 去理解

---

## 12. 推荐的调试顺序：从安全到性能

### 阶段 A：不上电机闭环，只看观测
目标：

- IMU、编码器、遥控输入全部对
- 符号和零点全部对

做法：

- 电机不使能或挂空载
- 观察 telemetry / log
- 人手推动车体，检查状态量响应

### 阶段 B：低风险执行器测试
目标：

- 电机方向、编码器方向、实际运动方向一致

做法：

- 极小速度指令
- 单独测前后、左右差速

### 阶段 C：关闭 NMPC，只跑基础平衡
目标：

- 先让 `estimate_state + generate_reference + state feedback` 能稳定工作

做法：

- `nmpc_enabled = false`
- steering 尽量为 0
- 先只试短时间扶持起立

### 阶段 D：压低复杂功能
目标：

- 去掉会干扰理解的变量

建议先临时减弱或关闭：

- 自适应 trim
- 过强的位置约束
- 复杂 steering 输入
- NMPC

### 阶段 E：基础稳定后，再逐项恢复
恢复顺序建议：

1. adaptive trim
2. position hold
3. steering
4. NMPC

---

## 13. 建议你直接加的调试打印

建议在 `BalanceController::update()` 每 N 个 tick 打一次：

```text
tick
pitch / pitch_rate
disp / vel
target_speed / target_disp
nominal_pitch / constrained_pitch
theta_error / x_error / v_error
base_velocity_cmd
left/right cmd
```

建议在 `Robot::control_tick()` 每 N 个 tick 打一次：

```text
dt
state
imu fresh?
rc fresh?
enc disp / enc vel
output left / right
```

建议在 NMPC 结果采纳处打印：

```text
nmpc valid?
result age
source_tick
state mismatch
applied or rejected
```

这样你很快就能知道问题在：

- 观测
- 参考
- 控制
- 执行
- 还是异步结果采纳

---

## 14. 对症下药：常见故障 → 首查位置

### 14.1 arm 后立刻倒
先查：

- `pitch_deg` 符号
- 左右轮正方向
- `base_velocity_cmd` 方向
- `balance_point_deg_` 是否错

### 14.2 能站几秒，但越来越偏
先查：

- `adaptive_trim_deg_`
- `target_displacement_turns_`
- 编码器速度零偏
- 轮子实际速度闭环是否对称

### 14.3 剧烈发抖
先查：

- `gyro_y` 噪声
- `wheel_velocity` 抖动
- `dt` 是否稳定
- 控制增益是否过大
- ODrive 速度环是否引入高频响应

### 14.4 推杆后响应很肉
先查：

- `target_speed_rate_limit`
- `speed_to_pitch_ff`
- `accel_to_pitch_ff`
- `lqr_k_v`
- ODrive 是否跟不上命令

### 14.5 松杆后停不住
先查：

- `stop_speed_threshold_tps`
- `stop_hold_capture_window_tps`
- `reference_position_gain`
- `target_displacement_catchup_rate`

### 14.6 退出/停机卡住
先查：

- `tcp_.stop()` 是否有阻塞 socket / join
- `imu_.stop_polling()` 后是否真的停止异步传输
- 是否还有线程在访问 USB 句柄
- `odrive_.detach()` 前是否仍有并发访问

---

## 15. 一个非常实用的最小化调试配置

如果你要快速隔离控制问题，建议先试一个“最小复杂度配置”：

- `nmpc_enabled = false`
- `ekf_enable = true`
- `adaptive_balance_enable = false`
- steering 输入固定 0
- 低速、小角度、小范围测试
- 开日志，降低 `logging_decimation`

这样你先把最短闭环打通：

```text
传感器 → EKF → reference → state feedback → 电机
```

基础站稳以后，再恢复 NMPC 和 trim。

---

## 16. 最后的调试原则

### 原则 1：先确认方向，再谈增益
方向错了，参数越大死得越快。

### 原则 2：先确认观测，再谈控制
估计状态错了，控制律再漂亮也没用。

### 原则 3：先确认执行器能跟，再谈算法先进
ODrive 跟不上、方向不一致，LQR/NMPC 都救不了。

### 原则 4：先关复杂功能，再逐项恢复
一次只改一个层级，你才知道是谁造成的变化。

---

## 17. 建议配合阅读的文件

和这份文档一起看，理解会最快：

- `Doc/09_BALANCE_ALGORITHM_FULL_REVIEW.md`
- `Doc/03_DEBUG_AND_TROUBLESHOOTING.md`
- `Doc/07_LOGGING_REPLAY_AND_TESTING.md`
- `robot/src/robot.cpp`
- `robot/src/balance_controller.cpp`
- `robot/src/nmpc_controller.cpp`

---

## 18. 一句话总结

这套系统最有效的调试法不是“盲调增益”，而是按下面顺序硬检查：

```text
状态机
→ 传感器新鲜度
→ 符号方向
→ 状态估计
→ 参考生成
→ 反馈控制
→ 执行器响应
→ NMPC 结果采纳
→ 参数优化
```

只要你按这个顺序做，问题一定能被快速缩到某一层，而不是在全系统里乱猜。
