# 控制算法与软件架构深度设计说明

## 1. 总体设计

系统采用“分层控制”思路：

1. 传感器采集
2. 状态估计（LPF / observer / EKF）
3. 参考生成
4. Tracking MPC 参考修正
5. LQR / 状态反馈稳定控制
6. 差速转向与安全保护

这套结构的目标不是让 MPC 直接做最内层稳定，而是让：

- 快环稳定由本地状态反馈完成
- 慢一点、带前瞻的参考修正由 MPC 完成

这样更符合树莓派 + USB + TCP + IMU 的实时条件。

## 2. 目录结构

### robot/

- `include/types.h`：系统参数、状态、调试结构体
- `include/control_model.h` / `src/control_model.cpp`：共享物理模型、离散线性模型、LQR 求解
- `include/ekf.h` / `src/ekf.cpp`：四状态 EKF
- `include/balance_controller.h` / `src/balance_controller.cpp`：控制主逻辑
- `include/nmpc_controller.h` / `src/nmpc_controller.cpp`：Tracking MPC / 增量式 LTV MPC
- `src/robot.cpp`：主状态机、设备管理、参数应用、主循环
- `src/tcp_server.cpp`：TCP 命令与 telemetry
- `src/odrive_usb.cpp` / `src/msp_imu_usb.cpp`：驱动层

### controller/

- `include/wayland_app.h`：参数面板、UI 状态、参数定义
- `src/main.cpp`：主循环、evdev 输入、TCP 收发、参数发送
- `src/hud_renderer.cpp`：HUD 和参数面板绘制
- `src/config_persistence.cpp`：参数保存与加载

## 3. 状态定义

基础状态向量：

`x = [position, velocity, pitch, pitch_rate]^T`

其中：

- `position`：平均轮位移（turns）
- `velocity`：平均轮速度（turns/s）
- `pitch`：车身俯仰角（deg）
- `pitch_rate`：俯仰角速度（deg/s）

## 4. 误差系统与增广系统

MPC 不直接在原始状态上优化，而是在误差系统上进行：

`e_k = x_k - r_k`

其中参考：

`r_k = [x_ref, v_ref, theta_ref, 0]^T`

进一步增广成 5 维状态：

`z_k = [e_x, e_v, e_theta, e_theta_dot, delta_u_{k-1}]^T`

这里第五维表示上一拍相对参考输入的输入偏差，使输入动态也纳入状态空间。

## 5. 增量式 LTV MPC

控制输入不是绝对值，而是增量：

`Δu_k = u_k - u_{k-1}`

增广系统形式：

`z_{k+1} = A_aug,k z_k + B_aug,k Δu_k + c_k`

其中：

- `A_aug,k`：随参考变化的线性时变矩阵
- `B_aug,k`：输入矩阵
- `c_k`：参考轨迹变化引入的仿射项

代价函数由：

- 状态误差项
- 输入偏差项
- 输入增量项
- 终端代价项

组成。

## 6. 为什么 MPC 不直接做主稳定器

原因：

- 树莓派算力有限
- USB / TCP / IMU 存在时延和异步性
- 快速平衡必须稳定可靠

所以本系统采用：

- `MPC`：负责参考层前瞻修正
- `LQR / 状态反馈`：负责最内层稳定

## 7. 共享物理模型

`control_model.*` 统一提供：

- 线性离散模型构建
- LQR Riccati 求解
- 物理参数到模型参数的映射

这样：

- 自动 LQR
- MPC terminal cost
- EKF 过程模型

都尽量基于同一套物理模型，减少模型不一致。

## 8. EKF

EKF 使用 4 维状态：

`[x, v, theta, theta_dot]`

观测量：

- 编码器位移
- 编码器速度
- 融合后的 pitch
- gyro_y

EKF 目前是“线性预测 + 线性观测”的工程实现，重点是：

- 可编译
- 可落地
- 可通过 Q/R 调整响应速度和噪声抑制

## 9. 参考生成

`balance_controller` 里先基于手柄输入生成：

- `v_ref`
- `x_ref`
- `theta_nominal`

其中 `theta_nominal` 由：

- 平衡偏置
- 速度前馈
- 加速度前馈
- 位置误差项

共同组成，再经过限幅和限斜率。

## 10. 状态反馈控制律

在不开自动 LQR 时，使用工程化的四状态反馈：

`u = v_ref - k_theta*e_theta - k_theta_d*e_theta_dot - k_x*e_x - k_v*e_v`

在开自动 LQR 时，反馈矩阵来自共享物理模型和 LQR 权重。

## 11. 参数链

完整链路：

`controller UI -> JSON -> TCP -> robot parameter callback -> Config -> controller logic`

控制面板的分组只影响显示，不影响参数发送和持久化。所有参数发送都是基于完整参数表遍历。

## 12. 设计原则

- **快环稳定优先**
- **前瞻优化辅助而不是接管**
- **所有参数尽量在线可调**
- **模型尽量统一**
- **提供回退路径**（例如 EKF 可关，MPC 可关）

