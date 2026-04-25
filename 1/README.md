# Self-Balancing Robot v2 — libusb + evdev

**Raspberry Pi 5 + ODrive v3.5 + SpeedyBee F405 V5 + MSI Claw 8 AI+ Controller**

USB 通信通过 libusb 直接操作 bulk endpoint，手柄输入通过 evdev 读取，启动时自动将 MSI Claw 从 XInput 切换到 DInput 模式。


## Documentation

发布级文档位于 `Doc/` 目录，建议按以下顺序阅读：

1. `Doc/00_DOC_INDEX.md`
2. `Doc/01_QUICK_START_AND_TUNING.md`
3. `Doc/02_CONTROL_AND_ARCHITECTURE.md`
4. `Doc/03_DEBUG_AND_TROUBLESHOOTING.md`
5. `Doc/06_FULL_CODE_REVIEW.md`

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    MSI Claw 8 AI+ A2VM                      │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  controller (Wayland + Cairo + evdev)          │  │
│  │  • 自动 HID 模式切换 (XInput→DInput)                   │  │
│  │  • evdev 读取 /dev/input/event* 手柄                   │  │
│  │  • 左摇杆: 油门/转向     • LT/RT: 速度限制             │  │
│  │  • Menu(≡): 解锁  B: 上锁  View: 急停                 │  │
│  └────────────────────┬───────────────────────────────────┘  │
│                       │ WiFi TCP :9000                       │
└───────────────────────┼─────────────────────────────────────┘
                        │
┌───────────────────────┼─────────────────────────────────────┐
│  Raspberry Pi 5       │                                     │
│  ┌────────────────────▼───────────────────────────────────┐ │
│  │  balance_controller (libusb + async bulk)               │ │
│  │  • UsbManager: VID/PID 枚举 + 热插拔 + CDC 发现         │ │
│  │  • ODriveUsb: 异步 bulk OUT + 4x bulk IN 流水线         │ │
│  │  • MspImuUsb: MSP 流式解析器 + 异步 bulk                │ │
│  │  • BalanceController: EKF + tracking MPC + 状态反馈 (200Hz)                  │ │
│  └────────┬─────────────────────────────┬──────────────────┘ │
│           │ USB bulk                    │ USB bulk            │
│  ┌────────▼────────────┐      ┌────────▼─────────────┐      │
│  │  ODrive v3.5         │      │  SpeedyBee F405 V5   │      │
│  │  1209:0D32           │      │  0483:5740            │      │
│  └──────────────────────┘      └──────────────────────┘      │
└──────────────────────────────────────────────────────────────┘
```

## MSI Claw 8 AI+ — DInput 模式切换

MSI Claw 8 AI+ 在 Linux 下默认启动为 XInput 模式（VID:PID `0db0:1901`），此模式下大部分按键不通过 evdev 上报，手柄无法正常使用。

**本程序启动时自动检测并切换到 DInput 模式**（`0db0:1902`）：

1. 扫描 `/sys/class/hidraw/` 查找 `0db0:1901` 设备
2. 向对应 `/dev/hidrawN` 发送 HID output report: `0F 00 00 3C 24 02 00 00`
3. 设备 USB 重新枚举，以 `0db0:1902` 重新出现
4. 等待 3 秒后扫描 evdev 设备

切换后所有按键和摇杆通过 evdev 正常工作。重启后需重新切换（程序每次启动自动执行）。

如需跳过自动切换：`--no-dinput-switch`

### 前置条件

```bash
# 用户需要在 input 组才能访问 /dev/hidraw* 和 /dev/input/event*
sudo usermod -aG input $USER
# 重新登录生效

# 可选：安装 MSI Claw HID 内核驱动
yay -S hid-msi-claw-dkms-git
```

## MSI Claw 8 AI+ 按键映射（DInput 模式）

经实机测试确认的完整 evdev 映射：

```
物理按键            evdev code    内核名称            本程序功能
──────────────────────────────────────────────────────────────
X  (左方)           304           BTN_SOUTH           —
A  (下方)           305           BTN_EAST            —
B  (右方)           306           BTN_C               DISARM 上锁
Y  (上方)           307           BTN_NORTH           —
LB (左肩)           308           BTN_WEST            —
RB (右肩)           309           BTN_Z               —
LT (左扳机·数字)    310           BTN_TL              —
RT (右扳机·数字)    311           BTN_TR              —
View                312           BTN_TL2             E-STOP 急停
Menu (≡)            313           BTN_TR2             ARM 解锁
L-Stick click       314           BTN_SELECT          —
R-Stick click       315           BTN_START           —
M1 (背面)           305           同 A                —
M2 (背面)           306           同 B                —
MSI 按键            —             WMI (不在evdev)     —

左摇杆 X            ABS_X (0)     -1.0 ~ +1.0        转向
左摇杆 Y            ABS_Y (1)     -1.0 ~ +1.0        油门(反转)
右摇杆 X            ABS_RX (3)    -1.0 ~ +1.0        —
右摇杆 Y            ABS_RY (4)    -1.0 ~ +1.0        —
LT (模拟)           ABS_Z (2)     0.0 ~ 1.0          速度限制
RT (模拟)           ABS_RZ (5)    0.0 ~ 1.0          速度限制
D-Pad 左右          ABS_HAT0X(16) -1 / 0 / +1        —
D-Pad 上下          ABS_HAT0Y(17) -1 / 0 / +1        —
```

## 依赖安装

### Raspberry Pi 5 (Raspberry Pi OS / Debian)

```bash
sudo apt update
sudo apt install -y cmake g++ libusb-1.0-0-dev pkg-config
```

### MSI Claw 8 AI+ (Arch Linux)

```bash
sudo pacman -S cmake gcc libusb wayland wayland-protocols cairo libxkbcommon pkg-config evtest
```

## 构建

### 树莓派 5 — 平衡控制器

```bash
cd robot
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
# 输出: ./balance_controller
```

### MSI Claw 8 — 遥控器

```bash
cd controller
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
# 输出: ./robot_controller
```

也可以在仓库根目录执行：

```bash
./build.sh Release
```

## 运行

### 1. 查看 USB 设备

```bash
sudo ./balance_controller --list-usb
```

### 2. 启动平衡控制器（树莓派）

```bash
sudo ./balance_controller
sudo ./balance_controller --pitch-kp 50 --pitch-ki 1.0 --pitch-kd 1.5
```

### 3. 启动遥控器（MSI Claw）

```bash
# 自动切换 DInput + 自动检测手柄
./robot_controller 10.0.0.218

# 指定 evdev 设备
./robot_controller 10.0.0.218 --joy /dev/input/event9

# 跳过 DInput 切换（已经是 DInput 模式）
./robot_controller 10.0.0.218 --no-dinput-switch
```

启动时终端输出示例：

```
[Main] Checking for MSI Claw in XInput mode (0db0:1901)...
[Main] Found MSI Claw XInput at /dev/hidraw2, switching to DInput...
[Main] Mode switch sent! Waiting for re-enumeration...
[Main] Waiting 3s for USB re-enumeration...
[Gamepad] Scanning /dev/input/event* ...
  [OK] /dev/input/event9: Љ
[Gamepad] Opened: Љ (/dev/input/event9)
  ABS_0: min=-32768 max=32767 flat=128
  ABS_1: min=-32768 max=32767 flat=128
  ABS_2: min=0 max=255 flat=0
  ABS_5: min=0 max=255 flat=0
[Main] Gamepad: Љ
[Main] Button mapping (evdev codes):
  ARM    = 313 (Menu/≡)
  DISARM = 306 (B)
  E-STOP = 312 (View)
```

### 4. 操作流程

1. 启动程序（自动切换 DInput 模式）
2. 等待终端显示 gamepad 连接成功
3. 竖直握住平衡车
4. 按 **Menu (≡)** 解锁 → 进入平衡模式
5. 左摇杆控制前进/后退/转向
6. **LT/RT 扳机**（模拟）控制速度上限
7. **B** 上锁 / **View** 急停

### 5. 摇杆映射检测模式

```bash
./robot_controller 0 --joy-map
```

显示每个按键按下和摇杆移动的 evdev 代码，用于调试或自定义映射。

## 遥测 HUD 面板

控制器 HUD 右侧面板显示 9 项实时遥测数据：

| 项目 | 颜色 | 说明 |
|------|------|------|
| PITCH | 黄色 | 俯仰角。IDLE 时为 IMU 原始值，BALANCING 时为融合滤波值 |
| ROLL | 青色 | 横滚角 |
| YAW | 紫色 | 航向角 |
| SPEED | 绿色 | 速度估计值 |
| VBUS | 橙色 | ODrive 电源电压 |
| L MOTOR | 蓝色 | 左电机速度 |
| R MOTOR | 蓝色 | 右电机速度 |
| SPD LIMIT | 紫色 | 速度限制百分比 |
| ARMED | 红色 | 解锁状态 |

## 遥测协议（TCP JSON）

### 服务端 → 控制器（20Hz）

```json
{"type":"telemetry","pitch":1.23,"roll":0.45,"yaw":178.5,"speed":2.1,
 "vbus":24.1,"state":"BALANCING","left_vel":3.2,"right_vel":-3.1}
```

### 控制器 → 服务端（50Hz）

```json
{"type":"control","throttle":0.50,"steering":-0.30,"speed_limit":0.80}
{"type":"arm"}
{"type":"disarm"}
{"type":"estop"}
{"type":"set_pid","pitch_kp":50,"pitch_ki":1.0,"pitch_kd":1.5}
```

## 热插拔行为

| 事件 | 系统响应 |
|------|---------|
| 拔出 ODrive | 停止控制 → WAITING_USB → 自动重连 |
| 拔出 IMU | E-STOP → WAITING_USB → 自动重连 |
| 插入设备 | hotplug 回调 → 500ms 延迟 → 重连 → IDLE |
| 控制器断开/重连 | 服务端自动接受新连接，无需重启 |

## 状态机

```
INITIALIZING
     │
     ▼
WAITING_USB ◄──────────── 任一设备断开
     │
     │ (两个设备都连接 + IMU 有数据)
     ▼
   IDLE ◄─────────────── disarm
     │
     │ arm (IMU valid + tilt < 20° + 清除 estop)
     ▼
CALIBRATING → ODrive 闭环 → BALANCING
                                │
                      tilt>45° / IMU超时 / E-STOP / 断开
                                │
                                ▼
                              FAULT ───→ arm ───→ CALIBRATING
```

## PID 调参

```bash
# 命令行
sudo ./balance_controller --pitch-kp 50 --pitch-ki 1.0 --pitch-kd 1.5

# 运行时 TCP
echo '{"type":"set_pid","pitch_kp":50,"pitch_ki":1.0,"pitch_kd":1.5}' | nc <rpi-ip> 9000
```

| 参数 | 范围 | 说明 |
|------|------|------|
| pitch_kp | 30 – 80 | 主平衡响应 |
| pitch_ki | 0 – 3 | 消除稳态偏差 |
| pitch_kd | 0.5 – 3 | 阻尼 |
| speed_kp | 0.2 – 2 | 速度→倾斜映射 |
| yaw_kp | 5 – 30 | 转向灵敏度 |
| pitch_offset | -5 – +5° | 重心偏移补偿 |

## udev 规则（免 sudo）

创建 `/etc/udev/rules.d/99-robot.rules`：

```
# ODrive v3.5
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d32", MODE="0666", GROUP="plugdev"
# SpeedyBee F405 V5
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5740", MODE="0666", GROUP="plugdev"
# MSI Claw HID (for DInput mode switch)
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0db0", MODE="0666", GROUP="input"
# MSI Claw evdev
SUBSYSTEM=="input", ATTRS{idVendor}=="0db0", MODE="0666", GROUP="input"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| 手柄无反应 | MSI Claw 在 XInput 模式 | 程序自动切换，确认 `/dev/hidraw*` 权限 |
| `Cannot open /dev/hidrawN` | 权限不足 | `sudo usermod -aG input $USER` 后重新登录 |
| 切换后找不到手柄 | 重枚举时间不够 | 增大等待时间或手动指定 `--joy /dev/input/eventN` |
| Menu/View 无反应 | 仍在 XInput 模式 | 手动切换：见「DInput 模式切换」章节 |
| MSI/Quick Settings 键无反应 | 这两个键走 WMI 通道 | 无法在 evdev 中使用，选择其他按键 |
| event 编号变化 | USB 重枚举 | 程序自动扫描，无需指定编号 |
| PITCH 显示 0.0 | 已修复 | IDLE 时显示 IMU 原始值 |
| 控制器重连后崩溃 | 已修复 | TCP recv_thread 安全 join |

## 文件结构

```
robot/
├── CMakeLists.txt                  # RPi5 构建 (libusb)
├── build.sh
├── README.md
├── include/
│   ├── types.h                     # USB VID/PID, Config
│   ├── usb_manager.h               # libusb 枚举/热插拔/CDC/异步
│   ├── odrive_usb.h                # ODrive async bulk
│   ├── msp_imu_usb.h               # MSP 流式解析器
│   ├── balance_controller.h        # PID + 互补滤波
│   ├── tcp_server.h                # TCP JSON 协议
│   └── robot.h                   # 编排器 + 热插拔
├── src/
│   ├── main.cpp                    # 入口, --list-usb
│   ├── usb_manager.cpp             # libusb 实现
│   ├── odrive_usb.cpp              # ODrive 实现
│   ├── msp_imu_usb.cpp             # MSP 实现
│   ├── balance_controller.cpp      # PID 实现
│   ├── tcp_server.cpp              # TCP 实现（安全重连）
│   └── robot.cpp                 # 编排器 + arm 诊断
└── controller/
    ├── CMakeLists.txt              # Wayland 构建
    └── robot_controller.cpp      # evdev + HID 切换 + HUD
```

## 安全机制

- 倾斜 > 45° → 电机 idle
- TCP 丢失 > 500ms → 油门归零
- USB 断开 → 停机 + 自动重连
- E-STOP (View) → 同步 bulk write
- arm 时清除残留 estop 标志
- arm 前检查 IMU 有效性和倾斜角
- 控制器断开/重连不崩溃
- MSI Claw 自动 DInput 模式切换
