# 接口一致性审计说明

## 1. 审计范围

本次重点审计四条链：

1. `controller` 参数面板
2. `controller` 参数持久化
3. TCP 参数协议
4. `robot` 侧接收与应用

## 2. 审计结论

在当前工程整合包中：

- 参数面板基于参数表驱动
- 序列化发送基于参数表遍历
- 持久化保存/加载基于参数表遍历
- `robot` 侧参数回调对暴露参数逐项接收

也就是说，当前 UI 中能看到并发送的控制参数，在协议和 robot 应用链路上是贯通的。

## 3. 本次整合包基线

本包以 `self_balance_robot_with_ekf.tar.gz` 为基础，叠加：

- `self_balance_robot_compilefix_maincpp.tar.gz` 中的 `controller/src/main.cpp`
- `self_balance_robot_panel_layout_fix2.tar.gz` 中的 `controller` 布局与 CMake 修正
- 新增 `Doc/` 文档目录

## 4. 已确认修复的具体问题

### 4.1 main.cpp 编译错误

问题：
- 事件循环中多了一个 `}`
- 导致 `for (int i = 0; i < nfds; ++i)` 作用域被提前关闭
- 后续 `events[i]` 报错

状态：已修复。

### 4.2 telemetry JSON 转义问题

问题：
- `msg/status_msg` 直接拼进 JSON
- 若含引号、反斜杠、换行，JSON 会损坏

状态：已在 `tcp_server.cpp` 中修复。

### 4.3 controller 目标名

问题：
- `controller/CMakeLists.txt` 旧目标名为 `robot_controller`

状态：已改为 `controller`。

## 5. 当前限制

虽然本整合包已经包含完整工程文件与文档，但当前环境并未对整工程做目标机全量链接验证，因为：

- 此环境缺 `libusb-1.0` 头文件
- 此环境缺 Wayland 开发头

因此，最终完整编译仍需在目标机上验证。

## 6. 推荐验证顺序

1. 编译 `controller`
2. 编译 `robot`
3. 启动 `robot`
4. 启动 `controller`
5. 改一项参数并按 `Y`
6. 确认 robot 侧收到参数更新
7. 再进行实车控制调试

