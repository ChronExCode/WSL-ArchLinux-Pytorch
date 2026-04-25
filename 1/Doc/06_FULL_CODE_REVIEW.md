# 全工程 Code Review（发布级审计）

## 1. 审计范围

本轮审计覆盖：

- `common/mini_json.h`
- `controller/`：UI、参数面板、TCP 客户端、持久化、主循环
- `robot/`：状态机、控制器、EKF、tracking MPC、TCP 服务、日志
- `tools/`（当前仓库未包含独立 `tests/` 目录）
- `Doc/` 现有文档

审计目标不是只清 warning，而是检查：

- 编译卫生
- 接口一致性
- 运行时鲁棒性
- 控制与通信安全边界
- 调试与维护可用性

---

## 2. 本轮确认并修复的问题

### 2.1 `controller/src/config_persistence.cpp` 存在损坏的换行字符串

问题：`f << obj.str() << "\n";` 在整合过程中曾被写坏成跨行字面量，属于明确编译风险。

修复：恢复为单行输出，并保留对 `server_ip` 的 JSON 转义。

影响：若不修，controller 可能无法编译，或保存配置文件时行为异常。

### 2.2 `controller/include/tcp_client.h::send_line()` 未处理部分发送

问题：旧实现只调用一次 `send()`，在负载高、网络短暂拥塞或较大 payload 时，存在部分发送风险。

修复：改为循环发送直到全部数据发完，失败则断开。

影响：修复后参数下发与控制命令的 TCP 语义更可靠。

### 2.3 `common/mini_json.h` 缺少 `cstdio` 头

问题：`snprintf()` 被使用，但头文件未显式包含。某些工具链上可能只是侥幸通过。

修复：显式包含 `<cstdio>`。

### 2.4 `Config::sanitize()/validate()` 覆盖不完整

问题：`tcp_port`、`msp_request_interval_us`、`usb_bulk_timeout_ms`、`usb_rx_queue_depth` 未统一检查。

修复：补充 sanitize 和 validate。

影响：避免远程参数更新或默认值异常导致系统/通信层进入未定义状态。

### 2.5 `controller/include/wayland_app.h` 参数分组索引存在不一致

问题：MODEL、AUTO LQR、MANUAL LQR、NMPC MODEL/COST/VALID 中部分参数落到了错误基础组，UI 虽能显示，但组语义会混乱。

修复：统一组索引，使参数分组和文档描述一致。

---

## 3. 高优先级剩余问题（建议尽快处理）

### 3.1 轻量 JSON 解析器仍是“足够用”而不是“严格 JSON”

`mini_json.h` 当前适合：
- 平坦对象
- 简单数值/布尔/字符串
- 明确受控的协议

它不适合：
- 深层嵌套
- 数组
- 完整 JSON 合法性校验
- 恶意输入防御

建议：
- 当前工程内继续保留，用于受控内网协议
- 若后续协议扩展，优先引入成熟 JSON 库

### 3.2 `TcpServer` 与 `TcpClient` 仍缺少发送队列/背压机制

当前实现已经处理“部分发送”，但仍是同步发送。高频 telemetry 或 UI 卡顿时，通信线程可能被发送阻塞拖慢。

建议：
- 后续把发送层改成 ring buffer + 单独发送线程
- 至少给 telemetry 加丢帧保护

### 3.3 `CsvLogger` 参数快照与更新记录

当前代码已经具备 Config 快照和远程参数更新事件记录能力；最新修订还在 logger 启动成功后写入初始 Config 快照。

建议：
- 保持 replay 工具解析 `#CONFIG_SNAPSHOT` 与 `#CONFIG_UPDATE` 注释行
- 后续把参数快照和数据行关联到实验 ID，便于多次调参比较

---

## 4. 中优先级问题

### 4.1 `robot/src/robot.cpp` 中参数回调太长

表现：单个 lambda 内完成大量参数解析。

风险：
- 审计困难
- 漏字段时不易发现

建议：
- 拆成 `apply_pid_params()`、`apply_observer_params()`、`apply_ekf_params()`、`apply_mpc_params()` 等小函数

### 4.2 `balance_controller.cpp` 兼容层较厚

当前同时保留 legacy PID、state feedback、EKF、MPC，适合过渡，但长期会提高维护复杂度。

建议：
- 下一阶段明确哪些路径保留，哪些只做 fallback
- 把 legacy 字段在文档中标记为“兼容层”

### 4.3 UI 参数表仍是手写初始化

虽然已经表驱动，但大量 `add(...)` 仍集中在一个函数里。

建议：
- 中期可迁移到 `constexpr` 参数表描述数组
- 再由 UI / 持久化 / 文档共用

---

## 5. 控制算法审计结论

### 5.1 总体结构是健康的

当前控制链：

- 状态估计：LPF/observer + EKF 可切换
- 参考生成：速度/位移/俯仰参考
- 外层：tracking MPC / beam-search 近似
- 内层：LQR 风格状态反馈或自动 LQR
- 执行层：速度命令到 ODrive

这种分层结构在树莓派 + USB + ODrive 速度模式下是合理的。

### 5.2 当前最大技术债不在控制律，而在“基础设施”

也就是：
- 参数快照
- replay 可复现性
- 更严格的协议与日志
- 更多最小测试

继续只堆算法，收益会低于先补这些基础设施。

---

## 6. 建议的下一步路线

1. 把 `Config` 参数清单集中成单源描述（single source of truth）
2. 给日志增加参数快照与事件行
3. 增加 replay 对比工具：同一日志下比较不同参数曲线
4. 给 `mini_json` 和 `Config sanitize/validate` 增加更多测试
5. 再考虑 QP MPC、系统辨识和 autotune

---

## 7. 本轮结论

本工程已经从“能跑”推进到“可维护、可调试、可继续工程化”的阶段。

本轮修复后，最关键的收益是：
- controller 与 robot 的参数链更可靠
- JSON 与 TCP 的基础设施更稳
- 文档更适合团队协作与后续调试

但它还没有到“完全工业发布版”。真正接近工业发布的下一个门槛，是：
- 参数快照
- replay 完整复现
- 更多自动化测试
- 更强的错误恢复与通信背压
