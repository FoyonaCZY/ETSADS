# 桥接协议与复核记录

## SDK 来源

使用 SCS 官方 SDK **1.15** 归档，不是设计文档中的 1.14。
遥测 API 仍为 1.01，输入 API 仍为 1.00。原始头文件与许可证在
`vendor/scs_sdk`，下载地址和 SHA-256 记录在该目录的 `PROVENANCE.md`。
`crates/sdk-sys/src/bindings.rs` 由 bindgen 0.72.1 实际生成；包含所有使用类型的
尺寸、对齐和字段偏移编译期断言。默认构建使用提交的绑定，不要求最终用户安装 LLVM。
需要重新生成时设置 `LIBCLANG_PATH` 并执行
`cargo build -p etsads-sdk-sys --features regenerate`。

## 内存与时钟

映射名统一使用 `Local\\`，协议头 64 字节：magic、schema、payload size、slot count、
latest index 均为对齐的 64 位原子值。每槽为序号加原子字节 payload，末尾按 8 字节对齐。
序号和 payload 采用顺序一致原子操作；不会将非原子的 `T` 暴露给并发读写线程。
传统 `UnsafeCell<T>` + volatile seqlock 在 Rust 中仍可能构成数据竞争，本实现不使用该方式。
每个 ring 只有一个发布者。读取方同时验证槽的代数和版本，拒绝被环绕覆盖的数据。
进程退出自动关闭视图和句柄；打开已有映射不会清零。

兼容性新增信息使用独立映射，旧 Telemetry/Control/Gate/Applied 布局保持不变：

| 名称 | payload 字节 | 槽数 | 用途 |
|---|---:|---:|---|
| `ETSADS_Info` | 192 | 1 | SDK 版本、进程与会话身份，补丁版本另读 EXE PE |
| `ETSADS_SafetyRequest` | 24 | 1 | 请求启停；监督进程验证序号和时效 |
| `ETSADS_VehicleCounters` | 56 | 1 | 官方 truck.odometer，单位 km，附有效位 |
| `ETSADS_InputDiagnostics` | 72 | 1 | 输入批次计数、flags、批次有界性证据 |
| `ETSADS_InputActions` | 320 | 256 | 完整SDK批次实际发出的按钮边沿、消费计数、prepare/end QPC及命令序号 |
| `ETSADS_FrameTiming` | 88 | 256 | start/end/input prepare 的实际回调顺序与 QPC |
| `ETSADS_SupervisorStatus` | 64 | 1 | 独立进程身份、实际已加载后向 profile 的 SHA-256、配置与 readiness |

完整 wire 声明与编译期大小断言在 `crates/msgs/src/protocol.rs`。

`WireHeader.sim_time_us` 保留原始 SDK `simulation_time`；官方头文件明确该时钟在暂停时
仍增加。`TelemetryFrame.paused_simulation_time_us` 在暂停时冻结，因此高层
`EgoState.header.sim_time_us` 使用它。QPC 不是微秒，必须用帧内 `qpc_frequency` 换算。
`Telemetry.header.qpc` 在 frame_start 采集；`AppliedControl.applied_qpc` 在输入帧开始
采集，可与 `source_frame_id`、render/simulation clocks 分析 E1/E9。

官方 `scssdk_telemetry_truck_common_channels.h` 规定角速度为 rotations/s、车轮转向为
rotations。wire 保留原值，高层 yaw_rate 和 wheel_angle 均乘 TAU 转为弧度制。

## 输入隔离

DLL 只读取 Control 和 Gate，回写 Applied。CLI/UI 通过 SafetyRequest 请求启停；
Gate 的业务写入者是独立安全监督进程。Gate 心跳超 300 ms、disabled、暂停或非有限值
均输出零轴并松开按住按钮。命令超 200 ms 时轻刹斜坡，超 1 s 释放；Gate 无效优先释放。
`enabled` 明确是 Off=0、Active=1、BrakeOnly=2 的模式，不可按非零当作自动驾驶许可。
BrakeOnly只允许独立有界制动，未知模式全零；新命令不能解除失联锁存。DLL独立保留最后
ACTIVE命令期限，最迟原命令年龄1秒归零。旧DLL的mode2为全零安全降级。原文矛盾与联合
进程验证的范围见 `BRIDGE.md`，不能把主动RAII停止误当作意外kill缓刹的证据。
所有输入事件在 first_in_frame 批量计算，仅返回变化值；first_after_activation 全量重发。
批次开始后直到返回 not_found 前绝不再次填充队列，即使 SDK 重复设置 first 标志；
批次上界由名称数组总长 `INPUT_COUNT` 推导；当前追加engine后为53个输入，单批最多53个
事件和54次取事件调用，旧pulse索引保持不变：quickload为44，engine为45。
`/input_actions`只在not_found完成整个批次后发布；中途激活状态变化或退出不会把未完成
批次认证为已消费。它记录已返回给SDK的事件，实际游戏mix生效仍须随后遥测状态变化确认。
单次按钮使用计数器并产生按下/松开边沿，失能期间不积压按钮动作。
设备名为 `etsads`。SDK 头文件说明 register_device 调用期间完整处理描述符，返回后
可以释放描述符和名称字符串。所有 FFI 入口及回调包含 catch_unwind。

当前源码的 steer/pulse CLI 注册 Ctrl+C handler，并在预热、等待许可、活动三个阶段
检查停止 token；正常退出、取消和错误均通过同一 RAII guard 发布零轴并请求 gate off。
隔离映射测试覆盖三个阶段及外部已 arm 的调用；它不向真实游戏映射写入。

## 扩展与帧捕获

ETS2LA 原始扩展映射在游戏主线程下一次 frame_start 复制，并标为前一物理 frame_id。
GameAdapter 使用 extension 的 frame_id 回读 64 槽遥测历史，绝不把 n-1 扩展与 n 遥测混合。
`require_extension` 可要求缺扩展时停止产出快照。默认允许显式 telemetry-only 快照。
扩展消息约 107 KB，DLL 复用堆缓冲；读取使用 `latest_boxed`，避免大结构多层返回造成栈溢出。

Capture Reader 按上游 `ETS2LA/capture/include/ets2la_capture/protocol.h` 核实：
DataHeader 中 slot_stride 在 16、slots_offset 在 24、width/height/stride/format 在
32/36/40/44，像素在槽首 +256。读者槽通过 CAS 认领，poll 更新 GetTickCount64 心跳。
图像通过 Windows ReadProcessMemory 复制为自有缓冲，之后复核 seq 和 generation；
验证所有尺寸、格式、偏移、映射长度后才访问像素。当前支持 BGRA/BGRX/RGBA 8 位格式，
其他格式明确拒绝。这里没有声称完成上游 DLL 实机帧捕获验收。

投影模块显式提供 8 种矩阵/四元数/前向约定候选，没有把未经实机验证的约定设为默认。
合成相机单元测试只能证明计算正确，E4/E6 仍需真实截图与相机数据标定。

## 自动验证的范围

单元测试覆盖输入超时边界、AEB、限幅、NaN、暂停，真实 Windows 命名映射、跨进程读取、
原子快照压力、schema 拒绝、帧对齐与暂停时钟，以及模拟 SDK 的 10 次 init/shutdown。
模拟 SDK 生命周期测试使用单独映射名，不会读写运行中游戏的 ETSADS 映射。
这些测试不能替代游戏控制台 sdk reload、实际转向响应、一小时录制或投影准确性验收。
