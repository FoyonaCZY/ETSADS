# 实测与源码核实发现

E 编号沿用 IMPLEMENTATION。源码结论与现场测量分开；未完成项保持待测。

> 文中括号里的 `artifacts/`、`runs/` 等路径是开发机上的本地实测记录，体积大且含存档信息，未随仓库发布。

**P0–P4实车阶段仍全部未通过。** 用户已明确授权，当前测试存档已应用`c_relatsteer=0`并重启验证；四段静止输入确认绝对响应，动态车辆标定仍未完成。

| 实验 | 状态 | 当前证据/下一步 |
|---|---|---|
| E1 物理步频率 | 静止现场已测；负载/行驶覆盖仍需补充 | `runs/live-stationary-01` 3602帧模拟步均16666µs；墙钟回调P95 32.9941ms，不能视为墙钟均匀60Hz |
| E2 人工/语义输入合成 | 静止模式及两段低速起步/制动已实测；人工叠加仍待测 | r8点火使RPM达到约550，前进4挡短测峰值0.98/5.54km/h，均正常制动、驻车；暂停后恢复的2.48秒观测中Gate持续Off、Applied轴全零。尚非高速制动、人工接管或模式切换完整验证 |
| E3 转向映射 | 当前测试存档的静止绝对模式已确认；动态辨识待测 | r7在±0.1、±0.25下effective稳定为相反符号同幅值，轮角稳定，释放后57–132ms内实际转向及轮角回零；r5相对模式的积分结果保留为历史 |
| E4 投影矩阵 | 固定版本C1获三组可见车外车辆支持，已有r7持久回归；全视角精确标定未完成 | 100帧穷举后，三张cam8实际车辆排除了其它七种；人工region不是精确body groundtruth。见实录复核（`artifacts/r5-projection-audit/README.md`）和[测试夹具](../crates/game/testdata/README.md) |
| E5 随机道路事件 | 待测 | 必须遇到真实事故/施工并对照列表与画面 |
| E6 相机编号 | 当前车型8个按键视角已有实录；自动分类标定未完成 | r6实录码4明确表示NearCabinUnclassified，码3是Other；均不是原生cam编号。按键完成边与原始姿态要单独关联，不由目录名推定整段视角 |
| E7 双 Hook 顺序 | 单一自研Hook独立GPU验证通过，静止实机已有绘制及捕获对照；逐Present证明/持续驾驶兼容性仍缺 | Sky已看到UNCAL及可见车辆框，但无精确截图QPC，不能声称与frame-dump同Present；详见实机报告（`artifacts/r6-exterior-and-release-rerun-findings.md`） |
| E8 DLC归档合并 | 已在真实归档核实 | TruckLib AssetLoader内置多归档叠加；base_map和10个DLC归档含1101扇区，具体以导出清单为准 |
| E9 输入/遥测回调顺序 | 静止主动输入与回调顺序已测；动态控制时延仍待测 | r5 112个Active样本write→prepare P95 3.1764ms/max6.352ms；物理采样中制动首次可见15.85/19.18ms，不把空闲旧命令年龄当延迟 |

## 官方 SDK 与设计文字的差异

实际下载并固定的是 SCS SDK 1.15，接口仍协商 telemetry 1.01/input 1.00。官方文件与许可证位于 `vendor/scs_sdk`，bindings由真实头文件生成。

- `scssdk_telemetry_event.h` 明确 `simulation_time` 在暂停时仍变化，`paused_simulation_time` 才冻结。原始协议保留两者；高层 EgoState 使用冻结时间，安全新鲜度使用 QPC。
- `truck.local.velocity.angular` 是 rotations/s；车轮 `steering` 是 rotations。适配层转为 rad/s 与 rad。
- ETS2LA/capture 的数据头有 magic/version/generation/slot数；尺寸并不是从偏移0开始。按上游真实 `protocol.h` 解析并验证边界。
- `ExtensionFrame` 在下一物理步开始转发前一帧。适配层必须回读同帧历史遥测，不能永远将最新遥测 n 和扩展 n−1 配对失败后静默退化。
- 106KB 的扩展状态避免在调试线程栈上多层按值传递；读端使用盒装快照，DLL复用预分配缓冲。

## 地图结构

- 车道没有一对一原生 u64 UID。保留 owner/node原生UID，并以复合字符串表达车道，运行时内部ID只作为索引。
- TruckLib `AssetLoader` 已实现归档联合，无需自制 `IHashFsReader`。
- PPD conflict 的 Position 是曲线距离标量，不能直接当作世界坐标或冲突车道对。
- 仅做 yaw 变换会在坡度/侧倾处产生较大放置误差，需根据实际控制点做3D刚体配准，并拒绝超阈值残差。

## 新录制中的在线对齐与输入实验

`runs/live-steer-02` 使用40ms有界延迟对齐，3,591个视频索引均有匹配的
`/video_alignment` bracketed证据，capture帧号和QPC逐条对应。对齐误差P95
9.0664ms、最大11.0402ms。该60秒片段满足本次对齐误差检查，但有9个捕获帧丢失，
且不是一小时驾驶录制，不能据此宣布P0完成。证据：
`artifacts/live-steer02-metrics-complete.json`。

三次短时转向试验均未完成预期转向。第一次遥测超过200ms而停止。后续检查发现
调用方先采样QPC、后读共享快照可能把并发发布的正常快照误判为未来数据；已改为
读快照后采样时钟，仍拒绝真正的未来/过期数据。第三次使用该修复仍遇到输入回调停顿。

旧DLL的`Applied.frame_index`在`live-steer-03`的21.8677ms窗口内跳过19225批，
对应879151批/秒；同一命令附近出现162.56ms的物理回调墙钟空隙和游戏UI队列溢出。
`live-steer-02`同样有16.3217ms内4970批。原实现依赖first标志决定是否重填输入队列；
修复后明确以一次prepare至返回not_found为有限批次，最多51个输入事件，重复first
标志不得清空/重填队列。重复first与activation标志的mock回归已通过，实际游戏效果
仍需加载新DLL确认。新增`/input_diagnostics`记录实际flags、批次与回调计数。
原始录制及`artifacts/live-steer02/03-callback-analysis.json`均保留，E2/E3保持未通过。

上述51项是当时修复版本的数量；追加quickload后批次为52个输入，随后第53次调用返回not_found。旧索引保持不变，r5已有52项批次和转向/制动/相机完成边实录；这不等于全部按钮效果或quickload已现场验收。

## 失联制动与Gate关闭的原文矛盾

r4进入驾驶后，`runs/live-r4-steer-01`首次得到有效轴响应，同时暴露了变速箱模式问题。零油门、0.5制动的3秒命令在“简易自动挡”中触发倒挡，有效油门接近0.5，有效刹车为0；峰值倒车7.35km/h。SDK正转向0.3同时产生负effective转向，不能直接把语义轴当成车体左正轮转角。原始失败、QPC关联和完整轨迹保留在分析报告（`artifacts/live-r4-steer-01-findings.md`）。

官方头文件`scssdk_telemetry_common_configs.h`提供`controls`配置与`shifter.type`字符串（arcade/automatic/manual/hshifter）。兼容旁路消息保留真实事件时间、会话和代际；DLL最终输出、Supervisor及Host都拒绝arcade/unknown，任何模式变化撤销旧启用状态。UI已切换真实自动挡，保存后的本机磁盘值是`g_trans=3`，不把该本机数字当跨版本接口。r5静止制动复验结果见下文，动态完整验证仍缺；旧r4包不包含此修复。

`IMPLEMENTATION` §3.5规定 Gate0 优先全零、命令超过200ms后斜坡轻刹；§3.11却同时要求
监督进程在200ms置Gate0并认为DLL会继续斜坡。两者组合会取消斜坡。原始设计文档没有改写。
实现改用保留原布局的显式 Off/Active/BrakeOnly 模式：失联后≤300ms撤销主栈转向、油门、
离合与全部按钮权限，独立短刹最迟原命令年龄1秒结束；主动off、人工接管与暂停立即全零。
失联锁存不会因恢复的新命令自动解锁。DLL也独立保留原命令期限，防止监督进程末段崩溃
延长刹车。未知模式及不支持mode2的旧DLL全零，不能将兼容降级写成缓刹验收通过。

三个隔离子进程联合kill回归通过，记录于 `artifacts/command-loss-joint-ipc.txt`；
SDK有限批次与按钮松开的mock也通过。它们是synthetic IPC/SDK回归，尚非实机失联验收。
详细时间边界与协议解释见 `BRIDGE.md`。

## r5 静止输入、视角与暂停的证据范围

r5固定包的真实自动挡（SDK `automatic`、代际2）已完成空挡、发动机关闭的零油门制动与正负转向试验。有效制动0.5时有效油门0，挡位始终0；60秒记录最大水平漂移约0.048mm，240秒记录约0.153mm。该结果证明本次静止制动没有变为倒车，不覆盖行驶制动或倒车AEB。制动对照报告（`artifacts/live-r5-steer-brake-01-findings.md`）。

此前正负0.1转向均呈积分响应，变化率约∓0.06/s，释放后仍保留转角。与当时`c_relatsteer=1`以及[SCS对相对/绝对转向的说明](https://forum.scssoft.com/viewtopic.php?t=204507)一致。用户明确授权后，游戏正常退出，按既有候选逐字节验证并仅改1个字符为0，随后重启。应用记录位于`artifacts/absolute-steering-applied-r7.json`，原文件仍保留。

`runs/live-r7-absolute-steering-01`的90.5489秒、5434遥测帧完整，5434帧均空挡、RPM0、有效油门0，最大水平位移约0.078mm。±0.1、±0.25各保持3秒，稳态effective分别为∓0.1、∓0.25，静态分析确认单调、负极性；左右前轮均值绝对角约0.06984rad和0.17495rad。四次释放到effective及两前轮归零分别约67.57、57.39、131.89、131.16ms，均保留实际采样精度。282组同序号write→Applied观测P95为3.0629ms，但这不是动态横摆响应或首次输入回调的保证。静态辨识未采到独立的主动零轴稳态点，neutral字段保持null；记录末尾实际轮角和输入均为零。没有生成驾驶标定证书。

240秒实录有14449个遥测帧、14383个视频帧，内部遥测无序号缺口，视频均有同帧相机，对齐P95 8.5769ms/max12.8786ms。聚合`dropped_frames=1`与24个capture drop分别保留。因为旧计数未记录来源，不能确定该1来自哪个流，也不能据内部连续性改为0。最后菜单暂停43.2269秒内，2608个相邻步的冻结仿真时钟增量全部为0。完整相机/时序报告及图册（`artifacts/live-r5-camera-steer-01-findings.md`）。

## r6/r7 可视化证据的适用范围

E4新增的三个真实例子是同一cam8车外视点下的蓝车、油罐组合和白车，游戏补丁为1.61.1.9，插件/录制/原图SHA均保留。Root与agent均看过原图及8候选对照；C1对应实际道路车辆，C3落到建筑，其他候选不符。r7测试以人工区域及12px不确定性为期望，另明示几何盒与可见mesh差异容差，不复制算法输出像素。具名`Convention::ETS2LA_1_61_REVIEWED_EXTERIOR`不改变默认选择；`fully_calibrated`/`phase_acceptance`保持false，不能推广到车内、其它视角/车型/补丁。

r6的`/recording_loss`只对已确认覆盖的环槽写流式事件，记录观察QPC而不推测丢失样本时间。`first_drain`是消费阶段，不等于已证明发生在用户开始前。旧r5聚合1没有来源信息，保持未定位。新实录已能单独记录Extension覆盖，不能与旧metadata口径或capture编号缺失混为一个计数。

实际Sky车外02图已确认`UNCAL C1`和可见白车青框，重载组合部分出屏也有框；白车元数据（`artifacts/r6-overlay-exterior-presented-car.json`）和重载元数据（`artifacts/r6-overlay-exterior-presented-heavy.json`）均为`exact_capture_qpc=null`，不称同Present或精确groundtruth。90秒生产者约60Hz只证明发布链路；捕获/视频计数、GPU消费、可见性正确性须分别判断。

优化版Rerun完成30秒只读观察，1130快照/165图像（164 Aligned、1 MissingPast），官方viewer验证并实际显示视频、3D地图与时间轴。该观察率不是游戏全量采样率，也不是与debug构建保持相同负载的性能对照。原始录制视角阶段、实际零控制证据、丢失与对齐数据集中见报告（`artifacts/r6-exterior-and-release-rerun-findings.md`），不推导为动态驾驶验收。

## r3 源码冻结时的证据范围

r3 整合日志（`artifacts/integration-check-r3.txt`）与格式检查（`artifacts/format-r3.txt`）通过。当前 recorder 为 8 项回归；Overlay 为 8 项库回归，隔离子进程辅助测试不重复计数。独立原生 GPU 日志实际为 2 项测试，其中 Present 测试内部循环 5 次；这不是欧卡驾驶中的 GPU/Hook 验收。Linux 证据仅为目标交叉检查，没有 Linux 原生执行结论。

MCAP 同行审查复现过超长分块头、错误 descriptor/外层时间及孤立跟车诊断问题，修复后均被拒绝。累计 schema/channel 元数据限制为 1 MiB，schema/channel 数量分别限制为 16/256，重复 summary 不重复扣预算。实际 Runtime Replay 的跟车诊断经 JSON/MCAP 往返仍归入 synthetic，即使测试运行元数据刻意标记 game 也不升级为实机样本。隔离复验见 完整日志（`artifacts/recorder-review-final-verification.txt`），其中明确标注的合成测试文件不是游戏证据。

r2/r3 打包日志均记录 264 项依赖声明、0 项缺失；旧检查点的 265 项是历史依赖集数量。r3 的中文/空格路径启动、系统 PATH、GUI 子进程生命周期已有独立检查，详细证据见验收记录。quickload 仅完成追加索引、旧索引冻结及 mock 回归，尚未实车执行。所有 P0–P4 实车阶段均保持未通过，源码测试和开发实验入口不能替代现场退出条件。

