# SCS 桥接核实与单位修正

本页依据仓库内未经修改的官方 SDK 1.15 头文件。详细共享协议、生命周期和测试范围见
[BRIDGE_PROTOCOL.md](BRIDGE_PROTOCOL.md)。SDK 下载来源与归档哈希见
[PROVENANCE.md](../vendor/scs_sdk/PROVENANCE.md)。

`ETSADS_Info` 独立元数据块保留游戏提供的 game_id、game_name、两套 SDK 各自的
game_version、协商后的 API 版本、游戏进程 ID、schema 及 session_start_qpc。
完整 SDK 重载后 session_start_qpc 变化；API 关闭后对应 API 版本写为 0。
GameAdapter 会依据该 session 值清理旧帧/时钟/事件缓存，并拒绝前一会话的 QPC 数据。

| 核实项 | 官方证据 | 实现含义 |
|---|---|---|
| `simulation_time` 在暂停时仍增加 | `vendor/scs_sdk/include/scssdk_telemetry_event.h:123`–130 | 原始 wire 保留该值用于 E1、E9；不能用来判断驾驶时间是否冻结 |
| `paused_simulation_time` 在暂停时冻结 | `vendor/scs_sdk/include/scssdk_telemetry_event.h:132`–136 | `GameAdapter` 的高层 `EgoState.header.sim_time_us` 使用此时钟 |
| 局部角速度单位为 rotations/s | `vendor/scs_sdk/include/common/scssdk_telemetry_truck_common_channels.h:49`–54 | 高层 yaw_rate 将 SCS 本地 Y 分量乘 `TAU`，得到 rad/s |
| 车轮转向单位为 rotations | `vendor/scs_sdk/include/common/scssdk_telemetry_truck_common_channels.h:712`–722 | 高层 wheel_angle 乘 `TAU`，得到 rad；正值向左 |
| 遥测 paused=3、started=4 | `vendor/scs_sdk/include/scssdk_telemetry_event.h:49`、54 | 回调使用生成绑定的具名常量，避免手写数字歧义 |
| 输入描述符在注册调用期间完成处理 | `vendor/scs_sdk/include/scssdk_input_device.h:179` | register_device 返回后允许释放临时 CString 和描述符数组 |
| `game_version` 不是游戏补丁版本 | `vendor/scs_sdk/include/scssdk.h:182`–187 | 不能拿 SDK `game_version` 当 Steam 1.xx 兼容性检查依据 |

SDK 的包版本 1.15 与遥测 API 1.01、输入 API 1.00 是三个不同版本号。
上述核实结果来自官方静态协议；E1 的实际物理节拍、E2 输入合成、E3 转向响应、
E4 投影约定、E6 视角、E9 回调延迟仍需要当前本机游戏的实验记录。

已执行真实 Windows 双进程共享内存测试：发布者写 10,000 个带校验字段的帧，
子进程检查快照完整性并回写最终帧 ACK。模拟 SDK 生命周期测试完成 10 次初始化、
输入输出和关闭循环。它们均不能替代游戏内 SDK 重载测试。

## 静止实机证据

源数据为 `runs/live-stationary-01`，车辆在特罗姆瑟维修站、空挡、速度为零。该次录制
包含 3,602 个真实遥测帧，没有遥测缺帧；模拟时钟与暂停时钟的相邻差分均为
16,666 µs，支持“本段约 60 Hz 固定模拟步”的结论。它不能证明高速、载重或控制跟踪验收。

墙钟回调并不均匀：按遥测 frame_start QPC 计算，相邻差分中位数 16.871 ms、
P95 32.9941 ms、最大 37.6139 ms。控制和录制不得把固定模拟步误当成墙钟每隔
16.666 ms 一次的计时器。

同段 2,235 个视频索引的原在线对齐误差 P95 为 25.0997 ms，最大 26.7617 ms，
未达到 60 Hz 下一渲染帧的目标。只读离线使用完整的前后遥测样本做最近邻匹配后，
P95 为 9.2283 ms，最大 13.5649 ms；其中 1,937 帧选择后一个物理样本更准确。
等待未来样本的 P95 为 9.9138 ms、最大 21.9853 ms，尾部有 1 帧缺少后续样本。

计算代码：`apps/etsads/examples/alignment_probe.rs`。
结果：`artifacts/alignment-stationary-01-offline.json`。原始 MCAP 和视频未被改写；
离线重匹配不能代替“修复后新录制在线通过”的证据。

录制器现将视频索引放入最多 256 项的有界队列，等待不超过 40 ms，使用**已经写入 MCAP**
的前后物理帧做最近邻。启用视频时记录最多 32 个物理帧的起始预录数据，结束时最多多收
40 ms 遥测补齐尾部。`VideoIndex` 仍是原来的 64 字节布局；新增 `/video_alignment`
记录 bracketed、缺过去样本、超时、尾部缺未来、无遥测等状态，不能把未对齐状态算作通过。

实机还发现 capture 映射首次连接会保留先前菜单画面。读者现在拒绝超过 500 ms 或来自未来
的 Present QPC，等待新图像；对应真实映射的过期/未来帧回归测试已通过。

## 输入批次有界性与时序证据

旧 DLL 的停车零油门试验 `runs/live-steer-02` 和 `runs/live-steer-03` 未成功施加转向。
只读 MCAP 分析发现，输入批次计数分别在 16.3217 ms 内增加 4,970、在 21.8677 ms
内增加 19,225；后者相当于约 879,151 次/秒。遥测 input/effective 和已记录的 applied
转向仍为零。同期游戏日志出现密集 UI queue overflow。这些证据支持排查输入事件批次
反复填充，但不能单凭日志断定 SDK flags 的实际值。

新版桥接在一个批次返回 `not_found` 前绝不重新准备事件，即使 `first_in_frame` 重复出现。
每批事件上界等于注册输入总数，下一次调用必定结束；最初修复版本为51/52，后续追加
quickload后为52/53，追加engine后为53/54，代码从名称数组计算上界。重复 flags 的模拟 SDK 回归已通过。
`ETSADS_InputDiagnostics` 独立块记录实际 flags、回调/批次数、重复 first 次数、每批事件数
和起止 QPC，录制为 `/input_diagnostics`。分析代码为
`apps/etsads/examples/input_timing_probe.rs`，结果保存在
`artifacts/live-steer02-callback-analysis.json` 和 `artifacts/live-steer03-callback-analysis.json`。

新增 `ETSADS_FrameTiming` 256 槽独立环，不改变已部署遥测帧布局。`/frame_timing` 原样记录
`frame_start`、`frame_end`、输入批次 prepare 的 QPC 和严格递增事件序号、物理帧号、
输入批次号、命令序号及 source_frame_id。输入事件的物理帧号和模拟时钟明确引用最近已完成
物理帧；不能据此假定 SDK 的输入/渲染先后顺序。frame_start 时间戳在扩展拷贝之前采样。
带 mandatory runtime trace 的录制若该环溢出立即失败；普通录制明确累计丢失条数。
上述模拟回归和构建通过不替代新版 DLL 的实机 E2/E9 验收。

新录制完成后可运行只读分析：

```powershell
cargo run -p etsads --example frame_timing_probe -- runs/<run>
```

输出包括事件顺序、缺失序号、start→end 和已完成 end→input prepare 的延迟分布、
每批事件上界及重复 flags 计数。命令块是 latest 采样，输出将命令写入到 prepare 的结果
明确标为 sampled；它不替代 effective/wheel 响应，也不会对缺少该 channel 的旧录制给出通过。

## 本机输入合成的静态核实

官方 `scssdk_input_device.h:29–42` 说明语义设备映射到同名 mix，并提醒只有部分 mix 支持。
`scssdk_input_event.h:39–60` 规定 first 标志、激活标志和反复取事件直到 `not_found`；
官方 `examples/input/input.cpp:154–182` 与 `examples/input_semantical/input_semantical.cpp:31–40`
展示同样的有限取事件机制。桥接在激活后重发全部状态，之后只发送变化；释放时发送零值。

只读核查本机当前 Steam 存档 `controls.sii`，第 339 行的 steering mix 是
`dsteering - joystick_term - semantical.steering`；第 344、345、349 行的模拟油门、刹车、
离合均为原输入加相应语义轴。第 230 行 cam2、以及喇叭、车灯、转向灯等 mix 使用 OR。
因此该配置下正语义转向预期产生负 telemetry input，不能因为符号相反就判定输入失效。
此为配置表达式推论，需本机 applied/input/effective/wheel 实测闭环确认。
零语义轴不会覆盖键盘或手柄输入，语义设备也不提供独占控制权；人工操作仍按游戏 mix 合成。
标定必须使用 `semantic_applied` 来源并学习正负极性，不把玩家 input 轴直接冒充命令轴。

## 主栈意外失联与主动释放的区别

原 `IMPLEMENTATION.md` §3.5（第167–168行）规定 Gate 关闭优先全零，又要求命令超过
200 ms 后在0.5秒内爬到0.3刹车、超过1秒释放。§3.11（第235行）同时要求监督进程在
200 ms关闭 enabled，并认为 DLL 会继续斜坡；这两条组合时斜坡实际上被优先级更高的
Gate0覆盖。原始 DESIGN/IMPLEMENTATION 保持原文，此处记录实现解释与差异。

新源码保留32字节 Gate 布局，将 enabled 明确为模式：Off=0、Active=1、BrakeOnly=2。
未知模式必须全零。主动 gate off、人工接管、暂停、非法遥测优先立即全零。
只有此前获得有效许可的同一命令意外失联，独立监督进程才锁存原始 write_qpc，撤销主栈
转向、油门、离合及全部按钮权限，并以 BrakeOnly 做有限制动退让：命令年龄200–700 ms
从0爬到0.3，年龄达到1秒时归零。恢复的新命令不能刷新计时或自动重新启用；需新的显式 arm。
物理帧过期先于墙钟阈值时可先撤权，但未到200 ms不施加斜坡刹车。

DLL 的 BrakeOnly 完全忽略当前主栈命令，仅接收监督进程有限刹车，并依据自己缓存的最后
ACTIVE 命令时间强制1秒上界。这样监督进程在斜坡末尾崩溃也不能借300 ms心跳宽限延长制动。
没有先前 ACTIVE 历史的 SDK 直接全零；Gate 心跳超过300 ms、暂停或未知模式也全零。
`AppliedControl.reason=5` 标明 BrakeOnly。初始化日志标明支持模式0/1/2；旧DLL遇到2会
全零，属于兼容性安全降级，不能声称旧DLL已经实现缓刹。

联合回归使用三个真实子进程：独立 supervisor、被直接 kill 的命令发布者、运行生产
InputPolicy 的 SDK 消费者；全部使用隔离命名映射和 synthetic 数据。该次测得212.954 ms
撤销主栈权限、峰值独立制动0.3、1秒原命令期限后全零，注入恢复命令仍保持失能。
另一变体在0.85秒直接kill监督进程，209.453 ms撤权，DLL仍在原始期限后全零；
主动停止与暂停的全零期限也通过。日志 `artifacts/command-loss-joint-ipc.txt`。
另有真实 FFI mock 验证 BrakeOnly 批次全部按钮归零且重复first标志仍≤51事件。
这些属于跨进程与SDK模拟回归，已部署的实机DLL未改变，实际游戏失联制动验收尚未执行。

## 相机记录尾帧补齐

只读复核 `live-steer-02` 的唯一camera缺失实际发生在最后一个视频帧3590：它对齐物理帧
73269，而录制中最后的extension只到73268。首段32帧telemetry预录虽然没有全部对应扩展，
却不是这次缺失的原因。证据为 `artifacts/live-steer02-camera-alignment.json`，旧run未修改。

源码收尾现在在原40ms硬期限内同时排空真实4槽extension历史，只记录已经写入MCAP、且
frame_id/simulation_time/QPC都匹配的物理帧，按物理帧去重。若尾部n帧的camera直到n+1
才发布，会继续等待有界时间；超时或无camera时不插值，`/video_camera_summary`明确计数
同帧camera存在/缺失、扩展环溢出及没有匹配telemetry的扩展。没有扩展插件的数据仍可作为
telemetry/video记录，但不能把缺失相机记为投影验收通过。延后发布、去重和真实miss的隔离
映射回归已通过，新的实机片段仍待验证。

## 重复起点试验的quickload入口

本机controls.sii第432行明确存在 `semantical.quickload`。它追加到pulse索引44、SDK输入
索引51，原44个pulse的位置和304字节ControlCmd保持不变。SDK mock核对实际注册名称、
布尔类型、真实回调返回值和完成批次动作证据；移动时请求被丢弃，不积压到之后停车时触发。

`etsads-cli pulse quickload --arm` 必须显式arm，并在预热、等待许可和发出前确认测得速度
≤0.05m/s；DLL也重复检查静止条件。成功仅表示完整SDK批次已返回quickload边沿，随后立即
归零并请求gate off，载入完成须另等新鲜遥测。代码不选择文件、不写存档，不自动循环加载。
此能力的代码回归通过，尚未在游戏中执行quickload验收。

## P0 原始字段补充与兼容侧路

为补齐 IMPLEMENTATION §3.3，新增三个独立64槽共享环；原1024字节TelemetryFrame、
544字节ConfigBlock以及已有MCAP channel均不变。结构定义和大小编译断言位于
`crates/msgs/src/protocol.rs`，旧录制没有这些channel时明确返回None，不补造数据。

| 映射 / MCAP topic | 单帧大小 | 内容与时序 |
|---|---:|---|
| `ETSADS_Dynamics` / `/dynamics` | 120 B | 原始局部角加速度、cabin动态位姿偏移、左右转向灯实际亮灭、前/顶辅助灯强度及各字段valid位；header逐字段等于同帧Telemetry |
| `ETSADS_ConfigFull` / `/config_full` | 5032 B | 旧truck/job配置完整副本、truck字段/轮属性存在位、10节trailer配置；header.qpc取SDK配置回调时刻，effective_from_frame_id为最近已完成物理帧+1 |
| `ETSADS_CameraView` / `/camera_view` | 192 B | cabin相对位置、两种相机旋转约定候选、视角类别、切换位和数据质量标志；header等于被观察的物理帧，不使用发布时刻替代 |

`scssdk_telemetry_truck_common_channels.h:63–70`规定角加速度为rotations/s²，原始侧路
保留该单位；需要rad/s²的调用方再乘TAU。该文件第13–17行允许单精度/双精度转换，
桥接请求dvector。第73–79行的`truck.cabin.offset`是车辆局部空间中的驾驶室动态偏移，
请求dplacement。第475–488行的`truck.light.lblinker/rblinker`是实际灯泡状态，与逻辑
转向开关`truck.lblinker/rblinker`分别保留；第511–531行的辅助灯是uint强度（0/1/2）。
这些新增通道每帧订阅并启用no_value回调；缺值、错误类型或非有限数清除对应valid位，
不能把上帧的有效状态当作本帧值。

`scssdk_telemetry_common_configs.h:94–120`说明`trailer`是第一节兼容别名，
`trailer.0`至`trailer.9`分别上报，未存在的节仍有空属性配置。实现将空属性清除为
不存在，并保留reported/present两个mask、各节更新QPC和config_seq。每节记录原始
hook位置、wheel位置、轮数、ID；radius/steerable仅在SDK实际提供且类型正确时填入。
各属性存在位区分零值和缺值；单节最多存16轮，若SDK轮数更多，仍保留原始轮数并显式
设置truncated标志，不伪造额外车轮。truck/job已有字段继续保留；u32的游戏分钟、
delivery.time、轮数与挡位数量直接读取整型，避免经f32转换损失大整数精度。

配置依据`(effective_from_frame_id, event_qpc)`关联，不能将后来发生的配置关联给环里更早的
遥测。effective_from是最早允许关联帧的下界，不是实测的首次影响帧；还必须满足事件QPC
不晚于样本frame_start QPC。因此发生在物理步中间的事件可能保守地晚一帧关联，E9实测
前不推断其游戏内部首次作用时刻。Live与Replay均优先用完整配置历史；存在侧路但缺少
合格的早期历史时返回None，只有整个侧路缺失的旧插件/旧录制才用legacy观测值。
录制保留尚在环内的配置事件历史；动态/相机侧路只写入已记录且frame/sim/QPC匹配
的物理帧，尾部复用40ms有界排空。强制驾驶录制遇到侧路环溢出会失败；普通录制累计丢失。
游戏事件也保留实际SDK回调QPC，而不是复用frame_start时间。

## Cabin相对视角输出的边界

相对位姿以官方SDK truck世界位姿、`cabin.position`和本帧`truck.cabin.offset`计算。
`scssdk_telemetry_common_configs.h:389–397`明确cabin.position是旋转关节位置，且没有
独立驾驶室的车型可以不提供；不是驾驶室包围盒。缺cabin配置时输出Unknown；缺动态偏移
时标明STATIC_CABIN。相机世界位置仅加相机自己的sector，不给truck位置再加sector。

上游源码[core.cpp:103–115](https://github.com/ETS2LA/plugin/blob/3b01d90b5be2469c0d94fcfdee4b354184ddeb02/src/core.cpp#L103)
把插值placement经get_center_coords转换后写入truck_position；
[traffic.cpp:27–39](https://github.com/ETS2LA/plugin/blob/3b01d90b5be2469c0d94fcfdee4b354184ddeb02/src/processing/traffic.cpp#L27)
与[common.cpp:7–9](https://github.com/ETS2LA/plugin/blob/3b01d90b5be2469c0d94fcfdee4b354184ddeb02/src/prism/common.cpp#L7)
显示该点是已全局化的bbox中心。它不能冒充SDK truck参考点。所审源码commit为
`3b01d90b5be2469c0d94fcfdee4b354184ddeb02`；已安装DLL的资源commit没有给出源码构建映射，
因此本次源码核读不构成该DLL与源码等价的证明。

输出始终带PHYSICS_REFERENCE：它将扩展相机与对应物理快照关联，没有把物理参考点冒充
已标定的渲染插值参考点。r4/r5的Interior/Chase/Other使用启发式范围，带HEURISTIC_BOUNDS；
r6将同一近驾驶室范围输出为NearCabinUnclassified（新码4），不再单凭距离产生Interior。
这些类别不等于已识别cam1–8。相机四元数是否需共轭仍无实测定论，故记录两套相对旋转，
selected_hypothesis默认UINT32_MAX。连续有效帧的位置跳变超过0.5m或对应候选角跳变超过
10度时设置switched_mask；丢帧、配置变更、会话变更和计时器重置清除比较基线，防止将
重载算作相机切换。接口允许分析者明确选择候选，默认桥接不偷偷选择约定。

官方SDK不能提供实际cam1–8编号、驾驶室/车身实测包围盒、扩展相机投影矩阵或渲染时刻
truck原点的实测转换。trailer配置也不保证提供radius/steerable。这些缺项用mask/Unknown
表达，不从轮距推导车身外形、不把缺失值猜成零。E4投影、E6切换与cam1–8实机记录仍待执行。

本轮回归包括真实官方SDK结构的同步注册/配置回调、trailer.9及空配置、角加速度双精度
单位/缺值、cabin旋转平移/大世界坐标/切换连续性、隔离共享环与MCAP时序关联。
`artifacts/p0-sidecars-tests.txt`和`artifacts/p0-sidecars-clippy.txt`保存源码验证结果。
只读回放旧`runs/live-steer-02`得到3634个快照和3591个视频索引，三个新侧路均为None，
缺33个extension保持明确；结果为`artifacts/p0-sidecars-legacy-replay.json`。旧run未改写，
这轮未部署DLL或执行游戏操作，以上不构成新增实机验收。

## Live泊车慢规划与执行许可

Hybrid A*搜索可以超过普通控制轨迹的200ms有效期。只有已测停稳（速度绝对值不超过
0.05m/s）的Parking模式可以进入`WaitingForParkingPlan`，最长30秒墙钟；道路和移动中
轨迹仍严格执行200ms。等待期间不产生初次arm请求；此前已获许可的运行只能保持当前
转向与至少0.4制动，零油门、零按钮，独立supervisor、输入回调和强制录制监测照常运行。
暂停、会话/配置改变、位置或关节漂移、超时会取消请求并退出控制。

旧搜索结果只作为几何候选。Runtime提交一个带当前真实快照的新请求，worker重新检查
实际剩余路径、物理边界、当前障碍及未来2秒占据，并生成局部扫掠索引。采用时对变化对象
查询4m网格，只检查命中的局部连续扫掠，最多8192工作单位和10ms；预算耗尽不能取得权限。
无害移动不再因整体AABB内任意坐标变化而无限等待，侵入路径则必须重新提交请求，仍共享
原30秒期限。当前姿态至首个剩余点的连接保留检查；200ms时限在局部检查后再次核验。
剩余路径使用实际位移推进的单调游标，不能以全局最近点跳到自交路径的后一次经过。
最终控制还以本帧测得挡位核对轨迹方向，不使用请求时的旧挡位确认。

搜索取消由每generation独立AtomicBool传到Hybrid A*及其辅助搜索。Runtime退出不等待
同步join长搜索；晚到的结果不能写入已退出的控制器。CLI写命令、记录动作后和申请arm时
再次检查停止标志，RAII最终归零并申请disarm。GUI进度携带产生时刻，队列积压不能刷新
旧控制状态的有效期。

`artifacts/parking-lifecycle-final-tests.txt`记录17项泊车相关源码测试，慢worker用260ms
注入延迟，与实际游戏证据区分；包括新障碍、无害移动、档位改变、游标、预算、暂停、会话、
30秒超时与有界Drop反例。`artifacts/parking-wait-ipc-final-tests.txt`记录6项隔离命名映射
测试，`artifacts/parking-lifecycle-final-clippy.txt`为Runtime/CLI全目标严格Clippy结果。
这些检查未向游戏发送控制，不能替代P4实机泊车验收。

## SDK换挡模式与控制语义联锁

真实r4实验发现简易自动挡会把`abackward`解释为倒车加速：即使发送零`aforward`，持续
制动仍可能使effective throttle非零。旧r4包与原始录制保留，不能以油门轴为零宣称静止。
官方`scssdk_telemetry_common_configs.h:36–41`定义`controls`配置组，
第497–508行定义字符串属性`shifter.type`及`arcade/automatic/manual/hshifter`。
游戏本地文件中的数字配置不能替代运行SDK状态，也不推测数字与这些枚举的映射。

新增`ETSADS_ControlConfiguration`（64槽、72字节POD）及MCAP `/control_configuration`，
不改旧Telemetry/ConfigFull/ControlCmd布局。字段包括实际SDK事件header、session_start_qpc、
generation、effective_from_frame_id下界、shifter_type、valid。枚举0未知、1简易自动、2真实
自动、3手动、4H挡；只有精确单一字符串属性才valid=1，空/错误类型/重复/未知属性值发布
新的未知代际，绝不保留旧安全模式。初始化先发布generation0未知。`safe_shifter()`只判断
枚举语义，调用者仍须检查会话、代际、事件QPC与物理帧下界。

DLL最后一层在未知/简易自动挡下释放所有轴、保持键和脉冲，包括AEB及BrakeOnly，
Applied.reason=6。每次controls事件清空待发批次与控制器历史、消费旧脉冲计数，并要求
先看到新鲜Gate Off，再接受Active；只把模式改回安全值不会恢复旧授权。遥测关闭同样
发布未知状态。暂停会立即清掉尚未完成批次的旧事件，恢复后同样需要Off→新Active，
不会继续吐出暂停前的按钮/轴。独立supervisor和Host另按最新代际撤权，不能以旧帧对应的安全模式继续。
历史GameAdapter/Replay按事件QPC及effective下界关联，未知记录替换旧模式；旧MCAP没有
侧路时返回None，不推断当时换挡设置。录制保存实际配置事件，静态配置不按200ms过期。

模拟官方SDK配置和输入回调覆盖四种模式、错误/缺失/重复属性、批次中途切换、旧脉冲
清除、安全模式恢复前仍需Off→Active，以及Active/BrakeOnly双路径拒绝；隔离ring→MCAP→
Replay覆盖未来事件、跨会话和未知代际。结果见`artifacts/control-semantics-transport-tests.txt`
及`artifacts/control-semantics-recorder-tests.txt`。修复后的真实SDK模式与实际轴语义仍须
退出游戏、部署新DLL并重新采集，不用源码测试替代现场确认。

## r8点火语义入口

`engine`追加到pulse索引45、SDK输入索引52，不改变既有按钮编号或304字节ControlCmd布局。
实际测试存档的controls.sii包含`semantical.engine`映射。`etsads-cli pulse engine --arm`
切换发动机状态，Host与DLL都要求测得停稳（绝对速度≤0.05m/s），移动时丢弃按钮计数，
不会在随后停车时补发。Host必须收到本次计数对应的完整SDK动作边沿才返回成功；旧DLL
没有此按钮时会报告未确认。SDK已接收并不表示发动机启动成功，仍须观察实际RPM。

完成菜单操作时，Host可在得到本次新鲜动作边沿后重新读取暂停遥测，解决跨映射快照
先后不同造成的误报；只看到Gate Off不能判成功，也不会继续发送控制。针对菜单和
点火的隔离回归分别覆盖正确边沿及错误按钮边沿，日志见`artifacts/r8-engine-ack-tests-fixed.txt`。

## r6近驾驶室视角未分类

`runs/live-r5-camera-steer-01`的完成SDK边沿与精确视频帧证实：cam5侧窗、cam6低位前向、
cam7近轮视角会进入原来的宽松位置范围，因而被旧规则标为Interior。保留该录制的原始
码1及其HEURISTIC标志，不把旧数据重写成新算法的结果。原始位置、两套方向候选、截图
和SHA256见`artifacts/live-r5-camera-steer-01-near-cabin-comparison.json`及同前缀summary。

r6追加`VIEW_NEAR_CABIN_UNCLASSIFIED=4`；0 Unknown、1旧Interior、2 Chase、3 Other
保持原值与历史含义，CameraViewFrame仍为192字节。相同近驾驶室空间范围的新观察一律
输出4，包含真正cam1及附近车外视角，明确表达“有效位姿、未确定视角”，不是无效数据。
远端Chase/Other仍是原来的启发式诊断。缺失/非法相机与未来配置仍返回Unknown，不放宽
验证条件。位置、两方向候选、时序连续性与switched计算不变，选择四元数候选也不等于
识别C1。本次没有加入车型坐标表、视角指纹或未经测量的驾驶室包围盒。

回归输入取自这段真实录制cam1/5/6/7及返回cam1的相对位姿，断言新结果均为4且位置/
方向有效性和切换特征保留；另验证远端分类、非法四元数拒绝、旧原始码1及192字节布局。
`artifacts/r6-camera-classification-tests.txt`记录6项相机测试，
`artifacts/r6-camera-classification-clippy.txt`记录相关全目标严格检查。它们证明保守标签
修复，不宣称完成精确C1识别或所有车型/头部运动的实车分类验收。
