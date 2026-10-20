# 验收记录

此文件只记录实际已有的证据。`通过` 表示对应检查完成，不推导为整个阶段完成。阶段退出条件必须使用与文档一致的真实游戏试验。

> 文中括号里的 `artifacts/`、`runs/` 等路径是开发机上的本地实测记录，体积大且含存档信息，未随仓库发布。

**停止交接：用户要求记录进度后停止任务。最新完整状态见 [开发与验证进度交接记录](DEVELOPMENT_STATUS.md)。下文保留各历史检查点，不把早期“当前/待复验”描述当作最新结果。** 最新MVP仍未通过20秒道路试验；最新应用包77 passed/2 ignored，最新全workspace检查是更早的340 passed/8 ignored，二者不可合并或当作同一源码的全量验收。

**当前 P0、P1、P2、P3、P4 均未通过实车退出验收。** r6 已有实机可视化和便携交付证据，r7 新增了真实投影夹具回归。经用户明确授权，当前测试存档的 `c_relatsteer` 已从1改为0；重启后的四段静止输入确认绝对响应。动态车辆标定、连续驾驶与困难泊车试验仍未完成。

## 阶段门槛

| 阶段 | 退出条件 | 当前状态 |
|---|---|---|
| P0 | E1–E9 全部结论；真实驾驶录制 1 小时并完整回放；车内/外交通框叠加；闭环跟圆稳态误差 <0.5 m | 未通过：已有静止绝对转向及实机可视化证据；动态标定、1小时驾驶录制、全视角标定与闭环跟圆仍缺 |
| P1 | 连续高速 100 km 无接管；中心 RMS <0.3 m；跟车时距 P95 ≥1.8 s；地图/辨识/MPC/自研 gfx 完成 | 未通过：地图几何/宽度验收和实测尚缺 |
| P2 | 跨城高速含匝道/收费站，全程无接管、零罚单 | 未通过：离线规划与状态机已实现，尚无实车证据 |
| P3 | 城市到城市，每百公里罚单 <1 | 未通过：路口控制与轨迹优化仍需完整现场验证 |
| P4 | ≥50 次困难泊车，成功率 ≥90% | 未通过：不得用简单直倒/合成目标替代 |

## 当前交付与现场证据

道路辅助MVP第一段：在真实50km/h道路上以30km/h上限运行，产生720个主动控制帧，约12秒；约10Hz诊断样本最高26.58km/h、最大绝对中心偏差0.369m，先后匹配两个相连的GPS路段，前后五项SDK损伤读数相同。原定20秒未完成：输入侧强制制动后出现短暂负速度，旧版倒车保护结束运行。该记录没有保存完整交通原始帧和当时的Gate AEB字段，因此当次制动对象和触发原因没有定论。见 [MVP说明](MVP.md)、第一段汇总（`artifacts/mvp-r2-validation.json`）。

MVP最新mvp-r7道路复验仍未通过：启用了计划结束停车，20秒计划实际610主动帧、约10.2秒；92个活跃诊断样本覆盖10.033秒、轨迹约15.847m，峰值15.125km/h、最大绝对横向偏差0.391m。主程序在lead gap为0时请求满制动，独立Gate AEB当时为0；随后制动转油门，约−0.2204m/s负速度触发退出，未到计划停车收尾。弯道交通直线外推误判是待验证假设，未记录完整候选轨迹；算法修复尚未实施。见 复验汇总（`artifacts/mvp-r7-road-summary.json`）、原始日志（`runs/assist/assist-39740-1119780541116.jsonl`）。两段均不计为零事故、跟车、贴限速或完整导航验收。

MVP mvp-r6静止停车握手通过：熄火、零油门条件下47个记录主动帧，最大绝对速度0.0003617m/s、漂移约0.054mm，实际确认释放驻车后重新驻车。成功日志运行时检查了SDK完成边，但没有序列化后来新增的详细ACK负载；尚未复做新版负载记录，也未验证从行驶速度减速停车。见 静止汇总（`artifacts/mvp-r6-static-finish-summary.json`）。

最新MVP代码检查：R6应用包77 passed、2 ignored；辅助模块17 passed包含在相关测试范围内，不另行累加；严格Clippy和UI构建通过。此前R5的全workspace检查为340 passed、8 ignored，R6后未重新执行全workspace。R5界面已实际检查，R6新增停车复选框尚未做实际视觉复核。详见 [检查范围与证据](DEVELOPMENT_STATUS.md)。

实体键盘接管已完成一次静止实测：用户在控制启用期间按住左方向键约1秒，监督记录`ManualTakeover`，原定60秒的命令提前结束，末尾Gate Off、Applied四轴全零。75.556秒记录共4535遥测帧，无缺帧，车辆保持熄火驻车。没有精确物理按键QPC，不据此宣称按键至撤权的毫秒延迟已验收。见`artifacts/live-r8-human-takeover-01-summary.json`。用户当前不方便完成1小时人工驾驶，该项明确保留待验。

r8实机补充：`engine`语义点火及熄火已观察到实际RPM变化；两段前进4挡测试最大位移分别0.176m、2.336m，峰值0.98km/h、5.54km/h。第二段0.5制动命令后约396ms降至绝对速度≤0.05m/s，随后驻车。暂停约29.29秒后恢复，记录剩余2.48秒内Gate始终Off、Applied四轴全零；这里没有把原请求自动重启。详见`artifacts/r8-engine-forward-pause-summary.json`。这只补充低速语义与暂停恢复证据，不代表速度相关动态标定、30km/h闭环或任何阶段退出通过。

r7 收尾源码检查：326 项通过、0 失败、8 项明确忽略，格式、严格 Clippy 与 UI 构建通过，见 `artifacts/integration-check-r7.txt` 和 `artifacts/r7-workspace-test-counts.json`。r7 便携包的生成、文件哈希与最小 PATH 检查结果统一记录在 `artifacts/r7-validation-summary.json`；该文件未生成或检查失败时，不将打包视为完成。

| 范围 | 已确认的结果 | 仍未证明 / 来源 |
|---|---|---|
| 转向与控制模式 | r7已应用授权的单字符修改并重启；真实自动挡下±0.1、±0.25均达到稳定轮角，释放后回零；90.55秒5434遥测帧无缺口，全程空挡、RPM0、零有效油门 | 只验证当前车型的静止响应，不是完整动态标定。见应用记录（`artifacts/absolute-steering-applied-r7.json`）、静态辨识（`artifacts/live-r7-absolute-steering-01-static-identification.json`）、稳态与释放（`artifacts/live-r7-absolute-steering-01-response.json`） |
| E4固定版本回归 | r5实录100帧×8约定；三张实际可见道路车辆支持C1。r7源码保存原始相机/bbox、图像SHA及近似人工区域，2项新回归和6项既有相机回归通过 | 仅ETS2 1.61.1.9及记录插件指纹下的cam8观测；不是全视角精确标定。具名常量不改变应用默认。见复核图与统计（`artifacts/r5-projection-audit/README.md`）、[持久夹具说明](../crates/game/testdata/README.md) |
| r6相机与丢失诊断 | 近驾驶室码4为未分类，不再自动称Interior；新增流式`/recording_loss`保留来源、阶段和环索引，旧聚合口径/旧录制不变 | 码3/4都不是相机编号或已标定许可；历史`dropped_frames=1`仍未知来源。见诊断说明（`artifacts/r6-recording-loss-diagnostics.md`） |
| 实机Overlay显示 | 车内与车外均实际显示`UNCAL C1`；车外02的Sky原图目视确认青框对应可见白车，以及部分出屏的重载组合 | Sky `exact_capture_qpc=null`，不能称与某一原始捕获同Present或精确像素真值。02的90秒生产者5399次发布约60.002Hz，不等于逐Present消费率。见白车原图（`artifacts/r6-overlay-exterior-presented-car.png`）、元数据（`artifacts/r6-overlay-exterior-presented-car.json`）、重载原图（`artifacts/r6-overlay-exterior-presented-heavy.png`） |
| 同期原始录制 | 车外02：7234遥测/7184视频，120.54674秒；遥测内部无缺帧，视频均有同帧camera，capture缺13帧，对齐P95 9.3849ms/max11.1982ms | 是静止、零控制记录；不能算高速距离或动态控制验收。车外01原始180秒的前112.15秒是近座舱，且末22.92秒Overlay没有同期raw覆盖，不按目录名混算视角。见分段报告（`artifacts/r6-exterior-and-release-rerun-findings.md`） |
| 优化版Rerun只读联调 | 30秒观察1130快照/165图像，164 Aligned、1 MissingPast；官方viewer verify通过，root实际检查视频、3D地图和时间轴 | 约37.67 snapshots/s与5.5 images/s是工具观察率，不是全量60Hz或受控性能对照。见统计（`artifacts/r7-rerun-release-observation-summary.json`）、viewer日志（`artifacts/live-r7-rerun-interior-release-01-view.txt`）、实际图（`crates/visualization/output/live-r7-interior-release-01-15s.png`） |
| 便携交付 | r6中文/空格路径与最小PATH smoke、623文件ZIP哈希验证通过 | r6包保留不改；r7投影夹具为后续源码，不冒充已在r6包中。交付检查不是实车阶段通过，见r6包记录（`artifacts/r6-validation-summary.json`） |

## 已执行的独立检查

| 检查 | 结果 | 证据 |
|---|---|---|
| 游戏安装与版本 | ETS2 1.61.1.9，Steam build 25904183 | EXE 版本信息、Steam manifest |
| Rust 运行程序与 egui 面板构建 | GNU x64 开发构建成功；MSVC 构建未验收 | `artifacts/app-build.txt` |
| 官方 SDK ABI、共享映射、心跳释放、跨帧关联 | 单元与真实 Windows 映射/双进程测试；测试数量随实现扩充 | `cargo test --workspace`，具体日志留在 artifacts |
| MCAP 一小时数据量基准 | 216000 帧，3599.983334 模拟秒；0 缺帧、0 逆序；写入+回放约 3.69 秒 | `artifacts/record-benchmark-1h`，source=synthetic |
| 合成弯道闭环 | RMS 0.0348066 m | `artifacts/simulation-initial.json`，不是游戏测量 |
| 合成 ACC stop-and-go | 0 碰撞，最小动态时距 2.73874 s | 同上 |
| 50 次简单直倒规划回归 | 50/50；只验证基础规划，不计 P4 | 同上，明确标记 straight reverse |
| ffmpeg NVENC | 本机 1080p/60 H.265 黑场编码成功 | `artifacts/nvenc-probe.mp4`，不是游戏视频 |
| 真实全图导出性能 | 约105秒；1101 扇区，约144.8万车道、6022万采样点 | 地图导出 manifest/validation；详见地图工具说明 |
| 存档备份 | Documents 配置与 Steam Cloud 本地 remote 存档已备份、SHA256 清单 | 游戏备份目录（`artifacts/game-backup-20261017-004230`） |
| 真实停车基线录制 | 60秒、3602物理帧、零遥测缺帧，模拟步16666µs | `runs/live-stationary-01`；不是人类驾驶一小时验收 |
| 修复后的实时视频对齐 | 3591帧逐条bracketed；误差P95 9.0664ms/max11.0402ms；9捕获帧丢失 | `runs/live-steer-02`、`artifacts/live-steer02-metrics-complete.json` |
| Linux目标可移植性 | r3 workspace 默认功能的 Linux 目标交叉检查通过；不是 Linux 原生构建、运行或测试验收 | r3 交叉检查（`artifacts/linux-cross-check-r3.txt`）；早期记录仍保留 |
| 自研DX11插件 | 2 项真实 GPU 测试通过：先捕获后叠加、Present Hook 启停；后者包含 5 轮启停，不是 5 项测试 | 原生 GPU 日志（`artifacts/native-gfx-final-gpu-tests.txt`）、[GFX说明](GFX.md)；仅独立隐藏窗口 GPU 测试，驾驶中兼容性待验收 |
| 原生面板录制浏览器 | 从目录列出真实/合成及不完整记录，后台逐帧分析；已在实际UI打开真实记录 | `apps/etsads/src/ui_runs.rs`；界面检查不替代驾驶验收 |
| r3 源码冻结检查 | workspace 测试、严格 Clippy、UI 构建及格式检查通过；显式忽略的现场/子进程辅助测试不计作默认通过 | 整合日志（`artifacts/integration-check-r3.txt`）、格式日志（`artifacts/format-r3.txt`）；早期 `workspace-tests-after-stop-fix.txt` 保留，均不代表实车阶段通过 |
| 发行依赖声明 | r3 包收集 264 项依赖声明，缺失声明文件数为 0；历史检查点曾为 265 项，数量随依赖集变化 | r3 打包日志（`artifacts/portable-r3-build.txt`）、`vendor/licenses/rust-supplements.json` |
| r3 便携包启动与界面 | 中文/空格路径、仅系统 PATH，CLI/sysid/Overlay 启动、1200 合成帧录制回放通过；GUI 中文显示正常，可启动监督并显示 stderr 日志，Gate 保持 0；关闭面板后 UI/监督子进程均为 0 | 便携测试（`artifacts/portable-r3-smoke/result.json`）、启动进程（`artifacts/portable-r3-child-running.json`）、退出检查（`artifacts/portable-r3-child-after-close.json`）、PE 依赖（`artifacts/portable-r3-imports.json`）；不是实车控制测试 |
| 便携包检查点 | 中文/空格路径启动GUI、CLI和sysid；仅系统PATH下1200合成帧录制/回放零缺帧；ZIP616文件哈希一致 | 便携数据检查目录（`artifacts/portable-data-smoke-20261017`）、`portable-archive-verified.json`；此为开发检查点，后续代码仍在改进 |
| Rerun实际录制回放 | 3634遥测帧、3591视频帧；官方viewer验证RRD，10/30/50秒seek已检查；1视频帧缺同帧camera明确保留 | `crates/visualization/RERUN.md`；此行为早期回放检查，最新实时观察见上表，完整AR标定仍未通过 |
| 真实地图双读取路径 | JSON slice与Parquet车道/点/邻接/audit一致；跨区域邻接保留原始权限并禁止有效变道 | `artifacts/map-region-base-neighbors-equivalence.txt`及boundary日志；真实宽度/限速未验证，driveable仍false |
| MCAP流式校验 | recorder 8 项回归通过，覆盖 CRC、截断、资源上限、描述符和包装时间一致性；旧真实记录 25314 条消息/3634 遥测帧复验成功；216000 合成帧回放约 2.20 秒 | r3 测试（`artifacts/integration-check-r3.txt`）、真实记录复验（`artifacts/live-steer02-streaming-metrics.json`）、合成基准复验（`artifacts/benchmark-1h-streaming-verification.json`）；不是一小时人类驾驶 |
| Overlay 生产者历史回归 | 当时8项库回归通过，含过期、暂停/会话/resize失效及独立IPC；内部辅助测试不重复计数 | 历史测试（`artifacts/overlay-staging-fix-tests.txt`）、录制回放（`artifacts/overlay-staging-fix-replay.txt`）的发布0帧仅指该离线回放；r6实机发布/显示另见上表，`fully_calibrated`始终false |
| quickload 语义输入 | 追加索引及旧按钮索引冻结、有限批次回归通过；尚未在真实游戏执行 quickload | 回归日志（`artifacts/quickload-append-tests.txt`）；不计入存档加载或现场恢复验收 |
| 实测车身包络与高级实验入口 | 共用后轴/挂点基准、同总长不同前后悬、转弯挂车扫掠及独立包络包含检查通过；Live 必须有测量与证据 | 包络回归（`artifacts/measured-envelope-tests.txt`）、[Runtime说明](../crates/runtime/README.md)；当前真实车型完整标定/实验前置证据仍缺，未开放已认证驾驶 |

## 仍需补齐的系统能力

主栈异步规划、系统辨识、受控路线/泊车Runtime已形成离线实现与测试，仍需真实游戏闭环验证。
Rerun与Overlay已有静止实机联调，当前仍缺全量实时吞吐、跨视角精确投影标定、gfx持续驾驶兼容性/性能、真实车辆标定、完整城市/泊车实测、
后向安全监督的真实尺寸/场景验证、Linux原生执行与Windows MSVC构建。算法库或Replay通过不代表实车阶段完成。

完整动态转向标定仍未通过；静止绝对转向已在后续r7实测通过，见上文。r4通过SDK提示并进入驾驶后的真实基线和失败控制试验作为历史记录保留在下文。
看门狗的显式 BrakeOnly 与输入策略已有隔离子进程联合回归，见 联合 IPC 日志（`artifacts/command-loss-joint-ipc.txt`）。
该结果仍不证明实机短刹：主栈意外失联、主动释放、人工接管、暂停必须分别记录真实 Applied/effective 结果；旧 DLL 不支持模式 2 时只会释放。

## r4 源码补齐与便携包检查点

- P0：完整配置、动力学、相机视角旁路消息已保留各自时间与有效性；统一 ReplayAdapter 按真实物理帧和配置事件下界重建 GameSnapshot。旧录制仍可回放，缺失字段不补造。见 `p0-sidecars-tests.txt`、`replay-adapter-tests.txt`。
- P1：Runtime 真正执行可选 LTV-MPC；独立优化构建 999 次 QP 的 P95 约0.3795 ms，合成闭环 RMS 约0.0993 m。澄清：这里只对线性化轨迹热启动，没有把 Clarabel 称为已支持原始/对偶 warm start。见 `runtime-control-performance-summary.json`。
- 地图与导航：实际欧洲地图分为985个分片；Tromsø 2 km 窗口与旧Parquet读取逐字段一致，冷读约39 ms。原生GPS已接入Runtime、2 km换图和稳定源ID重绑。未知UID、未经验证道路与重复经过同一车道的局部路线均明确拒绝；不支持的循环路线不能算长途已完成。
- P2：每目标2秒HMM、1秒加速度拟合和分支概率已接入实际规划；地图闸门profile、类型和作用车道已有实际归档证据。原生GPS候选自动生成任务，Manual/Automatic/Distance分类型动作已接入Runtime；手动脉冲还需同帧Closed状态及既有互动证据。未知profile保持停车，静态时序不能替代实时闸门状态。
- P3：录制自动切片及辨识自车／IDM交通闭环已接入同一Runtime；四类合成触发库全量回归通过，并绑定源录制、地图、窗口、配置和标定后比较基线。它仍是模型仿真，平路、刚性他车挂车和外生信号等限制见 `crates/sim/RECORDING_SCENARIOS.md`。真实危险场景库仍待驾驶录制积累。
- P4：前进拉直／换挡确认／重规划的恢复路径已有合成回归。Reeds–Shepp曲线族、二维Dijkstra启发式及铰接解析连接已接入，全部连接限速统一为0.3 rad/s；原有50次合成扰动场景仍50/50。额外困难几何中，堵住直线路径的绕行成功，垂直和斜入车位在80,000扩展预算下NoPath，失败未排除；这不等于50次游戏困难泊车。Live停稳等待、取消和新快照二次验证已经接入；17项Runtime与6项隔离IPC回归通过，证据见`parking-lifecycle-final-tests.txt`、`parking-wait-ipc-final-tests.txt`。
- Rerun：实录 `live-steer-02` 的3634遥测、3591视频帧通过新版流式导出和官方RRD校验；`live-steer02-r4-visible-video-30s.png` 实际显示视频和地图。8类Arrow实体的呈现时间回归、原生交通挂车三维框、独立capture映射均已检查。完整E4和游戏实时流仍未验收。

- 操作面板：新增“运行与目标速度”页，复用同一Host入口，默认观察，支持受证据约束的启动和停止。速度只保存为同目录新草稿，未保存禁止启动，不覆盖原配置或热改控制；8项回归及UI严格Clippy通过，见`ui-drive-tests.txt`、`ui-drive-clippy.txt`。
- 平台：这一批源码的默认workspace已通过Linux目标交叉检查，见`linux-cross-check-r4.txt`；仍不是Linux原生执行或MSVC构建证明。

上述日志位于 `artifacts/`，Rerun产物位于 `crates/visualization/output/`。冻结后的整体验证日志为`integration-check-r4-final.txt`，发行包的最终机器记录为`r4-validation-summary.json`；记录尚未产生或存在失败时不可视为通过。故障时撤销控制不是车辆瞬间停稳的证明；所有阶段门槛继续保持未通过。

## r4 实机基线与制动语义故障

- `runs/live-r4-stationary-01`：60秒静止、未启用控制，3635遥测帧无缺帧。动力学3635条全部有效；真实配置确认四轮、无挂车。CameraView3634条无错帧、跨会话或误切换；最后1条缺失明确保留。
- 视频3553帧全部有相机关联，对齐P95 9.3061ms、最大11.135ms；捕获丢43帧。FrameTiming无序号/时序错误，不能将静止回调间隔冒充控制生效延迟。完整哈希与输出见基线汇总（`artifacts/live-r4-stationary-01-summary.json`）。
- `runs/live-r4-steer-01`：发送3秒`steer=0.3, throttle=0, brake=0.5`，实际简易自动挡把`abackward`转换为倒车油门，峰值倒车7.35km/h。命令结束时全零撤权，但车辆继续滑行；不能用撤权证明停稳。录像末尾仍有速度，之后独立遥测确认停稳并进入菜单。详见失败分析（`artifacts/live-r4-steer-01-findings.md`）。
- UI确认原设置为“简易自动挡”，切换为“真实自动挡”后磁盘`g_trans`从0变为3。磁盘枚举与界面观察只作本次证据，程序使用官方SDK `controls / shifter.type`事件进行联锁。
- 此后源码增加实际控制模式联锁：简易自动挡、未知模式、会话错误、未来配置均禁止车辆输入；AEB/BrakeOnly同样受限，不能以制动名义继续输出倒车轴。任何模式代际变化撤销原启用状态。源码测试通过不等于新DLL现场验收；旧r4包保留为不可改写的历史检查点。
- r5同时修复暂停中途残留输入、单槽共享状态写入窗口的误撤权，以及真实地图毫米级末段的量化航向。新默认workspace 316项通过、8项明确忽略，严格Clippy和UI构建通过；Linux目标交叉检查通过，仍不是Linux原生执行或MSVC证明。见`integration-check-r5-final.txt`、`r5-workspace-test-counts.json`、`linux-cross-check-r5-final.txt`。MPC功能回归仅在该测试使用现有合法50ms预算，实际超时Fault与独立1ms性能目标未改变，先前并发失败记录保留。
- 20个原生路口326条successor端点最大间隙约0.000034m；5个朴素末段航向异常与2–9mm量化尾段有关。公共切向估计现用本车道至少0.25m有效基线，不改坐标、弧长或拓扑；无足够基线的曲线明确拒绝。当前Tromsø 2km窗口1964车道中没有整条不足0.25m的曲线。此项是几何审计与回归，实际道路宽度、场地净空、20路口游戏目测仍待验收，见`artifacts/field-map-audit/SUMMARY.md`。

## r5 静止实机复验

`runs/live-r5-steer-brake-01`和`runs/live-r5-camera-steer-01`确认当前SDK加载真实自动挡，零油门制动不再变为倒车；两段均空挡、RPM0，不代表行驶制动验收。正负语义转向确有轮角响应，但本机相对模式下是积分控制，尚无可用于驾驶的绝对转向模型。第二段已经逐一录到cam2–8及返回cam1的SDK完成边、原始相机和精确视频帧；8种画面已人工检查。菜单暂停时钟冻结已测，恢复后重新启用仍需另测。

第二段保留`dropped_frames=1`和24个capture drop，不认定为零丢失；现存消息不足以唯一定位聚合计数来源。近驾驶室的cam5/6/7被旧启发式误标Interior，不能据标签宣称自动识别成功。报告见`artifacts/live-r5-camera-steer-01-summary.json`和图册；r6补充流级丢失事件与NearCabinUnclassified标签，已在后续实录观察到码4及新的流级损失诊断，未倒改r5。

r6默认workspace 324项通过、8项明确忽略，严格Clippy与UI构建通过；Linux目标交叉检查通过。见`artifacts/integration-check-r6.txt`、`r6-workspace-test-counts.json`、`linux-cross-check-r6.txt`。新增`/recording_loss`单条事件限制4KiB，记录真实观察时钟和环索引区间；未知丢失物理帧/时刻保持null，旧总计数口径不变。真实64槽测试分别复现启动历史窗和运行中覆盖，事件明确区分；历史记录不补写来源。近驾驶室新码4不改旧码1或旧录制，也不选择未完成标定的相机约定。以上仍非完整P0–P4现场验收。

## 现场试验规则

- 每次试验使用新的运行目录，记录游戏/插件/地图/车辆版本与开始存档。
- 罚单验收必须先确认 `g_police=1`；选中存档已备份并从0改为1，重启日志已确认加载1；此前零事件不能算零违章。
- 每个接管要记录原因与时间；暂停/菜单期间不累计驾驶距离。
- 视频索引包含捕获帧、QPC、物理帧及对齐误差；丢失或停止必须判为不完整。
- P4 每次记录开始状态、目标车位、难度、成功/失败、姿态误差及 `job.delivered` 事件，至少50次，不排除失败样本。

