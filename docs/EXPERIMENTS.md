# 受控路线与泊车开发试验

`drive` 现在有明确的 P2/P3 受控路线与 P4 泊车执行入口。它们是开发试验许可，
不是阶段验收证书。默认仅观察、强制录制；只有额外指定 `--arm` 才请求一次独立监督许可。
P0 静态 `steer`、`pulse` 和原有 P0/P1 模式不要求这里的高阶证据文件。

当前仓库没有生成任何“已通过”的真实高阶试验证书。缺少动态标定、物理边界、语义输入、
停车目标或后向 AEB 实测时，命令会指出缺项；不能把 synthetic 回放报告改个文件名用于放行。

**当前P0–P4实车退出条件仍未通过。** 本机测试存档已获用户明确授权，将`c_relatsteer`从1改为0并重启；r7的±0.1、±0.25静止试验确认稳定绝对响应及释放回零。r5积分响应仅代表修改前的配置。原文件保存在`artifacts/absolute-steering-review-r5`，应用证据为`artifacts/absolute-steering-applied-r7.json`；本文命令或面板草稿不会自动修改游戏设置。静态响应仍不能代替行驶速度相关映射、动态延迟和纵向辨识。

r6实机Overlay/Rerun与r7三个真实投影夹具，只分别证明可视化链路和限定版本/视角的投影约定。它们不是`input_semantics`、实体边界、动态标定或后向AEB许可。Sky截图没有精确捕获QPC时不能单独满足同帧视频证据要求；`NearCabinUnclassified=4`、`Other=3`也不能替代相机身份/标定证明。完整当前范围见[验收记录](ACCEPTANCE.md)。

## 启动方式

先启动独立监督进程。泊车必须使用与试验证据绑定的同一安全 profile：

```powershell
etsads-supervisor.exe --safety-profile evidence/reverse-profile.json
etsads-cli.exe drive --config evidence/parking-runtime.json --calibration evidence/vehicle.json --map evidence/map/runtime-map.json --policy evidence/region.json --experiment parking --experiment-evidence evidence/parking-experiment.json --safety-profile evidence/reverse-profile.json --output runs/parking-observe-01 --seconds 60 --video
```

受控路线使用 `--experiment controlled-route`，RuntimeConfig 的模式必须为 `ControlledRoute`。
泊车配置必须为 `Parking`。省略 `--arm` 时所有轴保持零，所有脉冲计数不变。
只有检查同一套证据和场地后，才在上述命令显式追加 `--arm`，并使用新的输出目录。
程序不会在暂停、故障、人工接管或失联后自动重新 arm。

游戏 PE 版本、固定的 ETS2LA 插件哈希、地图来源/版本、动态标定的真实数据来源、车辆指纹、
后轴参考、挂车数量和载荷范围仍须满足。所有高阶模式要求地图与 policy；泊车还逐帧核验
独立 supervisor 的状态心跳、实际已加载 profile 的 SHA-256、当前配置序号和 reverse_ready。
只在 CLI 放一个 profile 文件不构成后向安全监督已运行的证据。

## 证据文件

数据结构以 `apps/etsads/src/driving/experiment.rs` 为准，拒绝未知字段。顶层字段：

| 字段 | 含义 |
|---|---|
| `schema_version` | 必须为 1 |
| `kind` | `controlled-route` 或 `parking`，与 CLI/RuntimeConfig 一致 |
| `game_version`、`vehicle_id` | 与当前游戏 PE 版本和动态标定身份一致 |
| `config_sha256`、`calibration_sha256` | 原始配置文件的 SHA-256，包含整个 mission |
| `map_manifest_sha256`、`map_data_sha256`、`policy_sha256` | 绑定实际加载的地图和区域许可 |
| `safety_profile_sha256` | 泊车必填；绑定监督进程实际加载的原始文件 |
| `calibration_recording` | 动态标定源录制的 `meta_sha256` 和完整 `segments` 字典 |
| `supporting_files` | `[{"path":"...","sha256":"..."}]`，绑定被配置引用的证据文件 |
| `evidence` | 下表所列 prerequisite 与录制/审阅文档的映射 |

支持文件路径相对证书所在目录解析。配置内的 evidence_path 相对其拥有者文件解析：
标定参考/载荷/实测前后悬包络/可选前保险杠相对标定文件，boundary/control/target 相对 RuntimeConfig，
区域证据相对 policy，后向几何证据相对 profile。每个被引用文件必须存在且出现在
`supporting_files` 中，不能只放一个非空路径或 true 布尔值。
Live使用统一的实测 `MeasuredEnvelope`，独立profile仍保留自己的测量；必须证明规划包络
包含独立安全包络，不能用扩大一个猜测总长的方式绕过前后悬匹配。

JSON 地图的 `map_data_sha256` 是文件 SHA-256。Parquet 目录使用固定次序
`lanes.parquet`、`lane_points.parquet`、`lane_links.parquet`、`lane_neighbors.parquet`：
依次哈希 UTF-8 文件名、零字节、文件 SHA-256 的 32 个原始字节；缺少可选 neighbors 时
用 UTF-8 `absent` 替代其摘要。实现为公开的 `experiment::map_data_hash`。

| prerequisite 名称 | 何时必须 | 可核对的录制条件 |
|---|---|---|
| `input_semantics` | 所有高阶试验 | 实际 `/input_actions` ACTIVE批次已经返回按钮边沿，随后灯态/挡位真正发生变化；收费activate还需后续paid事件。旧有状态和请求计数不能代替反馈 |
| `physical_boundary` | 所有高阶试验 | 视频索引与物理frame/QPC/time及status=0对齐逐条一致；所有MP4绑定哈希，ffprobe实际帧数与完整索引数一致 |
| `traffic_annotations` | 受控路线 | 解码真实 `/extension`，要求traffic+signals可用且状态/时钟/几何有效，单有topic不算证据 |
| `parking_target` | 泊车 | 有对齐视频，审阅明确目标所指的后轴/挂车参考，不允许 Unknown |
| `reverse_aeb` | 泊车 | raw后向扫掠确有危险，匹配profile为ready，fresh ACTIVE Gate、零主栈刹车、实际SDK刹车和随后effective响应组成因果链；Off/BrakeOnly不算独立AEB验收 |

每个 prerequisite 的值有如下结构。这里的占位符故意不能通过验证：

```json
{
  "run": "../runs/REAL_COMPLETED_RUN",
  "meta_sha256": "REQUIRED_SHA256",
  "segments": {"data-0000.mcap": "REQUIRED_SHA256"},
  "review": "reviews/physical-boundary.json",
  "review_sha256": "REQUIRED_SHA256",
  "video_files": {"video-0000.mp4": "REQUIRED_SHA256"}
}
```

`segments` 必须列出目录中的全部 MCAP；视频用途必须列出全部 MP4。预检会重放并核对
metadata 消息数、非单调/缺帧以及审阅区间的每个物理帧，不仅读取 completed 标签。
必须是 `source=game` 且版本匹配；区间内记录的车辆配置也必须匹配标定指纹。
文件哈希可用 `Get-FileHash -Algorithm SHA256 <path>` 获取，不能复制未知版本的摘要。
视觉证据需要可用的ffprobe：默认从PATH查找，也可设 `ETSADS_FFPROBE`，或使用
`ETSADS_FFMPEG`同目录的ffprobe。语义因果证据需要当前桥接的 `/input_actions` 和
`/frame_timing`；缺少这些原始记录会明确拒绝，不能给旧录制补写一个通过标志。

审阅 JSON 的字段为 `schema_version`、`purpose`、`decision`、`game_version`、`vehicle_id`、
`frame_range: [first,last]`、`reviewed_by`、`observations`、`limitations`、`observed_actions`。
`decision` 仅允许 `allow_bounded_development_experiment`，必须由审阅者在检查对应记录后填写；
观察、限制和审阅人不能为空。它明确批准一个有限开发试验，不声称道路、城市或泊车阶段通过。
程序不会自动代写这种审阅决定。

## 动作确认与退出

转向灯使用 SDK 逻辑灯状态（Telemetry.lights 第 6/7 位），不是闪烁亮灭。切换方向先确认旧灯关闭，
再开启新灯；1 秒无反馈即停止。危险报警灯/左右灯同时亮会要求人工处理，禁止开始未确认灯态的变道。

换挡只允许泊车试验；必须测得速度 ≤0.05 m/s 且保持刹车。一次请求只增加一次脉冲计数，
锁存直到 raw gear 正负号确认；3 秒未确认就退出。等待期间强制零油门并保持制动。
activate 仅用于受控路线中已解析为手动类型的闸门，在静止、制动、停留至少1秒且当前物理帧明确CLOSED时发送一次；持续意图不会逐帧反复点击。自动和距离触发类型不发送activate。原生导航即使初始`tolls`为空，也须先提供真实activate语义证据，后续地图窗口不能绕过该要求。
Runtime从选定GPS车道走廊的实际经过区间生成闸门任务，未知或矛盾类型停止。收费paid事件和整车清空闸门平面分开记录；手填已审查收费任务仍需paid事件，未分类原生闸门的物理通行不算收费成功。地图静态profile时长不作为实时剩余时间。发布动作前CLI重新检查当前帧栅栏状态，旧规划中的CLOSED不能替代新观察。

每次动作及反馈先进入 mandatory `/actuator_event` 队列，再发布命令；队列满时不发新脉冲。
录制包含 `/experiment_evidence`、配置、场景、轨迹、栈输出以及 `/supervisor_status`。
Ctrl+C、暂停、过期、配置变化、许可撤销和错误都结束命令并请求 gate off，等 MCAP/video 完成关闭。
正常文件关闭成功不等于实验验收通过，后续仍需按 ACCEPTANCE 的距离、误差、接管与事件指标审阅。

## 已验证范围

隔离测试验证真实 Runtime Replay 泊车请求进入命令映射、单次脉冲与测得挡位反馈，
观察模式无动作、换挡/灯态超时、输入轨迹证据反压、哈希篡改、synthetic 录制拒绝，
以及独立 profile/配置/心跳不匹配拒绝。它们没有连接真实游戏，不能替代实机阶段验收。
