# Rerun 只读回放与实时诊断

这是可选 Rust 功能；`cargo build` 默认不编译 Rerun。固定 SDK 与 portable viewer 为 **0.38.1**，可选功能要求 Rust ≥1.96。SDK 只启用 `sdk`，不把原生查看器、视频解码器或 Web 服务编入主控制栈。Windows、Linux/macOS 的离线导出使用同一代码；实时共享内存适配器面向 Windows。此工具没有控制映射、转向或键盘输入 API。

推荐 PowerShell 7.2+ 的统一入口，默认使用独立目录的 **release优化版**。首次构建需要额外时间与空间；构建失败立即退出，不启动残留旧程序。普通项目构建仍不启用Rerun。

```powershell
# 从项目根目录；路径含空格或中文时用引号。
./crates/visualization/run.ps1 -Mode Export -Run "runs/live-steer-02" -Output "crates/visualization/output/离线 回放.rrd"
# 仅构建；依赖已缓存时可加 -Offline。-NoBuild显式复用已有release产物。
./crates/visualization/run.ps1 -Mode Build -Offline
# 如默认cargo选择了不可用工具链，通过-CargoPath或ETSADS_CARGO指定已安装cargo.exe。
```

输入/输出路径按调用者当前目录解释；脚本自身不要求当前目录是仓库根目录。也可直接构建和运行：

```powershell
. ./tools/env.ps1
& $EtsadsCargo build --release --locked -p etsads-visualization --features rerun --bin etsads-rerun --target-dir target/visualization-rerun
& ./target/visualization-rerun/release/etsads-rerun.exe export --run runs/live-steer-02 --map tools/map_export/output/region-tromso-rerun/runtime-map.json --output crates/visualization/output/live-steer-02.rrd
./crates/visualization/view.ps1 -Recording crates/visualization/output/live-steer-02.rrd -InstallViewer
```

输出路径必须尚不存在。除了 `.rrd`，工具生成 `.report.json` 和 `.video-associations.json`。已有产物不会被悄悄覆盖。`view.ps1` 从官方 GitHub 发布下载便携查看器到项目 `.tools`，执行前校验固定 SHA256，并用 `rrd verify` 检查文件；不会安装全局软件。Linux/macOS 可从同一官方发布下载相应查看器。

只校验现有RRD，不打开GUI或后台查看器：

```powershell
./crates/visualization/view.ps1 -Recording "crates/visualization/output/离线 回放.rrd" -VerifyOnly
```

省略`-VerifyOnly`和`-HeadlessScreenshot`会正常打开GUI。`-InstallViewer`只在明确请求时下载固定版本。无界面截图仍会启动独立的无窗口viewer进行真实渲染；截图及旁路诊断文件必须均不存在：

```powershell
./crates/visualization/view.ps1 -Recording crates/visualization/output/live-steer-02.rrd -HeadlessScreenshot crates/visualization/output/rerun-viewer.png -AtSeconds 30
```

无界面截图脚本使用 PowerShell 7，通过官方 viewer MCP 等待录制载入、切到指定 elapsed 时间、等待渲染后再截图，并关闭自己的后台 viewer。省略`-AtSeconds`时选择录制中点。直接使用官方`--screenshot-to`可能在延迟加载完成前得到空画面，因此脚本不再采用该方式；解码日志保存在图片旁的`.viewer-logs.json`。

`elapsed` 是采集 QPC 相对首个遥测样本的秒数，适合主时间轴；`simulation` 是暂停感知仿真时钟；`physics_frame` 是物理帧号。视频呈现时，自车、交通、相机与图像都写在同一个曝光QPC处；原始物理QPC保留在关联元数据中，曲线仍使用物理采样时间。没有对应物理快照时不捏造暂停时钟。

图层包括：绝对 ENU 米制道路中心线、SDK 自车参考点与方向、交通平面轮廓，以及从原始扩展记录读取的交通车、其挂车、停放车辆三维线框。三维框使用各自中心、尺寸（含高度）与完整四元数，不沿用父车高度或仅使用yaw。自车完整外观高度尚未测量，所以仍只画参考点，不补造bbox。动态信号和原生导航节点路线分别保留原始身份；原生路线不是规划器轨迹。速度、轮转角、输入/实际转向、油门、刹车和离合分别记录时间序列。

地图读取直接调用 `etsads_map::importer::ExportedMap::from_path`，不另建车道 JSON schema。`status/map` 展示同一个 Runtime `ImportAudit`，schema 错误会在创建 `.rrd` 前报错；`driveable=false` 的地图仍允许诊断显示，显示成功不授予驾驶资格。新增原生 nodes 仅提供导航参考位置，不能替代该 audit 或区域政策。

路线只从地图 `nodes` 数组的原生 UID 和真实位置解析。该数组由 `map_export slice` 从 `nodes.parquet` 提取，不能用偏移过的车道端点替代。未解析 UID 会断开路线，不跨过未知点画虚假捷径。旧切片没有 nodes 仍可回放，其路线 UID 和缺失原因保留在文本实体中。

视频使用完整原始 MP4 内嵌 `AssetVideo`，无需在另一台机器复制额外视频路径。`ffprobe` 必须可用，或通过 `--ffprobe PATH` 指定。每个 `video_index.video_frame` 对应实际编码帧序号；工具使用与 Rerun 0.38.1 相同版本的官方 `re_mp4 0.5.1` 采样表，按 presentation 顺序排序及归一化，并以 `re_video::Time::into_nanos` 的舍入生成 `VideoFrameReference`。同时逐帧交叉检查 ffprobe 解码得到的有理 PTS（允许非零首 PTS，其偏移写入报告）。不使用固定 FPS 推算，也不把仿真秒数当视频 PTS。重复/缺号索引、帧数不一致或两种解析器的时间序列不一致会报错。

相机状态按 `video_index.physics_frame` 精确关联同物理帧 ExtensionFrame；缺失就标记 missing，不偷偷换成最近相机。索引的“同物理帧”并不证明渲染曝光与物理采样同一时刻，因此每帧保留 camera/capture QPC 差、P95、最大值和超过16 ms的数量。旧录制索引偏后问题会原样反映，不能因此算同步验收通过。

默认图像是独立 2D 视频，原始 CameraState 以文本保留，不隐式认证相机模型。`--interior-c1` 可明确启用转置投影、C→W四元数、−Z前向的相机显示；它仅有此前静止车内方向证据，绝对点投影及车外视角仍未验收。渲染相机坐标采用 RUB，位姿按原相机基向量经公共 SCS→ENU 函数计算，避免误用同时转换本地轴的 M R M⁻¹。

实时只读模式与离线模式共用 `GameAdapter` 的 scene 转换以及本 crate 的记录函数：

```powershell
# 推荐入口：只读live，保留本地RRD；不会启动游戏或自动打开viewer。
./crates/visualization/run.ps1 -Mode Live -Seconds 30 -Output "crates/visualization/output/实时 诊断.rrd"
# 只输出结构化状态可加-NoVideo；已有release产物可显式加-NoBuild。
# 直接调用优化版也可：
& ./target/visualization-rerun/release/etsads-rerun.exe live --seconds 30 --output crates/visualization/output/live-read-only.rrd
# 或先打开官方 viewer，再连接本地127.0.0.1:9876。
& ./.tools/rerun-0.38.1/rerun.exe
& ./target/visualization-rerun/release/etsads-rerun.exe live --seconds 30 --connect
```

实时模式默认采集画面，按最多10 Hz显示；`--no-video`可只输出结构化状态。单独的capture reader租约不会抢占录制器的读取槽。使用采集曝光QPC在64帧历史中寻找前后物理样本，选择误差不超过16 ms的历史相机，最多等待未来样本40 ms；GPU发布时间只用于报告回读延迟。画面、交通和自车均取同一个历史快照，不能使用Overlay准备下一次Present的“最新相机”策略。

默认图像仍独立显示。只有显式传入`--interior-c1`且历史关联有效时才输出Pinhole诊断模型；这不证明C1已完成绝对标定。缺相机、暂停、时钟不一致、resize和超误差均明确记录，画面不会套用旧相机。采集500 ms未更新会清除画面与相应动态场景。`.report.json`保留关联状态和`phase_acceptance=false`；没有遥测或请求视频却没有图像时，命令以错误退出。

实时和回放均提供`control/sdk_application_delay_ms`曲线：按ControlCmd.seq和真实AppliedControl匹配，每个命令只统计首次成功应用，不把持续持有命令的年龄重复算作延迟。未匹配、非因果时钟、释放/拒绝分别显示状态并清空数值，不填零。这是SDK应用延迟，实际轮胎/踏板响应仍须对照effective遥测测量。

回放场景由统一`etsads-replay::ReplayAdapter`流式重建，按配置事件允许关联的下界和同帧扩展转换。导出不再驻留全量遥测、相机或命令延迟记录：快照窗口为64帧，视频索引最多500000个、地图JSON最多128 MiB、内嵌MP4最多2 GiB；超限明确退出并保留原始数据。ffprobe输出有界且总超时300秒，SDK输出每256个物理帧刷盘，避免文件队列无限积累。实时画面用原尺寸JPEG质量80供诊断显示，属于有损显示数据，不替代原始录制作为测量依据。

历史曝光、真实隔离capture映射、resize、像素格式、原生挂车高度、媒体子进程限制及Arrow记录中的统一呈现时间均有回归。日志见`artifacts/rerun-final-feature-tests.txt`、`rerun-exposure-timeline-tests.txt`；它们仍不替代真实游戏实时联调。

官方依据：[Rust RecordingStream](https://docs.rs/rerun/0.38.1/rerun/struct.RecordingStream.html)、[视频与帧引用](https://rerun.io/docs/concepts/logging-and-ingestion/video)、[0.38.1 portable release](https://github.com/rerun-io/rerun/releases/tag/0.38.1)。发布 SHA256：`0c0205d63107b1fd286b0a4d2044f70feaf0f4a5a19ab6a49aaa1de7812b6a2a`。

## 本机实录验收记录

`runs/live-steer-02`：游戏 PE 1.61.1.9，3,634 遥测帧、3,601 场景帧，75,294 个车辆观测（单帧最多23辆）。3,591 张 HEVC 1920×1080 视频帧全部通过 MP4 采样表与 ffprobe 的逐帧 PTS 交叉检查。3,590 张图有索引指定的 CameraState，1张明确缺失；已关联帧 camera/capture 时间差 P95=9.0664 ms，最大11.0402 ms，超过16 ms为0。记录中没有原生导航路线或规划轨迹，没有填入示意路线。

`output/live-steer-02-runtime-schema.rrd` 已通过官方 `rrd verify`，离屏查看器在 elapsed=30秒实际显示了实录视频、道路和车辆参考；同源首版还在10/30/50秒审阅视频。`output/rerun-runtime-schema-30s.png` 是最终审阅截图。HEVC 后端仍报告两条 NALU type5 skip 警告；因此当前确认的是容器可读和所查时间点可显示，不宣称全视频逐像素解码无误。完整机器可读证据见 `evidence/rerun-validation.json` 和 `evidence/rerun-live-steer-02-report.json`。

以上是早期检查点。新版已增加统一ReplayAdapter、同曝光时间场景和原生三维框；14项库测试及2项CLI测试已有通过记录（显式忽略的现场/子进程辅助项不算通过）。早期一次5秒实时探测因不存在telemetry mapping退出1；后来已完成下述真实只读实时联调，但不据此宣称60Hz采样或完整阶段验收。绝对相机投影与完整E4仍不能由查看器成功显示推导。

新版 `view.ps1 -HeadlessScreenshot` 已自动调用官方 `rerun viewer-mcp` 的 `rerun_set_time_cursor`、等待渲染及 `rerun_save_screenshot`，保存状态和解码日志。可用`-SettleSeconds`调整额外等待；截图仍需实际检查。`output/live-steer02-r4-visible-video.rrd`已通过官方校验，`output/live-steer02-r4-visible-video-30s.png`实际可见驾驶室视频和地图，该次viewer日志为空。默认独立视频页始终保留，C1诊断开启时额外记录相机下的图像，避免默认布局选到空白页。工具只连接独立的无窗口查看器，未操作游戏。



## 优化构建的本机实时观察

同为请求30秒的只读运行：debug读取191个快照/29张图像（28关联、1 MissingPast）；release读取1130个快照/165张图像（164关联、1 MissingPast），均退出0且control_written=false。名义采样量分别约6.37/0.97与37.67/5.5次每秒；这是实际诊断采样量，不是游戏SDK总帧数，也不是严格同负载基准。不能称为60Hz通过，phase_acceptance仍为false。证据为`artifacts/live-r6-rerun-interior-01.txt`与`artifacts/live-r7-rerun-interior-release-01.txt`。

图像目前在轮询线程内同步转换RGBA并编码原尺寸JPEG80，完成后再等待100ms，因此图像目标最多约10Hz。debug构建会明显放大该路径开销；release仍可能因编码、场景写入和flush减少采样。进行性能对比前结束编译和其他高负载工作。查看器流不替代需要完整物理帧与丢失计数的MCAP录制。

新增的`Convention::ETS2LA_1_61_REVIEWED_EXTERIOR`是显式、非默认的测试约定名称；它没有自动更改显示选择。近驾驶室诊断码4为NearCabinUnclassified，不能据距离把cam5/6/7认成驾驶室。`--interior-c1`仍须显式指定，不能当作cam1识别或所有视角标定。