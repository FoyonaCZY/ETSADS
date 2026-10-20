# 独立未标定 AR 诊断发布器

`etsads-overlay` 是独立可执行程序，不依赖主控制应用。它读取 GameAdapter 与 capture metadata，使用已有 D3D11 覆盖图层的 `OverlayFrame` ABI；只有 `live` 子命令会发布到 `ETSADS_Overlay`。没有车辆控制客户端，也没有控制或授权门禁映射的读写。CaptureReader 会按其协议登记/续约读取者租约；不会取图像像素或写入采集图像。

每次必须显式选择 `--hypothesis 0..7`，没有隐藏默认值。输出、记录和屏幕角标始终为 `fully_calibrated=false` / `UNCAL Cn`。C1 只获得过车内方向证据，未获得绝对投影、遮挡或跨视角验收。此程序不是驾驶功能。

## 编译与离线回放

在项目根目录：

```powershell
. ./tools/env.ps1
& $EtsadsCargo build --offline --locked -p etsads-overlay --target-dir target/overlay-diagnostic
& ./target/overlay-diagnostic/debug/etsads-overlay.exe replay --run runs/live-steer-02 --output apps/overlay/output/my-replay-c1 --hypothesis 1
& ./target/overlay-diagnostic/debug/etsads-overlay.exe verify --run apps/overlay/output/my-replay-c1
```

第一次下载依赖后可离线编译。Replay 不打开任何游戏映射。视口尺寸来自录制 MP4 的 ffprobe 结果；视频或索引缺失时明确失败，不假设1920×1080。输入源必须是原始 `source=game` MCAP，派生结果标为 `game_replay_diagnostic`，不能代替实车验收记录。源文件 SHA256 写在 meta.json 中。

输出目录必须不存在，包含：

- `data-0000.mcap`：原始遥测、扩展、CameraState 与生成 OverlayFrame，以及采集元数据和逐帧诊断。
- `evidence.jsonl`：假设、矩阵、相机/采集时间、每个原生对象中心/尺寸/完整四元数、屏幕范围、剔除原因、矩形预算。
- `summary.json`：生成、清空和实际发布次数分开；回放的发布次数必为0。
- 三份 `projection-*.svg`：实际绘制原语的几何预览，不是游戏截图，不宣称对象可见。
- `verification.json`：执行 `verify` 后生成，检查MCAP完整性、矩形边界/颜色、禁用帧清空、未标定标识和记录计数。

旧录制没有 capture generation/publish QPC 时保留unknown；离线的评估时刻取已记录采集/相机/遥测时间最大值，仅是回放假设，不当作实时新鲜度验收。索引指定的物理帧缺失或相机与采集差超过16 ms则清空。源录制没有 BridgeInfo 时不补造SDK版本；原始 meta 中的游戏版本与SDK版本分开。

## 有界实时诊断

只有明确需要游戏内覆盖图层时才启动：

```powershell
& ./target/overlay-diagnostic/debug/etsads-overlay.exe live --seconds 30 --output runs/overlay-diagnostic-new --hypothesis 1
```

必须指定新的证据目录，时长限制1..3600秒，Ctrl+C可退出。默认最多32个框、200m范围、2px线宽，可用 `--maximum-boxes`、`--maximum-distance-m`、`--line-width-px` 调整到受限范围。没有 `--calibrated`、驾驶使能或任意映射目标参数。

只采用 ETS2LA Traffic/Parked 的原生bbox中心、三维尺寸和完整四元数；真实 Traffic trailer 记录具备完整尺寸时才画该挂车。SDK自车/挂车参考点没有可信尺寸，不推造高度或bbox。复用 `etsads_game::camera::bbox_corners_scs` 与 `project_bbox`；近裁面相交、在画外或无效尺寸的对象省略。按相机距离从近到远处理，最多256个矩形，每框4边；5×7字形的未标定角标最后绘制，保留预算且不会被对象框盖住。遮挡未知。

实时发布需要原始 BridgeInfo、最新同物理帧遥测/扩展、有效capture generation及同一QPC时钟。Live只保留最新GameSnapshot，遥测与相机自身年龄最多100 ms，capture元数据的**发布年龄**最多100 ms。gfx有三槽GPU回读，metadata的曝光时间可能属于Present n−3；该旧图只提供视口/producer活性信息，不用它倒选旧相机，也不假称当前相机与旧图匹配。Live要把最新状态投影到后续Present；它不预测那个Present的精确时刻或完成渲染的回执。

Live每帧明确记录 `timing_context=live_latest_for_presentation`、`capture_is_image_match=false`，并分别保存相机/遥测年龄、元数据发布年龄、GPU回读延迟、旧图曝光年龄和相机减曝光时间差。Replay继续要求所录图像与相机相差≤16 ms，两个规则不能互换。暂停、时钟错误、数据失效、分辨率/generation改变、会话改变都清空；viewport改变后必须等到该次viewport publication之后的相机样本。写证据后、发布前重新读取capture元数据及BridgeInfo再检查，I/O延误不继续绘制旧框。独占写者guard阻止多个发布器。

原始SDK `BridgeInfo` 按字节记录。实时 meta 中游戏patch为unknown，不把SDK game_version伪装成1.xx补丁号。记录包括 `/telemetry`、`/extension`、`/camera`、`/bridge_info`、`/capture_metadata`、`/overlay`（准备帧）、`/overlay_status` 与 `/overlay_published`（实际发布见证）。记录失败会停止并由RAII清空；正常退出和Ctrl+C也清空。进程被强杀不能执行RAII时，已有gfx消费者在200 ms后拒绝过期帧。

## 已完成的验收与限制

`output/replay-steer02-c1-reviewed` 使用真实 `runs/live-steer-02`。3,239个原始遥测/相机样本匹配到3,590张图，生成17,439个对象框观测；1个索引缺匹配数据而清空。最大101个矩形，3,590帧未标定标签，0帧发布到游戏。派生MCAP共20,489条消息，`verify` 检查通过，遥测非单调数为0。三张SVG及其计算光栅预览只用于核对原语/角标。

8项测试通过，覆盖80对象预算/近者优先、近裁面与未知高度、省略与清空、暂停/过期/resize/session、真实隔离映射单写者与Drop清空，并模拟三槽50/150 ms回读延迟仍选择最新相机、resize后等待新相机及Replay16 ms边界；其中一个父测试启动独立子进程验证IPC读取和无法抢占写者。另一个ignored测试只是该子进程入口，父测试已实际调用。Clippy `-D warnings` 通过。省略hypothesis的CLI调用退出2且不创建输出目录。

此次修复替换了r2的Live时序实现；r2便携包中的overlay需重建/重打包，gfx协议和DLL未因该修复改变。`evidence/live-staging-timing-audit.json`保留调查依据：旧截图metadata实测publish−exposure为1.4193/2.0211/2.0238 ms；它们不标识capture producer，旧MCAP也未记录publish QPC，不能当作native三槽约50 ms的实测证据。50 ms是三槽/60 Hz的推导，另有对应的隔离模拟回归。

修复后又在 `output/replay-steer02-c1-staging-fix` 重跑同一真实录制及完整 `verify`：3,590启用帧、17,439对象框观测、最大101矩形、1缺匹配清空、0实际发布，与修复前summary完全一致；三份代表帧SVG的SHA256也逐字节一致。该回归只证明Replay规则和几何未被Live修复改变，不能替代生产Live端到端验收。

当前没有启动过生产 `live` 发布器，没有游戏内AR渲染、绝对投影误差、遮挡或跨视角验收。以上通过项不能改写为E4完成。原生gfx消费链已有其独立测试，本程序与真实游戏的端到端联调仍待执行。
