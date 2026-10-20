# E4 相机投影离线诊断

该 crate 只读取已完成的 MCAP/视频及离线地图，不连接实时映射、不截图、不发送游戏或 UI 输入，也没有修改 game/bridge 协议。默认 feature 为空；图像、字体、MCAP 和 CLI 依赖只在显式启用 `diagnostics` 时编译，未引入 Rerun。世界标记 JSON 可供后续可选 Rerun 导出器使用。

## 当前结论：车内前视局部支持 C1

对 `runs/live-stationary-01` 的三个不同录制帧，优选：

```rust
etsads_game::camera::Convention {
    transpose: true,
    quaternion_world_to_camera: false,
    positive_z_forward: false,
}
```

即：数组经过 `Mat4::from_cols_array` 后转置；记录四元数按相机→世界理解，世界点转相机时使用共轭；相机朝 −Z。

这是**当前车内朝前视角的方向与约定检查**，`fully_calibrated` 保持 false。消失点与相机平移无关，因此本实验没有验证相机绝对位置/高度和完整 3D 点的像素重投影误差，更不能据此宣布车内、车外都通过。

## 可复现命令

在仓库根目录：

```powershell
. ./tools/env.ps1
& $EtsadsCargo test -p etsads-visualization
& $EtsadsCargo run -p etsads-visualization --features diagnostics `
  --bin etsads-projection-audit -- `
  --run runs/live-stationary-01 `
  --map tools/map_export/output/region-tromso-1km/runtime-map.json `
  --annotations crates/visualization/evidence/interior-verticals.json `
  --output crates/visualization/output/interior-e4
```

需要 PATH 中的 ffmpeg；也可指定 `--ffmpeg PATH`。默认字体为 Windows Segoe UI，可用 `--font PATH` 指向其它本机 TrueType 字体。`--frames 223,1117,2011` 可明确选择至少三个不同的编码视频帧。工具用 ffmpeg 的帧序号选择，不用录制实时时间推算视频 PTS。

不提供 annotations 时仍生成八种对照；没有充分区分证据时不会写入优选方案。工具使用已有 `Convention::candidates()` 与 `project()`，未复制一个行为不同的有限点投影实现。

## 数据与对齐

| 编码视频帧 | 对应相机帧 | 离线最近相机 QPC 误差 | 原索引误差 |
|---:|---:|---:|---:|
| 223 | 2257 | 8.6692 ms | 22.4395 ms |
| 1117 | 3987 | 7.5143 ms | 7.5143 ms |
| 2011 | 5195 | 6.8680 ms | 6.8680 ms |

三张原图已逐图查看，都是同一静止的车内前视姿态；时间不同不等于视角独立。第 223/1117 帧的离线最近相机样本位于 capture QPC 之后，报告明确标记，不把它伪装为当时在线可用的零延迟状态。录像旧 metadata 的 `game_version=unverified` 也原样保留。

`evidence/interior-report.json` 保存源视频、MCAP、地图和三张原图 SHA-256，原始相机位置/四元数/矩阵、SDK 朝向、时间戳、八候选逐帧分数与误差。

## 打分与图上证据

- 白线 V1/V2：从真实图中标注的前方及左侧车棚立柱，用于方向评分。
- 黄线 H1/H2：另一立柱及后方立柱，未参与评分，作为留出方向检查。
- 粉线：该候选预测的世界竖直方向，通过所标线段中点，便于直接比较斜率。
- 绿色方向标记：独立 SCS SDK heading 的车身水平前向。图像明确是正前挡风玻璃视角，因此该方向应位于主画面前方；它不是虚构的物理目标点。
- 青色：原生/校准道路中心线；紫色：地图 UI 地面几何参考；橙色：原生交通/停放车辆包围盒。它们可能被建筑、驾驶室或镜面遮挡，因此**不参与可见性或 IoU 评分**。

没有捏造自车外形尺寸。协议中没有真实自车 bbox 尺寸，工具只能投影配置中的 W0–W3 轮位，以及记录中的 render 旋转作为辅助；当前车内这些轮位被遮挡或在画外。

分数是便于排序的启发式，不是概率：55% 拟合立柱方向、25% 明确声明的车内前向先验、20% 投影内参一致性。缺少的项不计权。内参项会比较记录 FOV 与矩阵隐含的垂直、当前宽高比水平、4:3 参考水平 FOV；4:3 的吻合是数值推断，未作为未经证明的引擎规则硬编码。

| 候选 | 平均分 | 留出竖直方向 RMS（约） | 审阅结论 |
|---|---:|---:|---|
| C1：转置、C→W、−Z | 0.9827 | 0.38 px | 粉线贴合四根立柱，前向先验符合 |
| C7：转置、W→C、+Z | 0.6606 | 0.88 px | 竖直方向单独仍可能；车身前向落在相机后方，与当前前视图矛盾 |
| C3：转置、W→C、−Z | 0.4554 | 20.75 px | 竖直收敛方向相反 |
| C0：不转置、C→W、−Z | 0.3188 | 5.70 px | 竖直收敛/尺度明显不符 |

C1 的矩阵隐含垂直 FOV 约 64.366°、4:3 参考水平 FOV 约 80°，与记录字段约 80°吻合。不转置的结果约为 28.262°/37.112°。

标注精度约 ±1.5 px，**0.38 px 是方向线残差，不能宣传成亚像素 3D 重投影精度**。C1 与 C7 的竖直残差都在该标记不确定性内；排除 C7 明确依赖这三张图的“车内朝前”先验，不能推广到追车、回头或自由相机。

## 产物与审阅

生成目录包含三张原图、24 张完整对照、三张八宫格、每帧原生世界标记 JSON 和完整报告：

- `source-000223.png`、`source-001117.png`、`source-002011.png`
- `comparison-000223.png`、`comparison-001117.png`、`comparison-002011.png`
- `frame-000223-c1.png` 等各候选全分辨率图
- `frame-*-world-markers.json` 与 `report.json`

已实际查看三张八宫格、C1 及 C0 全分辨率对照，并核对 C1 的立柱方向贴合。地图地面参考在 C1 中位于前方地面附近；C0 的同类参考明显上移，但 UI 地图几何没有实体边界保证，只作为辅助观察。遮挡车辆框没有被误计为成功或失败的检测框。

单测覆盖投影的齐次尺度、竖直消失点误差，并以保存的三个真实相机帧做回归：C1 留出方向误差 <1.5 px，C0 >4 px、C3 >15 px；同时保留“仅凭竖直方向不能排除 C7”的事实。

后续仍需：停车状态的车外/追车录制，至少三个不共线且确实可见的原生 3D 点（可先尝试 SDK 轮位）；不同相机朝向或 FOV；无遮挡车辆的真实像素框。需要这些数据后才能核验绝对点投影、bbox 包围和跨视角 E4。当前工具不会把车内方向证据自动写成全局默认约定，也不会标记 P0 整体验收通过。

## 可选 Rerun

真实 MCAP 的交互回放、视频帧关联和实时只读模式见 [RERUN.md](RERUN.md)。默认 feature 仍为空。推荐 `run.ps1` 的release构建；`view.ps1 -VerifyOnly`仅校验RRD，普通模式打开GUI，`-HeadlessScreenshot`启动独立无窗口渲染。实际优化版30秒读取1130快照/165图像，不能称为60Hz验收通过。
