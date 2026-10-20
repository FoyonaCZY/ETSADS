# 离线地图导出器

`tools/map_export` 是独立的 C# / .NET 10 工具，通过 TruckLib 读取本机 ETS2 的 HashFS 归档，输出 Parquet。运行时驾驶栈不链接 TruckLib，也不依赖 .NET。

**当前状态：真实地图导出、数值几何与文件完整性验收通过；全图驾驶验收尚未通过。** `accepted_for_autonomous_driving=false` 表示没有完成整张地图的游戏验收，不是禁止使用已核验的区域。`capabilities` 和逐车道字段区分中心线、名义导航宽度、物理边界、车辆通行及实时限速来源；运行时用显式区域策略决定允许哪类测试。不能把“成功读取全图”解释为 P1 高速驾驶、P3 城市驾驶或 P4 泊车已通过。

## 使用

从仓库根目录运行。PowerShell 包装脚本优先使用仓库的 `.tools/dotnet/dotnet.exe`，其次使用 PATH 中的 .NET 10 SDK；不会修改或安装全局 SDK。

```powershell
# 只检查归档分布、游戏版本和道路定义。
./tools/map_export/export.ps1 `
  -GameDirectory 'D:\SteamLibrary\steamapps\common\Euro Truck Simulator 2' `
  -Command inspect

# 全图 Parquet 导出。
./tools/map_export/export.ps1 `
  -GameDirectory 'D:\SteamLibrary\steamapps\common\Euro Truck Simulator 2'

# 调试用：只导出一个扇区，同时生成 Rust importer 的 JSON 中间格式。
./tools/map_export/export.ps1 `
  -GameDirectory 'D:\SteamLibrary\steamapps\common\Euro Truck Simulator 2' `
  -Sector '0,0' -RuntimeJson
```

所有游戏归档均只读，工具不启动游戏、不改存档。`-Output` 可以指定任意输出目录。`-Mod` 接受显式 HashFS 模组路径，**先列出的优先级最高**；工具不会猜测哪个存档启用了哪些模组。目前 ZIP 格式模组和解压文件夹模组尚未支持。

输出目录为 `output/<12位缓存键>`。缓存键包含游戏版本、按优先级排序的归档绝对路径、大小、文件修改元数据、扇区选择及导出器程序集 SHA-256。生成前清除旧 `manifest.json`，只有完整成功后才重新写入。消费者必须要求 manifest 存在；不能读取失败留下的半成品。

```powershell
$dotnet = './.tools/dotnet/dotnet.exe'
$tool = './tools/map_export/bin/Release/net10.0/etsads-map-export.dll'
& $dotnet $tool self-test
& $dotnet $tool verify --input './tools/map_export/output/<缓存键>'
./tools/map_export/preview.ps1 -ExportDirectory './tools/map_export/output/<区域缓存键>'

# 从已完成全图中切出区域，保留全图校准结果，直接供 Rust 加载。
& $dotnet $tool slice --input './tools/map_export/output/<全图缓存键>' `
  --x 24914.1997 --y 110417.5116 --z 6.02976 --radius 1000 `
  --output './tools/map_export/output/region-tromso-1km'
```

`verify` 会解码每个 Parquet 文件的所有行组、所有列，核对 manifest 计数，并输出 `verification.json`。`slice` 支持半径不超过 5 km，读取点表行组而不把全图六千万点载入内存，删除区域外连接，保留控制注解，并输出含 PE 游戏版本、源 manifest SHA-256 和缓存键的区域 manifest。`area-assets.json` 包含附近原生车位、服务点、UI 多边形候选和低曲率道路；`--z` 可选，用于考虑高度的距离排序。

`preview.ps1` 读取区域 `runtime-map.json`，生成可离线打开的 `preview.html`，支持平移、缩放、图层开关及车道查询。内嵌 JavaScript 已通过 `node --check`。浏览器工具拒绝本地 `file://` 导航，因此没有计入浏览器视觉验收，也没有通过其它协议绕过限制。

全图有六千万个采样点，JSON 仅建议用于小区域；默认全图只生成 Parquet。点表按 65,536 行分组写出，避免把全部采样点保存在内存。TruckLib 全图解码本身仍占用较多内存，本机峰值约 12 GB；低内存机器应使用 `-Sector` 分区导出。

## 便携构建

依赖版本固定于 `MapExport.csproj` 和 `packages.lock.json`：TruckLib 0.5.1、Parquet.Net 6.1.0。常规脚本使用锁定还原模式，依赖变化会显式失败。

```powershell
./.tools/dotnet/dotnet.exe publish ./tools/map_export/MapExport.csproj `
  -c Release -r win-x64 --self-contained true `
  -p:RestoreLockedMode=true -o ./tools/map_export/output/portable-win-x64
./tools/map_export/output/portable-win-x64/etsads-map-export.exe self-test
```

可复制整个 `portable-win-x64` 目录到其它 Windows x64 机器，无须在目标机器安装 .NET SDK。该自包含版本已在本机独立执行 self-test 和真实归档 inspect；其它机器尚未验收。游戏数据由用户本机提供，不包含于源码和便携工具中。

## 原生标识与坐标

道路、预制件、地图节点有 SCS 原生 `u64 uid`；一条道路内部的各车道和预制件内部的各导航曲线**没有独立原生 uid**。因此导出器明确区分：

- `owner_uid`、`node_start`、`node_end`、`nodes.uid`：无损保存原生 `u64`；Parquet 使用 unsigned 64-bit，不经过浮点转换。
- `lanes.id`：稳定复合字符串，例如 `11599701801447065:road:left:0` 或 `<prefab_uid>:prefab:3`。
- `node_start/node_end = 0` 表示内部曲线端点没有对应地图节点，不是有效节点 uid。
- Rust 导入层可以建立字符串到内部整数的映射，但该内部编号不得被标记为 SCS 原生 uid，也不得直接当成 ETS2LA GPS 的 `node_uid`。

全部位置为 **ENU 米**：`(SCS X, SCS Y, SCS Z) -> (X, -Z, Y)`。角度为 ENU yaw 弧度。原始数据的 float 精度不会因为输出 double 而增加。车道包围盒由采样几何重新计算，不依赖 TruckLib 读取的编辑器包围盒。

## 表契约（schema_version = 1）

| 文件 | 内容与关键字段 |
|---|---|
| `nodes.parquet` | `uid:u64, x/y/z:f64`，原生地图节点 |
| `lanes.parquet` | `id:string, owner_uid:u64, node_start/end:u64, kind, width_m:f32?, speed_limit_mps:f32?, rule, geometry_status, sector_x/z` 和 3D AABB；另有 `navigation_width_m, centerline_validated, ai_truck_access, speed_class, speed_source` |
| `lane_points.parquet` | `lane_id:string, point_index:i32, x/y/z:f64`，行驶方向排序，每约 1 m 一个点，保留首尾端点 |
| `lane_links.parquet` | `lane_id, successor_id, source, endpoint_gap_m`；PPD 内部用原生 `NextLines`，道路连接用同一原生节点加几何/航向约束 |
| `lane_neighbors.parquet` | `lane_id, neighbor_id, side, change_allowed:bool?`；解析车道变体和 `no_chng_in/out`，无法完整解析的许可为 null |
| `signals.parquet` | 预制件作用域下的控制组 ID、原生 owner、`semaphore_id`、位置、yaw、类型、profile、周期延迟；同一组可能有多个灯头位置，因此 `id` 不是该表的唯一行键 |
| `signal_lanes.parquet` | 信号灯到导航曲线的关联 |
| `slots.parquet` | PPD spawn 和地图 Company 的车位；Company 车位保留原生 node uid、公司、城市、装卸类型与难度 |
| `conflicts.parquet` | 原始 PPD Intersection 的 `CurveId/Position/Radius/Flags`，保留供审计 |
| `conflict_pairs.parquet` | 实际 `lane_a/lane_b`、世界冲突点、半径、类型、优先权关系、来源与几何一致性间隙；来源分原生分组和几何回退 |
| `lane_controls.parquet` | 信号/停车/让行/优先道路/未知控制语义、原生 modifier、additive 标记和控制组关联 |
| `stop_lines.parquet` | 信号、stop、yield、unknown 的曲线入口代理，含 `control_kind`，标记 `curve_start_proxy`；不是已验证的油漆停止线 |
| `drivable_polygons.parquet` | PPD 的 UI map display polygon，标记 `ui_map_display_only`；不是可供扫掠碰撞约束使用的物理路面 |
| `pois.parquet` | PPD 服务、停车及其它 spawn 点的类型和世界位置 |

`runtime-map.json` 的 lanes 保留 `id,owner_uid,node_start,node_end,center:[{x,y,z}],width,speed_limit,geometry_status,successors,predecessors`；新增 `navigation_width,centerline_validated,physical_boundaries_verified,truck_access,speed_class,speed_source`。顶层还包含 `conflicts,lane_controls,signals,signal_lanes,stop_lines`，slice 另外附加 `slots,pois,boundary_candidates`。

实体 `width` 和离线 `speed_limit` 当前为 null；道路的 `navigation_width=4.5` 有官方来源，不能当作实体路面边界。速度来源声明为 `runtime_telemetry_required`，运行时可用新鲜 `truck.navigation.speed.limit`，过期时保持关闭。`ai_truck_access` 来自 traffic lane 的 actor_access 与原生 prefab AllowedVehicles，是 AI 曲线通行能力，不等价于玩家免除所有交通法规。

## E8：真实归档分布与 API 结论

只读检查本机：ETS2 `1.61.1.9`，Steam build `25904183`。TruckLib 0.5.1 支持地图格式 907（1.59–1.61），本机读取成功。

当前 TruckLib 的 `Map.Open(path, IFileSystem, sectors)` 接受文件系统；`TruckLib.HashFs.AssetLoader` 已经实现多文件系统优先级查找及目录文件集合合并，无须重新实现 `IHashFsReader`。本机 111 个被加载归档中，以下 11 个包含 `/map/europe/*.base`，合并后为 **1101 个扇区**：

| 归档 | base 扇区数 |
|---|---:|
| base_map.scs | 282 |
| dlc_polar.scs | 187 |
| dlc_iberia.scs | 128 |
| dlc_fr.scs | 76 |
| dlc_greece.scs | 71 |
| dlc_balkan_e.scs | 70 |
| dlc_north.scs | 68 |
| dlc_east.scs | 60 |
| dlc_balt.scs | 60 |
| dlc_it.scs | 57 |
| dlc_balkan_w.scs | 42 |

原始归档文件名、大小、修改时间、计数见 `tools/map_export/evidence/archives.json`。本次归档中扇区数相加等于并集数；其它版本或有模组时仍需检查覆盖优先级。`Map.Open` 对季节性 external map 的支持限制沿用上游，导出范围为 europe 主地图，不宣称覆盖季节性独立活动地图。

## 几何校验与未通过项

预制件放置最初只用平面 yaw 拟合，导致带坡度、侧倾的预制件残差大，7946 个被拒绝。修正为控制点的三维 Horn 刚体旋转拟合，并为共线控制点增加投影 up 约束，避免自由 roll；保留 origin 索引重排、垂直方向检查。相同全图中拒绝数降为 **4**；区域 `(0,0)` 最大控制点残差约 **0.0020 m**。3D 刚体拟合有独立的旋转回归测试。

剩余 4 个 uid 的控制点不能在当前刚体模型下满足 0.5 m 残差阈值，分别为 `237818647155834894`（0.925 m）、`347904248488722432`（0.559 m）、`3143690369692336129`（0.516 m）、`3430027782552772476`（0.556 m）。进一步比较距离不变量，PPD 与实际节点的最大两点距离差分别为 **0.924、0.417、0.516、0.669 m**。任何刚体旋转都保持两点距离，因此这些输入不能由单一刚体变换精确重合；剩余问题已定位为实际节点与 PPD 形状不全等，而非再换一个 yaw/四元数约定就能解决。究竟来自编辑器非刚体变形还是模板变体，仍需游戏画面对照。工具拒绝这 4 个预制件，没有放宽阈值。

道路定义样本实际含 `lanes_left/right`、`road_size_*`、`road_offset`、`lane_offsets_*`、`shoulder_space_*`、模板引用和变体，并无通用 `lane_width` 字段。原始样式估计和原生 PPD 边界交叉对照发现 213,169 个观察的 P95 误差约 0.3125 m；个别单向道路误差达到 4.5 m。因此没有把初版估计直接当成验收通过。

当前导出先处理预制件，再处理道路。`road-offset-audit.json` 用原生 PPD 边界中心点、相同 node uid、航向、左右侧及准确车道数量做交叉核对，按 road look + variant + side + lane index 分组。整个 road item 按固定 UID 哈希保留 20% 作为留出集；训练组至少 5 个观察且 P95 离差不超过 0.05 m 才使用中位数校准；跨变体回退要求至少 10 个观察、离差不超过 0.03 m。满足条件的几何标记 `prefab_boundary_calibrated`，其余仍是 `estimated_road_look`。manifest 分别记录校准前后和留出集误差。

进一步追到官方文档：SCS 明确给出一条道路导航车道的名义宽度为 **4.5 m**。本机 41 种 `traffic_lane_data` 的实际字段只有 `speed_class/rank/traffic_rules`，没有实体宽度字段；`road_size/shoulder_space` 不是通用导航车道宽度。当前基础计算改用官方 4.5 m 名义间距，并解析 `lane_variants/model_variants/lane_variant` 选择。见 [SCS 导航连接检查说明](https://modding.scssoft.com/wiki/Games/ETS2/Modding_guides/1.15)。

最终全图留出实验包含 **43,191 个未参与拟合的观察**，校准后 P95 中心偏移误差为 **0.00724 m**，全部观察的 P95 为 0.00754 m，336,016 条道路车道应用了校准。该对照验证**中心线偏移**，相邻中心线间距仍不能证明实体车道边界宽度，因此 `width_m` 保留 null。实体边界需要区域模型/游戏画面核验；名义导航宽度可由显式区域策略单独授权低速中心线跟随，不自动开放依赖边界的变道和泊车行为。

`slot-audit.json` 按原生 Company 的父 prefab 关联车位，对比实际地图 node 位姿和 PPD spawn 点。全图有 13,391 个原生 Company 位姿，12,150 个可对照当前 PPD 候选，位置 P95 差为 0.06756 m；只在位置差不超过 0.05 m 时统计 yaw，P95 为 0.00231 rad。1,241 个没有匹配候选，少量位置差很大，原因可能含自定义/变体 spawn，不能臆断已还原。**泊车任务优先选 `node_uid != 0` 的 Company 原生位姿**，不以 PPD 推算点覆盖它们。25,672 是两类记录总数，不是去重泊车位总数。

### 冲突、优先权和信号

已从 [SCS 官方 PPD 内存布局](https://modding.scssoft.com/wiki/Games/ETS2/Modding_guides/1.9) 及 [BlenderTools 实际导出调用](https://github.com/SCSSoftware/BlenderTools/blob/master/addon/io_scs_tools/exp/pip/exporter.py) 核实原生交点分组：首记录的 `(flags & 0xF0) >> 4` 是组内曲线数，成员紧跟且 sibling 位为零；START/END 分别为 0x10000/0x20000，官方工具另定义 CROSS_SHARP=0x40000。实际序列化 Position 是 `t * curve.Length`，还原 Hermite 参数要相除，不能当世界坐标，也不能直接当归一化参数。

原生分组在几何检查通过后生成 `native_ppd_group` 冲突对；缺失/非法分组按真实采样线段求交生成 `derived_geometry`，排除直接拓扑邻接、共同分叉及高差超过 0.5 m 的立交，几何回退明确使用 4 m 保守搜索半径。全图得到 **297,242 个原生冲突对、16,531 个几何回退对**；4,994 次原生记录/组检查未满足条件，保留诊断而不伪装成功。

优先权解析 `TrafficRule` 及其 `additional_rules`，支持官方 priority 的 major/yield 类型、标准 stop，以及 PPD priority modifier。不能确定的 additive/缺失/同级关系保持保守，3,634 对是 unknown；遇到信号控制必须等实时状态，不让静态优先权覆盖红灯。停止目标为入口代理；冲突目标同时给出 upstream 停车半径，车辆前缘补偿由控制器处理。

`crates/map/src/traffic.rs` 提供强类型解析、来源 lane ID 查询、`upcoming_stop_controls` 和 `signal_decision`。只允许新鲜、空间唯一匹配且状态一致的 GREEN=8；未知、缺失、陈旧、冲突、黄灯或闪烁均保持停车。ETS2LA 动态 id 不与 `<prefab_uid>:signal:<SemaphoreId>` 数值直连；先将 sector 坐标转换 ENU 后按位置消歧。闸门 kind=2、OPEN=3 属于单独闸门状态机。

每次导出还生成 `junction-samples.json`：按 uid 的固定乘法哈希选最多 20 个具有至少 3 个控制点、6 条导航曲线的路口，包含原生 uid、模型、SCS 坐标、扇区、所有曲线点、原生后继、数值连接间隙及 `game_visual_check_passed:null`。可重复选择，供游戏内叠加/传送到附近核验，不将数值零间隙视为人工验收。

以下仍阻止 P1/P3/P4 驾驶验收：

1. 模板实际路面、真实车道宽度、变体覆写、允许变道规则尚未全部还原；限速尚未合并国家、车辆类别和路牌规则。
2. 数值冲突对和静态优先权已恢复，但 unknown、实时信号、路口标志及区域运行行为仍需游戏验收；不能仅靠静态“有优先权”放弃动态碰撞检查。
3. 停车线采用曲线起点代理，可行驶多边形采用 UI 图形；二者不能用来宣称真实扫掠边界满足要求。
4. 尚未完成 20 个路口的游戏内视觉连接确认、车道线投影叠加和 P1 连续 100 km 实车测试。P4 的静态车位读取通过不等于困难泊车成功率通过。

## 已保存的实测证据

`tools/map_export/evidence` 保存验收元数据，不提交全图几何文件。最终全图实测 **105.27 秒**（低于 10 分钟），导出 1,447,808 条车道、60,217,810 个采样点、164,115 个信号灯定位记录、25,672 个车位记录。Parquet 共 **1,187,420,742 字节**。重复车道 ID 为 0，悬空后继为 0；**14 张表**已逐页解码且计数一致。完整数字与程序集 SHA-256 以 manifest 为准。

当前 C# 自测覆盖 ENU 转换、样条端点/采样间距、航向、三维刚体拟合、原生与几何冲突、立交/邻接排除、主次优先权、信号覆盖静态规则及 `u64::MAX` Parquet 往返。Rust traffic 模块 4 项单测和显式真实导出 fixture 解析均通过。Release 构建零警告、零错误。所有通过结论仅限列出的软件/数据检查。

### 特罗姆瑟实时停车点对照

根据集成任务提供的真实遥测 SCS `(24914.1997,6.02976,-110417.5116)`，已只读生成 `output/region-tromso-1km`：1034 条车道、240 个冲突对、769 个控制注解、451 个信号灯定位、143 个停止目标，附 20 条车位记录（13 个原生 Company 位姿）、21 个 POI、68 个 UI 多边形候选。

当前 ServiceStation 的地图 ENU 坐标为 `(24914.19727,110417.51563,6.01630)`，与遥测点水平差 **0.00470 m**、高度差 **-0.01346 m**，是一个已核对的真实世界坐标锚点，不能外推为全图视觉投影验收。修理厂 owner `5674731589549621249` 没有 NavCurves，20 个 UI 图形也不能冒充场内可驾驶边界。最近已校准道路约 40.64 m，完整资料见该区域 `area-assets.json`。

## 上游与许可证

导出器以 GPL-2.0-only 分发，许可证全文见 `tools/map_export/LICENSE`。TruckLib 作为独立离线工具依赖，不链接进 Rust 驾驶进程。

- [TruckLib 源码与支持版本](https://github.com/sk-zk/TruckLib)
- [AssetLoader 多归档文件系统](https://github.com/sk-zk/TruckLib.HashFs/blob/master/TruckLib.HashFs/AssetLoader.cs)
- [TruckLib PPD 数据结构](https://github.com/sk-zk/TruckLib.Models/tree/master/TruckLib.Models/Ppd)
- [SCS 道路交通车道与模板变体文档](https://modding.scssoft.com/wiki/Games/ETS2/Modding_guides/1.30)
- [Parquet.Net](https://github.com/aloneguid/parquet-dotnet)

`tools/map_export/.upstream` 仅为本机 API 调查缓存，已忽略，不属于构建必需输入。

Rerun 回放所用的最新 slice 还包含 `nodes` 数组（原生 node UID 与 ENU 坐标，来自 `nodes.parquet`）。它用于解析原生导航路线，不能用偏移过的车道中心端点替代。Tromsø 的单独 `region-tromso-rerun` 切片包含 3,938 个原生节点。

### 区域邻接与边界契约（slice_format_version=2）

每条 JSON 车道保留字符串 `left` / `right` 邻居ID（没有内部邻居时为null）；`can_change_left` / `can_change_right` 只有在原始 `lane_neighbors.parquet.change_allowed == true` 且邻居在当前区域内时为true。`left_change_allowed_raw` / `right_change_allowed_raw` 保留原始可空许可，未知不提升为允许。它们只代表原生交通规则许可，不证明物理边界或完整变道安全。

顶层 `lane_neighbors` 保留两端都在区域内的原始邻接行。`boundary_neighbors` 保存跨界原始行；对应运行时邻居引用清空、该方向变道禁用。`boundary_links` 保存原始 `lane_id, successor_id, source, endpoint_gap_m`，另带 `region_direction=incoming|outgoing`；它们不是可执行连接。车道 `successors/predecessors` 仅含区域内部连接。这样裁掉的边界资料可审计，不再静默丢弃。相同 `(lane_id,side)` 的重复/冲突声明会被C#导出器拒绝。

Rust JSON/Parquet两条读取路径按同一规则裁剪。实测小sector `output/b780b8a70964` → `output/region-base-neighbors`，中心ENU(2000,-2000)、半径1800m：398车道、30,487点、384内部连接、50真实允许邻接、8跨界连接。Runtime代理已逐车道、逐点f64、邻接、许可及audit精确比对通过，日志 `artifacts/map-region-equivalence.txt`；audit仍为不可驾驶，未把数据解析通过计作道路验收。专门的 `output/region-boundary-neighbor` 是1m半径的原生包围盒裁剪fixture：1车道/316点、0内部邻接、2跨界邻接、1跨界连接，保留raw=true而执行许可false，用于验证越界邻居禁用。

Rerun直接复用 Runtime 的 `ExportedMap::from_path` 和 `ImportAudit`。之前的 `region-tromso-rerun` 与 `region-tromso-1km` 在9组已有几何/交通数组上逐项相等，证据在 `crates/visualization/evidence/map-interface-comparison.json`；本轮新 `region-tromso-neighbors` 进一步增加邻接与边界元数据，没有另造展示地图。

边界邻居fixture也已完成两读取路径逐字段严格相等回归：1车道/316点、0内部邻接与许可、1跨界link、2跨界neighbor、audit_driveable=false；日志为 `artifacts/map-region-boundary-neighbor-equivalence.txt`。

## 扇区读取与缓存

`partition` 将已有 Parquet 车道图转换为独立可读的 gzip JSONL 分片，原始文件不变：

```powershell
& ./.tools/dotnet/dotnet.exe run --project tools/map_export -c Release --no-build -- partition --input tools/map_export/output/7ded328a0122 --output tools/map_export/output/sector-europe-v1
```

输出目录必须不存在。每条 lane 仅存于其原生 `sector_x,sector_z` 分片，包含完整 source metadata、原生 owner/start/end UID、点、后继及可空变道许可。索引保存每个分片的**实际几何包围盒**，不是仅根据4km网格猜覆盖范围，因此跨扇区道路仍会被区域查询选中。`sector_index.json` 在所有分片完成、计数和 SHA-256 记录后最后原子发布；未完成目录不可读取。gzip 允许多个连续 member，Rust 使用 `MultiGzDecoder` 完整解码。车道分片自身范围为 lanes / lane_points / lane_links / lane_neighbors；交通注解可额外生成独立分片，见下节。停车位、UI多边形等其它表仍保留在源 Parquet，不宣称已接入移动区域缓存。

`etsads_map::sector::SectorStore::open(path, CacheLimits::default())` 自动识别分片目录、旧 Parquet 目录、旧 JSON slice 文件或包含 `runtime-map.json` 的 slice 目录。`load_2km(center)` 是与原 loader 一致的中心 ±2000m ENU 方形包围盒查询，保留相交车道的完整折线，不裁断成碎片；返回的车道引用构成区域内部子图，越界邻居清空且禁止该方向变道，原始许可仍保留。

默认缓存最多16片、256MiB解码车道数据，单片解码128MiB、解压256MiB、单行16MiB、区域200万点；另有至多4个预取请求。预算不包含调用方持有的返回地图、索引/缓存少量管理结构及受单行/解压上限约束的临时解析空间。LRU在解码新片前预留空间，读取检查相对路径、实际文件路径、长度、SHA-256、点数、车道数与覆盖范围；非法分片不进入缓存。宽度、限速未知值不因缓存而改变。

`prefetch_2km(center)` 将请求送给一个后台工作线程并立即返回 ticket；队列满明确报错，不阻塞提交者。`wait_prefetch(ticket, Duration::ZERO)` 可无等待检查是否完成。完成后 `load_2km` 复用缓存；移动后应按新区域重建 source-string 到 runtime ID 的映射，不能沿用旧数字 ID。旧 Parquet/JSON仍能同步读，旧格式的后台预取会提示先分片，避免隐蔽全图扫描。

本机实际全图分片耗时94.095s，1,447,808车道、60,217,810点、985片、1,562,401,303压缩字节。gzip JSONL 比原车道图 Parquet 占用更多存储，以换取直接按片读和不依赖 Arrow 的轻量 Rust 运行时。

同一 Release 程序、特罗姆瑟中心 `(24914.1997,110417.5116)`、2km查询实测如下；这是软件读取性能，OS页缓存状态未清空，不代表物理磁盘冷启动速度。

| 路径 | 逻辑压缩读取量 | 解码点数 | 用时 |
| --- | ---: | ---: | ---: |
| 旧全图 Parquet | 1,030,889,896 B | 60,217,810 | 38,466.13 ms |
| 分片首次读，4/985片 | 2,238,465 B | 79,645 | 39.14 ms |
| 同一区域缓存命中 | 0 B | 0 | 2.32 ms |
| 后台预取后读取 | 0 B | 0 | 1.86 ms |

两路径返回的1,964车道、52,696点、全部 metadata / native UID / 邻接与许可及 audit **严格相等**，没有放宽f64容差。缓存持有约4,810,751B解码车道数据。原Parquet字节是传给解析器的逻辑读量，不是简单用文件大小代替；指标不计索引/manifest。证据 `tools/map_export/evidence/sector-europe-tromso-benchmark.json`，可复现程序 `crates/map/examples/sector_benchmark.rs`。小基础区域也完成398车道/30,487点等价对照，索引与压缩开销在小输入上可能比直接读Parquet更大，不宣称所有文件都加速。

## 原生导航路线走廊 API

`crates/map/src/corridor.rs` 保留原生端点映射，不把稳定的 source lane 字符串、原生 owner/node UID、区域内临时 runtime ID 混为一谈：

```rust,ignore
let loaded = SectorStore::open(map_path, CacheLimits::default())?.load_2km(center)?;
let native = NativeRouteIndex::from_exported(&loaded.map)?;
let corridor = native.resolve(&ordered_node_uids, start_source_lane, start_s_m,
                              &CorridorPolicy::default())?;
// 按既有证据/RegionPolicy 导入同一区域后，验证并绑定当前 runtime 图：
let bound = corridor.bind(&native, &region.imported)?;
```

输入路线应从当前或下一个原生 checkpoint 开始；已经经过的前缀必须由调用方用观察到的进度裁掉，不凭最近点猜掉。`0` 表示预制件内部/未知端点，不接受为路线UID。相邻重复checkpoint折叠，非相邻重复保留绕行语义；u64全程无损。分支只能按输入节点顺序推进，内部PPD曲线通过原始后继连通。默认要求已校验中心线且明确允许卡车，未知访问权限阻断。某个要求节点不在当前区域返回 `NeedsMoreMap`，没有有序连通通路返回 `NoCorridor`，不会静默改走无关A*路线；搜索有状态预算。

输出 `steps` 包含 source/owner/start/end、进入/离开弧长和checkpoint进度，`allowed_channels` 是选中通路允许的出口集合，不允许执行器任意取其它后继。侧向转换只接受同owner、同方向原生起终节点、已加载邻居且原生明确许可的道路车道。每个必需变道输出最早/最晚开始弧长、计划完成弧长；连续多次变道会预留全部横移距离及出口缓冲。策略距离是显式规划约束，不冒充游戏实体车道边界。结果始终 `topology_only=true`；动态间隙、实体边界、曲率/扫掠和控制能力仍要独立核验。`bind` 还会检查当前运行时图实际允许这些出口/变道。

`artifacts/map-native-corridor-tromso.json` 是从现有真实地图原生端点构造的3步/4节点拓扑fixture，**不是已观察到的ETS2LA游戏导航路线**，也不是驾驶通过。`native_corridor_probe` 可重新生成。单测覆盖错误分支、高位u64、内部曲线、缺地图、未知卡车许可、未知变道许可、变道截止、连续两次横移、路线环与搜索上限；现场游戏路线与地图端点的端到端行为仍需真实导航任务验收。

### 移动窗口、全源节点索引和交通注解

已有车道分片可补入同源元数据，不需要重新扫描6000万点：

```powershell
& ./.tools/dotnet/dotnet.exe run --project tools/map_export -c Release --no-build -- partition-metadata --input tools/map_export/output/7ded328a0122 --output tools/map_export/output/sector-europe-v1
```

输出目录必须已有完成的车道索引、尚无traffic分片。工具校验原始manifest完全一致，生成五表 `conflict_pairs/lane_controls/signals/signal_lanes/stop_lines` 的985个gzip分片，以及轻量 `native_route_index.json.gz`；最后原子写 `traffic_index.json`，记录源表、缺失表、所有内容校验和及关联车道索引的SHA-256。所有路径相对bundle根目录，移动目录不影响绑定。实测交通分片44,156,266B，全源节点索引9,772,668B、353,894个原生节点，加载约226.99ms。

原生节点图只将内部零端点曲线沿实际后继折叠到下一个非零checkpoint；它验证UID存在和有向节点连接，不代替精确车道方向、访问权限或物理边界检查。运行入口使用：

```rust,ignore
let global = GlobalNativeRouteIndex::load_source(bundle_path)?;
let local_policy = existing_policy.for_loaded_map(&loaded.map)?;
let region = loaded.map.clone().into_commissioning_region(local_policy, fresh_limit)?;
let native = NativeRouteIndex::from_region(&loaded)?.for_region_graph(&region)?;
let plan = native.resolve_local_prefix(&global, &remaining_gps_uids,
    source_lane, current_s, &CorridorPolicy::default())?;
let bound = plan.corridor.clone().bind(&native, &region.imported)?;
```

`for_loaded_map`只与既有白名单取交集，保留原证据路径、速度和功能限制；交集为空拒绝，不将新加载车道自动授权。`for_region_graph`进一步限定为已经导入且授权的车道/通道。`from_region`保留源索引身份，局部和全源索引不一致会拒绝。已知远处后缀返回 `LocalCorridor.next_boundary_checkpoint/remaining_node_uids/must_stop_at_local_terminal`，执行器必须在当前终点前能停住，只有后续窗口成功接通才能继续；未知UID、节点对不连通、已加载但未获授权分别返回不同错误。不存在“截短后缀即导航成功”。临时数字ID可以随窗口变化，来源字符串必须用于重绑定和保存任务状态。

`AnnotationStore::open(path, AnnotationLimits::default())` 支持该交通索引、旧JSON slice及原始Parquet元数据；生产移动窗口使用 `load_for_region(center, 2000.0, &loaded)`，验证与车道源身份一致。返回 `RegionAnnotations { traffic, gate_candidates, source_id, missing_sources, stats }`，其中分片source_id为traffic索引原始字节SHA-256。默认最多16片、64MiB原始解压JSON缓存，单片32MiB、区域25万记录；返回的解析对象由调用方持有。缓存命中免IO但仍需解析JSON，所以不保证warm耗时比cold大幅降低。

跨窗口冲突保留整条原始关系及外部lane身份。`TrafficContext.reference_only_lane_ids`明确外部身份没有完整控制注解，不能让 `has_annotations_for_route`误认其已覆盖。重复关系会去重，冲突的重复声明会拒绝；缺表或缺覆盖会令 `annotations_present=false`。原生闸门类型可生成纯元数据 `GateCandidate`，但 `toll_classification_verified/activation_verified`始终false，不生成自动激活权限。

未解析profile的v1分片仅保留 `UseProfile` 和原始profile，因此当时的candidate数量0并不表示没有收费站。此缺口已由下述profile-v3静态解析补齐；老产物仍明确未知，不按名称猜测激活规则。

特罗姆瑟2km五表读取与原Parquet逐字段严格相等：414冲突、1429控制、313信号定位、222信号车道连接、222停止目标。4片读取100,700B、20.49ms，旧原表扫描2315.23ms；warm读取0B、19.74ms，缓存持有1,037,569B。证据 `tools/map_export/evidence/annotations-tromso-benchmark.json`；该区域没有实际收费候选，闸门未知与外部冲突保留另有明确合成单测。

`crates/map/testdata/navigation-sectors`是**合成存储fixture**，不会生成真实验证许可：窗口 `(0,0)`给[a,m]，窗口 `(6000,0)`给[m,z]，m的临时ID由2变为1，全源路线[1,2,3,4]跨窗口。集成回归通过真实文件读取、索引SHA校验、白名单交集、边界停车标记、下一窗口重绑、未知/未授权区别及交通缓存；它不代替实车导航验收。

### 独立道路区域入口

`into_commissioning_region`仍严格限制30km/h。独立 `into_road_region(RoadRegionPolicy, fresh_limit)`返回 `RoadRegionGraph { region, policy }`；`RoadRegionPolicy { authorization:RegionPolicy, evidence:RoadEvidence }`额外带中心线、实体边界、车辆通行的三个证据路径。该入口禁止名义导航宽度回退，要求每条源车道 `centerline_validated=true`、`physical_boundaries_verified=true`、真实 `width=Some(...)`、`truck_access=Some(true)`，且限制在有效fresh限速、已知地图限速和明确授权上限的最小值。未知地图限速仅回退fresh遥测，非法数值拒绝；结构上限60m/s不构成允许开到该速度。

地图层只验证结构和来源字段，不认证证据文件真实性。Host/Runtime必须验证三个文件的内容、hash、source scope，并保留RoadEvidence及车辆辨识动态速度范围，不能丢弃道路权限来源后只传一个放宽速度的旧RegionPolicy。现有真实全图仍缺实体边界/宽度，不能通过该道路入口；本轮没有生成或修改任何真实驾驶验证许可。

## UseProfile 静态语义与最终 profile-v3 数据

完整车道分片准备好后，给 `partition-metadata` 增加 `--game-dir PATH`（原导出有mod时也按原优先级给 `--mod PATH`），即可从实际归档解析profile并生成独立版本。工具要求游戏PE版本、归档顺序、大小、修改时间与原地图导出一致，记录实际使用的每个定义资产及所属归档完整SHA-256。常规 `export` 现在也会执行相同解析。

[SCS 1.9定义](https://modding.scssoft.com/wiki/Games/ETS2/Modding_guides/1.9)规定国家变体优先、按SemaphoreId选择数组元素、超过长度取最后值、继承只补缺属性；[SCS 1.32变更](https://modding.scssoft.com/wiki/Games/ETS2/Modding_guides/1.32)允许预制件实例覆盖profile。实现读取 `Prefab.SemaphoreProfile`，从原生国界node两侧国家标记及道路/预制件连接推导国家，冲突或无种子的组件不猜国家。国家不确定时，仅在所有可能国家变体给出相同类型时确定静态类型，否则保留Unknown。名字中的`toll/border`不参与分类。

真实 `tr_sem_prof.ab_toll` 的ID 0–9定义为 `barrier_manual`，ID10及数组末值回退为 `barrier_automatic`；这来自实际SII的`type[]`字段。`border_long`的缺失字段来自明确的`inherited`引用。继承环、缺定义、未知类型、国家变体冲突都保留未知。显式PPD类型沿用PPD自身定义。

`Signal.type`为有效类型，同时保留可空的 `raw_type/raw_profile/effective_profile/profile_unit/profile_country/profile_resolution/profile_evidence_id/resolved_id_map` 及 `timing_resolved`。老JSON字段缺失仍可读取；未解析的UseProfile/Unknown在 `signal_decision` 中明确Hold，闸门也不能因收到数值8而误走普通绿灯放行逻辑。`BarrierAutomatic/BarrierGas/TrafficLightVirtual`有独立枚举值。

TruckLib.Sii将括号float4解析为Quaternion对象，原始数值顺序为W/X/Y/Z；整数四元组是ValueTuple。这里按原SII四个时间值还原，不当旋转归一化。长度、元素类型、负数、非有限interval/cycle均拒绝；仅在可确定的profile变体间时长一致时令 `timing_resolved=true`。静态周期/间隔**不是运行中剩余时间**，不能覆盖实时信号状态。

最终可用目录是 `tools/map_export/output/sector-europe-profile-v3-final`，几何、车道索引与原始权限保持不变，旧v1/v2证据保留。`traffic_index.json` SHA-256为 `f9476726ca248cbfa4699c828553591cb32abd689bb89c3f3f92d8e1a6158c02`。它显式绑定两份补充文件：

| 文件 | 字节 | SHA-256 |
| --- | ---: | --- |
| signal-profile-evidence.json | 22,683,020 | 4924bf36934f3f92a6f39187e023f7ddb024c8aa588dd77435bccf29886e5382 |
| signals-resolved.parquet | 5,590,908 | df8dc16e0708a9faf3eb1a633171aca25463816e7a32b7bb1e3c0818d9dea220 |

`signal_profile_resolution`对象中的路径/hash/bytes各两组三项均必需。AnnotationStore与GlobalNativeRouteIndex打开时复验本地路径包含关系、大小、内容hash；实验校验器将两文件纳入完整bundle哈希。proof内部的SCS归档/asset路径是原始来源指纹，不要求重放机器安装游戏。

完整实际信号中，1829为BarrierManualTimed、2266为BarrierAutomatic、689为BarrierDistanceActivated、7557为BarrierGas；4017仍为Unknown，其余为普通灯/模型类型。原生profile解析输出不赋予收费类别或激活实测资格：`GateCandidate`通过真实signal_lanes和停止目标绑定source lane，携带owner/semID/profile/provenance及 `semantics_resolved`，而 `activation_verified` 和 `toll_classification_verified` 始终false。人工/自动/距离/加油条件必须由运行时分别处理，激活仍受现有实测与现场状态门槛约束。

`profile_probe`通过生产地图loader读取真实区域：ENU `(-844.3327026367188,-27943.841796875)` 周围2km的46个gate候选逐一匹配定义proof及有效类型；另一实际区域 `(-185.4271240234375,2172.609375)` 的6个Unknown信号在明确标为**合成绿灯观察**的逻辑检查中仍全部Hold。9317条唯一时长proof通过非负/有限值检查。报告在 `tools/map_export/evidence/profile-v3-real-probe.json` 与 `profile-v3-unknown-probe.json`。这验证静态解析和保守逻辑，**没有游戏激活、过闸或P2/P3阶段通过**。

最新局部检查：30项Rust地图测试通过（28单测+2跨窗口集成），严格Clippy通过，.NET零警告构建及profile继承/国家/实例覆盖/数组回退/两种tuple语法/非法时间回归通过。另修复路线搜索的终点代价漏算：终点成本包含剩余车道距离，横移成本包含真实纵向推进和正换道代价，直行不再通过往返变道“缩短”路线；必要出口变道仍通过。日志 `artifacts/map-profile-tests.txt`、`map-profile-clippy.txt`、`map-profile-dotnet-tests.txt`。

## 原生闸门候选到运行任务

`runtime::controlled_route::gates::derive_native_gates`以已绑定GPS走廊、稳定source lane ID和实际经过的entry/exit弧长选取候选，将同一原生semaphore group的有效heads组合为任务。未知、矛盾类型、重复任务、与真实投影不一致的手填覆盖均拒绝。`NativeNavigation::bind_mode`已调用此接口，不再要求把全部候选手填到`tolls`；候选不改变地图边界、车辆通行或激活权限。手动、自动、距离触发分别接入行为状态机；加油服务类型仍明确拒绝，属于未启用的服务交互。

任务保留原生观察点、投影闸门平面、profile proof ID及原停止目标来源。`GateDiagnostic`把付款观察、完整车身通行、尝试次数和当前状态分别记录；静态profile时间不输入实时闸门状态机。手填已审查收费覆盖仍要求实际paid事件。未分类原生闸门只能报告物理通行，不赋予收费站类别或P2付款验收资格。窗口换图保留稳定group/source语义和已开始交互的状态，不因临时数字ID重排重复激活。

真实 `sector-europe-profile-v3-final` 在ENU `(-844.3327026367188,-27943.841796875)` 的46个候选通过生产使用的几何投影函数，结果在 `artifacts/gate-real-candidate-audit.json`。该检查只读取原始中心线并生成 `geometry_only=true` 预览，没有补宽度/限速、没有构造驾驶许可；原始地图 `driveable=false`，候选 `activation_verified=false`，`input_published=false`、`phase_acceptance=false`。生产任务仍须通过原有实体边界、标定、来源hash、实测交互和supervisor门槛。

局部回归覆盖空原生任务模板经真实Runtime接线派生闸门、未授权/缺观察不激活、自动与距离类型不误当普通绿灯、挂车后端清空、临时lane ID重绑不重复脉冲，以及CLI发布时重新核对当前帧CLOSED。测试日志在 `artifacts/gate-runtime-tests.txt`、`gate-behavior-tests.txt`、`gate-actions-tests.txt`、`gate-permission-test.txt`；地图原投影回归在 `gate-map-projection-tests.txt`。这些为合成消息/模型及真实静态地图离线检查，**没有实车过闸或P2阶段通过**。
