# ETSADS 实现计划（步骤级）

> 配套 [DESIGN.md](DESIGN.md)。DESIGN 讲架构和"为什么"，这里讲"按什么顺序、具体怎么做、怎么验收"。
>
> 版本 0.1

**怎么读这份文档**

- 每一步的格式：**做什么 → 关键细节 → 验收**。验收不过，不进下一步。
- 标 ✅ 的事实已经对照源码或官方头文件核实过（SCS SDK 头文件、ETS2LA 各仓库源码、TruckLib 源码）。标 🧪 的是必须实测才能确定的未知项，统一列在 §1，P0 期间逐个解决。
- P0–P2 写到"照着能直接写代码"的粒度。P3 以后的步骤依赖前面的实测结果，粒度粗一些，到时再细化。

---

## 1. 待实测清单（P0 期间全部回答）

| # | 问题 | 为什么重要 | 怎么测 |
|---|---|---|---|
| E1 | 物理步频率是多少？是否固定？ | SDK 说明：遥测的一"帧"是一个物理步（固定步长），不是渲染帧。控制频率、积分步长都以它为准 ✅ | 记录 `frame_start.simulation_time` 的差分，看分布 |
| E2 | 语义输入设备的值和玩家自己的键盘/方向盘输入如何合成？取最大值、相加，还是覆盖？ | 决定人工接管怎么检测 | 脚本输出 steering=0.3，同时人手按方向键，对比 `truck.input.steering` 和 `truck.effective.steering` |
| E3 | 语义 `steering` 轴值到车轮转角的映射，受不受游戏里转向灵敏度、非线性等设置影响？ | 系统辨识的前提 | 固定车速，扫描轴值 -1..1，记录 `truck.wheel.steering` |
| E4 | ETS2LA 投影矩阵的存储约定（行主序还是列主序、相机坐标轴朝向）和视图矩阵的构造方式 | 所有投影、标注、叠加都依赖它 | §3.7 的"约定穷举法" |
| E5 | 随机道路事件（事故车、施工锥桶）会不会出现在 ETS2LA 的交通车或停放车列表里？ | 决定视觉障碍物检测要提前到哪个阶段 | 遇到事故现场时，对比画面和列表 |
| E6 | `cam1`–`cam8` 分别对应哪个视角？ | 视角自动切换、数据采集 | 逐个按，记录 `CameraState` 相对驾驶室的位置 |
| E7 | 同一游戏进程里，两个 DLL 各自 Hook `Present` 时谁先执行？ | 决定"先截图、后画叠加层"能不能靠两个 DLL 实现 | 见 §3.10 |
| E8 | DLC 地图扇区在哪些 `.scs` 文件里？TruckLib 的 `Map.Open` 能否从多个归档合并读取？ | 地图导出能否覆盖全部 DLC | §4.1 第 2 步 |
| E9 | 物理步与渲染帧的先后关系：输入事件回调（渲染帧开始）和 `frame_end`（物理步结束）在一个渲染帧内的顺序 | 控制链路延迟到底是 1 步还是 2 步 | 在两个回调里记录 QPC 时间戳，排出时序图 |

---

## 2. P-1：环境与仓库

### 2.1 安装工具链
1. 安装 **Visual Studio 2022 Build Tools**，勾选"使用 C++ 的桌面开发"（提供 MSVC 链接器和 Windows SDK）。
2. 安装 **rustup**，执行 `rustup default stable-x86_64-pc-windows-msvc`。现有的独立 GNU 版 Rust（`C:\Program Files\Rust stable GNU 1.98`）要从 PATH 里移除，免得两套工具链冲突。
3. 安装 **.NET 10 SDK**。TruckLib 的目标框架是 `net10.0` ✅。
4. 安装 **LLVM**（`bindgen` 要用 libclang）。设置环境变量 `LIBCLANG_PATH`。
5. ffmpeg 已经装好了（9.0.1，带 NVENC）。

**验收**：`cargo build` 能编出一个空的 `cdylib`；`dotnet --version` 输出 10.x。

### 2.2 仓库骨架
```
git init；建 Cargo workspace（结构见 DESIGN §15）
crates/sdk-sys      bindgen 生成的 SCS SDK 绑定（头文件放 vendor/scs_sdk，附 sdk_license.txt）
crates/shm          共享内存通用原语：命名映射、seqlock、环形缓冲
crates/msgs  crates/game  crates/bridge ……
```
- `crates/shm` 是第一个要写好的库，后面几乎所有进程都依赖它。内容：
  - `Mapping::create(name, size)` 和 `Mapping::open(name)`：封装 `CreateFileMappingW` / `MapViewOfFile`，名字统一加 `Local\` 前缀。
  - `SeqLock<T: Pod>`：写端 `seq += 1`（变成奇数）→ 写入数据 → `seq += 1`（变回偶数），中间用 `Release` 内存序。读端读 `seq`，奇数则重试；读完数据再读一次 `seq`，前后不一致也重试。
  - `Ring<T, N>`：N 个 `SeqLock<T>` 槽位，加一个 `latest: AtomicU64`。
  - 共享结构全部 `#[repr(C)]` + `bytemuck::Pod`，并用 `const _: () = assert!(size_of::<T>() == …)` 在编译期锁定布局。

### 2.3 游戏侧准备
1. Steam → ETS2 → 属性 → 测试版：记下当前版本号。版本锁定方式写进 `docs/GAME_SETUP.md`。
2. `config.cfg`：`uset g_developer "1"`、`uset g_console "1"`；渲染器选 DX11；无边框窗口 1920×1080；帧率上限 60。
3. 新建一个专用存档（profile），只用于开发，避免污染正常游戏存档。
4. 写 `tools/deploy.ps1`。流程：`cargo build --release` →（游戏正在运行时）提示先在游戏控制台执行 `sdk unload` → 把 DLL 复制到 `<游戏目录>\bin\win_x64\plugins\` → 提示执行 `sdk reload`。SDK 自带这几个控制台命令 ✅。Windows 会锁住已加载的 DLL，所以必须先 unload 再复制。

---

## 3. P0：基础设施

目标：数据能读出来、能录下来、能看到；脚本能控制方向盘；安全开关有效。

### 3.1 坐标与单位（`crates/msgs::frames`）

✅ SDK 约定：世界坐标 X 朝东、Y 朝上、Z 朝南；局部坐标 X 朝右、Y 朝上、Z 朝后；heading 取值范围 [0,1)，0 = 北，逆时针增加（0.25 = 西）；pitch 取值 [-0.25,0.25]，对应 ±90°；roll 取值 [-0.5,0.5]，对应 ±180°。

内部统一使用 ENU 世界坐标（x 东、y 北、z 上）和车体坐标（x 前、y 左、z 上）：

```rust
// 世界坐标：SCS (X,Y,Z) -> ENU (x,y,z)
fn scs_to_enu(p: DVec3) -> DVec3 { DVec3::new(p.x, -p.z, p.y) }
// 航向角：heading=0 是北，在 ENU 下 yaw = π/2；逆时针方向一致
fn heading_to_yaw(h: f32) -> f64 { (h as f64) * TAU + FRAC_PI_2 }
// 局部向量：SCS 局部 (X右,Y上,Z后) -> 车体 (x前,y左,z上)
fn scs_local_to_body(v: Vec3) -> Vec3 { Vec3::new(-v.z, -v.x, v.y) }
```

ETS2LA 插件的坐标约定 ✅（读源码确认）：
- 相机和信号灯给的是"扇区坐标"：`global = (cx*512 + x, y, cz*512 + z)`，其中 `cx`、`cz` 是 i16。
- 交通车和停放车给的已经是全局坐标（f32），而且是**包围盒中心**，不是车辆原点。
- 四元数是 SCS 世界坐标系下的旋转，转成 ENU 时同样要换轴：先转成旋转矩阵 R，再算 `M·R·Mᵀ`，M 是上面的轴置换矩阵。

**验收**：单元测试覆盖四个方向的 heading、ENU 与 SCS 的往返转换、四元数换轴（车头朝北时，车体前向 = ENU +y）。

### 3.2 桥接 DLL 骨架（`crates/bridge`，产物 `etsads_bridge.dll`）

1. `crate-type = ["cdylib"]`，导出四个函数（同一个 DLL 可以同时提供遥测和输入两套接口，SDK 说明里明确支持 ✅）：
   - `scs_telemetry_init(version, params)` / `scs_telemetry_shutdown()`
   - `scs_input_init(version, params)` / `scs_input_shutdown()`
2. 版本协商：遥测接口要求 `SCS_TELEMETRY_VERSION_1_01`，输入接口要求 1.00 ✅。不支持的版本返回 `SCS_RESULT_unsupported`，游戏会自动降级重试。
3. **两条硬规矩**：
   - 每个 `extern "C"` 入口和回调都包在 `std::panic::catch_unwind` 里，出错只记日志、返回错误码，panic 绝不能传进游戏进程。
   - 所有全局状态放进一个 `static STATE: Mutex<Option<State>>`。init 时创建，shutdown 时彻底清空。SDK 说明里写了 init/shutdown 可能反复发生多次 ✅。
4. 日志：init 参数里带有游戏的 `log` 函数，用它把日志写进游戏控制台，同时写一份到 `%LOCALAPPDATA%\ETSADS\bridge.log`。
5. 线程：SDK 规定所有回调都在游戏主线程上调用，而且接口不是线程安全的 ✅。所以 DLL 里不开任何线程，也不在回调之外调用 SDK 函数。

**验收**：游戏控制台出现 `etsads_bridge loaded, telemetry 1.01, input 1.00`。`sdk reload` 连续执行 10 次不崩溃。

### 3.3 遥测采集（bridge 内）

1. **订阅通道**（名字已核对 ✅）。值类型尽量选 double 版本：
   - 自车：`truck.world.placement`（dplacement）、`truck.local.velocity.linear/angular`、`truck.local.acceleration.linear/angular`、`truck.speed`、`truck.engine.rpm`、`truck.engine.gear`、`truck.displayed.gear`
   - 输入与生效值：`truck.input.steering/throttle/brake/clutch`、`truck.effective.steering/throttle/brake/clutch`
   - 制动：`truck.brake.parking/motor/retarder`、`truck.brake.temperature`、`truck.brake.air.pressure`
   - 车轮：`truck.wheel.steering`、`truck.wheel.on_ground`、`truck.wheel.substance`（带索引）
   - 灯光：`truck.light.*`、`truck.lblinker`、`truck.rblinker`、`truck.hazard.warning`、`truck.wipers`
   - 其他：`truck.cruise_control`、`truck.navigation.speed.limit/distance/time`、`truck.fuel.amount`、`truck.wear.*`、`game.time`、`local.scale`、`rest.stop`
   - 挂车：`trailer.N.world.placement`、`trailer.N.velocity.linear`、`trailer.N.connected`，N 取 0–9（SDK 最多支持 10 节 ✅）
2. **配置事件**（`SCS_TELEMETRY_EVENT_configuration`）：解析 `truck`（`wheel.position[i]`、`wheel.steerable`、`wheel.radius`、`hook.position`、`cabin.position`、`head.position`、`retarder.steps`、`gears.forward`）、`trailer.N`（`hook.position`、`wheel.position`）、`job`（`cargo.mass`、`destination.city.id`、`destination.company.id`、`delivery.time`、`job.market`）。放进 `ConfigBlock` 共享内存，每变一次 `config_seq += 1`。
3. **游戏事件**（`SCS_TELEMETRY_EVENT_gameplay`）：`player.fined`（属性 `fine.offence` 的取值有 crash / avoid_sleeping / wrong_way / speeding_camera / no_lights / red_signal / speeding / avoid_weighing / illegal_trailer / avoid_inspection / illegal_border_crossing / hard_shoulder_violation / damaged_vehicle_usage / generic ✅）、`job.delivered`、`job.cancelled`、`player.tollgate.paid`、`player.use.ferry`、`player.use.train`、`player.refuel.paid`。写进事件环形缓冲（256 条）。
4. **帧节拍**：
   - `frame_start` 回调：记下 `simulation_time`、`paused_simulation_time`、`render_time` ✅，再取一次 QPC。
   - `frame_end` 回调：`frame_id += 1`，把这一步的快照 `TelemetryFrame { header{frame_id, sim_time_us, qpc}, … }` 写进 `Local\ETSADS_Telemetry` 环形缓冲（64 槽）。
   - `paused` / `started` 事件：写入 `paused` 标志。
5. **顺带转发 ETS2LA 扩展状态**：在 `frame_start` 回调里打开并读取 `Local\ETS2LATraffic` 等映射，复制进我们自己的环形缓冲，打上上一个 `frame_id`。
   - 这样做不会读到半写的数据：ETS2LA 插件在它自己的 `frame_end` 回调里写入，而所有插件回调都在同一个主线程上依次执行 ✅，到下一步的 `frame_start` 时，上一步的写入一定已经完成。
   - 外部进程因此只需要读我们自己的、带帧号的数据，不用直接读 ETS2LA 那几块没有序号的共享内存。

**验收**：外部写一个 `etsads-cli telemetry` 命令（只读，打印 10 Hz 摘要）。开车时位姿、速度、挡位都在合理范围；`frame_id` 连续不丢号。顺手回答 E1。

### 3.4 ETS2LA 扩展状态的布局（`crates/game::ets2la`）

全部是紧凑布局（`#pragma pack(1)`），Rust 侧用 `#[repr(C, packed)]`，并断言大小 ✅：

| 映射名 | 单条记录 | 条数 | 总字节 |
|---|---|---|---|
| `ETS2LACameraProps` | `fov f32, x y z f32, cx cz i16, qw qx qy qz f32, m[16] f32, truck xyz f32, truck q[4] f32` | 1 | 128 |
| `ETS2LATraffic` | 车辆 54 B：`x y z, qw qx qy qz, width height length, speed, accel (f32×12), trailer_count i16, id i16, is_tmp u8, is_trailer u8`；后接 3 个挂车，每个 40 B：`x y z, q×4, w h l` | 40 | 6960 |
| `ETS2LAParkedVehicles` | `x y z, q×4, w h l (f32×10), id i16, is_trailer u8` | 40 | 1720 |
| `ETS2LASemaphore` | `x y z f32, cx cz i16, q×4 f32, type i32 (1 = 信号灯, 2 = 闸门), time_remaining f32, state i32, id i32` | 40 | 1920 |
| `ETS2LARoute` | `node_uid i64, distance_to_end f32, time_to_end f32` | 6000 | **96000**（README 里写的 80000 是错的，以源码 `static_assert` 为准） |

注意事项：
- 空槽位怎么识别：车辆 `id == -1`，或者整条记录全为零，两种都要判断。
- 车辆 `id` 是插件按对象指针分配的 i16。对象释放后指针可能被复用，所以跟踪器不能只认 id，还要叠加位置连续性校验（同 id 一帧内跳变超过 10 m，就当作新目标）。
- 信号灯状态：OFF=0、ORANGE_TO_RED=1、RED=2、ORANGE_TO_GREEN=4、GREEN=8、SLEEP=32（黄灯闪烁）；闸门：CLOSING=0、CLOSED=1、OPENING=2、OPEN=3 ✅。`time_remaining` 是当前状态还剩多少秒。
- 坐标精度：交通车坐标是 f32 全局坐标，在 50 万米处大约只有 3 cm 精度，够用。

### 3.5 输入设备（bridge 内）

1. 在 `scs_input_init` 里注册一个设备：`type = SCS_INPUT_DEVICE_TYPE_semantical`（=2），`name = "etsads"` ✅。语义设备的输入会直接映射到同名的游戏控制项（mix），不需要玩家做任何按键绑定 ✅。
2. 输入列表（名字取自游戏的 mix 名，ETS2LA 的 `controls_spec.txt` 里有完整清单；标"?"的是他们也没确定的，需要实测）：
   - 轴：`steering`（-1..1）、`aforward`（油门 0..1）、`abackward`（刹车 0..1）、`clutch`
   - 按住型：`horn`、`lighthorn`、`motorbrake`(?)
   - 单次触发型：`lblinker`、`rblinker`、`flasher4way`、`light`、`hblight`、`wipers0`–`wipers4`、`retarder0`–`retarder5`、`engbraketog`、`parkingbrake`、`activate`、`gearup`、`geardown`、`gear0`、`geardrive`、`gearreverse`、`cam1`–`cam8`、`attach`、`quicksave`、`pause`、`menu`，以及菜单导航 `j_up/j_down/j_left/j_right/selectfcs/back`（留给 P6 用）
   - 上限是 400 个 ✅。
3. 控制命令共享内存 `Local\ETSADS_Control`：
   ```rust
   #[repr(C)] struct ControlCmd {
       seq: u64, write_qpc: u64, source_frame_id: u64,
       steer: f32, throttle: f32, brake: f32, clutch: f32,
       holds: u64,             // 按住型按钮的位图
       pulses: [u32; 64],      // 单次触发型：计数器加 1 表示按一下（参考 ETS2LA 的做法）
   }
   ```
4. **安全闸门**共享内存 `Local\ETSADS_Gate`。由安全监督进程写入，主栈不能写：
   ```rust
   #[repr(C)] struct Gate { seq: u64, heartbeat_qpc: u64, enabled: u32,
                            aeb_brake: f32, max_throttle: f32, max_steer_rate: f32 }
   ```
5. 输入事件回调的逻辑（每个渲染帧开始时调用；带 `first_in_frame` 标志的那次调用里算出整帧要发的事件，之后逐条返回，没有了就返回 `SCS_RESULT_not_found` ✅）：
   ```
   gate 失效（enabled == 0，或心跳超过 300 ms） → 所有轴归零，按住型按钮全部松开
   命令过期（距 write_qpc 超过 200 ms）        → 油门 0；刹车在 0.5 s 内斜坡升到 0.3；超过 1 s 后全部释放
   否则                                         → 用命令值，再经 gate 处理：
       刹车 = max(刹车, aeb_brake)；aeb_brake > 0 时油门强制为 0
       油门 = min(油门, max_throttle)
       转向变化率按 max_steer_rate 限幅
   只发生变化的事件；遇到 first_after_activation 时全部重发一遍 ✅
   ```
6. 在 `ETSADS_Control` 的状态区回写 `applied_seq`、`applied_qpc`、`frame_index`，外部进程据此测量命令从写入到被游戏采用的延迟。

**验收**：`etsads-cli steer 0.3` 能让方向盘转过去；进程退出 200 ms 后车辆开始轻刹；`etsads-cli gate off` 能立即释放控制。顺手回答 E2、E3、E9。

### 3.6 外部读取层（`crates/game::adapter`）

1. `GameAdapter::poll()` 以最新遥测帧为基准，组装一份 `GameSnapshot { ego, trailers, camera, agents_raw, signals_raw, route_raw, events, config }`。所有部分都必须属于同一个 `frame_id`，不一致就等下一帧。
2. 自车状态：位姿从 SCS 转成 ENU，速度、加速度转到车体坐标；铰接角 = 牵引车航向 − 挂车航向，归一化到 (-π, π]。
3. 局部原点：首次运行时取自车位置所在的 1 km 网格中心作为原点；离开原点超过 2 km 就换新原点，并通知所有下游重建缓存。下游一律用 `f32` 局部坐标，`f64` 只在这一层出现。
4. 不变量检查：四元数模长约为 1、速度小于 70 m/s、时间单调递增。违反时计数并报警，不 panic。

### 3.7 相机与投影（`crates/game::camera`）

1. 每帧读取 `CameraState`：`pos = cx*512 + xyz`、四元数、投影矩阵、FOV，以及渲染插值后的车辆位姿。
2. **E4 约定穷举法**：投影约定可能的组合有：矩阵是否转置（2）× 四元数作为"相机→世界"还是"世界→相机"（2）× 相机前向是 -Z 还是 +Z（2），共 8 种。
   - 取 100 帧录制数据，用每一种组合把交通车包围盒中心投影到画面上。
   - 打分：投影点落在画面内的比例，加上投影出的框和车辆实际像素区域的重合度（P0 阶段可以先人工看 3 张截图）。
   - 得分明显最高的就是正确约定。把它写成一个函数，配上回归测试：固定一帧数据和期望的像素坐标。
3. 视角分类（DESIGN §6.7）：把相机位置变换到驾驶室坐标系（以 `cabin.position` 为参考）。在驾驶室包围盒内为 `Interior`；在车后上方且离车 5–30 m 为 `Chase`；其余为 `Other`。相对位姿一帧内变化超过 0.5 m 或 10°，记为一次 `switched`。顺手回答 E6。

### 3.8 画面读取（`crates/game::capture`）

P0 先读 ETS2LA `capture` 插件的共享内存。协议已从源码确认 ✅：

- 控制块 `Local\ETS2LA_Capture_Control`（4096 B）：`magic = 0x32435445`，`version = 1`，偏移 64 处是 `generation`（u32），偏移 72 处是 `latest_frame`（u64），偏移 128 起是 32 个读者槽位，每个 64 B，内容为 `{id, heartbeat_ms}`。
- 数据块 `Local\ETS2LA_Capture_Data_{generation}`：头部 4096 B，依次是 `width`、`height`、`stride`、`format`（DXGI 格式编号，87 = BGRA8）、`slot_stride`、`slots_offset`。
- 第 i 帧所在槽位：`slots_offset + (i % 6) * slot_stride`。槽位头 64 B，`seq` 为偶数表示数据稳定；像素从槽位偏移 256 处开始。
- 读者必须先在读者表里登记，方式是对 `id` 做 CAS，写入 `(pid << 32) | counter`，然后每 250 ms 写一次心跳。插件可能只在有读者时才截图。
- 新帧通知：事件 `Local\ETS2LA_Capture_Frame0/1`，按帧号奇偶交替使用。
- 读取流程：读 `seq` → 复制像素（或在 4 帧时间内处理完）→ 再读 `seq`，前后一致才算有效。
- 时间对齐：槽位头里的 `timestamp_qpc` 是游戏调用 `Present` 时的 QPC；桥接 DLL 也给每个物理步记了 QPC（§3.3）。按 QPC 找最近的物理步，就能把画面对齐到 `frame_id`。

**验收**：`etsads-cli frame-dump` 能存出 PNG，打印的 QPC 和帧号对得上。

### 3.9 录制器（`apps/recorder`，独立进程）

1. 结构化数据写 MCAP（`mcap` crate），消息用 protobuf 编码（`prost`），`.proto` 文件放在 `crates/msgs/proto/`。用 protobuf 是因为字段可以演进，Foxglove 也能直接解码。
   - 每路数据一个通道：`/telemetry`、`/config`、`/events`、`/camera`、`/agents_raw`、`/signals_raw`、`/route_raw`、`/control_cmd`、`/gate`；后期加 `/scene`、`/trajectory`。
2. 视频：启动 `ffmpeg -f rawvideo -pix_fmt bgra -s 1920x1080 -r 60 -i - -c:v hevc_nvenc -preset p4 -cq 23 out.mp4`，把 BGRA 像素经 stdin 管道送进去。另外把 `(video_frame_no, capture_frame_index, qpc, physics_frame_id)` 写进 MCAP 的 `/video_index` 通道。
3. 按 30 分钟或 4 GB 切分文件；每次运行生成一个独立目录，例如 `runs/session-example/`，里面放 `meta.json`（游戏版本、DLC、mod 列表、车辆配置、代码的 git 提交号）。
4. 回放工具 `etsads-cli replay <run>`：按时间轴读出 MCAP + 视频，以 `GameSnapshot` 流的形式提供给下游，和实时运行走同一套接口。这是后面离线仿真的基础。

**验收**：录 1 小时人类驾驶，回放时帧号连续，视频和数据的对齐误差不超过 1 个渲染帧。

### 3.10 可视化与叠加层

1. **Rerun**（`rerun` crate）：实时和回放模式下都推送以下内容：
   - `world/ego`：牵引车和挂车的包围盒
   - `world/agents`：交通车的框和 id
   - `world/signals`：信号灯位置，按状态着色
   - `world/route`
   - `camera/image`：画面，设置 `Pinhole` 相机模型，让 3D 物体直接投影显示在图像上
   - 时间序列：速度、转向、油门刹车、命令延迟
2. **P0 阶段的叠加层用外置透明窗口**（`egui` + `winit`）：窗口无边框、置顶、可鼠标穿透（`WS_EX_LAYERED | WS_EX_TRANSPARENT`），跟随游戏窗口的位置和大小，用当帧相机矩阵画投影。要求游戏是无边框窗口模式。比游戏内绘制晚 1–2 帧，调试够用了。
3. **E7 实验**：游戏内 AR 叠加必须满足"先截图、后画叠加层"。两个 DLL 各自 Hook `Present` 时，执行顺序取决于安装顺序，不可控。所以终态的做法是**自研 `etsads_gfx.dll`，用一个 Hook 依次完成截图和叠加绘制**，替换掉 ETS2LA 的 capture（P1 末尾做，见 §4.9）。

### 3.11 安全监督 v0（`apps/supervisor`，独立进程）

1. 每 10 ms 循环一次：读遥测和 `ControlCmd`，写 `Gate`，带心跳。
2. 规则：
   - 主栈心跳（`ControlCmd.write_qpc`）超过 200 ms 没更新 → 关闭 `enabled`（DLL 会自己执行斜坡刹车再释放）
   - 轮询 `F8` 按键边沿 → 切换 `enabled`（r9 实测：Ctrl/Shift 是游戏默认换挡键，且游戏前台时 `RegisterHotKey` 收不到，原 Ctrl+Shift+F12 方案作废）
   - 游戏暂停或进入菜单 → 关闭 `enabled`
   - 人工接管检测（按 E2 的实测结果实现）：玩家输入超过阈值 → 关闭 `enabled`，并发出提示音
3. 监督进程用 `HIGH_PRIORITY_CLASS` 运行。它只依赖 `crates/shm` 和 `crates/msgs`，不链接主栈的任何模块。

**验收**：主栈被 kill、热键、暂停这三种情况，都能在 300 ms 内让控制释放（通过录制数据确认）。

### 3.12 第一次闭环

1. 找一个空旷场地（开发存档里事先停好的位置，或者夜间车少的直路）。
2. `apps/etsads` 的最小版本：读快照，用 PD 控制器跟踪一个"固定曲率圆"（参考圆在启动时按当前位姿生成），油门用 PI 控制把车速维持在 30 km/h。
3. 在 Rerun 里画出参考圆、实际轨迹、横向误差。

**P0 退出条件**：§1 的 E1–E9 全部有结论，写进 `docs/FINDINGS.md`；1 小时录制能完整回放；交通车框在车内、车外视角下都能准确叠加；闭环跟圆时稳态横向误差小于 0.5 m。

---

## 4. P1：高速 L1（车道保持 + ACC）

### 4.1 地图导出器（`tools/map_export`，C# / .NET 10）

**许可证** ✅：TruckLib 是 **GPL v2**。导出器作为独立程序，自身受 GPL v2 约束；它导出的数据文件不受影响，主项目也不链接它。

1. 依赖：NuGet 包 `TruckLib`（HashFs、ScsMap、Sii、Models 四个包）和 `Parquet.Net`。TruckLib 当前支持的地图格式版本是 907，对应游戏 1.59–1.61 ✅。先确认你的游戏版本在这个范围内。
2. 打开归档：`HashFsReader.Open(path)`，对 `base.scs`、`base_map.scs`、`def.scs`、`dlc_*.scs` 逐个打开 ✅。
   - E8 实验：用 `GetDirectoryListing("/map/")` 看各个 DLC 归档里有哪些扇区文件。
   - 如果 `Map.Open("/map/europe.mbd", reader)` 只接受一个读取器，就自己实现一个**叠加读取器**：实现 `IHashFsReader` 接口，按"mod > DLC > base"的优先级依次查找条目。
3. 读地图：`Map.Open("/map/europe.mbd", reader)` ✅，加载完调用一次重算包围盒（TruckLib 加载时不会算包围盒 ✅）。遍历 `map.MapItems`，找出 `Road` 和 `Prefab`；节点用 `map.Nodes`。
4. **道路车道**：
   - 先写一个小工具，把 `def/world/road_look*.sii` 用 `TruckLib.Sii` 解析后，dump 出一个高速道路样式（road look）的全部字段，人工确认车道相关字段的名字和含义（左右车道数、车道宽、偏移、中央分隔、路肩）。truckermudgeon/maps 的 parser 里有处理这些字段的现成代码，可以对照。
   - 道路中心线：两个节点之间用三次 Hermite 曲线，切线方向取节点的朝向，切线长度取两节点间的距离（SCS 的惯例，用 TruckLib 的 Road 几何函数交叉验证）。
   - 每条车道 = 中心线按车道偏移平移后，每 1 m 采样一个点。
5. **预制件车道** ✅：`PrefabDescriptor.Open(ppd路径, reader)`，其中 `NavCurves` 列表的每条曲线包含 `StartPosition/EndPosition`、`StartRotation/EndRotation`、`Length`、`NextLines/PreviousLines`、`SemaphoreId`、`TrafficRule`、`NavNodeIndex`。
   - 按预制件在地图上的放置（原点节点、旋转、所选变体）把曲线变换到世界坐标，同样用 Hermite 曲线采样。
   - `Semaphores` 列表（`Type`、`Position`、`SemaphoreId`、`Intervals`、`CycleDelay`）→ 信号灯表。
   - `SpawnPoints` 列表（类型包括 `UnloadEasy/Medium/Hard`、`Trailer`、`CompanyPoint`、`GasStation`、`Parking`、`TruckStop`、`WeighStation`、`ServiceStation` 等）→ 场站和服务点表。
   - `Intersections` 列表（`CurveId`、`Position`、`Radius`、`Flags`）→ 路口冲突表。
6. 输出 Parquet 表（字段见 DESIGN §5.1）。所有 id 都用 SCS 原生的 uid（u64），这样才能和 ETS2LA 路线里的 `node_uid` 对上。
7. 缓存：输出目录名 = `sha256(游戏版本 + 归档列表及其修改时间 + mod 列表)` 的前 12 位。

**验收**：导出全图耗时小于 10 分钟；用 Rerun 画出某个城市的全部车道，目测没有断裂或错位；随机抽 20 个路口，车道连接关系正确。

### 4.2 车道图（`crates/map`）

1. 加载 Parquet → `Lane { id, polyline: Vec<Vec2> (局部坐标), z: Vec<f32>, width, speed_limit, successors, predecessors, left, right, can_change_left/right, kind }`。
2. 道路车道和预制件曲线的拼接：在共享节点处，满足"端点距离 < 0.5 m 且航向差 < 10°"就连接。连不上的端点记录下来作为导出问题，回头修导出器。
3. 空间索引：用 `rstar` 建 R*-树，每 20 m 一段线段作为一个条目。按自车所在扇区懒加载周围 2 km，后台线程预取。
4. 查询接口：`nearest_lanes(pos, radius)`、`project(lane, pos) -> (s, d, heading)`、`lane_sequence(start, route)`。

### 4.3 地图匹配（`crates/map::matcher`）

1. 候选：5 m 半径内、航向差小于 30°、高度差小于 3 m（用来区分立交上下层）的车道。
2. HMM：发射概率 ∝ exp(-d²/2σ²)，σ = 0.5 m；转移概率按车道图连通性计算，s 方向推进量与车速一致的转移得分更高。用滑动窗口 Viterbi，窗口取 2 s。
3. 输出 `LaneState { lane_id, s, d, heading_err, confidence }`。置信度低于 0.6 时，下游降级（只保持车道，不变道）。

**验收**：一条 50 km 高速线路，匹配错误率小于 0.1%（与人工标注的车道序列对比）。

### 4.4 系统辨识（`apps/sysid`）

前提：要有一段空旷的路，脚本自动执行：
1. **转向**：车速 20/40/60/80 km/h 下，轴值依次取 ±0.05、±0.1、±0.2、±0.4，每档保持 3 s，记录稳态横摆角速度 r，算出曲率 κ = r / v。拟合表 `κ(steer, v)`（二维插值）。
2. **执行延迟**：在转向阶跃信号和横摆角速度响应之间做互相关，得到纯延迟 τ。再拟合一阶滞后时间常数，作为转向执行器模型。
3. **纵向**：油门 0.2–1.0 阶跃、刹车 0.1–1.0 阶跃、松开踏板滑行，分别在不同车速和挡位下做。用加速度残差回归总质量（货重从配置事件里读，牵引车自重也通过回归得到）和坡度（由 pitch 算出）。最后得到表 `a(v, throttle | brake, mass, grade)`。
4. 结果存到 `configs/vehicles/<品牌>_<型号>_<挂车配置哈希>.toml`。

### 4.5 横向控制（`crates/control::lateral`）

1. **v1 纯追踪 + 前馈**：预瞄距离 L = clamp(1.2·v, 8 m, 40 m)。目标曲率 κ* = 2·sin α / L，加上参考线曲率作为前馈；通过 `κ(steer, v)` 的逆表换算成轴值。对延迟 τ 做补偿：先把自车状态按当前速度和横摆角速度前推 τ，再算控制量。
2. **v2 LTV-MPC**：状态取横向误差 e_y、航向误差 e_ψ、转向状态 δ；输入为 δ 的变化率；时域 2 s、步长 0.1 s。代价项包括 e_y、e_ψ、曲率跟踪、Δδ。约束包括 |δ|、|δ̇| 和横向加速度上限。每个控制周期用 `clarabel` 求解一次 QP，目标耗时小于 1 ms。

### 4.6 纵向与 ACC（`crates/planning::speed` 和 `crates/control::longitudinal`）

1. 前车选择：把交通车投影到本车道及其后继车道上，取 s 方向最近、且横向距离 |d| < 车道宽一半 + 0.3 m 的那一辆。挂车同样参与计算，取最后一节挂车的尾部作为前车的车尾。
2. 期望加速度（IDM 形式，按卡车调参）：期望车速取 min(限速, 弯道限速)。弯道限速 = sqrt(a_lat_max / κ)，a_lat_max 满载取 1.2 m/s²、空载取 1.8 m/s²，曲率取前方 300 m 内的最大值。时距 2.0 s，最小间距 8 m。舒适减速 1.5 m/s²，紧急减速 4 m/s²。
3. 逆模型：用 `a(v, cmd, mass, grade)` 表反查油门或刹车量，外加 PI 修正残差。长下坡时优先用缓速器档位，按"所需减速度 → 档位"查表；刹车温度超过阈值时提高缓速器档位。
4. 限速来源：`truck.navigation.speed.limit`（游戏给的当前限速）和地图里的车道限速，取两者较小值。

### 4.7 主循环（`apps/etsads`）

```
线程 A（物理步驱动，约 60 Hz）：poll 快照 → 地图匹配 → 横纵向控制 → 写 ControlCmd
线程 B（10 Hz）：            场景构建 → 速度规划（ACC）→ 生成参考轨迹 → Arc 交换给线程 A
线程 C：                    Rerun 推送、egui 控制面板（启用开关、设定车速、状态灯）
```

### 4.8 测试与指标

- 回放回归：每次提交都在 10 段录制数据上开环跑一遍匹配和控制，比较控制量与人类驾驶的差异分布，检查有没有 NaN 或越界。
- 实车测试：固定 3 条高速线路，统计 km/接管、车道中心偏差 RMS、跟车时距分布、最大横向加速度、加加速度。

### 4.9 自研 `etsads_gfx.dll`（P1 末尾）

1. 作为遥测插件加载进游戏进程（只导出 `scs_telemetry_init/shutdown`，不注册任何通道）。init 时安装 Hook，shutdown 时卸载。
2. 找到 `Present`：创建一个临时的 D3D11 设备和交换链，从虚表里取出 `IDXGISwapChain::Present` 的地址（虚表第 8 项，从 0 开始数），再用 detour 库挂钩。具体用哪个 crate 在 P1 选型：候选是 `retour` 和 MinHook 的 Rust 绑定。
3. Hook 里的顺序：
   - ① `GetBuffer(0)` 拿到后台缓冲区，`CopyResource` 到 6 个 staging 纹理组成的环里；
   - ② 在上一帧的 staging 纹理上调用 `Map`，拷进共享内存，协议与 §3.8 相同，但改成我们自己的映射名并加上物理步 `frame_id`；
   - ③ 用 Dear ImGui 画叠加层。可选 `hudhook` 的 DX11 渲染部分，或 `imgui` crate 加一个自写的 DX11 渲染器；
   - ④ 调用原始的 `Present`。
4. 卸载要安全：维护一个"正在执行 Hook 的线程数"计数器，shutdown 时先禁用 Hook，再等计数归零，然后才允许 DLL 卸载。
5. 完成后在 `plugins` 目录里移除 ETS2LA 的 capture，项目就不再依赖 GPL 组件（地图导出器除外，见 §4.1）。

**P1 退出条件**：高速上连续 100 km 无接管；车道中心偏差 RMS 小于 0.3 m；跟车时距的 95% 分位数不低于 1.8 s。

---

## 5. P2：高速 L2（路线、变道、汇入、收费站）

### 5.1 路线
1. 读 `ETS2LARoute` → 节点 uid 序列 → 在车道图里转成"可行车道序列"，在每个路段上标出允许所在的车道集合（例如前方要下匝道，就必须在最右侧车道）。
2. 自研车道级 A* 只在两种情况下用：游戏没有设置路线时，或者要比较备选路线时。代价 = 行驶时间 + 换道惩罚 + 收费 + 卡车难度惩罚（转弯半径小于 15 m、路宽不足）。
3. 输出"必须完成变道的最后位置"，即距离路口还剩多少米时必须已经在目标车道上，作为行为层的硬约束。

### 5.2 预测 v1（`crates/prediction`）
1. 每个目标匹配到车道上（复用 §4.3 的 HMM，每个目标一个实例）。
2. 沿车道图做常加速度外推 4 s，加速度用最近 1 s 的拟合值，限制在 [-3, 1] m/s²。在分叉处，按"目标与各分支的横向偏差趋势 + 是否在路线上"分配概率。
3. 输出 `PredictedAgent { modes: [(prob, Vec<Pose2 @ 0.2 s>)] }`。

### 5.3 行为层框架（`crates/planning::behavior`）
1. 场景状态机：`trait Scenario { fn applicable(&Scene) -> bool; fn plan(&mut self, &Scene) -> BehaviorGoal; fn done(&Scene) -> bool }`。
2. P2 实现的场景：`LaneFollow`、`LaneChange{dir}`、`Merge`、`Exit`、`TollBooth`。
3. 变道决策条件：路线需要变道，或者前车比期望速度慢 15 km/h 以上并持续 10 s，同时目标车道满足安全条件（下一条）。

### 5.4 Frenet 采样规划器（`crates/planning::frenet`）
1. 参考线：路线车道中心线，变道时取目标车道中心线。
2. 采样：横向终点 d ∈ {当前车道中心, 目标车道中心, 两者中点}；完成时间 T ∈ {4, 5, 6, 8} s；目标车速 ∈ 期望车速 ± {0, 5, 10} km/h。横向用五次多项式，纵向用四次多项式。
3. 碰撞检查：把牵引车和各节挂车的包围盒沿轨迹逐点放置（挂车位姿用 DESIGN §9.3 的运动学方程积分得到），与预测目标的每个模态逐时刻做 OBB 相交检测，空间上留 0.5 m 余量，时间上留 0.3 s 余量。
4. 代价 = 加加速度 + 横向偏差 + 进度损失 + 风险（距离的倒数，按模态概率加权）。
5. 变道安全条件：目标车道后车按"假设它减速 2 m/s²"计算，与我方之间仍保持至少 1.5 s 时距；前车同理。
6. 转向灯：变道开始前 3 s 打灯，变道完成后关灯。打灯期间对方车让出空间时，允许提前开始变道。

### 5.5 收费站 / 边检
1. 识别：路线前方 500 m 内出现收费站类预制件（闸门信号灯类型为 `BarrierManualTimed/BarrierDistanceActivated/BarrierAutomatic` ✅）。
2. 选通道：收费通道一般就是预制件里的若干条导航曲线，选与路线衔接的那一条。
3. 流程：减速 → 停在闸门前 3 m → 等 1 s → 发一次 `activate` → 等到 `ETS2LASemaphore` 里该闸门状态变成 OPEN → 通过 → 等待 `player.tollgate.paid` 事件确认。超过 20 s 闸门没开，就重试一次 `activate`；仍然不开就请求人工接管。

**P2 退出条件**：跨城高速全程（含匝道和收费站）无接管、零罚单（以 `player.fined` 事件为准）。

---

## 6. P3：全路网 L3

1. **信号灯关联**：地图里的信号灯（`SemaphoreId` 加预制件实例 uid）对应 ETS2LA 的信号灯记录，按世界坐标最近、距离小于 1.5 m 匹配，匹配结果按预制件实例缓存。
   - 导航曲线上带有 `SemaphoreId`，可以据此得到"这条曲线受哪个灯控制"；停止线取该曲线的起点。
2. **信号灯决策**：RED、ORANGE_TO_RED 停车；GREEN 通行；遇到黄灯时，如果以舒适减速度停不到停止线前，就继续通过。`time_remaining` 用来预判：剩余绿灯时间小于通过路口所需时间时，提前减速。SLEEP（黄闪）按让行规则处理。
3. **让行与冲突**：`Intersections` 给出导航曲线之间的冲突点（`CurveId`、`Position`、`Radius`）✅，优先级从 `TrafficRule` 和 ppd 的 Priority 查表得到。
   - 对冲突车流估计到达冲突点的时间（TTC）：对方比我方先到，且时间差小于 4 s，就让行。
   - 无保护左转额外要求对向车流间隙大于 8 s（卡车清空路口慢）。
4. **城市转弯**（扫掠约束）：
   - 用 iLQR 以牵引车加挂车模型优化轨迹，状态 [x, y, θ1, θ2, v, δ]，约束挂车四角在 `drivable_polygons` 之内，用对数障碍函数处理，|θ1−θ2| < 60°。
   - 初值取导航曲线。离线把常见路口预先计算好、缓存起来，在线只做热启动。
5. **环岛**：作为一个专门场景处理。进环让行用冲突检测，环内保持车道，出环前 30 m 打右转灯。
6. **离线闭环仿真 v1**（`crates/sim`）：
   - 自车：辨识得到的动力学模型。
   - 其他车辆：从录制里取初始状态，之后按 IDM 沿车道图行驶，并对自车做出反应。
   - 场景库：从录制里自动切出接管前 10 s、急刹（减速度大于 3 m/s²）、闯红灯边缘、环岛等片段。
   - CI 每次提交都跑完整场景库，任何指标退化都拒绝合入。

**P3 退出条件**：城市到城市（场站门口为止），每百公里罚单少于 1 次。

---

## 7. P4：场站倒车入位

1. 目标位姿：任务目的地公司预制件里 `UnloadEasy/Medium/Hard` 类型的 `SpawnPoints` ✅，按任务难度选取，取挂车停放的位置和朝向。
2. 场站局部地图：预制件的可行驶多边形，叠加停放车（`ETS2LAParkedVehicles`）作为静态障碍。
3. **Hybrid A\***：状态 (x, y, θ1, θ2, 方向)；运动基元覆盖前进和倒车 × 5 档转角。启发式取 max(无障碍 Reeds-Shepp 距离, 二维 Dijkstra 距离)。约束包括挂车折叠角上限 60° 和车身包络无碰撞。每隔 N 次扩展，尝试一次解析连接（Reeds-Shepp 加铰接校验）。
4. **倒车控制**：外环对挂车轴的位置做纯追踪，得到期望铰接角 φ*；内环用 LQR 把铰接角稳定到 φ*（倒车时挂车是不稳定系统），输出转向。车速固定 3 km/h。铰接角误差超过 15° 就停车，向前拉直后重新规划。
5. 完成判定：等到 `job.delivered` 事件，或挂车位姿误差小于 0.5 m、角度误差小于 3°。
6. 练习方式：用开发存档反复加载同一场站的起点（`quickload`），批量跑。

**P4 退出条件**：困难难度的车位，成功率不低于 90%（至少 50 次尝试）。

---

## 8. P5：学习化

1. **自动标注**（`crates/labeling`）：录制帧 + 当帧真值 + 投影 → 2D/3D 框、车道线、可行驶区域、信号灯小图和状态、自车遮挡掩码（做法见 DESIGN §6.5、§6.7）。输出 WebDataset 风格的分片（tar）。遮挡剔除：P5 初期用"框与框的深度排序"近似；需要更准的话，再在 gfx DLL 里额外拷出深度缓冲（需要实测能否拿到深度缓冲）。
2. **训练和推理都在 `burn` 里完成**：`burn` 目前只能导入 ONNX，不能导出 ✅。所以自训练的模型直接用 `burn` 的 CUDA 后端推理（f16）；外部现成的 ONNX 模型才走 `ort` + TensorRT。
   - 推理预算按 DESIGN §6.6，P5 开始前先实测 `burn` 在 5060 Ti 上的推理耗时。
3. 模型顺序（每个都是"先影子模式对比，再上线"）：信号灯小图分类 → 2D 检测 → BEV 目标与车道（LSS 风格）→ 可行驶区域分割 → 学习型预测 → 学习型规划。
4. 纯视觉降级模式：关掉 ETS2LA 扩展状态（模拟游戏更新导致失效），在场景库和实车上评估。

---

## 9. P6：全职业自动化

1. 菜单操作：语义输入里的 `j_up/j_down/j_left/j_right/selectfcs/back/menu` 可以操作菜单（取自 ETS2LA 的 controls_spec）。界面识别：先用模板匹配加 OCR（以 ONNX 模型的形式通过 `ort` 运行），后期换成小型视觉模型。
2. 接单：读遥测里的任务配置事件确认结果；选单策略按收益除以预计时间计算。
3. 加油、休息：`fuel.range` 不够到下一个加油点，就规划到地图里 `GasStation` 类型的生成点；疲劳由 `rest.stop` 通道给出，就近进入 `Parking`/`TruckStop`。
4. 轮渡、火车：路线经过时进入对应场景，等待 `player.use.ferry/train` 事件确认。
5. 天气和夜间：在场景库里按天气和时间分层统计指标，不达标的层专门补采数据。

---

## 10. 每一步都适用的工程规矩

- **共享内存结构一旦发布就只能追加字段**，改布局必须提升 `schema_version`，读端检查不过就拒绝连接。
- **不信任任何来自游戏内存的数据**：每次读取都做合理性检查（NaN、量级、单调性），失败计数进 Rerun 时间序列。
- **每个新模块先在回放数据上跑通，再上实车。**
- **每次实车测试都录制。** 没有录制的测试不算数。
- 测试：纯函数（坐标、几何、控制律）写单元测试；跨模块的写回放回归；共享内存协议写双进程集成测试（`cargo test` 里起一个模拟写端）。
