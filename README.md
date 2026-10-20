# ETSADS

欧洲卡车模拟 2（ETS2）单机模式的自动驾驶程序：跟随游戏 GPS 导航行驶，按限速和弯道调速，跟车、过路口（红绿灯、让行、停车让行）、过环岛，到达目的地后自动停车拉手刹。游戏里有 HUD 显示状态，F8 启用，方向键 / WASD / 空格随时接管（Esc 打开游戏菜单时自动驾驶保持，回到游戏后继续）。

> 试验软件：请始终注意路况、随时准备接管。只能在单机模式使用，不支持 TruckersMP。目前不会掉头，不处理收费站；支持带挂车行驶（试验，倒车入位需自己操作）。遇到不支持的情况会提前提醒你接管。**程序看不到施工护栏、事故、警车拦车等静止障碍物，也不会自己去加油站**，遇到时请自己接管。

## 快速开始（玩家）

需要：Windows 10/11 x64，Steam 版欧卡2（已验证 1.61），不需要安装 Rust、.NET 或 Python。

1. 从 [Releases](../../releases) 下载 `ETSADS-…-win-x64.zip`，解压到任意可写目录，双击 `ETSADS.cmd`。
2. 面板上方的“首次设置”会逐项检查并给出按钮：
   1. **安装插件**：先退出游戏再点，约 10 秒。会安装本程序的两个插件，并下载、校验 ETS2LA 交通数据插件（MIT）。
   2. **生成地图**：从你本机的游戏文件生成，约 5 分钟，占用约 1.6 GB。
   3. 从 Steam 启动游戏（DX11），进入单机驾驶。游戏设置为**真实自动挡**（不是简易自动挡），并关闭转向辅助；转向方式（相对 / 绝对）不用改，程序自动适配。建议关闭“休息状态模拟”（司机打盹时游戏会忽略所有控制）。
   4. **标定转向**：停稳并拉起驻车制动后点击，约 30 秒到 1 分钟，每辆车只需一次。
3. 在游戏里设置 GPS 目的地，把车开到车道中央；在面板点 **开启自动驾驶**，HUD 显示“可以启用”后，在游戏里按 **F8**。

详细说明、各状态含义和已知限制见 [使用说明](docs/MVP.md)。游戏更新后，面板会提示重新生成地图或更新插件。

开发进度与实测记录见 [开发与验证进度交接记录](docs/DEVELOPMENT_STATUS.md)。原始设计目标与 P0～P4 阶段验收门槛见 [设计](docs/DESIGN.md)，这些门槛仍未全部通过。

## 开发者

以下是开发平台的说明：录制、回放、离线仿真和各阶段实验工具。

### 从源码运行

当前仓库的短时道路试用入口是双击 `ETSADS-MVP.cmd`，打开“自动驾驶”页，使用步骤与玩家相同。`tools/package.ps1` 生成与 Release 相同的便携包；推送 `v*` 标签时，GitHub Actions 自动构建并发布。

便携包中双击 `ETSADS.cmd`。驾驶面板提供录制开关、状态和控制释放；“运行与目标速度”页可填写配置、标定、地图、策略和实测证据路径，默认观察，显式“启用并运行”仍经过原有Host与独立安全监督校验。“录制与回放”页可直接浏览本地记录并后台分析，“离线实验室”可查看闭环回放。中文字体随程序提供，运行面板不需要安装开发工具。

目标速度修改只会以新文件保存到原配置同目录，保留相对引用及未知JSON字段，不覆盖原文件或热改运行。未保存的速度草稿会禁用启动，可先保存或还原。切换草稿会清空关联实验/道路许可证据；新指纹须重新校验，保存不授予驾驶资格。泊车保持固定3km/h上限。驾驶运行自行录制，不能同时启动独立录制；停止或关闭面板会请求释放并等待记录收尾。安全几何配置同时用于面板新启动的监督进程，已运行的监督不会自动换配置。

Windows 11 x64；Steam 已安装欧卡2。推荐 Rust MSVC + Visual Studio C++ Build Tools；本机也验证 GNU Rust 用于纯 Rust/Win32 开发。地图导出单独需要 .NET 10，运行已导出地图的主程序不需要 .NET。视频录制需要带 `hevc_nvenc` 的 ffmpeg 与 NVIDIA GPU。

```powershell
# 在仓库目录内
cargo build --workspace --features etsads/ui
.\target\debug\etsads-cli.exe doctor
.\target\debug\etsads.exe
```

游戏接入：

```powershell
# 游戏退出时：备份 Documents 配置 + Steam Cloud 本地实际存档
.\tools\prepare-game.ps1
# 复制自研桥接插件；不覆盖其他插件
.\tools\deploy.ps1 -GameDir 'D:\SteamLibrary\steamapps\common\Euro Truck Simulator 2'
```

从 Steam 以 DX11 启动游戏，确认 SDK 提示，进入单机驾驶。扩展数据需要与游戏版本匹配的 ETS2LA MIT 插件；固定版本的下载、校验和安装步骤见 [游戏设置](docs/GAME_SETUP.md)。早期录制使用 ETS2LA capture GPL-3.0，当前安装已切换到自研 gfx；历史录制保留各自插件指纹。

自研 `etsads_gfx.dll` 已通过独立 GPU 测试，并在本机游戏中完成录像、车内及车外诊断框显示检查；投影仍标记未完成全局标定。便携安装脚本的 `-NativeGraphics` 会备份并停用旧 capture，避免同时加载两套捕获 Hook。源代码打包运行 `tools/package.ps1`；包内包含依赖许可证与 SHA256 清单。

```powershell
.\target\debug\etsads-cli.exe telemetry --seconds 10
.\target\debug\etsads-supervisor.exe
.\target\debug\etsads-cli.exe record runs/first --seconds 60 --video
.\target\debug\etsads-cli.exe replay runs/first
.\target\debug\etsads-cli.exe replay runs/first --snapshots
.\target\debug\etsads-cli.exe analyze runs/first
.\target\debug\etsads-cli.exe frame-dump artifacts/game.png
```

安全监督默认关闭；游戏中按 `F8` 请求开启/关闭，方向键 / WASD / 空格请求释放。已启用时游戏暂停（Esc、地图）会保持控制、零油门，恢复后继续；未启用时暂停不允许启用。数据过期、非法数据或交通通路故障均会释放；重新启用必须显式请求。方向盘/踏板接管合成尚须 E2 实测；面板的启用按钮是受证据门槛约束的开发试验入口，不代表已认证自动驾驶。

以下 CLI 示例需独立终端运行。短时输入实验需要安全监督已运行、游戏未暂停，且 SDK 确认真正的自动挡或手动挡；简易自动挡和未知模式禁止输入。相对转向会累积轮角，不能直接当作绝对转角输入。本机测试存档经授权改为绝对模式后已通过小幅静止响应检查；其它存档与动态驾驶仍须各自验证。`--arm` 在数据持续新鲜的预热阶段后只请求开启一次，结束时释放；零油门转向实验无需开动车辆：

```powershell
.\target\debug\etsads-cli.exe steer 0.3 --seconds 10 --throttle 0 --brake 0 --arm
# 需要提前停止时，在另一终端请求释放
.\target\debug\etsads-cli.exe gate off
```

停稳后可用 `etsads-cli pulse engine --arm` 切换点火/熄火。命令成功表示游戏 SDK 已接收本次按钮，仍应检查仪表或遥测中的实际转速；旧版 DLL 未实现该按钮时会明确返回未确认。点火不会自动挂挡或授予持续驾驶权限。

## 离线验证

```powershell
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
cargo run -p etsads -- sim --output artifacts/simulation.json
cargo run -p etsads -- record-bench artifacts/benchmark --frames 216000
```

`record-bench` 生成 216,000 个合成遥测帧，相当于 60 Hz 一小时的数据量。它仅验证录制器吞吐与完整性，不能替代一小时真实驾驶验收。`sim` 输出轨迹、误差、跟车时距和逐项离线判定；简单直线倒车回归不能替代困难车位验收。

`replay --snapshots` 会将原始录制按物理帧重建为与实时读取相同的 `GameSnapshot`，关联配置、事件、扩展和视频索引。`replay-drive` 可将这些观测送入实际 Runtime，输出控制/规划的开环结果；必须使用 `execution: "Replay"`，不会连接游戏控制。用法和缺失数据处理见 [回放适配器](crates/replay/README.md)。

`scenario-extract` 自动选择接管前窗、急刹、红灯接近及已分类的环岛片段；`scenario-run` 用辨识模型和会响应自车的 IDM 交通运行整库，保存逐步状态并与指标基准比较。说明与模型限制见 [录制场景仿真](crates/sim/RECORDING_SCENARIOS.md)。

## 目录

| 路径 | 用途 |
|---|---|
| `crates/msgs`、`shm`、`sdk-sys` | 定长协议、共享内存、官方 SDK 绑定 |
| `crates/bridge`、`game` | 游戏 DLL、时间/坐标/相机/捕获适配 |
| `crates/map`、`prediction`、`planning`、`control` | 地图、预测、规划和控制算法 |
| `crates/safety`、`apps/supervisor` | 独立安全策略与进程 |
| `crates/recorder`、`apps/etsads` | MCAP、命令行和原生中文面板 |
| `crates/sim` | 可重复的离线闭环回归 |
| `tools/map_export` | 独立 GPL-2.0 C# 地图导出器 |
| `runs`、`artifacts` | 本机证据与数据，默认不纳入 Git |

Windows 游戏接入依赖 Win32；核心数学、规划和离线录制模块独立于游戏进程。环境路径通过 `ETSADS_GAME_DIR`、`ETSADS_FFMPEG` 或命令行指定；`ETSADS_FONT` 可覆盖内置中文字体，`ETSADS_CARGO` 可指定源码构建工具链。不把本机绝对路径写进运行数据契约。

