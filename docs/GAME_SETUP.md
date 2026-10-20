# 本机游戏设置与恢复

## 便携包安装

1. 将便携包解压到可写目录，退出游戏。打开该目录中的 PowerShell，运行 `./install-plugin.ps1 -GameDir '<Steam 中欧卡2的安装目录>' -NativeGraphics`。它会备份并安装 `etsads_bridge.dll` 和 `etsads_gfx.dll`，停用旧 `ets2la_capture.dll`，避免两套捕获插件同时加载。
2. 交通与相机扩展使用 ETS2LA 的独立 MIT 插件。对于本次实际测试的 **ETS2 1.61.1.9**，下载[固定提交的 ets2la_plugin.dll](https://raw.githubusercontent.com/ETS2LA/ETS2LA/26f32720b5c1dd7e8d7d14efaa483835f044e2a5/Assets/SDKs/1.61/Windows/ets2la_plugin.dll)，运行 `Get-FileHash ./ets2la_plugin.dll -Algorithm SHA256`，确认结果为 `0E1893719F84F28079857451B82A22A263401D98B8FC299CD561EB7695BFA88F` 后，复制到游戏的 `bin/win_x64/plugins/`。已有同名文件应先另存备份。本包不包含该第三方二进制；许可证与版本说明见[上游插件仓库](https://github.com/ETS2LA/plugin)。
3. 从 Steam 以 DX11 启动游戏，确认 SDK 提示，选择存档并进入驾驶。双击包内 `ETSADS.cmd`；等待面板出现实时遥测后，可开始数据录制。视频另需 ffmpeg/NVENC，可在面板关闭视频选项，仅录数据。
4. 停止录制并确认完成后，再退出面板。每次更新插件都先退出游戏，保留旧插件和存档备份。

上述扩展 DLL 仅记录本次固定版本的来源与校验值；其他游戏版本必须另行匹配、验证，不直接沿用。运行诊断命令 `./etsads-cli.exe doctor` 可检查当前环境。实际捕获尺寸以帧元数据为准，本次游戏窗口曾为 2560×1440，不能把配置中的 1920×1080 当作验收事实。

## 本次现场设置

已核实游戏目录：`D:\SteamLibrary\steamapps\common\Euro Truck Simulator 2`，版本1.61.1.9，Steam build25904183。工具运行时自行发现 Steam 库或使用 `ETSADS_GAME_DIR`，不要求其他机器使用此路径。

运行 `tools/prepare-game.ps1` 前退出游戏。脚本备份 Documents配置、profiles/steam_profiles，以及 Steam userdata中227300应用的remote目录（云存档的实际game.sii位于这里），然后设置开发控制台、1920×1080窗口与60FPS。

脚本把备份放在 `artifacts/game-backup-<时间>` 目录。恢复时先退出游戏，将备份中对应目录复制回原位置；恢复 Steam Cloud 存档时先在Steam确认云同步状态，避免较新云副本覆盖本地恢复。不要删除备份或自动批量替换所有账户内容。

游戏中使用单机模式、真实自动挡、无转向辅助、线性/零死区控制。简易自动挡会把刹车用于倒车，不适用于本项目的独立油门/制动控制。程序以SDK实际报告的`shifter.type`检查模式，未知或简易自动挡会拒绝控制；仅修改磁盘配置不能证明游戏已加载。保留原存档以便恢复。现场罚单验收须开启交通违章处罚；本次在退出游戏后备份当前profile配置并将g_police从0改为1，重启日志已确认加载1。此前的静止录制不用于罚单指标。

在 Steam 游戏属性→测试版选择匹配版本以锁定版本；本次尚未修改Steam版本选项。更新后必须先重新检查扩展插件、地图缓存与投影。

`tools/deploy.ps1` 遇到运行中的游戏会停止，防止覆盖被加载的DLL。P0热重载仍需要单独在游戏控制台实测10次 `sdk unload` / `sdk reload` 并记录。

