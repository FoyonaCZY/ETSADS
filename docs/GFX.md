# 自研 DX11 画面与叠加插件

`crates/gfx` 的 `native-dx11` 功能生成 `etsads_gfx.dll`。默认 workspace 构建保留
可移植协议与单元测试，不启用 Windows 原生 hook；发布插件必须显式构建：

```powershell
cargo build --release -p etsads-gfx --features native-dx11
```

复制 `target/release/etsads_gfx.dll` 到游戏 `bin/win_x64/plugins/` 前应退出游戏。
默认 workspace 构建可能覆盖同名 DLL，打包时最后执行上面的显式构建。
自研 gfx 与 ETS2LA capture 应只部署一个；ETS2LA 扩展状态插件可以保留。

## 捕获顺序与协议

只有一个 `IDXGISwapChain::Present` detour，依次执行：

1. 使用 `D3D11_MAP_FLAG_DO_NOT_WAIT` 读取先前已提交的 staging 纹理；尚未完成就跳过。
2. 将当前 backbuffer 复制到三槽 staging 队列，记录进入 Present 的 QPC。
3. 通过 `ID3D11DeviceContext1::ClearView` 绘制新鲜且通过边界校验的矩形叠加。
4. 调用游戏原来的 Present。

同一 immediate context 保证 CopyResource 排在叠加写入之前，因此自己录制的图像
不含自己画出的叠加。三个 staging 槽避免同步等待 GPU，代价是图像发布晚若干帧；
记录的是原始 Present QPC，不能用发布 QPC 代替其物理帧对齐时间。
无捕获读者时不分配 staging 纹理、不执行 GPU 回读。

写入 `Local\\ETSADS_Capture_Control` 和 `Local\\ETSADS_Capture_Data_<generation>`。
使用 Capture v1 公开协议，以便现有 Rust 读者复用；实现是独立编写的 Rust 代码，
未复制 GPL capture 实现。根目录项目许可证为 MIT；MinHook Rust 包和原生库的
相应许可证仍适用。`CaptureReader::connect` 优先连接自研映射，再回退 ETS2LA。

发生分辨率或格式变化时创建新 generation；按 RowPitch 逐行拷贝，输出紧密排列。
当前支持 8 位 BGRA、BGRX、RGBA 格式，不支持 MSAA backbuffer 的捕获。
不实现 HDR 浮点或 10 位格式的色调映射，也不声称支持 DX12/Vulkan。

## 叠加接口

主程序向 `ETSADS_Overlay` 写入 `OverlayFrame`（`crates/msgs/src/protocol.rs`）。
最多 256 个带 RGBA 色值的像素矩形，画边框时提交四条细矩形。过期 200 ms、禁用、
NaN、反向边界、超量输入都会被拒绝或裁剪。默认没有发布者，因此不会显示测试图形。
ClearView 使用不透明诊断色，不改变游戏的 shader、blend、sampler 或 viewport 状态。
本模块不擅自假定相机投影约定；交通框和轨迹的世界到像素变换由完成 E4 标定的上游提供。

## 游戏内 HUD

主程序（`apps/etsads/src/hud.rs`）把 HUD 渲染成一张预乘 RGBA 图，最大 1024×512，写入 `ETSADS_Hud`。写入方式是 `SharedRing<HudImage, 2>`，双槽加序号，持有 `ETSADS_Hud_Writer` 单写者锁。中文由 `ab_glyph` 用随包字体光栅化，按渲染端回报的分辨率缩放（高度/940），保证像素清晰。HUD 在独立线程中以 10 Hz 发布，显示内容超过 400 ms 没更新就发布“关闭”。

DLL（`crates/gfx/src/hud.rs`）在叠加矩形之后绘制 HUD，步骤如下：

1. 镜像序号变化时，`Map(WRITE_DISCARD)` 上传纹理；
2. `SwapDeviceContextState` 切换到私有状态对象，绘制一个预乘 Alpha 混合的四边形（着色器在首次使用时由 `d3dcompiler_47` 编译）；
3. 解绑渲染目标，再切换回游戏原状态。

整个过程不读取、不逐项修改游戏的管线状态。图像 `write_qpc` 超过 500 ms 就不再绘制。每帧向 `ETSADS_HudStatus` 回报 Present QPC、分辨率和已绘制序号，`etsads-cli hud-status` 可读取。

写者若在复制图像中途被强杀，槽序号会停在奇数，后续发布一直 `Busy`。新写者拿到锁后先调用 `recover_after_writer_crash` 回滚，r9 实测过这种情况。

## SDK 关闭与代码寿命

`sdk unload` 使 gfx 停止接受新渲染、关闭 detour，并在回调退出后释放 GPU 和共享内存资源。
DLL 代码和 disabled trampoline **保留到游戏进程退出**，防止另一个 CPU 在禁用之前
已经取到 detour 地址、稍后才进入函数时跳到卸载后的内存。SDK 可以重新初始化插件，
但更换 DLL 文件必须退出 ETS2。此限制是明确的卸载策略，不是完整物理卸载已通过验收。
如果在 100 ms 内仍有未退出回调，保留资源，不冒险销毁正在使用的对象。

## 已执行的验证

在本机 Windows/GNU 工具链完成：

- native-dx11 原生代码真实编译；核心默认构建不依赖该 feature。
- 真实 Windows 映射的 RowPitch 剥离、分辨率 generation 切换与读者注册/释放测试。
- 真实 D3D11 隐藏窗口 GPU 测试：场景红色，叠加绿色；共享图像首像素为红色，最终
  GPU backbuffer 首像素为绿色，验证先捕获后叠加。
- 真实 Present detour 的五次初始化、捕获、停用、重新启用循环全部通过。
- 相关 crates 的 `clippy --all-targets -D warnings` 通过。

GPU 测试需要 Windows 图形会话，因此默认标记为 ignored，实际执行命令：

```powershell
cargo test -p etsads-gfx --features native-dx11 gpu_capture_precedes_overlay_without_game -- --ignored
cargo test -p etsads-gfx --features native-dx11 real_present_hook_can_deactivate_and_reactivate -- --ignored
```

这些是本机真实 GPU 与 hook 测试，**不是 ETS2 实机画面、相机标定或 AR 框精度验收**。
ETS2 截图、与游戏叠加的兼容性、低于 2 ms 的采集预算和长期录制仍须分别测量。
- r9 真实 D3D11 隐藏窗口测试 `gpu_hud_blends_after_capture_and_restores_game_state`：50% 绿色 HUD 叠在红色场景上读回约 (127,128,0)，截图仍为纯红，游戏设置的视口与渲染目标在绘制后原样恢复；ETS2 1.61 实机 2560×1440 下 HUD 正常显示。
