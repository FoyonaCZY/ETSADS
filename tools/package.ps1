[CmdletBinding()]
param([string]$Output)
. (Join-Path $PSScriptRoot 'env.ps1')
function Get-SourceInventory {
    # Git is optional for source archives; do not make it a runtime dependency.
    if (-not (Get-Command git -ErrorAction SilentlyContinue)) { return @() }
    $checkout=(& git -C $EtsadsRoot rev-parse --show-toplevel 2>$null | Out-String).Trim()
    if ($LASTEXITCODE -ne 0 -or [IO.Path]::GetFullPath($checkout) -ne [IO.Path]::GetFullPath($EtsadsRoot)) { return @() }
    $names=(& git -C $EtsadsRoot -c core.quotepath=false ls-files --cached --others --exclude-standard -z | Out-String).TrimEnd("`r","`n") -split "`0"
    foreach($name in ($names | Where-Object {$_} | Sort-Object -Unique)) {
        $absolute=[IO.Path]::GetFullPath((Join-Path $EtsadsRoot $name))
        if (-not $absolute.StartsWith($EtsadsRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Source inventory escaped workspace' }
        if (Test-Path -LiteralPath $absolute -PathType Leaf) {
            [pscustomobject]@{file=$name.Replace('\','/');sha256=(Get-FileHash -LiteralPath $absolute -Algorithm SHA256).Hash}
        }
    }
}
if (-not $Output) { $Output=Join-Path $EtsadsRoot ('artifacts\portable-' + (Get-Date -Format 'yyyyMMdd-HHmmss')) }
if (Test-Path -LiteralPath $Output) { throw '输出目录已存在，请使用新目录，以免覆盖旧版本。' }
Push-Location $EtsadsRoot
try {
    $sourceBefore=@(Get-SourceInventory)
    & $EtsadsCargo build --release -p etsads -p etsads-supervisor -p etsads-bridge -p etsads-sysid -p etsads-overlay --features etsads/ui --locked
    if ($LASTEXITCODE) { throw 'Release build failed' }
    # Build this feature last: a featureless workspace build does not include
    # the native entrypoints and must never replace the packaged gfx plugin.
    & $EtsadsCargo build --release -p etsads-gfx --features native-dx11 --locked
    if ($LASTEXITCODE) { throw 'DX11 plugin build failed' }
    New-Item -ItemType Directory -Path $Output | Out-Null
    foreach ($name in @('etsads.exe','etsads-cli.exe','etsads-supervisor.exe','etsads-sysid.exe','etsads-overlay.exe','etsads_bridge.dll','etsads_gfx.dll')) {
        Copy-Item -LiteralPath (Join-Path $EtsadsRoot ('target\release\' + $name)) -Destination $Output
    }
    Copy-Item -LiteralPath (Join-Path $EtsadsRoot 'LICENSE'),(Join-Path $EtsadsRoot 'README.md') -Destination $Output
    Copy-Item -LiteralPath (Join-Path $EtsadsRoot 'docs') -Destination $Output -Recurse
    Copy-Item -LiteralPath (Join-Path $EtsadsRoot 'apps\overlay\README.md') -Destination (Join-Path $Output 'docs\OVERLAY.md')
    Copy-Item -LiteralPath (Join-Path $EtsadsRoot 'crates\visualization\RERUN.md') -Destination (Join-Path $Output 'docs\RERUN.md')
    Copy-Item -LiteralPath (Join-Path $EtsadsRoot 'crates\replay\README.md') -Destination (Join-Path $Output 'docs\REPLAY.md')
    Copy-Item -LiteralPath (Join-Path $EtsadsRoot 'crates\sim\RECORDING_SCENARIOS.md') -Destination (Join-Path $Output 'docs\RECORDING_SCENARIOS.md')
    Copy-Item -LiteralPath (Join-Path $EtsadsRoot 'vendor\licenses') -Destination $Output -Recurse
    Copy-Item -LiteralPath (Join-Path $EtsadsRoot 'vendor\scs_sdk\sdk_license.txt') -Destination (Join-Path $Output 'licenses\scs-sdk.txt')
    & (Join-Path $PSScriptRoot 'collect-licenses.ps1') -Output (Join-Path $Output 'licenses\rust-dependencies')
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'install-plugin.ps1') -Destination $Output
    # Map exporter, self-contained so players need no .NET install. It is a
    # separate GPL-2.0 program; its licence and source location ship with it.
    $dotnet=(Get-Command dotnet -ErrorAction Stop).Source
    $env:DOTNET_CLI_TELEMETRY_OPTOUT='1'
    & $dotnet publish (Join-Path $EtsadsRoot 'tools\map_export\MapExport.csproj') -c Release -r win-x64 --self-contained true -o (Join-Path $Output 'map-export')
    if ($LASTEXITCODE) { throw 'Map exporter publish failed' }
    Copy-Item -LiteralPath (Join-Path $EtsadsRoot 'tools\map_export\LICENSE') -Destination (Join-Path $Output 'map-export\LICENSE-GPL-2.0.txt')
    [IO.File]::WriteAllText((Join-Path $Output 'map-export\SOURCE.txt'),"地图导出工具（GPL-2.0）的完整源代码在本项目仓库的 tools/map_export 目录。`r`n",[Text.UTF8Encoding]::new($true))
    $launch='@echo off' + "`r`n" + 'cd /d "%~dp0"' + "`r`n" + 'start "" "%~dp0etsads.exe"' + "`r`n"
    [IO.File]::WriteAllText((Join-Path $Output 'ETSADS.cmd'),$launch,[Text.ASCIIEncoding]::new())
    $quickstart=@'
ETSADS 欧卡2自动驾驶（试验版）

运行本程序不需要安装 Rust、.NET 或 Python。

1. 双击 ETSADS.cmd 打开面板，默认就在“自动驾驶”页。
2. 按面板上方“首次设置”的提示依次完成：
   ① 退出游戏，点“安装插件”（自动安装本程序的两个插件，并下载校验 ETS2LA 交通数据插件）；
   ② 点“生成地图”（从你本机的游戏文件生成，约 5 分钟、1.6 GB）；
   ③ 从 Steam 启动欧卡2（DX11），进入单机驾驶；
   ④ 停稳、拉起驻车制动，点“标定转向”（约 30 秒，每辆车一次）。
3. 游戏设置：真实自动挡（不是简易自动挡）、关闭转向辅助；建议关闭“休息状态模拟”
   （司机疲劳打盹时游戏会忽略所有方向和刹车输入）。
4. 设置好 GPS 目的地，开到车道中央，在面板点“开启自动驾驶”，回到游戏按 F8。
   方向键、WASD、空格或 Esc 随时接管；F8 退出。

详细说明见 docs/MVP.md。只能在单机模式使用，不支持 TruckersMP。
这是试验软件：请始终注意路况、随时准备接管。会自动换道；不掉头，不处理收费站和挂车。
本包不包含任何游戏资源；地图在你本机从你自己的游戏文件生成。
'@
    [IO.File]::WriteAllText((Join-Path $Output '开始使用.txt'),$quickstart,[Text.UTF8Encoding]::new($true))
    $sourceAfter=@(Get-SourceInventory)
    if ((ConvertTo-Json -InputObject $sourceBefore -Compress) -cne (ConvertTo-Json -InputObject $sourceAfter -Compress)) { throw 'Source changed during build; rebuild after writers have stopped.' }
    ConvertTo-Json -InputObject $sourceAfter | Set-Content -LiteralPath (Join-Path $Output 'source-sha256.json') -Encoding utf8
    [pscustomobject]@{
        format_version=1
        product='ETSADS development'
        built_utc=[DateTime]::UtcNow.ToString('o')
        cargo=(& $EtsadsCargo --version | Out-String).Trim()
        rustc=(& rustc -vV | Out-String).Trim()
        cargo_lock_sha256=(Get-FileHash -LiteralPath (Join-Path $EtsadsRoot 'Cargo.lock') -Algorithm SHA256).Hash
        source_inventory_files=$sourceAfter.Count
        source_inventory_scope='Git tracked and non-ignored untracked files when checkout inventory is available; zero means unavailable, not clean.'
        features=@('etsads/ui','etsads-gfx/native-dx11')
        live_phase_acceptance='incomplete; see docs/ACCEPTANCE.md'
    } | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $Output 'build-info.json') -Encoding utf8
    $packageRoot=(Resolve-Path -LiteralPath $Output).Path
    Get-ChildItem -LiteralPath $packageRoot -Recurse -File | ForEach-Object {
        [pscustomobject]@{file=$_.FullName.Substring($packageRoot.Length+1).Replace('\','/');sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash}
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $Output 'sha256.json') -Encoding utf8
    Write-Output "便携开发包：$Output"
    Write-Output '双击 ETSADS.cmd，按面板“首次设置”完成插件、地图和转向标定。'
} finally { Pop-Location }

