[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$GameDir,[switch]$NativeGraphics)
$ErrorActionPreference='Stop'
if (Get-Process eurotrucks2 -ErrorAction SilentlyContinue) { throw '请先退出欧卡2。' }
$game=(Resolve-Path -LiteralPath $GameDir).Path
if (-not (Test-Path -LiteralPath (Join-Path $game 'bin\win_x64\eurotrucks2.exe'))) { throw '指定目录不是欧卡2安装目录。' }
$source=Join-Path $PSScriptRoot 'etsads_bridge.dll'
if (-not (Test-Path -LiteralPath $source -PathType Leaf)) { throw '此脚本用于便携包。源码工作区请使用 deploy.ps1。' }
# Validate every requested source before creating directories, replacing the
# bridge, or disabling another capture plugin. A partial package must not
# leave a previously working installation partially modified.
$graphicsSource=Join-Path $PSScriptRoot 'etsads_gfx.dll'
if ($NativeGraphics -and -not (Test-Path -LiteralPath $graphicsSource -PathType Leaf)) { throw '便携包缺少 etsads_gfx.dll；未修改游戏插件。' }
$plugins=Join-Path $game 'bin\win_x64\plugins'
New-Item -ItemType Directory -Force -Path $plugins | Out-Null
$target=Join-Path $plugins 'etsads_bridge.dll'
if (Test-Path -LiteralPath $target) { Copy-Item -LiteralPath $target -Destination ($target+'.backup-'+(Get-Date -Format 'yyyyMMdd-HHmmss')) }
Copy-Item -LiteralPath $source -Destination $target -Force
if ($NativeGraphics) {
    $capture=Join-Path $plugins 'ets2la_capture.dll'
    if (Test-Path -LiteralPath $capture) {
        # Preserve the original binary; disable it by changing the extension.
        $resolved=(Resolve-Path -LiteralPath $capture).Path
        if (-not $resolved.StartsWith($plugins + [IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Capture path escaped the plugins directory' }
        Move-Item -LiteralPath $resolved -Destination ($resolved + '.disabled-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
    }
    $gfx=Join-Path $plugins 'etsads_gfx.dll'
    if (Test-Path -LiteralPath $gfx) { Copy-Item -LiteralPath $gfx -Destination ($gfx+'.backup-'+(Get-Date -Format 'yyyyMMdd-HHmmss')) }
    Copy-Item -LiteralPath $graphicsSource -Destination $gfx -Force
}
Write-Output "插件已安装：$target"
Write-Output '下一步：从 Steam 启动游戏、确认 SDK 提示，然后进入单机驾驶。'
