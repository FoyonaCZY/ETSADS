[CmdletBinding()]
param([string]$GameDir = $env:ETSADS_GAME_DIR, [switch]$SkipBuild)
. (Join-Path $PSScriptRoot 'env.ps1')
if (-not $GameDir) {
    $candidates = @('D:\SteamLibrary\steamapps\common\Euro Truck Simulator 2', (Join-Path ${env:ProgramFiles(x86)} 'Steam\steamapps\common\Euro Truck Simulator 2'))
    $GameDir = $candidates | Where-Object { Test-Path -LiteralPath (Join-Path $_ 'bin\win_x64\eurotrucks2.exe') } | Select-Object -First 1
}
if (-not $GameDir) { throw '未找到欧卡2。请指定 -GameDir <Steam 游戏目录>。' }
$GameDir = (Resolve-Path -LiteralPath $GameDir).Path
if (-not (Test-Path -LiteralPath (Join-Path $GameDir 'bin\win_x64\eurotrucks2.exe'))) { throw '指定目录不是欧卡2安装目录。' }
if (Get-Process eurotrucks2 -ErrorAction SilentlyContinue) { throw '游戏正在运行。请先退出游戏，确保 DLL 不被占用。' }
Push-Location $EtsadsRoot
try {
    if (-not $SkipBuild) { & $EtsadsCargo build --release -p etsads-bridge --locked; if ($LASTEXITCODE) { throw 'Bridge build failed' } }
    $dll = Join-Path $EtsadsRoot 'target\release\etsads_bridge.dll'
    if (-not (Test-Path -LiteralPath $dll)) { throw "Missing build artifact: $dll" }
    $plugins = Join-Path $GameDir 'bin\win_x64\plugins'
    New-Item -ItemType Directory -Force -Path $plugins | Out-Null
    $target = Join-Path $plugins 'etsads_bridge.dll'
    if (Test-Path -LiteralPath $target) { Copy-Item -LiteralPath $target -Destination ($target + '.backup-' + (Get-Date -Format 'yyyyMMdd-HHmmss')) }
    Copy-Item -LiteralPath $dll -Destination $target -Force
    Write-Output "Deployed $target"
    Get-FileHash -LiteralPath $target -Algorithm SHA256
} finally { Pop-Location }
