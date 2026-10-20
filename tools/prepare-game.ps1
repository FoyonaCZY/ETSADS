[CmdletBinding()]
param([string]$UserDir = (Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'Euro Truck Simulator 2'), [string]$SteamDir = (Join-Path ${env:ProgramFiles(x86)} 'Steam'))
$ErrorActionPreference = 'Stop'
if (Get-Process eurotrucks2 -ErrorAction SilentlyContinue) { throw '请退出游戏后准备配置，避免游戏退出时覆盖文件。' }
$root = Split-Path $PSScriptRoot -Parent
$source = (Resolve-Path -LiteralPath $UserDir).Path
$backup = Join-Path $root ('artifacts\game-backup-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
New-Item -ItemType Directory -Path $backup | Out-Null
foreach ($name in @('config.cfg','profiles','steam_profiles')) {
    $path = Join-Path $source $name
    if (Test-Path -LiteralPath $path) { Copy-Item -LiteralPath $path -Destination $backup -Recurse }
}
# Steam Cloud saves live under userdata, not Documents/steam_profiles (which
# often contains only controller/config files). Back up the ETS2 app only.
$userdata = Join-Path $SteamDir 'userdata'
if (Test-Path -LiteralPath $userdata) {
    foreach ($account in Get-ChildItem -LiteralPath $userdata -Directory) {
        $remote = Join-Path $account.FullName '227300\remote'
        if (Test-Path -LiteralPath $remote) {
            $destination = Join-Path $backup ('steam-cloud\' + $account.Name)
            New-Item -ItemType Directory -Force -Path $destination | Out-Null
            Copy-Item -LiteralPath $remote -Destination $destination -Recurse
        }
    }
}
Get-ChildItem -LiteralPath $backup -Recurse -File | ForEach-Object {
    [pscustomobject]@{ path = $_.FullName.Substring($backup.Length + 1); sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $backup 'hashes.json') -Encoding utf8
$configPath = Join-Path $source 'config.cfg'
$config = Get-Content -LiteralPath $configPath -Raw
$settings = @{g_developer='1';g_console='1';r_mode_width='1920';r_mode_height='1080';r_fullscreen='0';r_fullscreen_borderless='1';t_limit_fps='60'}
foreach ($key in $settings.Keys) {
    $pattern = '(?m)^uset ' + [regex]::Escape($key) + ' "[^"]*"'
    $line = 'uset ' + $key + ' "' + $settings[$key] + '"'
    if ($config -match $pattern) { $config = [regex]::Replace($config,$pattern,$line) } else { $config += "`r`n$line`r`n" }
}
[IO.File]::WriteAllText($configPath,$config,[Text.UTF8Encoding]::new($false))
Write-Output "已备份存档及配置：$backup"
Write-Output '已设置开发者控制台、1920×1080 无边框、60 FPS。存档内容未改动。'

