# Isolated filesystem regression. No real game path or process is accessed.
[CmdletBinding()]
param([string]$Output)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if (-not $Output) { $Output=Join-Path ([IO.Path]::GetTempPath()) ('ETSADS 安装器 测试 '+[Guid]::NewGuid().ToString('N')) }
$root=[IO.Path]::GetFullPath($Output)
if (Test-Path -LiteralPath $root) { throw 'Test output exists; choose a new directory' }
New-Item -ItemType Directory -Path $root | Out-Null
$utf8=[Text.UTF8Encoding]::new($false)
function Snapshot([string]$Directory) {
    @(Get-ChildItem -LiteralPath $Directory -Recurse -File | Sort-Object FullName | ForEach-Object {
        [pscustomobject]@{path=$_.FullName.Substring($Directory.Length+1);hash=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash}
    }) | ConvertTo-Json -Compress
}
function Invoke-FakeInstaller([string]$Installer,[string]$Game,[bool]$Running) {
    if (-not $Game.StartsWith($root+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Fake game escaped test directory' }
    # Explicit test double for the process gate. The child script only sees
    # these synthetic process results and the literal fake installation.
    function Get-Process {
        param([string]$Name,[object]$ErrorAction)
        if ($Running) { [pscustomobject]@{ProcessName='synthetic eurotrucks2'} }
    }
    & $Installer -GameDir $Game -NativeGraphics | Out-Null
}
$results=@()
foreach ($case in @('missing-gfx-existing','gfx-is-directory','missing-bridge-new','running-game','complete-package')) {
    $caseRoot=Join-Path $root $case
    $package=Join-Path $caseRoot '便携 包'
    $game=Join-Path $caseRoot '假游戏 Euro Truck Simulator 2'
    $bin=Join-Path $game 'bin\win_x64'
    New-Item -ItemType Directory -Path $package,$bin | Out-Null
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'install-plugin.ps1') -Destination $package
    [IO.File]::WriteAllText((Join-Path $bin 'eurotrucks2.exe'),'SYNTHETIC FILE, NOT AN EXECUTABLE',$utf8)
    if ($case -ne 'missing-bridge-new') { [IO.File]::WriteAllText((Join-Path $package 'etsads_bridge.dll'),'synthetic new bridge',$utf8) }
    if ($case -eq 'gfx-is-directory') { New-Item -ItemType Directory -Path (Join-Path $package 'etsads_gfx.dll') | Out-Null }
    elseif ($case -in @('missing-bridge-new','running-game','complete-package')) { [IO.File]::WriteAllText((Join-Path $package 'etsads_gfx.dll'),'synthetic new gfx',$utf8) }
    $plugins=Join-Path $bin 'plugins'
    if ($case -ne 'missing-bridge-new') {
        New-Item -ItemType Directory -Path $plugins | Out-Null
        foreach ($name in @('etsads_bridge.dll','etsads_gfx.dll','ets2la_capture.dll')) {
            [IO.File]::WriteAllText((Join-Path $plugins $name),('synthetic old '+$name),$utf8)
        }
    }
    $before=Snapshot $game
    $failure=$null
    try { Invoke-FakeInstaller (Join-Path $package 'install-plugin.ps1') $game ($case -eq 'running-game') }
    catch { $failure=$_.Exception.Message }
    if ($case -eq 'complete-package') {
        if ($failure) { throw "Complete fake package failed: $failure" }
        foreach ($name in @('etsads_bridge.dll','etsads_gfx.dll')) {
            if ((Get-FileHash -LiteralPath (Join-Path $package $name)).Hash -ne (Get-FileHash -LiteralPath (Join-Path $plugins $name)).Hash) { throw 'Fake installation content mismatch' }
            if (@(Get-ChildItem -LiteralPath $plugins -Filter ($name+'.backup-*')).Count -ne 1) { throw 'Previous fake plugin was not backed up' }
        }
        if ((Test-Path -LiteralPath (Join-Path $plugins 'ets2la_capture.dll')) -or
            @(Get-ChildItem -LiteralPath $plugins -Filter 'ets2la_capture.dll.disabled-*').Count -ne 1) { throw 'Fake capture was not preserved by renaming' }
    } else {
        if (-not $failure) { throw "Expected installer failure: $case" }
        if ($before -cne (Snapshot $game)) { throw "Partial fake installation detected: $case" }
        if ($case -eq 'missing-bridge-new' -and (Test-Path -LiteralPath $plugins)) { throw 'Missing source created a plugins directory' }
    }
    $results+=[pscustomobject]@{case=$case;passed=$true;expected_failure=$failure}
}
[IO.File]::WriteAllText((Join-Path $root 'result.json'),([pscustomobject]@{
    source='synthetic-installer-filesystem-regression';real_game_access=$false;process_gate='explicit test double';cases=$results
} | ConvertTo-Json -Depth 5),$utf8)
Write-Output "Five isolated installer cases passed: $root"
