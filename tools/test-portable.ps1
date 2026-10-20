[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Package,[string]$Output)
$ErrorActionPreference='Stop'
$packageRoot=(Resolve-Path -LiteralPath $Package).Path
if (-not $Output) { $Output=Join-Path (Split-Path $packageRoot -Parent) ('portable-smoke-'+(Get-Date -Format 'yyyyMMdd-HHmmss')) }
if (Test-Path -LiteralPath $Output) { throw '验证输出目录已存在，请使用新目录。' }
$entries=Get-Content -LiteralPath (Join-Path $packageRoot 'sha256.json') -Raw | ConvertFrom-Json
foreach ($entry in $entries) {
    $file=[IO.Path]::GetFullPath((Join-Path $packageRoot $entry.file))
    if (-not $file.StartsWith($packageRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Package manifest path escapes the package' }
    if ((Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash -ne $entry.sha256) { throw "Package hash mismatch: $($entry.file)" }
}
New-Item -ItemType Directory -Path $Output | Out-Null
$outputRoot=(Resolve-Path -LiteralPath $Output).Path
$savedPath=$env:PATH
Push-Location -LiteralPath $packageRoot
try {
    # Verify that executables do not accidentally rely on a developer's GNU,
    # Rust, .NET or ffmpeg directories. This test never opens game mappings.
    $env:PATH="$env:SystemRoot\System32;$env:SystemRoot"
    & .\etsads-cli.exe --help > (Join-Path $outputRoot 'cli-help.txt')
    if ($LASTEXITCODE) { throw 'Standalone CLI did not start' }
    & .\etsads-sysid.exe --help > (Join-Path $outputRoot 'sysid-help.txt')
    if ($LASTEXITCODE) { throw 'Standalone sysid did not start' }
    $overlayAvailable=Test-Path -LiteralPath '.\etsads-overlay.exe'
    if ($overlayAvailable) {
        & .\etsads-overlay.exe --help > (Join-Path $outputRoot 'overlay-help.txt')
        if ($LASTEXITCODE) { throw 'Standalone overlay diagnostics did not start' }
    }
    $recording=Join-Path $outputRoot '合成 回放'
    & .\etsads-cli.exe record-bench $recording --frames 1200 > (Join-Path $outputRoot 'record.json')
    if ($LASTEXITCODE) { throw 'Portable recording benchmark failed' }
    & .\etsads-cli.exe replay $recording > (Join-Path $outputRoot 'replay.json')
    if ($LASTEXITCODE) { throw 'Portable replay failed' }
    $report=Get-Content -LiteralPath (Join-Path $outputRoot 'replay.json') -Raw | ConvertFrom-Json
    if ($report.source -ne 'synthetic' -or -not $report.completed -or $report.telemetry_frames -ne 1200 -or $report.frame_gaps -ne 0 -or $report.nonmonotonic -ne 0) { throw 'Portable recording integrity mismatch' }
    [pscustomobject]@{
        source='portable-offline-smoke'
        live_game_test=$false
        package_manifest_sha256=(Get-FileHash -LiteralPath (Join-Path $packageRoot 'sha256.json')).Hash
        package_files_checked=$entries.Count
        system_only_path=$true
        recording_and_replay='passed'
        telemetry_frames=$report.telemetry_frames
        overlay_executable_tested=$overlayAvailable
        ui='separate visual check required'
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $outputRoot 'result.json') -Encoding utf8
    Write-Output "便携验证通过：$outputRoot"
} finally {
    $env:PATH=$savedPath
    Pop-Location
}
