# Offline only: never starts the UI, supervisor, installer or game mappings.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$Package,
    [string]$Output,
    # Optional exact comparison with the checkout used for this build.
    [string]$SourceRoot,
    [ValidateRange(5,60)][int]$CommandTimeoutSeconds=30
)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
$utf8=[Text.UTF8Encoding]::new($false)
function Write-JsonFile($Value,[string]$Path) {
    [IO.File]::WriteAllText($Path,($Value | ConvertTo-Json -Depth 12),$utf8)
}
function Read-BoundedJson([string]$Path) {
    $item=Get-Item -LiteralPath $Path
    if ($item.PSIsContainer -or $item.Length -gt 16MB) { throw "JSON file is invalid or exceeds 16 MiB: $Path" }
    Get-Content -LiteralPath $Path -Raw -Encoding UTF8 | ConvertFrom-Json
}
function Resolve-ManifestFile([string]$Root,[string]$Name) {
    if ([string]::IsNullOrWhiteSpace($Name) -or [IO.Path]::IsPathRooted($Name) -or $Name.Contains(':') -or
        ($Name -split '[/\\]' | Where-Object { $_ -eq '..' -or $_ -eq '.' -or $_ -eq '' })) {
        throw "Unsafe manifest path: $Name"
    }
    $path=[IO.Path]::GetFullPath((Join-Path $Root $Name))
    if (-not $path.StartsWith($Root+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) {
        throw "Manifest path escaped its root: $Name"
    }
    $path
}
function Read-Inventory([string]$Path,[string]$Root) {
    # Windows PowerShell 5.1 emits a JSON array as one pipeline object; unroll it.
    $rows=@(Read-BoundedJson $Path | ForEach-Object { $_ })
    if ($rows.Count -eq 0 -or $rows.Count -gt 50000) { throw "Inventory must contain 1..50000 files: $Path" }
    $names=[Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
    foreach ($row in $rows) {
        if ($row.file -isnot [string] -or $row.sha256 -isnot [string] -or $row.sha256 -notmatch '^[0-9a-fA-F]{64}$') {
            throw "Malformed inventory entry in $Path"
        }
        $full=Resolve-ManifestFile $Root $row.file
        $normalized=$row.file.Replace('\','/')
        if (-not $names.Add($normalized)) { throw "Duplicate inventory entry: $normalized" }
    }
    $rows
}
function Get-BoundedFiles([string]$Root) {
    $dirs=[Collections.Generic.Queue[string]]::new()
    $dirs.Enqueue($Root)
    $files=[Collections.Generic.List[object]]::new()
    [long]$bytes=0
    $directories=0
    while ($dirs.Count -gt 0) {
        $directories++
        if ($directories -gt 50000) { throw 'Package directory budget exceeded' }
        foreach ($item in Get-ChildItem -LiteralPath $dirs.Dequeue() -Force) {
            if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) { throw "Reparse points are not portable package files: $($item.FullName)" }
            if ($item.PSIsContainer) { $dirs.Enqueue($item.FullName) } else {
                $bytes+=$item.Length
                if ($bytes -gt 2GB -or $files.Count -ge 50000) { throw 'Package exceeds 2 GiB or 50000 files' }
                $files.Add($item)
            }
        }
    }
    $files.ToArray()
}
function ConvertTo-NativeArgument([string]$Value) {
    # Windows command-line escaping for Windows PowerShell/.NET Framework.
    # On PowerShell 7, ProcessStartInfo.ArgumentList is used instead.
    '"'+[regex]::Replace([regex]::Replace($Value,'(\\*)"','$1$1\"'),'(\\+)$','$1$1')+'"'
}
function Invoke-BoundedExecutable([string]$Executable,[string[]]$Arguments,[string]$Name,[string]$WorkingDirectory) {
    $start=[Diagnostics.ProcessStartInfo]::new()
    $start.FileName=$Executable
    $start.WorkingDirectory=$WorkingDirectory
    $start.UseShellExecute=$false
    $start.CreateNoWindow=$true
    $start.RedirectStandardOutput=$true
    $start.RedirectStandardError=$true
    $start.StandardOutputEncoding=$utf8
    $start.StandardErrorEncoding=$utf8
    $start.EnvironmentVariables['PATH']=$minimalPath
    foreach ($key in @($start.EnvironmentVariables.Keys)) {
        if ($key -match '^(ETSADS_|CARGO_|RUST|DOTNET_)') { $start.EnvironmentVariables.Remove($key) }
    }
    if ($start.PSObject.Properties['ArgumentList']) {
        foreach ($argument in $Arguments) { $start.ArgumentList.Add($argument) }
    } else {
        $start.Arguments=($Arguments | ForEach-Object { ConvertTo-NativeArgument $_ }) -join ' '
    }
    $process=[Diagnostics.Process]::new()
    $process.StartInfo=$start
    $timer=[Diagnostics.Stopwatch]::StartNew()
    try {
        if (-not $process.Start()) { throw "Failed to start $Name" }
        $stdout=$process.StandardOutput.ReadToEndAsync()
        $stderr=$process.StandardError.ReadToEndAsync()
        if (-not $process.WaitForExit($CommandTimeoutSeconds*1000)) {
            $process.Kill()
            [void]$process.WaitForExit(2000)
            throw "$Name exceeded $CommandTimeoutSeconds seconds; the test process was stopped"
        }
        if (-not $stdout.Wait(2000) -or -not $stderr.Wait(2000)) { throw "$Name output streams did not close" }
        $outText=$stdout.Result
        $errText=$stderr.Result
        if ($outText.Length -gt 4MB -or $errText.Length -gt 4MB) { throw "$Name output exceeded the smoke-test budget" }
        [IO.File]::WriteAllText((Join-Path $outputRoot ($Name+'.stdout.txt')),$outText,$utf8)
        [IO.File]::WriteAllText((Join-Path $outputRoot ($Name+'.stderr.txt')),$errText,$utf8)
        $commands.Add([pscustomobject]@{name=$Name;arguments=$Arguments;exit_code=$process.ExitCode;elapsed_seconds=$timer.Elapsed.TotalSeconds})
        if ($process.ExitCode -ne 0) { throw "$Name exited with $($process.ExitCode); see captured stderr" }
        $outText
    } finally { $process.Dispose() }
}
function Assert-SyntheticIntegrity($Report,[string]$Name) {
    if ($Report.source -cne 'synthetic' -or $Report.completed -ne $true -or
        $Report.telemetry_frames -ne 1200 -or $Report.messages -ne 1200 -or
        $Report.frame_gaps -ne 0 -or $Report.nonmonotonic -ne 0 -or
        $Report.topics.'/telemetry' -ne 1200) { throw "$Name did not preserve the complete 1200-frame synthetic recording" }
}

$packageRoot=(Resolve-Path -LiteralPath $Package).Path.TrimEnd('\','/')
if (-not (Test-Path -LiteralPath $packageRoot -PathType Container)) { throw 'Package must be an unpacked directory' }
if (-not $Output) { $Output=Join-Path ([IO.Path]::GetTempPath()) ('ETSADS 便携 验收 '+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+[Guid]::NewGuid().ToString('N')) }
$outputRoot=[IO.Path]::GetFullPath($Output)
if ($outputRoot.Equals($packageRoot,[StringComparison]::OrdinalIgnoreCase) -or
    $outputRoot.StartsWith($packageRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw 'Smoke output must be outside the package' }
if (Test-Path -LiteralPath $outputRoot) { throw 'Smoke output already exists; use a new directory to preserve previous evidence' }
New-Item -ItemType Directory -Path $outputRoot | Out-Null
$minimalPath=(Join-Path $env:SystemRoot 'System32')+';'+$env:SystemRoot
$commands=[Collections.Generic.List[object]]::new()
try {
    $manifest=Join-Path $packageRoot 'sha256.json'
    $inventory=@(Read-Inventory $manifest $packageRoot)
    $files=@(Get-BoundedFiles $packageRoot)
    $indexed=[Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
    foreach ($entry in $inventory) {
        if ($entry.file.Replace('\','/') -ieq 'sha256.json') { throw 'Package manifest cannot hash itself' }
        [void]$indexed.Add($entry.file.Replace('\','/'))
        $path=Resolve-ManifestFile $packageRoot $entry.file
        if ((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ine $entry.sha256) { throw "Package hash mismatch: $($entry.file)" }
    }
    foreach ($file in $files) {
        $relative=$file.FullName.Substring($packageRoot.Length+1).Replace('\','/')
        if ($relative -ine 'sha256.json' -and -not $indexed.Contains($relative)) { throw "Unlisted package file: $relative" }
    }
    foreach ($name in @('etsads.exe','etsads-cli.exe','etsads-sysid.exe','etsads-overlay.exe','etsads-supervisor.exe','etsads_bridge.dll','etsads_gfx.dll','install-plugin.ps1','source-sha256.json','build-info.json','LICENSE','docs/ACCEPTANCE.md')) {
        if (-not $indexed.Contains($name)) { throw "Required package file is missing from the manifest: $name" }
    }
    $build=Read-BoundedJson (Join-Path $packageRoot 'build-info.json')
    $source=@(Read-Inventory (Join-Path $packageRoot 'source-sha256.json') $packageRoot)
    if ($build.format_version -ne 1 -or $build.source_inventory_files -ne $source.Count -or
        [string]::IsNullOrWhiteSpace($build.source_inventory_scope) -or
        $build.live_phase_acceptance -notmatch '^incomplete' -or
        $build.features -notcontains 'etsads/ui' -or $build.features -notcontains 'etsads-gfx/native-dx11') { throw 'Build metadata/source inventory is incomplete or inconsistent' }
    $lock=@($source | Where-Object { $_.file -ceq 'Cargo.lock' })
    if ($lock.Count -ne 1 -or $lock[0].sha256 -ine $build.cargo_lock_sha256) { throw 'Cargo.lock is not consistently bound to the source inventory' }
    $sourceComparison='not requested; packaged inventory consistency checked, source tree not certified'
    if ($SourceRoot) {
        $sourceDirectory=(Resolve-Path -LiteralPath $SourceRoot).Path.TrimEnd('\','/')
        foreach ($entry in $source) {
            $path=Resolve-ManifestFile $sourceDirectory $entry.file
            if ((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ine $entry.sha256) { throw "Source differs from the packaged build: $($entry.file)" }
        }
        $sourceComparison='all inventoried files match the supplied source directory; extra files are outside this comparison'
    }
    # Copy exactly the authenticated package files. Both executable location
    # and working/recording paths exercise Unicode plus spaces on every run.
    $copyRoot=Join-Path $outputRoot '便携 程序'
    New-Item -ItemType Directory -Path $copyRoot | Out-Null
    foreach ($file in $files) {
        $relative=$file.FullName.Substring($packageRoot.Length+1)
        $destination=Resolve-ManifestFile $copyRoot $relative
        [IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($destination)) | Out-Null
        Copy-Item -LiteralPath $file.FullName -Destination $destination
    }
    foreach ($entry in $inventory) {
        if ((Get-FileHash -LiteralPath (Resolve-ManifestFile $copyRoot $entry.file) -Algorithm SHA256).Hash -ine $entry.sha256) { throw "Copied package hash mismatch: $($entry.file)" }
    }
    $working=Join-Path $outputRoot '独立 工作目录'
    New-Item -ItemType Directory -Path $working | Out-Null
    foreach ($binary in @('etsads-cli','etsads-sysid','etsads-overlay')) {
        $help=Invoke-BoundedExecutable (Join-Path $copyRoot ($binary+'.exe')) @('--help') ($binary+'-help') $working
        if ($help -notmatch '(?i)usage:') { throw "$binary help output did not contain usage information" }
    }
    $cli=Join-Path $copyRoot 'etsads-cli.exe'
    $recording=Join-Path $working '合成 1200 帧'
    $record=Invoke-BoundedExecutable $cli @('record-bench',$recording,'--frames','1200') 'record' $working | ConvertFrom-Json
    Assert-SyntheticIntegrity $record.report 'record-bench'
    $metadata=Read-BoundedJson (Join-Path $recording 'meta.json')
    if ($metadata.source -cne 'synthetic' -or $metadata.completed -ne $true) { throw 'Stored metadata is not a completed synthetic recording' }
    $replay=Invoke-BoundedExecutable $cli @('replay',$recording,'--snapshots') 'replay' $working | ConvertFrom-Json
    Assert-SyntheticIntegrity $replay 'replay --snapshots'
    if ($replay.associations.snapshots -ne 1200 -or $replay.phase_acceptance -ne $false -or
        $replay.execution -cne 'read-only recorded snapshot replay' -or
        $replay.associations.extension_frames -ne 0 -or $replay.associations.missing_extension_frames -ne 1200 -or
        $replay.associations.missing_config_frames -ne 1200) { throw 'Replay snapshot association or evidence classification mismatch' }
    Write-JsonFile ([pscustomobject]@{
        schema_version=1;source='portable-offline-smoke';passed=$true;live_game_test=$false;phase_acceptance=$false
        package=$packageRoot;package_manifest_sha256=(Get-FileHash -LiteralPath $manifest -Algorithm SHA256).Hash
        package_files_checked=$inventory.Count;source_files_checked=$source.Count;source_comparison=$sourceComparison
        source_manifest_sha256=(Get-FileHash -LiteralPath (Join-Path $packageRoot 'source-sha256.json') -Algorithm SHA256).Hash
        copied_package=$copyRoot;system_only_path=$minimalPath;unicode_space_executable_and_recording_paths=$true
        telemetry_frames=1200;snapshots=$replay.associations.snapshots;recording_source=$replay.source
        commands=$commands.ToArray();ui='not launched; separate visual check required';game_or_supervisor_ipc='not opened'
    }) (Join-Path $outputRoot 'result.json')
    Write-Output "Offline portable smoke passed: $outputRoot"
} catch {
    Write-JsonFile ([pscustomobject]@{schema_version=1;source='portable-offline-smoke';passed=$false;live_game_test=$false;phase_acceptance=$false;error=$_.Exception.Message;commands=$commands.ToArray()}) (Join-Path $outputRoot 'result.json')
    throw
}
