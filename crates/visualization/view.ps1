#requires -Version 7.2
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Recording,
    [switch]$InstallViewer,
    # Validate an existing RRD and exit without opening a viewer window/server.
    [switch]$VerifyOnly,
    [string]$HeadlessScreenshot,
    # Omit to render the midpoint of the recorded elapsed timeline.
    [Nullable[double]]$AtSeconds,
    # Video decode/catalog fetch may outlive a few rendered UI frames.
    [ValidateRange(1,45)][int]$SettleSeconds=10
)
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$viewerDirectory = Join-Path $projectRoot '.tools/rerun-0.38.1'
$viewer = Join-Path $viewerDirectory 'rerun.exe'
$expectedSha256 = '0c0205d63107b1fd286b0a4d2044f70feaf0f4a5a19ab6a49aaa1de7812b6a2a'
$source = 'https://github.com/rerun-io/rerun/releases/download/0.38.1/rerun-cli-0.38.1-x86_64-pc-windows-msvc.exe'
$recordingPath = (Resolve-Path -LiteralPath $Recording).Path
if(-not (Test-Path -LiteralPath $recordingPath -PathType Leaf)){throw 'Recording must be an existing .rrd file.'}
if($VerifyOnly -and ($HeadlessScreenshot -or $null -ne $AtSeconds)){throw '-VerifyOnly cannot render or select screenshot time.'}
if($null -ne $AtSeconds -and (-not $HeadlessScreenshot -or -not [double]::IsFinite([double]$AtSeconds))){throw '-AtSeconds requires a screenshot and a finite time.'}
if($HeadlessScreenshot){
    $HeadlessScreenshot=$ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($HeadlessScreenshot)
    foreach($path in @($HeadlessScreenshot,($HeadlessScreenshot+'.viewer-state.json'),($HeadlessScreenshot+'.viewer-logs.json'))){
        if(Test-Path -LiteralPath $path){throw 'Screenshot or its diagnostics already exist; choose a new output path.'}
    }
}
if ($InstallViewer -and !(Test-Path -LiteralPath $viewer)) {
    New-Item -ItemType Directory -Force -Path $viewerDirectory | Out-Null
    $download = Join-Path $viewerDirectory 'rerun.exe.download'
    Invoke-WebRequest -Uri $source -OutFile $download
    if ((Get-FileHash -LiteralPath $download -Algorithm SHA256).Hash -ne $expectedSha256) {
        throw 'Official viewer download SHA256 does not match the pinned release.'
    }
    Move-Item -LiteralPath $download -Destination $viewer
}
if (!(Test-Path -LiteralPath $viewer)) { throw 'Viewer is absent. Re-run with -InstallViewer to install the pinned portable release locally.' }
if ((Get-FileHash -LiteralPath $viewer -Algorithm SHA256).Hash -ne $expectedSha256) { throw 'Viewer SHA256 mismatch; refusing to execute an unknown binary.' }
& $viewer rrd verify $recordingPath
if ($LASTEXITCODE -ne 0) { throw 'Rerun rejected the recording.' }
if($VerifyOnly){Write-Output "Verified without opening viewer: $recordingPath";return}
$viewerArgs = @($recordingPath, '--port', 'auto', '--hide-welcome-screen')
if ($HeadlessScreenshot) {
    $screenshotPath = [IO.Path]::GetFullPath($HeadlessScreenshot)
    New-Item -ItemType Directory -Force -Path (Split-Path $screenshotPath -Parent) | Out-Null
    if (Test-Path -LiteralPath $screenshotPath) {throw 'Screenshot already exists; choose a new output path.'}
    # --screenshot-to can fire before the lazy catalog has fetched video and
    # scene chunks. Use the official viewer API to seek, settle and capture.
    $portLease=[Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback,0)
    $portLease.Start()
    $viewerPort=$portLease.LocalEndpoint.Port
    $portLease.Stop()
    $start=[Diagnostics.ProcessStartInfo]::new()
    $start.FileName=$viewer
    foreach ($arg in @($recordingPath,'--headless','--port',"$viewerPort",'--bind','127.0.0.1','--window-size','1600x1000','--video-decoder','prefer_software','--hide-welcome-screen')) {$start.ArgumentList.Add($arg)}
    $start.UseShellExecute=$false
    $start.CreateNoWindow=$true
    $start.WindowStyle=[Diagnostics.ProcessWindowStyle]::Hidden
    $start.Environment['CI']='1'
    $ownedViewer=[Diagnostics.Process]::Start($start)
    $client=Join-Path $PSScriptRoot 'viewer-mcp.ps1'
    try {
        $viewerState=$null
        for ($attempt=0;$attempt -lt 5;$attempt++) {
            if ($ownedViewer.HasExited) {throw 'Headless viewer exited before loading the recording.'}
            try {
                $result=& $client -Port $viewerPort -Tool rerun_get_viewer_state
                $viewerState=$result.content[0].text | ConvertFrom-Json
                if ($viewerState.recordings.Count -gt 0) {break}
            } catch {if ($attempt -eq 4) {throw}}
            Start-Sleep -Milliseconds 200
        }
        $timeline=$viewerState.recordings[0].timelines | Where-Object {$_.timeline.name -eq 'elapsed'} | Select-Object -First 1
        if (!$timeline) {throw 'Recording has no elapsed timeline for deterministic screenshot selection.'}
        $startNs=if ($null -eq $timeline.time_range.start) {0L} else {[long]$timeline.time_range.start}
        $endNs=[long]$timeline.time_range.end
        $timeNs=if ($null -ne $AtSeconds) {[long]($AtSeconds*1000000000)} else {$startNs+[long](($endNs-$startNs)/2)}
        if ($timeNs -lt $startNs -or $timeNs -gt $endNs) {throw 'Requested screenshot time is outside the recorded range.'}
        $null=& $client -Port $viewerPort -Tool rerun_set_time_cursor -ArgumentsJson (@{timeline=@{name='elapsed'};time=@{time=$timeNs};play=$false} | ConvertTo-Json -Depth 5 -Compress)
        $null=& $client -Port $viewerPort -Tool wait_for -ArgumentsJson '{"min_steps":60,"timeout_secs":10}'
        Start-Sleep -Seconds $SettleSeconds
        # Retain viewer state for diagnosing blank video; file validation and
        # render waiting alone never certify a successful image decode.
        $renderState=& $client -Port $viewerPort -Tool rerun_get_viewer_state
        $renderState | ConvertTo-Json -Depth 30 | Set-Content -LiteralPath ($screenshotPath+'.viewer-state.json')
        $null=& $client -Port $viewerPort -Tool rerun_save_screenshot -ArgumentsJson (@{file_path=$screenshotPath} | ConvertTo-Json -Compress)
        $logs=& $client -Port $viewerPort -Tool rerun_get_viewer_logs
        $logs | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath ($screenshotPath+'.viewer-logs.json')
        if (!(Test-Path -LiteralPath $screenshotPath)) {throw 'Viewer did not produce the requested screenshot.'}
        Write-Output "Rendered elapsed $($timeNs/1000000000) s: $screenshotPath"
    } finally {
        if (!$ownedViewer.HasExited) {$ownedViewer.Kill($true);$null=$ownedViewer.WaitForExit(3000)}
        $ownedViewer.Dispose()
    }
    return
}
# In verification mode the official viewer disables analytics when CI is set.
# Restore the caller's environment rather than changing a machine preference.
$priorCi = $env:CI
try {
    if ($HeadlessScreenshot) { $env:CI = '1' }
    & $viewer @viewerArgs
    if ($LASTEXITCODE -ne 0) { throw "Viewer exited with code $LASTEXITCODE." }
} finally { $env:CI = $priorCi }
