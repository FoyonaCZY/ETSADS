#requires -Version 7.2
[CmdletBinding()]
param(
    [ValidateRange(1,65535)][int]$Port=9876,
    [string]$Tool,
    [string]$ArgumentsJson='{}'
)
$ErrorActionPreference='Stop'
$projectRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$start=[Diagnostics.ProcessStartInfo]::new()
$start.FileName=Join-Path $projectRoot '.tools/rerun-0.38.1/rerun.exe'
if(-not (Test-Path -LiteralPath $start.FileName -PathType Leaf)){throw 'Pinned viewer is absent; use view.ps1 -InstallViewer.'}
if((Get-FileHash -LiteralPath $start.FileName -Algorithm SHA256).Hash -ne '0c0205d63107b1fd286b0a4d2044f70feaf0f4a5a19ab6a49aaa1de7812b6a2a'){throw 'Viewer SHA256 mismatch.'}
$start.ArgumentList.Add('viewer-mcp')
$start.ArgumentList.Add('--endpoint')
$start.ArgumentList.Add("http://127.0.0.1:$Port")
$start.RedirectStandardInput=$true
$start.RedirectStandardOutput=$true
# Inherit stderr: an unread redirected pipe can deadlock a noisy helper.
$start.RedirectStandardError=$false
$start.UseShellExecute=$false
$start.CreateNoWindow=$true
$start.Environment['CI']='1'
$clientProcess=[Diagnostics.Process]::Start($start)
function Send-ViewerRequest($requestId,$method,$parameters) {
    $request=@{jsonrpc='2.0';id=$requestId;method=$method;params=$parameters}
    $clientProcess.StandardInput.WriteLine(($request | ConvertTo-Json -Depth 40 -Compress))
    $clientProcess.StandardInput.Flush()
    do {
        $line=$clientProcess.StandardOutput.ReadLineAsync().WaitAsync([TimeSpan]::FromSeconds(15)).GetAwaiter().GetResult()
        if ($null -eq $line) {throw 'Viewer MCP process exited before responding'}
        $response=$line | ConvertFrom-Json
    } while ($response.id -ne $requestId)
    if ($response.error) {throw ($response.error | ConvertTo-Json -Compress)}
    return $response.result
}
try {
    $null=Send-ViewerRequest 1 'initialize' @{protocolVersion='2024-11-05';capabilities=@{};clientInfo=@{name='etsads-viewer-validation';version='1'}}
    $clientProcess.StandardInput.WriteLine('{"jsonrpc":"2.0","method":"notifications/initialized"}')
    $clientProcess.StandardInput.Flush()
    $result=if ($Tool) {Send-ViewerRequest 2 'tools/call' @{name=$Tool;arguments=($ArgumentsJson | ConvertFrom-Json -AsHashtable)}} else {Send-ViewerRequest 2 'tools/list' @{}}
    if ($result.isError) {throw ($result.content | ConvertTo-Json -Depth 8 -Compress)}
    $result
} finally {
    try {$clientProcess.StandardInput.Close()} catch {}
    if (!$clientProcess.WaitForExit(3000)) {$clientProcess.Kill($true);$null=$clientProcess.WaitForExit(3000)}
    $clientProcess.Dispose()
}
