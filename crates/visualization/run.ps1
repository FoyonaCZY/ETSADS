#requires -Version 7.2
# Optional Rerun diagnostic. Paths are resolved from the caller's directory.
[CmdletBinding()]
param(
    [ValidateSet('Build','Export','Live','Help')][string]$Mode='Help',
    [string]$Run,
    [string]$Output,
    [string]$Map,
    [string]$Ffprobe='ffprobe',
    [ValidateRange(1,3600)][int]$Seconds=30,
    [switch]$Connect,
    [switch]$NoVideo,
    [switch]$InteriorC1,
    [switch]$NoBuild,
    [switch]$Offline,
    [string]$CargoPath
)
$ErrorActionPreference='Stop'
$projectRoot=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$target=Join-Path $projectRoot 'target/visualization-rerun'
$binary=Join-Path $target ('release/etsads-rerun'+$(if($IsWindows){'.exe'}else{''}))
if($Mode -eq 'Help') {
    Write-Output 'Optional Rerun: -Mode Build | Export -Run DIR -Output NEW.rrd | Live -Output NEW.rrd (or -Connect). Uses release; -NoBuild explicitly reuses an existing release binary. See RERUN.md.'
    return
}
if($Mode -eq 'Build' -and $NoBuild){throw '-Mode Build cannot use -NoBuild.'}
if($Mode -ne 'Live' -and ($Connect -or $NoVideo)){throw '-Connect and -NoVideo are Live-only.'}
if($Mode -eq 'Export' -and (-not $Run -or -not $Output)){throw 'Export requires -Run and a new -Output .rrd path.'}
if($Mode -eq 'Live' -and ([bool]$Output -eq [bool]$Connect)){throw 'Live requires exactly one of -Output or -Connect.'}
if($Run){
    $Run=(Resolve-Path -LiteralPath $Run).Path
    if(-not (Test-Path -LiteralPath (Join-Path $Run 'meta.json') -PathType Leaf)){throw 'Run must be a recording directory containing meta.json.'}
}
if($Map){$Map=(Resolve-Path -LiteralPath $Map).Path}
if($Output){
    $Output=$ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($Output)
    foreach($path in @($Output,[IO.Path]::ChangeExtension($Output,'report.json'),[IO.Path]::ChangeExtension($Output,'video-associations.json'))){
        if(Test-Path -LiteralPath $path){throw 'Output or its report already exists; choose a new .rrd path.'}
    }
}
if($CargoPath){$CargoPath=(Get-Command $CargoPath -ErrorAction Stop).Source}
if($Ffprobe.Contains('/') -or $Ffprobe.Contains('\')){$Ffprobe=(Resolve-Path -LiteralPath $Ffprobe).Path}
# Resolve paths before changing location so quoted Chinese/space paths retain
# the same meaning when the script is launched outside the repository root.
$previousPath=$env:PATH
$previousCargo=$env:ETSADS_CARGO
Push-Location -LiteralPath $projectRoot
try {
    if(-not $NoBuild){
        if($CargoPath){$env:ETSADS_CARGO=$CargoPath}
        . (Join-Path $projectRoot 'tools/env.ps1')
        $buildArgs=@('build','--release','--locked','-p','etsads-visualization','--features','rerun','--bin','etsads-rerun','--target-dir',$target)
        if($Offline){$buildArgs+='--offline'}
        & $EtsadsCargo @buildArgs
        if($LASTEXITCODE -ne 0){throw "Rerun build failed with exit $LASTEXITCODE; no old binary was launched."}
    }
    if(-not (Test-Path -LiteralPath $binary -PathType Leaf)){throw 'Release executable is absent. Run -Mode Build first or omit -NoBuild.'}
    if($Mode -eq 'Build'){Write-Output $binary;return}
    $arguments=@($Mode.ToLowerInvariant())
    if($Mode -eq 'Export'){$arguments+=@('--run',$Run,'--output',$Output,'--ffprobe',$Ffprobe)}
    else {
        $arguments+=@('--seconds',"$Seconds")
        if($Connect){$arguments+='--connect'}else{$arguments+=@('--output',$Output)}
        if($NoVideo){$arguments+='--no-video'}
    }
    if($Map){$arguments+=@('--map',$Map)}
    if($InteriorC1){$arguments+='--interior-c1'}
    & $binary @arguments
    if($LASTEXITCODE -ne 0){throw "Rerun $Mode failed with exit $LASTEXITCODE. Keep its error and incomplete output for diagnosis."}
} finally {
    Pop-Location
    $env:PATH=$previousPath
    $env:ETSADS_CARGO=$previousCargo
}
