[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$GameDirectory,
    [ValidateSet('inspect', 'export')][string]$Command = 'export',
    [string]$Output = (Join-Path $PSScriptRoot 'output'),
    [string[]]$Sector = @(),
    [string[]]$Mod = @(),
    [switch]$RuntimeJson
)
$ErrorActionPreference = 'Stop'
$repository = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$localDotnet = Join-Path $repository '.tools/dotnet/dotnet.exe'
if (Test-Path -LiteralPath $localDotnet) {
    $dotnet = $localDotnet
} else {
    $installed = Get-Command dotnet -ErrorAction SilentlyContinue
    if (-not $installed) { throw '.NET 10 SDK is required. Use the repository bootstrap or install it from https://dotnet.microsoft.com/download/dotnet/10.0.' }
    $dotnet = $installed.Source
}
$env:DOTNET_CLI_TELEMETRY_OPTOUT = '1'
& $dotnet build (Join-Path $PSScriptRoot 'MapExport.csproj') -c Release -p:RestoreLockedMode=true
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$exportArguments = @($Command, '--game-dir', $GameDirectory, '--output', $Output)
foreach ($path in $Mod) { $exportArguments += @('--mod', $path) }
foreach ($coordinate in $Sector) { $exportArguments += @('--sector', $coordinate) }
if ($RuntimeJson) { $exportArguments += '--runtime-json' }
& $dotnet (Join-Path $PSScriptRoot 'bin/Release/net10.0/etsads-map-export.dll') @exportArguments
exit $LASTEXITCODE
