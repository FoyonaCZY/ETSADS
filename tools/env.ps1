$ErrorActionPreference = 'Stop'
$EtsadsRoot = Split-Path $PSScriptRoot -Parent
$localDotnet = Join-Path $EtsadsRoot '.tools\dotnet'
if (Test-Path (Join-Path $localDotnet 'dotnet.exe')) { $env:PATH = $localDotnet + ';' + $env:PATH }
# Respect the selected toolchain. Merely finding vswhere.exe does not mean
# the C++ linker/Windows SDK is installed, or that rustup selects MSVC.
# A portable SDK may be selected without changing the machine's global PATH.
if ($env:ETSADS_CARGO) {
    $EtsadsCargo = (Get-Command $env:ETSADS_CARGO -ErrorAction Stop).Source
} else {
    $EtsadsCargo = (Get-Command cargo.exe -ErrorAction Stop).Source
}
$env:PATH = (Split-Path $EtsadsCargo -Parent) + ';' + $env:PATH

