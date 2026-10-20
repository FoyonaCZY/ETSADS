[CmdletBinding()]
param([switch]$IncludeUi)
. (Join-Path $PSScriptRoot 'env.ps1')
$toolDir=Split-Path $EtsadsCargo -Parent
$formatTool=Join-Path $toolDir 'cargo-fmt.exe'
$clippyTool=Join-Path $toolDir 'cargo-clippy.exe'
if (-not (Test-Path -LiteralPath $formatTool) -or -not (Test-Path -LiteralPath $clippyTool)) { throw '所选Rust工具链缺少rustfmt或clippy组件。请为该工具链安装组件后再运行。' }
Push-Location $EtsadsRoot
try {
    # Cargo may prefer rustup shims from CARGO_HOME even with a portable Cargo
    # selected. Invoke that SDK's companion tools to keep toolchains consistent.
    & $formatTool fmt --all -- --check
    if ($LASTEXITCODE) { throw 'Rust formatting failed' }
    & $EtsadsCargo test --workspace --locked
    if ($LASTEXITCODE) { throw 'Rust tests failed' }
    & $clippyTool clippy --workspace --all-targets --locked -- -D warnings
    if ($LASTEXITCODE) { throw 'Clippy failed' }
    if ($IncludeUi) { & $EtsadsCargo build -p etsads --features ui --locked; if ($LASTEXITCODE) { throw 'UI build failed' } }
} finally { Pop-Location }

