[CmdletBinding()]
param([switch]$Build)
. (Join-Path $PSScriptRoot 'env.ps1')
Push-Location $EtsadsRoot
try {
    $exe=Join-Path $EtsadsRoot 'target\release\etsads.exe'
    if ($Build -or -not (Test-Path -LiteralPath $exe)) {
        & $EtsadsCargo build --release -p etsads -p etsads-supervisor --features etsads/ui --locked
        if ($LASTEXITCODE) { throw '构建失败。请查看上方错误或运行 tools/check.ps1。' }
    }
    & $exe
} finally { Pop-Location }

