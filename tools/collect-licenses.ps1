[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Output)
. (Join-Path $PSScriptRoot 'env.ps1')
Push-Location $EtsadsRoot
try {
    # Cargo writes UTF-8; Windows PowerShell would decode it with the OEM code
    # page and corrupt non-ASCII package descriptions.
    [Console]::OutputEncoding=[Text.UTF8Encoding]::new($false)
    $compilerInfo=(& rustc -vV | Out-String)
    $target=([regex]::Match($compilerInfo,'(?m)^host: (\S+)')).Groups[1].Value
    if (-not $target) { throw 'Cannot determine the selected Rust host target.' }
    $json=& $EtsadsCargo metadata --format-version 1 --offline --locked --features etsads/ui,etsads-gfx/native-dx11 --filter-platform $target
    if ($LASTEXITCODE) { throw 'Cargo dependency metadata failed.' }
    if ($PSVersionTable.PSVersion.Major -ge 6) {
        $metadata=($json | Out-String) | ConvertFrom-Json
    } else {
        # Windows PowerShell 5.1 caps ConvertFrom-Json at 2 MB; cargo metadata
        # for this workspace is larger. Dictionaries support the same .key access.
        Add-Type -AssemblyName System.Web.Extensions
        $serializer=New-Object System.Web.Script.Serialization.JavaScriptSerializer
        $serializer.MaxJsonLength=[int]::MaxValue
        $metadata=$serializer.DeserializeObject(($json | Out-String))
    }
    $packages=@{}
    $nodes=@{}
    foreach ($package in $metadata.packages) { $packages[$package.id]=$package }
    foreach ($node in $metadata.resolve.nodes) { $nodes[$node.id]=$node }
    $pending=[Collections.Generic.Queue[string]]::new()
    foreach ($package in $metadata.packages) {
        if ($package.name -in @('etsads','etsads-supervisor','etsads-sysid','etsads-overlay','etsads-bridge','etsads-gfx')) { $pending.Enqueue($package.id) }
    }
    $included=[Collections.Generic.HashSet[string]]::new()
    while ($pending.Count) {
        $id=$pending.Dequeue()
        if (-not $included.Add($id)) { continue }
        foreach ($dependency in $nodes[$id].deps) {
            # Do not distribute notices solely for test-only dependencies.
            if (@($dependency.dep_kinds | Where-Object { $_.kind -ne 'dev' }).Count) { $pending.Enqueue($dependency.pkg) }
        }
    }
    New-Item -ItemType Directory -Path $Output -Force | Out-Null
    $manifest=@()
    $supplements=Get-Content -LiteralPath (Join-Path $EtsadsRoot 'vendor\licenses\rust-supplements.json') -Raw | ConvertFrom-Json
    foreach ($id in ($included | Sort-Object)) {
        $package=$packages[$id]
        if (-not $package.source) { continue } # Workspace source is covered by the root license.
        $crate=(Split-Path $package.manifest_path -Parent)
        $destination=Join-Path $Output ($package.name + '-' + $package.version)
        $licenseFiles=@(Get-ChildItem -LiteralPath $crate -Recurse -File | Where-Object { $_.Name -match '(?i)(licen[cs]e|copying|notice|copyright|^OFL)' })
        if ($package.license_file) {
            $declared=[IO.Path]::GetFullPath((Join-Path $crate $package.license_file))
            if (-not $declared.StartsWith($crate + [IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { throw "License path escaped package $($package.name)" }
            $licenseFiles+=Get-Item -LiteralPath $declared
        }
        $copied=@()
        foreach ($file in ($licenseFiles | Sort-Object FullName -Unique)) {
            # Keep nested font/native licenses as well as top-level notices.
            $relative=$file.FullName.Substring($crate.Length+1)
            $targetFile=Join-Path $destination $relative
            New-Item -ItemType Directory -Force -Path (Split-Path $targetFile -Parent) | Out-Null
            Copy-Item -LiteralPath $file.FullName -Destination $targetFile
            $copied+=$relative.Replace('\','/')
        }
        $supplement=$null
        if (-not $copied.Count) {
            $supplement=@($supplements | Where-Object { $_.name -eq $package.name -and $_.version -eq $package.version }) | Select-Object -First 1
            if ($supplement) {
                $manifestHash=(Get-FileHash -LiteralPath $package.manifest_path -Algorithm SHA256).Hash
                if ($manifestHash -ne $supplement.registry_cargo_toml_sha256) { throw "Supplement registry manifest changed: $($package.name)" }
                $upstream=Join-Path $EtsadsRoot ('vendor\licenses\rust-' + $package.name + '-' + $package.version)
                New-Item -ItemType Directory -Path $destination -Force | Out-Null
                foreach ($notice in $supplement.files) {
                    $noticeFile=Join-Path $upstream $notice.file
                    if ((Get-FileHash -LiteralPath $noticeFile -Algorithm SHA256).Hash -ne $notice.sha256) { throw "Supplement notice hash mismatch: $noticeFile" }
                    Copy-Item -LiteralPath $noticeFile -Destination (Join-Path $destination $notice.file)
                    $copied+=$notice.file
                }
                Copy-Item -LiteralPath (Join-Path $upstream 'SOURCE.json'),(Join-Path $upstream 'cargo-vcs-info.json') -Destination $destination
            }
        }
        $manifest+=[pscustomobject]@{name=$package.name;version=$package.version;license_expression=$package.license;files=$copied;notice_files_found=($copied.Count -gt 0);supplement_basis=$supplement.basis}
    }
    $manifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $Output 'dependencies.json') -Encoding utf8
    $missing=@($manifest | Where-Object { -not $_.notice_files_found })
    Write-Output "Third-party dependency notice entries: $($manifest.Count); without bundled notice files: $($missing.Count)"
    if ($missing.Count) { throw ('Missing dependency notices: '+(($missing | ForEach-Object { $_.name+'@'+$_.version }) -join ', ')) }
} finally { Pop-Location }
