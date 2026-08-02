$ErrorActionPreference = 'Stop'

$bundleRoot = Split-Path -Parent $PSScriptRoot
$requiredList = Join-Path $bundleRoot 'manifest\required-files.txt'
$missing = @()
$failures = @()

Get-Content -LiteralPath $requiredList -Encoding UTF8 | ForEach-Object {
    $relativePath = $_.Trim()
    if ($relativePath -and -not (Test-Path -LiteralPath (Join-Path $bundleRoot $relativePath))) {
        $missing += $relativePath
    }
}

if ($missing.Count -gt 0) {
    $missing | ForEach-Object { Write-Error "MISSING: $_" -ErrorAction Continue }
    exit 1
}

$packagePath = Join-Path $bundleRoot 'source_snapshot\ucar_fast_nav\package.xml'
[xml]$packageXml = Get-Content -LiteralPath $packagePath -Raw -Encoding UTF8
if ($packageXml.package.name -ne 'ucar_fast_nav') {
    $failures += "package name is '$($packageXml.package.name)', expected 'ucar_fast_nav'"
}
$maintainerEmail = [string]$packageXml.package.maintainer.email
if ($maintainerEmail -notmatch '^[^@\s]+@[^@\s]+\.[^@\s]+$') {
    $failures += "maintainer email is not a fully qualified address: $maintainerEmail"
}

$mapYaml = Join-Path $bundleRoot 'source_snapshot\ucar_fast_nav\maps\002.yaml'
$imageLine = Get-Content -LiteralPath $mapYaml -Encoding UTF8 | Where-Object { $_ -match '^image\s*:' }
if ($imageLine -notmatch '^image\s*:\s*002\.pgm\s*$') {
    $failures += "002.yaml must reference relative image 002.pgm"
}

$runtimeRoot = Join-Path $bundleRoot 'source_snapshot\ucar_fast_nav'
$forbidden = Get-ChildItem -LiteralPath $runtimeRoot -File -Recurse |
    Where-Object { $_.Extension -in '.launch', '.xml', '.yaml' } |
    Select-String -Pattern '\$\(find ucar_nav\)|/home/ucar|dynamic_avoidance' -SimpleMatch:$false
if ($forbidden) {
    $failures += 'runtime package contains a forbidden ucar_nav, absolute path, or dynamic_obstacle reference'
}

$sourceMapPath = Join-Path $bundleRoot 'manifest\source-map.csv'
$sourceRows = Import-Csv -LiteralPath $sourceMapPath -Encoding UTF8
foreach ($row in $sourceRows) {
    $archivedPath = Join-Path $bundleRoot $row.bundle_path
    if (-not (Test-Path -LiteralPath $archivedPath)) {
        $failures += "source-map target missing: $($row.bundle_path)"
    }
}

$hashListPath = Join-Path $bundleRoot 'manifest\files.sha256'
Get-Content -LiteralPath $hashListPath -Encoding UTF8 | ForEach-Object {
    if ($_ -notmatch '^([0-9a-f]{64})  (.+)$') {
        $failures += "invalid hash line: $_"
        return
    }
    $expected = $Matches[1]
    $relative = $Matches[2]
    $filePath = Join-Path $bundleRoot $relative
    if (-not (Test-Path -LiteralPath $filePath)) {
        $failures += "hashed file missing: $relative"
        return
    }
    $actual = (Get-FileHash -LiteralPath $filePath -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actual -ne $expected) {
        $failures += "hash mismatch: $relative"
    }
}

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error "FAIL: $_" -ErrorAction Continue }
    exit 1
}

Write-Output 'PASS: navigation vendor bundle is internally consistent'
exit 0
