$ErrorActionPreference = 'Stop'

$bundleRoot = Split-Path -Parent $PSScriptRoot
$requiredList = Join-Path $bundleRoot 'manifest\required-files.txt'
$missing = @()

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

Write-Output 'PASS: required navigation bundle files exist'
exit 0
