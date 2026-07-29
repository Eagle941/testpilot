[CmdletBinding()]
param(
    [string]$PackagePath = "D:\MSFS\Packages\Community\flybywire-aircraft-a320-neo",

    [string[]]$RelativePath = @(
        "SimObjects/AirPlanes/FlyByWire_A320_NEO/panel/panel.cfg",
        "SimObjects/AirPlanes/FlyByWire_A320_NEO/panel/replay.wasm"
    )
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $PackagePath -PathType Container)) {
    throw "MSFS package directory does not exist: $PackagePath"
}
if ($RelativePath.Count -eq 0) {
    throw "At least one package-relative file path is required."
}

$entries = @()
$normalizedPaths = @()
foreach ($path in $RelativePath) {
    if ([string]::IsNullOrWhiteSpace($path)) {
        throw "Package-relative file paths cannot be empty."
    }

    $normalizedPath = $path.Replace("\", "/").TrimStart("/")
    if ($normalizedPath -eq "layout.json" -or
        $normalizedPath.StartsWith("../") -or
        $normalizedPath.Contains("/../")) {
        throw "Invalid package-relative file path: $path"
    }
    if ($normalizedPaths -contains $normalizedPath) {
        throw "Duplicate package-relative file path: $normalizedPath"
    }

    $installedPath = Join-Path $PackagePath ($normalizedPath -replace "/", "\")
    if (-not (Test-Path -LiteralPath $installedPath -PathType Leaf)) {
        throw "Package file does not exist: $installedPath"
    }

    $file = Get-Item -LiteralPath $installedPath
    $entries += [PSCustomObject]@{
        path = $normalizedPath
        size = [Int64]$file.Length
        date = [Int64]$file.LastWriteTimeUtc.ToFileTimeUtc()
    }
    $normalizedPaths += $normalizedPath
}

$entries | ConvertTo-Json -Depth 3
