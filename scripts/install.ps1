param(
    [Parameter(Mandatory = $true, Position = 0, HelpMessage = "A32NX package root path")]
    [string] $PackagePath,

    [Parameter(Mandatory = $false, Position = 1)]
    [string] $WorkPath
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
Set-Location $repoRoot

if (-not (Test-Path -Path $PackagePath -PathType Container)) {
    throw "Package path not found: $PackagePath"
}

$packageName = Split-Path $PackagePath -Leaf
if ([string]::IsNullOrWhiteSpace($WorkPath)) {
    $localAppData = $env:LOCALAPPDATA
    if ([string]::IsNullOrWhiteSpace($localAppData)) {
        throw "LOCALAPPDATA environment variable is not set."
    }

    $WorkPath = Join-Path $localAppData "Packages/Microsoft.FlightSimulator_8wekyb3d8bbwe/LocalState/packages/$packageName/work"
}

$aircraft = Join-Path $PackagePath "SimObjects/AirPlanes/FlyByWire_A320_NEO"
$panel = Join-Path $aircraft "panel"

& .\scripts\dev-env\run.cmd ./scripts/build-wasm.sh
if ($LASTEXITCODE -ne 0) {
    throw "Build command failed with exit code $LASTEXITCODE."
}

$wasmSource = Join-Path $repoRoot "target/wasm32-wasip1/release/testpilot-msfs.wasm"
$wasmTarget = Join-Path $panel "testpilot.wasm"
Copy-Item -Force -Path $wasmSource -Destination $wasmTarget

New-Item -ItemType Directory -Path $WorkPath -Force | Out-Null
Copy-Item -Force -Path "example/replayer_config.toml" -Destination (Join-Path $WorkPath "replayer_config.toml")
Copy-Item -Force -Path "example/scenario.csv" -Destination (Join-Path $WorkPath "scenario.csv")

$panelFile = Join-Path $panel "panel.cfg"
if (-not (Test-Path -Path $panelFile -PathType Leaf)) {
    throw "Panel configuration not found: $panelFile"
}

$gauge = "htmlgauge04 = WasmInstrument/WasmInstrument.html?wasm_module=testpilot.wasm&wasm_gauge=testpilot,0,0,1,1"
$panelText = Get-Content -Raw -Path $panelFile
$start = $panelText.IndexOf("[VCockpit17]")
if ($start -lt 0) {
    throw "Cannot find [VCockpit17] section in panel.cfg."
}

$end = $panelText.IndexOf("`n[", $start + 1)
if ($end -lt 0) {
    $end = $panelText.Length
} else {
    $end += 1
}

$prefix = $panelText.Substring(0, $start)
$section = $panelText.Substring($start, $end - $start)
$suffix = $panelText.Substring($end)

if ($section -notmatch [regex]::Escape($gauge)) {
    $section = $section.TrimEnd("`r","`n") + "`r`n`r`n" + $gauge + "`r`n`r`n"
}

Set-Content -Path $panelFile -Value ($prefix + $section + $suffix) -NoNewline

$layoutFile = Join-Path $PackagePath "layout.json"
if (-not (Test-Path -Path $layoutFile -PathType Leaf)) {
    throw "layout.json not found: $layoutFile"
}

$layout = Get-Content -Raw -Path $layoutFile | ConvertFrom-Json
$paths = @(
    "SimObjects/AirPlanes/FlyByWire_A320_NEO/panel/panel.cfg",
    "SimObjects/AirPlanes/FlyByWire_A320_NEO/panel/testpilot.wasm"
)

foreach ($relative in $paths) {
    $file = Join-Path $PackagePath $relative
    if (-not (Test-Path -Path $file -PathType Leaf)) {
        throw "Expected package file not found for layout update: $file"
    }

    $metadata = [ordered]@{
        path = $relative
        size = (Get-Item $file).Length
        date = (Get-Item $file).LastWriteTimeUtc.ToFileTimeUtc()
    }

    $entry = $layout.content | Where-Object { $_.path -eq $relative } | Select-Object -First 1
    if ($null -eq $entry) {
        $layout.content += [PSCustomObject]$metadata
    } else {
        $entry.path = $metadata.path
        $entry.size = $metadata.size
        $entry.date = $metadata.date
    }
}

($layout | ConvertTo-Json -Depth 10) | Out-File -FilePath $layoutFile -Encoding utf8
