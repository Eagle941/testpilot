[CmdletBinding()]
param(
    [ValidateSet("Install", "Rollback")]
    [string]$Action = "Install",

    [string]$PackagePath = "D:\MSFS\Packages\Community\flybywire-aircraft-a320-neo",

    [string]$BackupPath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$GaugeLine = "htmlgauge04 = WasmInstrument/WasmInstrument.html?wasm_module=replay.wasm&wasm_gauge=replayer,0,0,1,1"
$PanelRelativePath = "SimObjects/AirPlanes/FlyByWire_A320_NEO/panel/panel.cfg"
$WasmRelativePath = "SimObjects/AirPlanes/FlyByWire_A320_NEO/panel/replay.wasm"
$RepositoryRoot = Split-Path -Parent $PSScriptRoot
$BuildScript = Join-Path $PSScriptRoot "build-wasm.sh"
$BuiltWasm = Join-Path $RepositoryRoot "target/wasm32-wasip1/release/replay-msfs.wasm"
$PanelPath = Join-Path $PackagePath ($PanelRelativePath -replace "/", "\")
$InstalledWasm = Join-Path $PackagePath ($WasmRelativePath -replace "/", "\")
$LayoutPath = Join-Path $PackagePath "layout.json"

function Write-Utf8FileAtomically {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,

        [Parameter(Mandatory = $true)]
        [string]$Content
    )

    $temporaryPath = "$Path.replay-tmp"
    try {
        [System.IO.File]::WriteAllText(
            $temporaryPath,
            $Content,
            [System.Text.UTF8Encoding]::new($false)
        )
        Move-Item -LiteralPath $temporaryPath -Destination $Path -Force
    }
    finally {
        if (Test-Path -LiteralPath $temporaryPath) {
            Remove-Item -LiteralPath $temporaryPath -Force
        }
    }
}

function Restore-Backup {
    param(
        [Parameter(Mandatory = $true)]
        [string]$From
    )

    $backupPanel = Join-Path $From "panel.cfg"
    $backupLayout = Join-Path $From "layout.json"
    $backupWasm = Join-Path $From "replay.wasm"
    $wasmAbsentMarker = Join-Path $From "replay.wasm.absent"

    if (-not (Test-Path -LiteralPath $backupPanel -PathType Leaf)) {
        throw "Backup is missing panel.cfg: $backupPanel"
    }
    if (-not (Test-Path -LiteralPath $backupLayout -PathType Leaf)) {
        throw "Backup is missing layout.json: $backupLayout"
    }
    if (-not (Test-Path -LiteralPath $backupWasm -PathType Leaf) -and
        -not (Test-Path -LiteralPath $wasmAbsentMarker -PathType Leaf)) {
        throw "Backup does not describe the previous replay.wasm state: $From"
    }

    Copy-Item -LiteralPath $backupPanel -Destination $PanelPath -Force
    Copy-Item -LiteralPath $backupLayout -Destination $LayoutPath -Force

    if (Test-Path -LiteralPath $backupWasm -PathType Leaf) {
        Copy-Item -LiteralPath $backupWasm -Destination $InstalledWasm -Force
    }
    elseif (Test-Path -LiteralPath $InstalledWasm) {
        Remove-Item -LiteralPath $InstalledWasm -Force
    }
}

function Assert-InstallationPaths {
    if (-not (Test-Path -LiteralPath $PackagePath -PathType Container)) {
        throw "A32NX package directory does not exist: $PackagePath"
    }
    if (-not (Test-Path -LiteralPath $PanelPath -PathType Leaf)) {
        throw "A32NX panel.cfg does not exist: $PanelPath"
    }
    if (-not (Test-Path -LiteralPath $LayoutPath -PathType Leaf)) {
        throw "A32NX layout.json does not exist: $LayoutPath"
    }
}

Assert-InstallationPaths

if ($Action -eq "Rollback") {
    if ([string]::IsNullOrWhiteSpace($BackupPath)) {
        throw "Rollback requires -BackupPath. Use the path printed by a previous install."
    }

    $resolvedBackup = (Resolve-Path -LiteralPath $BackupPath).Path
    Restore-Backup -From $resolvedBackup

    Write-Host "Restored A32NX smoke-test backup:"
    Write-Host "  $resolvedBackup"
    Write-Host "Restored files:"
    Write-Host "  $PanelPath"
    Write-Host "  $LayoutPath"
    Write-Host "  $InstalledWasm"
    exit 0
}

if (-not (Test-Path -LiteralPath $BuildScript -PathType Leaf)) {
    throw "Build script does not exist: $BuildScript"
}

$runningSimulator = Get-Process -Name "FlightSimulator" -ErrorAction SilentlyContinue
if ($null -ne $runningSimulator) {
    throw "MSFS is running. Close it before modifying the installed A32NX package."
}

Write-Host "Building the MSFS WASM module..."
Push-Location $RepositoryRoot
try {
    & sh "scripts/build-wasm.sh"
    if ($LASTEXITCODE -ne 0) {
        throw "WASM build failed with exit code $LASTEXITCODE."
    }
}
finally {
    Pop-Location
}

if (-not (Test-Path -LiteralPath $BuiltWasm -PathType Leaf)) {
    throw "Build completed without producing the deployable artifact: $BuiltWasm"
}

$panelText = [System.IO.File]::ReadAllText($PanelPath)
$sectionMatch = [regex]::Match(
    $panelText,
    "(?ms)^\[VCockpit17\][^\r\n]*(?:\r?\n)(.*?)(?=^\[|\z)"
)
if (-not $sectionMatch.Success) {
    throw "panel.cfg does not contain a [VCockpit17] section."
}

$sectionBody = $sectionMatch.Groups[1].Value
$replayGaugeMatches = [regex]::Matches(
    $sectionBody,
    "(?im)^\s*htmlgauge\d+\s*=.*wasm_module=replay\.wasm&wasm_gauge=replayer.*$"
)
if ($replayGaugeMatches.Count -gt 1) {
    throw "[VCockpit17] contains more than one replay gauge entry; refusing an ambiguous update."
}

$gauge04Match = [regex]::Match($sectionBody, "(?im)^\s*htmlgauge04\s*=.*$")
if ($gauge04Match.Success -and
    $gauge04Match.Value -notmatch "wasm_module=replay\.wasm&wasm_gauge=replayer") {
    throw "[VCockpit17] already uses htmlgauge04 for another gauge."
}

if ($replayGaugeMatches.Count -eq 1) {
    $updatedSectionBody = $sectionBody.Remove(
        $replayGaugeMatches[0].Index,
        $replayGaugeMatches[0].Length
    ).Insert($replayGaugeMatches[0].Index, $GaugeLine)
}
else {
    $lineEnding = "`n"
    if ($panelText.Contains("`r`n")) {
        $lineEnding = "`r`n"
    }
    $updatedSectionBody = $sectionBody.TrimEnd([char[]]"`r`n") + $lineEnding + $GaugeLine + $lineEnding + $lineEnding
}

$updatedPanelText = $panelText.Remove(
    $sectionMatch.Groups[1].Index,
    $sectionMatch.Groups[1].Length
).Insert($sectionMatch.Groups[1].Index, $updatedSectionBody)

try {
    $layout = $null
    $layout = [System.IO.File]::ReadAllText($LayoutPath) | ConvertFrom-Json
}
catch {
    throw "Unable to parse A32NX layout.json: $($_.Exception.Message)"
}
if ($null -eq $layout.content) {
    throw "A32NX layout.json does not contain a content array."
}

$timestamp = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssfffZ")
$backupRoot = Join-Path $RepositoryRoot "target/a32nx-smoke-test-backups"
$createdBackup = Join-Path $backupRoot $timestamp
New-Item -ItemType Directory -Path $createdBackup -Force | Out-Null
Copy-Item -LiteralPath $PanelPath -Destination (Join-Path $createdBackup "panel.cfg")
Copy-Item -LiteralPath $LayoutPath -Destination (Join-Path $createdBackup "layout.json")
if (Test-Path -LiteralPath $InstalledWasm -PathType Leaf) {
    Copy-Item -LiteralPath $InstalledWasm -Destination (Join-Path $createdBackup "replay.wasm")
}
else {
    New-Item -ItemType File -Path (Join-Path $createdBackup "replay.wasm.absent") | Out-Null
}

$changesStarted = $false
try {
    $changesStarted = $true
    Write-Utf8FileAtomically -Path $PanelPath -Content $updatedPanelText

    $temporaryWasm = "$InstalledWasm.replay-tmp"
    try {
        Copy-Item -LiteralPath $BuiltWasm -Destination $temporaryWasm -Force
        Move-Item -LiteralPath $temporaryWasm -Destination $InstalledWasm -Force
    }
    finally {
        if (Test-Path -LiteralPath $temporaryWasm) {
            Remove-Item -LiteralPath $temporaryWasm -Force
        }
    }

    $layoutEntries = @($layout.content)
    foreach ($relativePath in @($PanelRelativePath, $WasmRelativePath)) {
        $installedPath = Join-Path $PackagePath ($relativePath -replace "/", "\")
        $file = Get-Item -LiteralPath $installedPath
        $matchingEntries = @($layoutEntries | Where-Object { $_.path -ieq $relativePath })

        if ($matchingEntries.Count -gt 1) {
            throw "layout.json contains duplicate entries for $relativePath"
        }

        if ($matchingEntries.Count -eq 1) {
            $matchingEntries[0].path = $relativePath
            $matchingEntries[0].size = [Int64]$file.Length
            $matchingEntries[0].date = [Int64]$file.LastWriteTimeUtc.ToFileTimeUtc()
        }
        else {
            $layoutEntries += [PSCustomObject]@{
                path = $relativePath
                size = [Int64]$file.Length
                date = [Int64]$file.LastWriteTimeUtc.ToFileTimeUtc()
            }
        }
    }

    $layout.content = @($layoutEntries | Sort-Object -Property path)
    $layoutJson = $layout | ConvertTo-Json -Depth 20
    Write-Utf8FileAtomically -Path $LayoutPath -Content ($layoutJson + [Environment]::NewLine)

    $verificationLayout = [System.IO.File]::ReadAllText($LayoutPath) | ConvertFrom-Json
    $verificationPanel = [System.IO.File]::ReadAllText($PanelPath)
    $gaugeCount = [regex]::Matches(
        $verificationPanel,
        "(?im)^\s*htmlgauge04\s*=\s*WasmInstrument/WasmInstrument\.html\?wasm_module=replay\.wasm&wasm_gauge=replayer,0,0,1,1\s*$"
    ).Count
    if ($gaugeCount -ne 1) {
        throw "Post-install verification found $gaugeCount replay gauge entries instead of one."
    }

    foreach ($relativePath in @($PanelRelativePath, $WasmRelativePath)) {
        $entryCount = @($verificationLayout.content | Where-Object { $_.path -ieq $relativePath }).Count
        if ($entryCount -ne 1) {
            throw "Post-install verification found $entryCount layout entries for $relativePath instead of one."
        }
    }
}
catch {
    if ($changesStarted) {
        try {
            Restore-Backup -From $createdBackup
            Write-Warning "Installation failed; the original package files were restored."
        }
        catch {
            Write-Warning "Automatic rollback also failed: $($_.Exception.Message)"
        }
    }
    throw
}

Write-Host "Installed the replay smoke-test gauge successfully."
Write-Host "Changed files:"
Write-Host "  $PanelPath"
Write-Host "  $InstalledWasm"
Write-Host "  $LayoutPath"
Write-Host "Backup:"
Write-Host "  $createdBackup"
Write-Host "Rollback command:"
Write-Host "  powershell -ExecutionPolicy Bypass -File `"$PSCommandPath`" -Action Rollback -PackagePath `"$PackagePath`" -BackupPath `"$createdBackup`""
