[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$PluginDirectory, [switch]$Apply)
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$PluginDirectory = (Resolve-Path -LiteralPath $PluginDirectory).Path
if (-not (Test-Path -LiteralPath (Join-Path $PluginDirectory 'AirSim.uplugin'))) {
    throw 'Expected the AirSim plugin directory containing AirSim.uplugin.'
}
$support = Join-Path $root 'ue_support'
$manifest = Get-Content -LiteralPath "$support\manifest.json" -Raw -Encoding UTF8 | ConvertFrom-Json
$changed = @()
$backup = Join-Path $PluginDirectory ('evtol_sensor_backup_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff'))
foreach ($entry in $manifest) {
    $relative = $entry.path.Replace('/', '\')
    $source = [IO.Path]::GetFullPath((Join-Path $support $relative))
    $target = [IO.Path]::GetFullPath((Join-Path $PluginDirectory $relative))
    if (-not $target.StartsWith($PluginDirectory+'\') -or -not $source.StartsWith($support+'\')) {
        throw 'Manifest path escapes expected directory.'
    }
    if ((Get-FileHash -LiteralPath $source).Hash.ToLower() -ne $entry.sha256) {
        throw "Bundled source checksum mismatch: $relative"
    }
    $matches = (Test-Path -LiteralPath $target) -and ((Get-FileHash -LiteralPath $target).Hash.ToLower() -eq $entry.sha256)
    if (-not $matches) {
        $changed += $relative
        if ($Apply) {
            if (Test-Path -LiteralPath $target) {
                $backupFile = Join-Path $backup $relative
                New-Item -ItemType Directory -Force -Path (Split-Path -Parent $backupFile) | Out-Null
                Copy-Item -LiteralPath $target -Destination $backupFile
            }
            New-Item -ItemType Directory -Force -Path (Split-Path -Parent $target) | Out-Null
            Copy-Item -LiteralPath $source -Destination $target -Force
        }
    }
}
if ($Apply) {
    Write-Host "Applied $($changed.Count) sensor source files. Backup: $backup"
    Write-Host 'Rebuild the matching UE 4.27 project before starting simulation.'
} elseif ($changed.Count) {
    $changed | ForEach-Object { Write-Host "Different or missing: $_" }
    throw 'Plugin sources differ from this project snapshot. Review before using -Apply; this is for the matching UE 4.27/CosysAirSim base.'
} else { Write-Host 'All bundled UE sensor source checksums match.' }
