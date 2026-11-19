[CmdletBinding()]
param([string]$Config = '', [switch]$Mock)
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $Config) { $Config = "$root\src\evtol_simulation\config\simulation.source_2k.yaml" }
$Config = (Resolve-Path -LiteralPath $Config).Path
. "$PSScriptRoot\activate.ps1"
if (-not $Mock) {
    $games = @(Get-CimInstance Win32_Process -Filter "Name='UE4Editor.exe'" | Where-Object {
        $_.CommandLine -match '(?i)(?:^|\s)-game(?:\s|$)|PIEVIACONSOLE'
    })
    if ($games.Count -gt 1) { throw 'Multiple Unreal game instances can overwrite the same LiDAR mapping. Keep one game instance.' }
    & ros2 run evtol_simulation preflight --config $Config
    if ($LASTEXITCODE -ne 0) { throw 'AirSim settings preflight failed.' }
}
$backend = if ($Mock) { 'mock' } else { 'airsim' }
& ros2 launch evtol_bringup simulation.launch.py "config:=$Config" "backend:=$backend" rviz:=false
if ($LASTEXITCODE -ne 0) { throw 'Simulation publishers exited with failure.' }
