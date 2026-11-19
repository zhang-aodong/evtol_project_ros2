[CmdletBinding()]
param([string]$Config='')
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $Config) { $Config=Join-Path $root 'config\targeting.yaml' }
. "$PSScriptRoot\activate.ps1"
& python -u -c 'from evtol_targeting.trajectory_node import main; main()' --config $Config --ros-args -p use_sim_time:=true
if ($LASTEXITCODE -ne 0) { throw 'Trajectory visualization node exited with an error.' }
