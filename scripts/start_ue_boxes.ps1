[CmdletBinding()]
param([string]$Config='')
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $Config) { $Config=Join-Path $root 'config\targeting.yaml' }
. "$PSScriptRoot\activate.ps1"
& python -u -c 'from evtol_targeting.ue_detection_boxes import main; main()' --config $Config --ros-args -p use_sim_time:=true
if ($LASTEXITCODE -ne 0) { throw 'UE detection box node exited with an error.' }
