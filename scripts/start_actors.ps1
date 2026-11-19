[CmdletBinding()]
param([string]$Config='')
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $Config) { $Config=Join-Path $root 'config\targeting.yaml' }
. "$PSScriptRoot\activate.ps1"
$deps=Join-Path $root '.algorithm_deps'
if (Test-Path -LiteralPath $deps) { $env:PYTHONPATH="$deps;$env:PYTHONPATH" }
& ros2 launch evtol_bringup actors.launch.py "config:=$Config"
