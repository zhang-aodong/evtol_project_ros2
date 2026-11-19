[CmdletBinding()]
param([double]$Duration = 60, [double]$Warmup = 10, [switch]$Mock, [string]$Config = '')
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\activate.ps1"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$configPath = if ($Config) { [IO.Path]::GetFullPath($Config) } else { Join-Path $projectRoot 'src\evtol_simulation\config\simulation.yaml' }
$reportPath = Join-Path $projectRoot ('reports\' + $(if ($Mock) {'mock_acceptance.json'} else {'airsim_acceptance.json'}))
$extraArgs = @()
if ($Mock) { $extraArgs += '--allow-mock' }
& python "$PSScriptRoot\validate_live.py" --config $configPath --duration $Duration --warmup $Warmup --output $reportPath @extraArgs
if ($LASTEXITCODE -ne 0) { throw "Sensor acceptance FAILED. See $reportPath" }
