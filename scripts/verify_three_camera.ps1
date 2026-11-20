[CmdletBinding()]
param([ValidateSet('baseline','holdout')][string]$Scenario='baseline',[ValidateRange(90,600)][int]$Seconds=100)
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
. (Join-Path $root 'scripts\activate.ps1')
$env:PYTHONPATH="$root\.visualization_deps;$root\.algorithm_deps;$env:PYTHONPATH"
$env:PYTHONIOENCODING='utf-8'
$python=(Get-Command python -ErrorAction Stop).Source
$label='three_camera_'+$Scenario+'_'+(Get-Date -Format yyyyMMdd_HHmmss)
$arguments=@((Join-Path $root 'tests\probe_perception_layers.py'),'--root',$root,'--dual-front','--seconds',"$Seconds",'--label',$label)
if($Scenario -eq 'holdout'){$arguments+=@('--bearing-offset-deg','7','--layer-heights','7','13')}
& $python @arguments
if($LASTEXITCODE -ne 0){throw 'Three-camera experiment failed to run'}
$result=Get-Content -Raw -Encoding UTF8 (Join-Path $root ('reports\perception_'+$label+'\result.json')) | ConvertFrom-Json
if(-not $result.meets_90){throw 'Warning acceptance failed: inspect the saved full-hemisphere result'}
Write-Host ('PASS: timely warning rate '+('{0:P2}' -f $result.timely_warning_success_rate))
