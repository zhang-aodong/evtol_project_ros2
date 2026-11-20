param(
    [ValidateSet('straight','turns','ground_hold','raw_emergency','crossing')]
    [string[]]$Cases=@('raw_emergency','crossing'),
    [ValidateRange(30,600)][int]$Seconds=120
)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
. (Join-Path $root 'scripts\activate.ps1')
$env:PYTHONPATH="$root\.visualization_deps;$root\.algorithm_deps;$env:PYTHONPATH"
$env:PYTHONIOENCODING='utf-8'
$python=(Get-Command python.exe).Source
$stamp=Get-Date -Format yyyyMMdd_HHmmss
$ErrorActionPreference='Continue'
foreach($case in $Cases){
    $label='fusion_2m_'+$case+'_'+$stamp
    & $python (Join-Path $root 'tests\probe_flight_layers.py') --root $root --case $case --label $label --seconds $Seconds 1> (Join-Path $root ('reports\'+$label+'.log')) 2> (Join-Path $root ('reports\'+$label+'.stderr.log'))
    if($LASTEXITCODE -ne 0){throw ('Probe failed: '+$case+'; see reports\'+$label+'.log')}
    $result=Get-Content -Raw (Join-Path $root ('reports\'+$label+'\result.json'))|ConvertFrom-Json
    $result|Select-Object case,completed,final_position,longest_stationary_waypoint_s,airborne_collisions,config_unchanged|Format-List
    if(-not $result.completed -or @($result.airborne_collisions).Count -gt 0){throw ('Flight acceptance did not pass: '+$case)}
}
