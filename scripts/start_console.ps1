[CmdletBinding()]
param(
    [ValidateRange(1,65535)][int]$Port=9040,
    [ValidateRange(1,65535)][int]$BridgePort=8765,
    [switch]$NoBrowser,
    [switch]$SkipPanelInstall,
    [string]$Config = ''
)
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if($Port -eq $BridgePort){throw 'Console and bridge ports must differ.'}
if(-not $PSBoundParameters.ContainsKey('BridgePort')){
    while(Get-NetTCPConnection -LocalPort $BridgePort -State Listen -ErrorAction SilentlyContinue){
        $BridgePort++
        if($BridgePort -gt 8799){throw 'No free Foxglove bridge port in 8765..8799.'}
    }
}
foreach($p in @($Port,$BridgePort)) {
    $occupied=Get-NetTCPConnection -LocalPort $p -State Listen -ErrorAction SilentlyContinue
    if($occupied){throw "Port $p is occupied. Stop the previous console/bridge or choose another port."}
}
. "$PSScriptRoot\activate.ps1"
$env:PYTHONPATH="$root\.visualization_deps;$env:PYTHONPATH"
if(-not $SkipPanelInstall){
    $panel=Join-Path $root 'foxglove\evtol-console'
    $manifest=Get-Content -LiteralPath (Join-Path $panel 'package.json') -Raw -Encoding UTF8|ConvertFrom-Json
    if(-not(Test-Path -LiteralPath (Join-Path $panel 'dist\extension.js'))){throw 'Panel is not built. Run scripts/build_console_panel.ps1 once.'}
    $destination=Join-Path $env:USERPROFILE ('.foxglove-studio\extensions\'+$manifest.publisher+'.'+$manifest.name+'-'+$manifest.version)
    New-Item -ItemType Directory -Force -Path $destination | Out-Null
    foreach($name in @('package.json','README.md','CHANGELOG.md','dist')){
        Copy-Item -LiteralPath (Join-Path $panel $name) -Destination $destination -Recurse -Force
    }
    Write-Host "Foxglove panel installed: $destination"
    Write-Host 'Reload Foxglove if it was already open, then add EVTOL Simulation Console.'
}
$python=(Get-Command python).Source
& $python -c 'import evtol_console.server, foxglove'
if($LASTEXITCODE -ne 0){throw 'Build the console packages first; see the console tutorial.'}
Write-Host "Foxglove connection: ws://127.0.0.1:$BridgePort"
Write-Host "Local console: http://127.0.0.1:$Port"
Write-Host 'UE and the experiment remain stopped until started from the panel.'
$managerArgs=@('-u','-m','evtol_console.server','--root',$root,'--port',"$Port",'--bridge-port',"$BridgePort")
if($Config){$managerArgs+=@('--config',[IO.Path]::GetFullPath($Config))}
if(-not $NoBrowser){$managerArgs+='--open-browser'}
& $python @managerArgs
if($LASTEXITCODE -ne 0){throw 'Console manager exited with an error.'}
