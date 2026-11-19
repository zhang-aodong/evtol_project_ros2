[CmdletBinding()]
param(
    [string]$Settings = '',
    [string]$Editor = '',
    [string]$Project = '',
    [switch]$Visible,
    [ValidateRange(320,7680)][int]$WindowWidth = 1280,
    [ValidateRange(240,4320)][int]$WindowHeight = 720
)
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
. "$PSScriptRoot\machine_config.ps1"
if (-not $Editor) { $Editor = Get-EvtolMachineValue 'UnrealEditor' 'EVTOL_UE_EDITOR' }
if (-not $Project) { $Project = Get-EvtolMachineValue 'UnrealProject' 'EVTOL_UE_PROJECT' }
if (-not $Editor -or -not $Project) { throw 'Set UnrealEditor/UnrealProject in config/machine.local.json, or pass -Editor and -Project.' }
if (-not $Settings) { $Settings = Join-Path $projectRoot 'src\evtol_simulation\config\airsim.navigation_2k.json' }
$Settings = (Resolve-Path -LiteralPath $Settings).Path
foreach ($requiredPath in @($Editor, $Project)) {
    if (-not (Test-Path -LiteralPath $requiredPath)) { throw "Missing UE path: $requiredPath" }
}
$listening = Get-NetTCPConnection -State Listen -LocalPort 41451 -ErrorAction SilentlyContinue
if ($listening) { throw 'AirSim port 41451 is already in use. Stop that simulation first.' }
$activeGames = @(Get-CimInstance Win32_Process -Filter "Name='UE4Editor.exe'" | Where-Object {
    $_.CommandLine -match '(?i)(?:^|\s)-game(?:\s|$)|PIEVIACONSOLE'
})
if ($activeGames.Count) { throw 'An Unreal game instance is already running. Stop it before starting another shared-memory writer.' }
$reportDir = Join-Path $projectRoot 'reports'
New-Item -ItemType Directory -Path $reportDir -Force | Out-Null
$logPath = Join-Path $reportDir ('ue_' + (Get-Date -Format 'yyyyMMdd_HHmmss') + '.log')
# Sensor capture dimensions remain defined by the selected AirSim settings.
$ueArgs = @(('"' + $Project + '"'), '-game', '-windowed', '-nosound',
    ('-settings="' + $Settings + '"'), ('-abslog="' + $logPath + '"'))
if ($Visible) {
    $settingsObject=Get-Content -LiteralPath $Settings -Raw -Encoding UTF8|ConvertFrom-Json
    if ($settingsObject.ViewMode -eq 'NoDisplay') { throw 'Visible UE requires a display ViewMode, e.g. SpringArmChase, in the selected settings file.' }
    $ueArgs += @("-ResX=$WindowWidth", "-ResY=$WindowHeight")
    $windowStyle='Normal'
} else {
    $ueArgs += @('-RenderOffscreen','-ResX=640','-ResY=480','-unattended')
    $windowStyle='Hidden'
}
$process = Start-Process -FilePath $Editor -ArgumentList $ueArgs -WindowStyle $windowStyle -PassThru
$uePython = Join-Path (Get-EvtolMachineValue 'PixiWorkspace' 'EVTOL_PIXI_WORKSPACE' 'C:\pixi_ws') '.pixi\envs\default\python.exe'
if (Test-Path -LiteralPath $uePython) {
    & $uePython "$PSScriptRoot\configure_ue_process.py" --pid $process.Id
    if ($LASTEXITCODE -ne 0) { Write-Warning 'UE timer policy setup failed; verify simulation clock speed before flight.' }
}
Write-Host "UE process: $($process.Id)"
Write-Host "UE log: $logPath"
Write-Host 'Wait for startup, then run ros2 run evtol_simulation preflight --config <matching simulation.yaml>.'
