[CmdletBinding()]
param([string]$Config='', [switch]$Execute, [switch]$Actors, [switch]$NewSession,
    [ValidateSet('gnss_imu','gnss_imu_5g','camera_imu')][string]$LocalizationMode,
    [switch]$UseFusedState, [switch]$GnssOutage)
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $Config) { $Config=Join-Path $root 'config\targeting.yaml' }
. "$PSScriptRoot\activate.ps1"
if (($UseFusedState -or $GnssOutage) -and -not $LocalizationMode) {
    throw 'Specify -LocalizationMode with -UseFusedState or -GnssOutage.'
}
if ($LocalizationMode) {
    $options = @('--config', $Config, '--mode', $LocalizationMode)
    if ($UseFusedState) { $options += '--fused' }
    if ($GnssOutage) { $options += '--outage' }
    $Config = & python -m evtol_localization.settings @options
    if ($LASTEXITCODE -ne 0) { throw 'Failed to prepare localization configuration.' }
    Write-Host "Localization experiment config: $Config"
}
if ($NewSession) {
    & python "$PSScriptRoot\prepare_targeting.py" --config $Config --new-session
    if ($LASTEXITCODE -ne 0) { throw 'Session initialization failed.' }
}
& ros2 launch evtol_bringup target_experiment.launch.py "config:=$Config" "execute:=$($Execute.IsPresent.ToString().ToLower())" "actors:=$($Actors.IsPresent.ToString().ToLower())"
if ($LASTEXITCODE -ne 0) { throw 'Experiment launch failed.' }
