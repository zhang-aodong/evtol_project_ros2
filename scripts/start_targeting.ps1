[CmdletBinding()]
param([string]$Config='', [switch]$Execute, [switch]$Actors, [switch]$NewSession)
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $Config) { $Config=Join-Path $root 'config\targeting.yaml' }
. "$PSScriptRoot\activate.ps1"
$deps=Join-Path $root '.algorithm_deps'
if (Test-Path -LiteralPath $deps) { $env:PYTHONPATH="$deps;$env:PYTHONPATH" }
if ($NewSession) {
    # Resolve the configured path in Python; reject changing origin while any
    # state adapter is running. Source paths are never inferred from the cwd.
    & python "$PSScriptRoot\prepare_targeting.py" --config $Config --new-session
    if ($LASTEXITCODE -ne 0) { throw 'Session initialization failed.' }
}
& ros2 launch evtol_bringup target_localization.launch.py "config:=$Config" "execute:=$($Execute.IsPresent.ToString().ToLower())" "actors:=$($Actors.IsPresent.ToString().ToLower())"
if ($LASTEXITCODE -ne 0) { throw 'Targeting launch failed.' }
