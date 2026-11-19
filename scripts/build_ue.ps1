[CmdletBinding()]
param([string]$Editor = '', [string]$Project = '', [string]$Target = '')
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\machine_config.ps1"
if (-not $Editor) { $Editor = Get-EvtolMachineValue 'UnrealEditor' 'EVTOL_UE_EDITOR' }
if (-not $Project) { $Project = Get-EvtolMachineValue 'UnrealProject' 'EVTOL_UE_PROJECT' }
if (-not $Editor -or -not $Project) { throw 'Configure UnrealEditor/UnrealProject or pass -Editor and -Project.' }
$Editor = (Resolve-Path -LiteralPath $Editor).Path
$Project = (Resolve-Path -LiteralPath $Project).Path
$active = @(Get-CimInstance Win32_Process -Filter "Name='UE4Editor.exe'" | Where-Object {
    $_.CommandLine -like ('*' + $Project + '*')
})
if ($active.Count) { throw 'Close this UE project and its game instances before rebuilding its plugin.' }
$engine = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $Editor))
$build = Join-Path $engine 'Build\BatchFiles\Build.bat'
if (-not (Test-Path -LiteralPath $build)) { throw "UE Build.bat missing: $build" }
if (-not $Target) {
    $source = Join-Path (Split-Path -Parent $Project) 'Source'
    $targets = @(Get-ChildItem -LiteralPath $source -Filter '*Editor.Target.cs' -File)
    if ($targets.Count -ne 1) { throw 'Pass -Target using the editor target defined in Source/*Editor.Target.cs.' }
    $Target = $targets[0].Name -replace '\.Target\.cs$', ''
}
& $build $Target Win64 Development "-Project=$Project" -WaitMutex -NoHotReload
if ($LASTEXITCODE -ne 0) { throw "UE build failed: $LASTEXITCODE" }
