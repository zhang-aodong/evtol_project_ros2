[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\activate.ps1"
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
& python -m pip install --disable-pip-version-check --no-deps --upgrade --target "$root\.visualization_deps" 'foxglove-sdk==0.30.3' 'websockets==15.0.1'
if ($LASTEXITCODE -ne 0) { throw 'Project-local visualization dependency installation failed.' }
Push-Location $root
try {
    $env:COLCON_EXTENSION_BLOCKLIST = 'colcon_core.event_handler.summary'
    & colcon build --merge-install --packages-select evtol_visualization --event-handlers console_direct+
    if ($LASTEXITCODE -ne 0) { throw 'Visualization package build failed.' }
} finally { Pop-Location }
