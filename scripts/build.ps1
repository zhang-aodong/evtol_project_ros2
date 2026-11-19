[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
if (Get-Process -Name sensor_node,foxglove_node,preview_node -ErrorAction SilentlyContinue) {
    throw 'Stop running project nodes before building: Windows cannot replace an executable while it is in use.'
}
$runningPython = @(Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object {
    $_.CommandLine -match 'evtol_simulation\.sensor_node|sensor_node-script\.py|evtol_visualization\.(bridge|preview)'
})
if ($runningPython.Count) { throw 'Stop running acquisition and visualization Python processes before building.' }
. "$PSScriptRoot\activate.ps1"
$vsWhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path -LiteralPath $vsWhere)) { throw 'Visual Studio 2022 C++ build tools are required.' }
$vsInstallation = & $vsWhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsInstallation) { throw 'Visual Studio x64 C++ tools were not found.' }
$vsDev = Join-Path $vsInstallation 'Common7\Tools\VsDevCmd.bat'
$vsEnvironment = & cmd.exe /d /s /c "`"$vsDev`" -no_logo -arch=x64 -host_arch=x64 >nul && set"
if ($LASTEXITCODE -ne 0) { throw 'Visual Studio developer environment failed.' }
foreach ($entry in $vsEnvironment) {
    if ($entry -match '^([^=]+)=(.*)$') { [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], 'Process') }
}
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
Push-Location $projectRoot
try {
    $rosPython = (Join-Path $env:CONDA_PREFIX 'python.exe').Replace('\','/')
    # The installed colcon-output summary plugin targets a newer colcon-core.
    # Disable only that optional presentation extension; keep all build checks.
    $env:COLCON_EXTENSION_BLOCKLIST = 'colcon_core.event_handler.summary'
    & colcon build --merge-install --executor sequential --event-handlers console_direct+ `
      --cmake-args -G 'Visual Studio 17 2022' -A x64 `
      "-DPython3_EXECUTABLE=$rosPython" -DBUILD_TESTING=OFF
    if ($LASTEXITCODE -ne 0) { throw 'colcon build failed.' }
} finally { Pop-Location }
