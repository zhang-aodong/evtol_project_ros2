[CmdletBinding()]
param(
    [ValidateSet('rviz','foxglove')][string]$Viewer = 'rviz',
    [string]$Namespace = 'drone1',
    [double]$PreviewHz = 5,
    [int]$ImageWidth = 640,
    [int]$MaxPoints = 16000,
    [int]$Port = 8765,
    [switch]$LocalFrame,
    [switch]$SoftwareOpenGL
)
$ErrorActionPreference = 'Stop'
if ($Namespace -ne 'drone1' -and $Viewer -eq 'rviz') { throw 'RViz config uses drone1; use a custom config for another namespace.' }
. "$PSScriptRoot\activate.ps1"
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$python = (Get-Command python).Source
$env:PYTHONPATH = "$root\.visualization_deps;$env:PYTHONPATH"
& $python -c 'import evtol_visualization'
if ($LASTEXITCODE -ne 0) { throw 'Run scripts/install_visualization.ps1 first.' }
if ($Viewer -eq 'foxglove') {
    & $python -c 'import foxglove'
    if ($LASTEXITCODE -ne 0) { throw 'Run scripts/install_visualization.ps1 first.' }
    if (Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue) {
        throw "Port $Port is occupied. Stop the previous bridge, or select another -Port."
    }
}
New-Item -ItemType Directory -Force -Path "$root\reports" | Out-Null
$runId = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
$preview = $null
try {
    $preview = Start-Process -FilePath $python -ArgumentList @('-m','evtol_visualization.preview',
        '--namespace',$Namespace,'--hz',"$PreviewHz",'--width',"$ImageWidth",'--max-points',"$MaxPoints") `
        -WindowStyle Hidden -PassThru -RedirectStandardOutput "$root\reports\preview_$runId.out.log" `
        -RedirectStandardError "$root\reports\preview_$runId.err.log"
    Start-Sleep -Seconds 2
    if ($preview.HasExited) { throw "Preview exited; see reports/preview_$runId.err.log" }
    if ($Viewer -eq 'foxglove') {
        Write-Host "Open Foxglove -> Open connection -> Foxglove WebSocket -> ws://127.0.0.1:$Port"
        & $python -m evtol_visualization.bridge --namespace $Namespace --port $Port
    } else {
        $env:QT_OPENGL = 'desktop'
        if ($SoftwareOpenGL) { $env:QT_OPENGL = 'software' }
        $env:QT_AUTO_SCREEN_SCALE_FACTOR = '0'
        $env:QT_ENABLE_HIGHDPI_SCALING = '0'
        $env:QT_SCALE_FACTOR = '1'
        $config = "$root\src\evtol_bringup\rviz\preview.rviz"
        if ($LocalFrame) { $config = "$root\src\evtol_bringup\rviz\preview_local.rviz" }
        & rviz2 -d $config --ros-args -p use_sim_time:=true
    }
    if ($LASTEXITCODE -ne 0) { throw "Viewer exited with code $LASTEXITCODE" }
} finally {
    # Only the preview process started by this invocation belongs to this script.
    if ($preview -and -not $preview.HasExited) { Stop-Process -Id $preview.Id -ErrorAction SilentlyContinue }
}
