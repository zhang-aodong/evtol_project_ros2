[CmdletBinding()]
param(
    [string]$PixiWorkspace = '',
    [string]$PixiExe = '',
    [ValidateSet('udp','shm')][string]$DdsTransport = 'udp'
)

$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\machine_config.ps1"
if (-not $PixiWorkspace) { $PixiWorkspace = Get-EvtolMachineValue 'PixiWorkspace' 'EVTOL_PIXI_WORKSPACE' 'C:\pixi_ws' }
if (-not $PixiExe) { $PixiExe = Get-EvtolMachineValue 'PixiExe' 'EVTOL_PIXI_EXE' "$env:USERPROFILE\.pixi\bin\pixi.exe" }

$manifestPath = Join-Path $PixiWorkspace 'pixi.toml'
$rosRoot = Join-Path $PixiWorkspace 'ros2-windows'
$rosSetup = Join-Path $rosRoot 'local_setup.ps1'

foreach ($requiredPath in @($PixiExe, $manifestPath, $rosSetup)) {
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "ROS 2 activation failed: required path does not exist: $requiredPath"
    }
}

# Activate the exact dependency environment used by the ROS 2 Windows build.
# --no-install keeps activation offline and prevents unexpected package changes.
$pixiHook = & $PixiExe shell-hook `
    --shell powershell `
    --manifest-path $manifestPath `
    --no-install `
    --change-ps1 false
if ($LASTEXITCODE -ne 0) {
    throw "ROS 2 activation failed: pixi shell-hook exited with code $LASTEXITCODE"
}
Invoke-Expression ($pixiHook -join [Environment]::NewLine)

# Small IMU/TF matrix operations should not spawn a BLAS pool per sensor process.
$env:OPENBLAS_NUM_THREADS = '1'
$env:OMP_NUM_THREADS = '1'
$algorithmDeps = Join-Path $PSScriptRoot '..\.algorithm_deps'
if (Test-Path -LiteralPath $algorithmDeps) {
    $algorithmDeps = [IO.Path]::GetFullPath($algorithmDeps)
    if (($env:PYTHONPATH -split ';') -notcontains $algorithmDeps) { $env:PYTHONPATH="$algorithmDeps;$env:PYTHONPATH" }
}

# This affects ROS/DDS only. UE camera/LiDAR named shared-memory rings stay on.
# UDP is the tested workaround for native Fast DDS crashes on this build.
$ddsDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\src\evtol_bringup\config'))
$projectProfiles = @((Join-Path $ddsDir 'fastdds.xml'), (Join-Path $ddsDir 'fastdds_udp.xml'))
if (-not $PSBoundParameters.ContainsKey('DdsTransport') -and $env:EVTOL_DDS_TRANSPORT) {
    if ($env:EVTOL_DDS_TRANSPORT -notin @('udp','shm')) { throw 'EVTOL_DDS_TRANSPORT must be udp or shm.' }
    $DdsTransport = $env:EVTOL_DDS_TRANSPORT
}
$legacyProfile = $env:FASTRTPS_DEFAULT_PROFILES_FILE
$modernProfile = $env:FASTDDS_DEFAULT_PROFILES_FILE
$managedProfiles = (-not $legacyProfile -or $legacyProfile -in $projectProfiles) -and `
                   (-not $modernProfile -or $modernProfile -in $projectProfiles)
if ($PSBoundParameters.ContainsKey('DdsTransport') -or $managedProfiles) {
    $ddsName = if ($DdsTransport -eq 'udp') { 'fastdds_udp.xml' } else { 'fastdds.xml' }
    $ddsProfile = Join-Path $ddsDir $ddsName
    if (-not (Test-Path -LiteralPath $ddsProfile)) { throw "Missing DDS profile: $ddsProfile" }
    $env:FASTRTPS_DEFAULT_PROFILES_FILE = $ddsProfile
    $env:FASTDDS_DEFAULT_PROFILES_FILE = $ddsProfile
    $env:EVTOL_DDS_TRANSPORT = $DdsTransport
}
# External/custom profiles are preserved unless -DdsTransport was explicit.
if ($env:EVTOL_DDS_TRANSPORT -eq 'udp' -and -not $env:RMW_FASTRTPS_PUBLICATION_MODE) {
    # Large BGR frames must not block the acquisition thread while UDP fragments
    # are sent to every subscriber. History/QoS and original messages stay intact.
    $env:RMW_FASTRTPS_PUBLICATION_MODE = 'ASYNCHRONOUS'
}

# pixi.toml uses cmd.exe's %cd% syntax, which PowerShell does not expand.
$env:QT_QPA_PLATFORM_PLUGIN_PATH = Join-Path `
    $PixiWorkspace '.pixi\envs\default\Library\plugins\platforms'

# Use the PowerShell setup file. It adds vendor DLL directories such as the
# liblz4 path required by rosbag2's default MCAP storage plugin.
. $rosSetup

Write-Host "ROS 2 $env:ROS_DISTRO activated."
Write-Host "  ROS root: $rosRoot"
Write-Host "  Python:   $((Get-Command python).Source)"
Write-Host "  ros2:     $((Get-Command ros2).Source)"
Write-Host "  DDS XML:  $(if ($env:FASTDDS_DEFAULT_PROFILES_FILE) { $env:FASTDDS_DEFAULT_PROFILES_FILE } else { $env:FASTRTPS_DEFAULT_PROFILES_FILE })"
Write-Host "  ROS domain: $(if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { '0' })"
$projectOverlay = Join-Path $PSScriptRoot '..\install\local_setup.ps1'
if (Test-Path -LiteralPath $projectOverlay) { . $projectOverlay }
