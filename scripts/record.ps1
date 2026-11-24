[CmdletBinding()]
param([string]$Output = '')
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\activate.ps1"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $Output) { $Output = Join-Path $projectRoot ('bags\sim_' + (Get-Date -Format 'yyyyMMdd_HHmmss')) }
$qosFile = Join-Path $projectRoot 'src\evtol_bringup\config\bag_qos.yaml'
& ros2 bag record -s mcap --qos-profile-overrides-path $qosFile -o $Output --topics `
  /clock /tf /tf_static /diagnostics `
  /drone1/camera/rgb/image_raw /drone1/camera/rgb/camera_info `
  /drone1/camera/semantic/image /drone1/camera/semantic/camera_info /drone1/camera/semantic/class_ids `
  /drone1/lidar/points /drone1/imu/data /drone1/gnss/fix /drone1/gnss/raw_observations `
  /drone1/ground_truth/odometry /drone1/five_ga/observations `
  /simulation/reference_origin /drone1/gnss/home /drone1/ground_truth/local_odometry `
  /drone1/localization/odometry /drone1/localization/vio/odometry /drone1/state/odometry `
  /drone1/navigation/waypoint_path /drone1/navigation/traveled_path
if ($LASTEXITCODE -ne 0) { throw 'rosbag2 recording failed.' }
