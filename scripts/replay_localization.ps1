[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Bag,[string]$Namespace='drone1',[double]$Rate=1.0,
    [ValidateSet('gnss_imu','gnss_imu_5g','camera_imu')][string]$Mode='gnss_imu_5g',
    [switch]$UseRecordedClock)
$ErrorActionPreference='Stop'
. "$PSScriptRoot\activate.ps1"
if($Namespace -notmatch '^[A-Za-z_][A-Za-z0-9_]*$' -or $Rate -le 0 -or $Rate -gt 2){throw 'Invalid namespace/rate'}
$prefix='/'+$Namespace
$topics=@("$prefix/imu/data","$prefix/ground_truth/local_odometry")
if ($Mode -eq 'camera_imu') {
    $topics += "$prefix/camera/rgb/image_raw","$prefix/camera/rgb/camera_info"
} else {
    $topics += '/simulation/reference_origin','/simulation/geodetic_reference',
        "$prefix/gnss/home","$prefix/gnss/fix","$prefix/control/flight_status"
    if ($Mode -eq 'gnss_imu_5g') { $topics += "$prefix/five_ga/observations" }
}
# Wall-time recordings need their original simulation clock. Use exactly one source.
$clockArgs=@('--clock','100')
if ($UseRecordedClock) {
    $clockArgs=@()
    $topics += '/clock'
}
& ros2 bag play $Bag @clockArgs --rate $Rate --topics @topics
if($LASTEXITCODE -ne 0){throw 'Localization bag replay failed'}
