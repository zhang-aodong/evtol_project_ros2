[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Bag,[string]$Namespace='drone1',[double]$Rate=1.0)
$ErrorActionPreference='Stop'
. "$PSScriptRoot\activate.ps1"
if($Namespace -notmatch '^[A-Za-z_][A-Za-z0-9_]*$' -or $Rate -le 0 -or $Rate -gt 2){throw 'Invalid namespace/rate'}
$prefix='/'+$Namespace
# Generate /clock from the bag timeline. Exclude recorded estimates, live state,
# original /clock and dynamic TF to avoid competing sources.
$topics=@('/simulation/reference_origin','/simulation/geodetic_reference',"$prefix/gnss/home",
    "$prefix/imu/data","$prefix/gnss/fix","$prefix/five_ga/observations",
    "$prefix/ground_truth/local_odometry","$prefix/control/flight_status")
& ros2 bag play $Bag --clock 100 --rate $Rate --topics @topics
if($LASTEXITCODE -ne 0){throw 'Localization bag replay failed'}
