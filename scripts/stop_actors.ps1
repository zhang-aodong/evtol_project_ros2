[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
. "$PSScriptRoot\activate.ps1"
# Invoke before stopping the launch process: on Windows forced process exit
# cannot guarantee Python finally blocks run. The service performs cleanup.
& ros2 service call /scenario/stop std_srvs/srv/Trigger '{}'
if ($LASTEXITCODE -ne 0) { throw 'Actor cleanup service failed.' }
