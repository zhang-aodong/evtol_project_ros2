[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$processes=@(Get-CimInstance Win32_Process)
# Match the actual launch interpreter and this workspace's config. Do not
# terminate arbitrary Python processes, UE or the visualization service.
$launches=@($processes | Where-Object {
    $_.Name -eq 'python.exe' -and $_.CommandLine -like "*$root*" -and
    $_.CommandLine -match 'ros2-script\.py' -and
    $_.CommandLine -match 'target_experiment\.launch\.py'
})
foreach ($launch in $launches) {
    if ($launch.CommandLine -match 'execute:=true|actors:=true') {
        throw 'This helper stops shadow experiments with actors disabled only. Complete vehicle cleanup before stopping a flight/actors launch.'
    }
    if ($launch.CommandLine -notmatch 'execute:=false' -or $launch.CommandLine -notmatch 'actors:=false') {
        throw 'Cannot establish that this experiment is shadow with actors disabled; leaving it running.'
    }
}
foreach ($launch in $launches) {
    $owned=@{}; $owned[[int]$launch.ProcessId]=$launch
    do {
        $added=$false
        foreach ($process in $processes) {
            if ($owned.ContainsKey([int]$process.ParentProcessId) -and -not $owned.ContainsKey([int]$process.ProcessId)) {
                $owned[[int]$process.ProcessId]=$process; $added=$true
            }
        }
    } while ($added)
    # Stop the supervisor first so it cannot change the set being cleaned up.
    $ordered=@($launch)+@($owned.Values | Where-Object {$_.ProcessId -ne $launch.ProcessId})
    foreach ($process in $ordered) {
        $live=Get-CimInstance Win32_Process -Filter "ProcessId=$($process.ProcessId)"
        if ($live -and $live.CreationDate -eq $process.CreationDate -and $live.CommandLine -eq $process.CommandLine) {
            Stop-Process -Id $live.ProcessId -Force -ErrorAction SilentlyContinue
        }
    }
    Write-Host "Stopped shadow experiment $($launch.ProcessId) and its acquisition/algorithm children."
}
if (-not $launches.Count) { Write-Host 'No matching shadow experiment found.' }
$remaining=@(Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object {
    $_.CommandLine -like "*$root*" -and $_.CommandLine -match 'from evtol_targeting\.state_node import main'
})
if ($remaining.Count) {
    throw "A separately launched state adapter is still active (PID $($remaining.ProcessId -join ', ')); stop its owning launch before using -NewSession."
}
Write-Host 'UE, Foxglove and saved origin were preserved. You can now start_experiment.ps1 -NewSession before takeoff.'
