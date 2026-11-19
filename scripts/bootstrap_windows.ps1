[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$RosArchive,
    [Parameter(Mandatory=$true)][string]$PixiWorkspace,
    [string]$PixiExe = "$env:USERPROFILE\.pixi\bin\pixi.exe"
)
$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$RosArchive = (Resolve-Path -LiteralPath $RosArchive).Path
$release = Get-Content -LiteralPath "$root\environment\ros_release.json" -Raw -Encoding UTF8 | ConvertFrom-Json
if ((Get-FileHash -LiteralPath $RosArchive).Hash.ToLower() -ne $release.sha256) {
    throw "ROS archive differs from the validated release: $($release.filename). Review compatibility before changing environment/ros_release.json."
}
if (-not (Test-Path -LiteralPath $PixiExe)) { throw 'Install pixi first or provide -PixiExe.' }
if (Test-Path -LiteralPath (Join-Path $PixiWorkspace 'pixi.toml')) {
    throw 'Use a new dependency workspace; this script will not overwrite an existing manifest.'
}
New-Item -ItemType Directory -Force -Path $PixiWorkspace | Out-Null
Copy-Item -LiteralPath "$root\environment\pixi.toml","$root\environment\pixi.lock" -Destination $PixiWorkspace
& $PixiExe install --locked --manifest-path (Join-Path $PixiWorkspace 'pixi.toml')
if ($LASTEXITCODE -ne 0) { throw 'Pinned dependency installation failed.' }
# The ROS Windows release zip has a ros2-windows top-level directory.
Expand-Archive -LiteralPath $RosArchive -DestinationPath $PixiWorkspace
if (-not (Test-Path -LiteralPath (Join-Path $PixiWorkspace 'ros2-windows\local_setup.ps1'))) {
    throw 'Expected a Windows ROS 2 binary archive containing ros2-windows/local_setup.ps1.'
}
New-Item -ItemType Directory -Force -Path "$root\config" | Out-Null
$machine = @{PixiWorkspace=[IO.Path]::GetFullPath($PixiWorkspace);PixiExe=(Resolve-Path $PixiExe).Path}
$machine | ConvertTo-Json | Set-Content -LiteralPath "$root\config\machine.local.json" -Encoding UTF8
Write-Host 'ROS dependency workspace installed. Add UnrealEditor and UnrealProject to config/machine.local.json, then run scripts/build.ps1.'
