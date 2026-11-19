[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$npm=(Get-Command npm.cmd -ErrorAction Stop).Source
Push-Location (Join-Path $root 'foxglove\evtol-console')
try {
    & $npm ci --no-audit --no-fund
    if($LASTEXITCODE -ne 0){throw 'npm ci failed'}
    & $npm run typecheck
    if($LASTEXITCODE -ne 0){throw 'Panel typecheck failed'}
    & $npm run package
    if($LASTEXITCODE -ne 0){throw 'Panel package failed'}
} finally {Pop-Location}
