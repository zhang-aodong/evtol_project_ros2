[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][ValidateSet('bag','dataset')][string]$Mode,
    [string]$Config = 'config/collection.example.json',
    [string]$OutputRoot = '',
    [ValidateRange(0,86400)][double]$Duration = 0
)
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
. "$PSScriptRoot\activate.ps1"
if(-not [IO.Path]::IsPathRooted($Config)){$Config=Join-Path $root $Config}
if(-not(Test-Path -LiteralPath $Config -PathType Leaf)){throw "Collection config not found: $Config"}
$options=Get-Content -LiteralPath $Config -Raw -Encoding UTF8 | ConvertFrom-Json
if($OutputRoot){$options.output_dir=$OutputRoot}
$output=[string]$options.output_dir
if(-not [IO.Path]::IsPathRooted($output)){$output=Join-Path $root $output}
$session=$Mode+'_'+(Get-Date -Format 'yyyyMMdd_HHmmss')+'_'+[guid]::NewGuid().ToString('N').Substring(0,8)
$folder=Join-Path $output $session
New-Item -ItemType Directory -Path $folder | Out-Null
$snapshot=$folder+'.config.json'
$options | ConvertTo-Json -Depth 30 | Set-Content -LiteralPath $snapshot -Encoding UTF8
Write-Host "Collecting $Mode -> $folder"
Write-Host 'Press Ctrl+C once to drain queues and finalize. Do not close the terminal while saving.'
& python -u -m evtol_collection.recorder --config $snapshot --mode $Mode --output $folder --session $session --duration $Duration
if($LASTEXITCODE -ne 0){throw "Collection failed. Inspect $folder\status.json and collection.json"}
Write-Host "Collection saved: $folder"
