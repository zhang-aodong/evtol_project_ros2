$ErrorActionPreference = 'Stop'
. "$PSScriptRoot\activate.ps1"
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
Push-Location $projectRoot
try {
    & python -m unittest discover -s tests -v
    if ($LASTEXITCODE -ne 0) { throw 'Unit tests failed.' }
} finally { Pop-Location }
