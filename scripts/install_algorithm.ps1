[CmdletBinding()]
param([string]$ModelPython='', [switch]$InstallModelDependencies)
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
. "$PSScriptRoot\activate.ps1"
& python -m pip install --no-deps --target "$root\.algorithm_deps" -r "$root\environment\requirements-algorithm.txt"
if ($LASTEXITCODE -ne 0) { throw 'Algorithm dependency installation failed.' }
if ($ModelPython) {
    if (-not (Test-Path -LiteralPath $ModelPython -PathType Leaf)) { throw 'ModelPython must point to an existing Python 3.11 executable.' }
    & $ModelPython -c 'import sys; assert sys.version_info[:2] == (3,11), sys.version'
    if ($LASTEXITCODE -ne 0) { throw 'Use a dedicated Python 3.11 model environment.' }
    if ($InstallModelDependencies) {
        & $ModelPython -m pip install torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu128
        if ($LASTEXITCODE -ne 0) { throw 'PyTorch installation failed.' }
        & $ModelPython -m pip install -r "$root\environment\requirements-model.txt"
        if ($LASTEXITCODE -ne 0) { throw 'Model dependency installation failed.' }
    }
    $file=Join-Path $root 'config\machine.local.json'
    $machine=if (Test-Path -LiteralPath $file) { Get-Content -LiteralPath $file -Raw | ConvertFrom-Json } else { [PSCustomObject]@{} }
    $machine | Add-Member -NotePropertyName ModelPython -NotePropertyValue ([IO.Path]::GetFullPath($ModelPython)) -Force
    $machine | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath $file -Encoding UTF8
}
Write-Host 'Dependencies installed. Run scripts/build.ps1, then scripts/check_model.py --config config/targeting.yaml.'
