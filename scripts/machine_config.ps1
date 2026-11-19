function Get-EvtolMachineValue {
    param([string]$Key, [string]$EnvironmentName, [string]$Fallback = '')
    $value = [Environment]::GetEnvironmentVariable($EnvironmentName, 'Process')
    if ($value) { return $value }
    $configPath = Join-Path $PSScriptRoot '..\config\machine.local.json'
    if (Test-Path -LiteralPath $configPath) {
        $config = Get-Content -LiteralPath $configPath -Raw -Encoding UTF8 | ConvertFrom-Json
        if ($config.$Key) { return [string]$config.$Key }
    }
    return $Fallback
}
