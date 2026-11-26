[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
. "$PSScriptRoot\activate.ps1"
$env:PYTHONPATH="$root\.visualization_deps;$root\.algorithm_deps;$env:PYTHONPATH"
$env:PYTHONIOENCODING='utf-8'
& python -c "import unittest,sys;patterns=['test_ego*.py','test_localization*.py','test_console.py','test_three_camera.py','test_detection_launch.py','test_collection.py','test_targeting_nodes.py','test_fused_experiment.py'];suite=unittest.TestSuite(unittest.defaultTestLoader.discover(r'$root\tests',pattern=p) for p in patterns);result=unittest.TextTestRunner(stream=sys.stdout,verbosity=2).run(suite);sys.exit(not result.wasSuccessful())"
if($LASTEXITCODE -ne 0){throw 'Localization regression failed'}
