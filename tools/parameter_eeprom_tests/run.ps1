$ErrorActionPreference = 'Stop'
$repoPath = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$testOutput = Join-Path $repoPath 'tools/ground_station/build/parameter_eeprom_test.exe'
New-Item -ItemType Directory -Force -Path (Split-Path $testOutput) | Out-Null
& g++ -std=c++17 -O2 -I $PSScriptRoot -I (Join-Path $repoPath 'src') (Join-Path $PSScriptRoot 'test.cpp') (Join-Path $repoPath 'src/parameters/parameter_eeprom.cpp') (Join-Path $repoPath 'src/parameters/parameter_record.cpp') (Join-Path $repoPath 'src/parameters/parameter_text.cpp') -o $testOutput
if ($LASTEXITCODE -ne 0) { throw 'EEPROM test build failed' }
& $testOutput
if ($LASTEXITCODE -ne 0) { throw 'EEPROM tests failed' }
