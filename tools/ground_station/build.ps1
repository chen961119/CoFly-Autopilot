$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
& (Join-Path $PSScriptRoot './prepare_python.ps1') -ToolDirectory $PSScriptRoot -Modules serial
& ./.venv/Scripts/python.exe -m pip install -r requirements.txt 'pyinstaller==6.22.3'
if ($LASTEXITCODE -ne 0) { throw 'Failed to install packaging dependencies.' }
& ./.venv/Scripts/python.exe -m PyInstaller --noconfirm --onefile --windowed `
    --hidden-import serial.urlhandler.protocol_socket `
    --name CoFlyGroundStation --distpath dist --workpath build --specpath build main.py
if ($LASTEXITCODE -ne 0) { throw 'Failed to build desktop software.' }
Write-Output (Join-Path $PSScriptRoot 'dist/CoFlyGroundStation.exe')
