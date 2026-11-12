param(
    [Parameter(Mandatory = $true)][string]$ToolDirectory,
    [Parameter(Mandatory = $true)][string]$Modules
)

$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false
$toolPath = (Resolve-Path -LiteralPath $ToolDirectory).Path
$venvPath = Join-Path $toolPath '.venv'
$pythonPath = Join-Path $venvPath 'Scripts/python.exe'
$requirementsPath = Join-Path $toolPath 'requirements.txt'
$runtimeCheck = "import sys, tkinter; assert sys.version_info >= (3, 10), 'Python 3.10+ is required'"

function Test-Python {
    param([string]$Executable, [string]$Code,
          [string[]]$PythonArguments = @(), [string[]]$ScriptArguments = @())
    try {
        & $Executable @PythonArguments -c $Code @ScriptArguments *> $null
        return $LASTEXITCODE -eq 0
    } catch {
        return $false
    }
}

if (-not (Test-Path -LiteralPath $requirementsPath -PathType Leaf)) {
    throw "Requirements file not found: $requirementsPath"
}

# A copied venv can still contain python.exe even though its base Python is gone.
$venvCheck = $runtimeCheck + "; assert sys.prefix != sys.base_prefix, 'Expected a virtual environment'"
if (-not (Test-Python -Executable $pythonPath -Code $venvCheck)) {
    $baseCheck = $runtimeCheck + '; import venv, ensurepip'
    if ($env:COFLY_PYTHON) {
        $candidates = @(@{ Command = $env:COFLY_PYTHON; Arguments = @() })
    } else {
        $candidates = @(
            @{ Command = 'py'; Arguments = @('-3') },
            @{ Command = 'python'; Arguments = @() },
            @{ Command = 'python3'; Arguments = @() }
        )
    }
    $basePython = $null
    foreach ($candidate in $candidates) {
        if (Test-Python -Executable $candidate.Command -PythonArguments $candidate.Arguments -Code $baseCheck) {
            $pythonArgs = $candidate.Arguments
            $basePython = & $candidate.Command @pythonArgs -c 'import sys; print(sys.executable)'
            if ($LASTEXITCODE -eq 0) { break }
            $basePython = $null
        }
    }
    if (-not $basePython) {
        throw 'No usable Python found. Install Python 3.10+ with Tcl/Tk and pip, or set COFLY_PYTHON to its full python.exe path. If already set, check COFLY_PYTHON.'
    }

    if (Test-Path -LiteralPath $venvPath) {
        # Only rename the tool's own .venv, and retain it for recovery.
        $existing = Get-Item -LiteralPath $venvPath -Force
        if ($existing.Parent.FullName -ne $toolPath -or
            ($existing.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
            throw "Refusing to move an environment outside the tool directory or through a link: $venvPath"
        }
        $backupName = '.venv.broken-' + [Guid]::NewGuid().ToString('N')
        Rename-Item -LiteralPath $venvPath -NewName $backupName
        Write-Host "[CoFly] Unusable environment saved as $backupName"
    }
    Write-Host "[CoFly] Creating .venv with $basePython"
    & $basePython -m venv $venvPath
    if ($LASTEXITCODE -ne 0) { throw 'Failed to create the Python environment. See the error above.' }
    if (-not (Test-Python -Executable $pythonPath -Code $venvCheck)) {
        throw 'The new environment cannot run Python 3.10+ with Tcl/Tk. Check the Python installation.'
    }
}

Write-Host "[CoFly] Python: $pythonPath"
if (-not (Test-Python -Executable $pythonPath -Code 'import pip')) {
    & $pythonPath -m ensurepip --upgrade
    if ($LASTEXITCODE -ne 0) { throw 'Failed to restore pip. See the error above.' }
}

$importCheck = "import importlib, sys; [importlib.import_module(name) for name in sys.argv[1].split(',')]"
if (-not (Test-Python -Executable $pythonPath -Code $importCheck -ScriptArguments @($Modules))) {
    Write-Host '[CoFly] Installing dependencies...'
    & $pythonPath -m pip install -r $requirementsPath
    if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed. Check the pip error above and retry start.cmd.' }
    & $pythonPath -c $importCheck $Modules
    if ($LASTEXITCODE -ne 0) { throw 'Dependencies are installed but cannot be loaded. See the error above.' }
}
