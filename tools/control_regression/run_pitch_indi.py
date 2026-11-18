"""Run the host regression with Python 3 and a C++17 compiler (CXX, default c++)."""

import os
from pathlib import Path
import subprocess


test_dir = Path(__file__).resolve().parent
root = test_dir.parents[1]
build = root / "build" / "control_regression"
build.mkdir(parents=True, exist_ok=True)
executable = build / ("pitch_indi.exe" if os.name == "nt" else "pitch_indi")
compiler = os.environ.get("CXX", "c++")
sources = [
    test_dir / "pitch_indi.cpp",
    "src/config/aircraft_config.cpp",
    "src/control/pitch_indi_controller.cpp",
    "src/control/control_output.cpp",
    "src/parameters/control_parameters.cpp",
    "src/allocation/master_mixer.cpp",
    "src/allocation/slave_mixer.cpp",
    "src/allocation/command_fusion.cpp",
]
subprocess.run(
    [compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-I", test_dir / "host",
     "-I", "src", *sources, "-o", executable],
    cwd=root,
    check=True,
)
subprocess.run([executable], check=True)
