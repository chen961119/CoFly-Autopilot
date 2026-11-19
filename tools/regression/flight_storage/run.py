"""Run the hardware-independent flight storage regression with a C++17 compiler."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

here = Path(__file__).resolve().parent
src = here.parents[2] / "src"
compiler = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("CXX", "g++")
build = src.parent / "build"
build.mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix="flight-storage-", dir=build) as temp:
    out = Path(temp)
    for name in ("Arduino.h", "SPI.h", "SBUS.h", "SD.h", "FreeRTOS.h", "task.h", "queue.h", "semphr.h"):
        (out / name).write_text('#include "host_devices.h"\n', encoding="utf-8")
    exe = out / ("check.exe" if os.name == "nt" else "check")
    env = os.environ.copy()
    env["TEMP"] = env["TMP"] = str(out)
    env["PATH"] = str(Path(compiler).parent) + os.pathsep + env["PATH"]
    sources = [str(src / "parameters" / f"parameter_{module}.cpp") for module in ("record", "text", "sd")]
    subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-I", str(out), "-I", str(src),
                    "-I", str(here), str(here / "check.cpp"), *sources, "-o", str(exe)], check=True, env=env)
    subprocess.run([str(exe)], check=True, env=env)
print("Flight storage regression passed")
