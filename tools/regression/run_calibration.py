"""Run the firmware calibration state machine against simulated clocks and sensors."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
cache = root / "build"
cache.mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix="calibration-", dir=cache) as directory:
    build = Path(directory)
    (build / "FreeRTOS.h").write_text("#pragma once\n")
    (build / "task.h").write_text("#define taskENTER_CRITICAL() ((void)0)\n"
                                  "#define taskEXIT_CRITICAL() ((void)0)\n")
    (build / "Arduino.h").write_text("#include <stdint.h>\nuint32_t millis(); uint32_t micros();\n"
                                     "extern uint32_t testCycles;\n#define ARM_DWT_CYCCNT testCycles\n"
                                     "#define F_CPU_ACTUAL 600000000u\n"
                                     "void delayMicroseconds(unsigned);\n")
    exe = build / ("calibration.exe" if os.name == "nt" else "calibration")
    env = os.environ.copy()
    env["TMP"] = env["TEMP"] = str(build)
    compiler = env.get("CXX", "g++")
    if Path(compiler).is_absolute():
        env["PATH"] = str(Path(compiler).parent) + os.pathsep + env["PATH"]
    subprocess.run([compiler, "-std=c++17", "-O2", "-I", str(build),
                    "-I", str(root / "src"), str(root / "src/sensors/sensor_calibration.cpp"),
                    str(root / "src/system/flight_clock.cpp"),
                    str(root / "tools/regression/calibration.cpp"), "-o", str(exe)], check=True, env=env)
    subprocess.run([str(exe)], check=True, env=env)
print("Calibration, lock, persistence, strain frames, cycle timing and counter rollover passed.")
