"""Replay identical estimator samples and serial bytes on the pre-RTOS and current code.

Run after resolving the teensy41 PlatformIO dependencies, with a C++17 compiler:
    python tools/regression/flight_links/run.py [--cxx /path/to/g++]
"""
import argparse
import difflib
import io
import os
from pathlib import Path
import subprocess
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BUILD = ROOT / "build" / "flight-link-replay"
BASE = "3b38f89"


def build(source, compiler, env, baseline):
    exe = BUILD / ("baseline" if baseline else "current")
    if os.name == "nt":
        exe = exe.with_suffix(".exe")
    eigen = ROOT / ".pio/libdeps/teensy41/ArduinoEigen/ArduinoEigen"
    if not (eigen / "Eigen/Dense").exists():
        raise SystemExit("Resolve the teensy41 ArduinoEigen dependency with PlatformIO first.")
    src = source / "src"
    units = sorted((src / "control").glob("*.cpp"))
    units += [f for f in sorted((src / "allocation").glob("*.cpp")) if f.name != "control_allocation.cpp"]
    units += [src / f for f in (
        "actuators/actuator_output.cpp", "config/aircraft_config.cpp",
        "interaircraft_comm/interaircraft_common.cpp", "interaircraft_comm/interaircraft_comm.cpp",
        "radio_comm/radio_comm.cpp", "parameters/control_parameters.cpp",
        "sensors/filters.cpp", "math/math_utils.cpp")]
    units += [source / "lib/SBUS/SBUS.cpp", HERE / "replay.cpp"]
    command = [compiler, "-std=c++17", "-O1", "-ffast-math", "-DARDUINO=10800",
               "-D__IMXRT1062__", f"-DREPLAY_BASELINE={int(baseline)}",
               "-I", str(BUILD / "include"), "-I", str(HERE), "-I", str(src),
               "-I", str(source / "lib/SBUS"), "-I", str(eigen), "-include", "host.h"]
    subprocess.run([*command, *map(str, units), "-o", str(exe)], check=True, env=env)
    return exe


def main():
    args = argparse.ArgumentParser()
    args.add_argument("--cxx", default=os.environ.get("CXX", "g++"))
    compiler = args.parse_args().cxx
    env = os.environ.copy()
    if Path(compiler).is_absolute():
        env["PATH"] = str(Path(compiler).parent) + os.pathsep + env["PATH"]
    headers = BUILD / "include"
    (headers / "util").mkdir(exist_ok=True, parents=True)
    for name in ("Arduino.h", "Servo.h", "SPI.h", "util/atomic.h"):
        (headers / name).write_text('#include "host.h"\n', encoding="utf-8")
    (headers / "FreeRTOS.h").write_text("#pragma once\n", encoding="utf-8")
    (headers / "task.h").write_text(
        "#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n",
        encoding="utf-8")
    (headers / "ArduinoEigenDense.h").write_text(
        "#pragma once\n#undef ARDUINO\n#include <Eigen/Geometry>\n#define ARDUINO 10800\n", encoding="utf-8")
    archive = subprocess.check_output(
        ["git", "archive", "--format=zip", BASE, "src", "lib/SBUS"], cwd=ROOT)
    source = BUILD / BASE
    with zipfile.ZipFile(io.BytesIO(archive)) as snapshot:
        snapshot.extractall(source)
    reference = build(source, compiler, env, True)
    current = build(ROOT, compiler, env, False)
    for aircraft in ("1", "2"):
        def trace(exe):
            return subprocess.check_output([str(exe), aircraft], env=env, text=True).splitlines()

        expected, actual = trace(reference), trace(current)
        if expected != actual:
            raise SystemExit("\n".join(difflib.unified_diff(expected, actual, BASE, "current")))
        print(f"Aircraft {aircraft}: {sum(' pwm ' in x for x in actual)} identical PWM/RC samples, "
              f"{sum(x.startswith('uart') for x in actual)} identical wire frames")
    print("Passed fragmented/bad frames, snapshot handoff, RX budget, SBUS validity, "
          "freshness expiry, TX backpressure, stalled and disabled control.")


if __name__ == "__main__":
    main()
