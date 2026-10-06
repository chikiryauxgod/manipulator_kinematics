"""Prepare a CI container; deliberately never called by CMake."""
import json
import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
lock = json.loads((ROOT / "ci/test-toolchains.lock").read_text())
parser=argparse.ArgumentParser()
parser.add_argument("--mode",choices=["host","embedded","all"],default="all")
args=parser.parse_args()
selected={"gcc-13","g++-13","cmake","ninja-build"}
if args.mode in {"host","all"}:
    selected.update({"clang-18","libclang-rt-18-dev","libgtest-dev"})
if args.mode in {"embedded","all"}:
    selected.update({"gcc-arm-none-eabi","binutils-arm-none-eabi","libnewlib-arm-none-eabi",
                     "libnewlib-dev","qemu-system-arm"})
packages = [f"{name}={lock['apt_packages'][name]}" for name in sorted(selected)]
subprocess.run(["apt-get", "update"], check=True)
subprocess.run(["apt-get", "install", "-y", "--no-install-recommends", *packages,
                "make", "python3-venv", "ca-certificates"], check=True)
subprocess.run(["python3", "-m", "venv", str(ROOT / ".venv")], check=True)
subprocess.run([str(ROOT / ".venv/bin/python"), "-m", "pip", "install",
                "-r", str(ROOT / lock["python_requirements"])], check=True)
