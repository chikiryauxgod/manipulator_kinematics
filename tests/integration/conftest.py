from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]


def pytest_addoption(parser):
    parser.addoption("--driver-path", help="Use an already built driver (one selected profile)")
    parser.addoption("--profile", choices=["float", "double"], default="double")
    parser.addoption("--embedded", action="store_true", help="Require cross build and QEMU")
    parser.addoption("--sanitizers", action="store_true", help="Instrument host integration binaries")


def run_checked(args, **kwargs):
    result = subprocess.run([str(a) for a in args], capture_output=True, text=True, timeout=120, **kwargs)
    assert result.returncode == 0, f"{args}\n{result.stdout}\n{result.stderr}"
    return result


@pytest.fixture(scope="session", params=["double", "float"])
def profile(request):
    return request.param


@pytest.fixture(scope="session")
def built_profile(profile, tmp_path_factory, request):
    build = tmp_path_factory.mktemp(f"cmake-{profile}")
    run_checked(["cmake", "-S", ROOT, "-B", build, f"-DMANIPULATOR_REAL_TYPE={profile}",
                 "-DMANIPULATOR_BUILD_TEST_DRIVER=ON", "-DCMAKE_BUILD_TYPE=Release",
                 f"-DMANIPULATOR_ENABLE_SANITIZERS={'ON' if request.config.getoption('--sanitizers') else 'OFF'}"])
    run_checked(["cmake", "--build", build, "--config", "Release", "--parallel", "2"])
    manifest = build / "driver-path-Release.txt"
    assert manifest.is_file()
    driver = Path(manifest.read_text().strip())
    assert driver.is_file()
    return profile, build, driver


@pytest.fixture(scope="session")
def driver(request, profile):
    supplied = request.config.getoption("--driver-path")
    if supplied:
        if profile != request.config.getoption("--profile"):
            pytest.skip("Only the supplied driver's selected profile")
        path=Path(supplied).resolve()
        assert path.is_file()
        return profile,path
    built_profile=request.getfixturevalue("built_profile")
    return built_profile[0], built_profile[2]
