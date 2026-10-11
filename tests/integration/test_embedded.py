import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest

from conftest import ROOT, run_checked


@pytest.mark.embedded
def test_cross_link_qemu_and_no_heap(profile, request, tmp_path):
    if not request.config.getoption("--embedded"):
        pytest.skip("Enable mandatory embedded job with --embedded")
    tool_dir=os.environ.get("K_ARM_TOOLCHAIN_DIR", "")
    compiler=shutil.which("arm-none-eabi-gcc",path=tool_dir or None)
    qemu=os.environ.get("K_QEMU_PATH") or shutil.which("qemu-system-arm")
    assert compiler and qemu, "Embedded job requires installed ARM GCC and QEMU"
    build=tmp_path/"cross"
    run_checked(["cmake","-S",ROOT,"-B",build,
                 f"-DCMAKE_TOOLCHAIN_FILE={ROOT/'cmake/toolchains/arm_none_eabi.cmake'}",
                 "-DK_BUILD_EMBEDDED_SMOKE=ON",f"-DK_REAL_TYPE={profile}","-DCMAKE_BUILD_TYPE=Release"])
    run_checked(["cmake","--build",build,"--parallel","2"])
    elf=build/"manipulator_smoke.elf"
    result=subprocess.run([str(qemu),"-M","mps2-an386","-cpu","cortex-m4","-nographic",
                           "-semihosting-config","enable=on,target=native","-kernel",str(elf)],
                          capture_output=True,text=True,timeout=15)
    transcript=result.stdout+result.stderr
    (build/"qemu-result.txt").write_text(transcript)
    assert result.returncode==0,transcript
    assert "SMOKE_OK" in transcript
    stacks={int(n):int(size) for n,size in re.findall(r"(?m)^STACK (\d+) (\d+)",transcript)}
    assert set(stacks)=={3,6,64}
    assert max(stacks.values())-min(stacks.values())<=128
    assert max(stacks.values())<16384
    assert "IK_SMOKE_OK" in transcript
    ik_stacks={int(n):int(size) for n,size in re.findall(r"(?m)^IK_STACK (\d+) (\d+)",transcript)}
    assert set(ik_stacks)=={3,6,64}
    assert max(ik_stacks.values())-min(ik_stacks.values())<=128
    assert max(ik_stacks.values())<16384
    prefix=Path(compiler).parent
    nm=prefix/"arm-none-eabi-nm"
    size=prefix/"arm-none-eabi-size"
    symbols=run_checked([nm,elf]).stdout
    assert not re.search(r"\b(malloc|calloc|realloc|free|_sbrk|_malloc_r|_calloc_r|_realloc_r|_free_r|printf|puts)\b",symbols)
    (build/"symbols.txt").write_text(symbols)
    (build/"size.txt").write_text(run_checked([size,elf]).stdout)
    reports=list((build/"CMakeFiles/manipulator_kinematics.dir").rglob("*.su"))
    assert len(reports)==8
    for report in reports:
        for line in report.read_text().splitlines():
            assert line.split()[-1]=="static",line
