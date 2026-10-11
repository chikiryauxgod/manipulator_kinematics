from pathlib import Path
import os
import shutil
import subprocess

import pytest

from conftest import ROOT, run_checked

CC = os.environ.get("CC", "cc")
CXX = os.environ.get("CXX", "c++")

CONSUMER = '''#include <manipulator/forward.h>
#include <manipulator/transform.h>
int main(void) {
    k_dh_joint joints[3] = {{K_REVOLUTE,1,0,0,0}, {K_REVOLUTE,2,0,0,0}, {K_REVOLUTE,3,0,0,0}};
    k_real q[3] = {0,0,0}; k_model model; k_transform out, unit; k_joint_config storage[3];
    if(k_model_begin(&model,storage,3,3)!=K_OK) return 1;
    k_joint_limits limits = {K_UNBOUNDED,0,0};
    for(size_t i=0;i<3;++i) {
        if(k_model_set_joint(&model,i,&joints[i])!=K_OK) return 4;
        if(k_model_set_limits(&model,i,&limits)!=K_OK) return 5;
    }
    if(k_transform_identity(&unit)!=K_OK || k_model_set_base(&model,&unit)!=K_OK
        || k_model_set_tool(&model,&unit)!=K_OK || k_model_finalize(&model)!=K_OK) return 6;
    if(k_forward(&model,q,3,&out)!=K_OK) return 2;
    return out.m[0][3]==6 && out.m[1][3]==0 ? 0 : 3;
}
'''


@pytest.mark.parametrize("language", ["c", "c++"])
def test_headers_are_self_contained(built_profile, tmp_path, language):
    for header in ["types", "transform", "model", "forward", "inverse"]:
        source=tmp_path/f"{header}.c"
        source.write_text(f"#include <manipulator/{header}.h>\n")
        run_checked([CC if language=="c" else CXX, "-x", language,
                     "-std=c11" if language=="c" else "-std=c++17", "-pedantic-errors", "-fsyntax-only",
                     "-I", ROOT/"include", "-I", built_profile[1]/"generated", source])


@pytest.mark.parametrize("custom_dirs", [False, True])
def test_relocated_install_and_external_consumers(profile, tmp_path, custom_dirs):
    library_source=tmp_path/"library source"
    library_source.mkdir()
    shutil.copy2(ROOT/"CMakeLists.txt",library_source/"CMakeLists.txt")
    for directory in ("cmake", "include", "src"):
        shutil.copytree(ROOT/directory,library_source/directory)
    license_notice=(ROOT/"LICENSE").read_text()
    (library_source/"LICENSE").write_text(license_notice)
    library_build=tmp_path/"library build"
    configure=["cmake","-S",library_source,"-B",library_build,
               f"-DK_REAL_TYPE={profile}","-DCMAKE_BUILD_TYPE=Release"]
    if custom_dirs:
        configure.extend(["-DCMAKE_INSTALL_INCLUDEDIR=headers", "-DCMAKE_INSTALL_LIBDIR=libraries",
                          "-DCMAKE_INSTALL_DOCDIR=manuals/manipulator"])
    run_checked(configure)
    run_checked(["cmake","--build",library_build,"--parallel","2"])
    prefix=tmp_path/"install prefix"
    run_checked(["cmake","--install",library_build,"--prefix",prefix])
    moved=tmp_path/"relocated prefix"; prefix.rename(moved)
    docdir="manuals/manipulator" if custom_dirs else "share/doc/manipulator"
    assert (moved/docdir/"LICENSE").read_text()==license_notice
    for config in moved.rglob("*.cmake"):
        text=config.read_text()
        assert str(ROOT) not in text
        assert str(library_source) not in text
        assert str(library_build) not in text
    shutil.rmtree(library_source)
    shutil.rmtree(library_build)
    source=tmp_path/"consumer project"; source.mkdir()
    (source/"main.c").write_text(CONSUMER)
    (source/"main.cpp").write_text(CONSUMER)
    (source/"CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.20)
project(consumer LANGUAGES C CXX)
find_package(manipulator 0.2 CONFIG REQUIRED)
add_executable(c_consumer main.c)
add_executable(cpp_consumer main.cpp)
target_link_libraries(c_consumer PRIVATE manipulator::kinematics)
target_link_libraries(cpp_consumer PRIVATE manipulator::kinematics)
set_target_properties(cpp_consumer PROPERTIES CXX_STANDARD 17)
file(GENERATE OUTPUT "${CMAKE_BINARY_DIR}/consumers-$<CONFIG>.txt"
    CONTENT "$<TARGET_FILE:c_consumer>\\n$<TARGET_FILE:cpp_consumer>\\n")
''')
    build=tmp_path/"consumer build"
    package_location=(f"-Dmanipulator_DIR={moved}/libraries/cmake/manipulator"
                      if custom_dirs else f"-DCMAKE_PREFIX_PATH={moved}")
    run_checked(["cmake","-S",source,"-B",build,"-DCMAKE_BUILD_TYPE=Release",package_location])
    run_checked(["cmake","--build",build,"--config","Release","--parallel","2"])
    for exe in (build/"consumers-Release.txt").read_text().splitlines():
        run_checked([exe])
        symbols=run_checked(["nm",exe]).stdout
        assert "inverse_position" not in symbols
        assert "k_internal_dls" not in symbols


def test_add_subdirectory_is_c_only(built_profile, tmp_path):
    source=tmp_path/"parent"; source.mkdir()
    (source/"main.c").write_text(CONSUMER)
    (source/"CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.20)
project(parent LANGUAGES C)
set(K_REAL_TYPE {built_profile[0]} CACHE STRING "" FORCE)
add_subdirectory([[{ROOT}]] library)
add_executable(consumer main.c)
target_link_libraries(consumer PRIVATE manipulator::kinematics)
file(GENERATE OUTPUT "${{CMAKE_BINARY_DIR}}/exe-$<CONFIG>.txt" CONTENT "$<TARGET_FILE:consumer>\\n")
''')
    build=tmp_path/"parent-build"
    run_checked(["cmake","-S",source,"-B",build,"-DCMAKE_BUILD_TYPE=Release",
                 "-DCMAKE_CXX_COMPILER=/does/not/exist"])
    run_checked(["cmake","--build",build,"--parallel","2"])
    run_checked([(build/"exe-Release.txt").read_text().strip()])
    assert not list(build.rglob("manipulator_unit_tests"))


def test_library_has_no_allocator_or_io_dependencies(built_profile):
    library=next(built_profile[1].glob("libmanipulator_kinematics_*.a"))
    result=run_checked(["nm","-u",library])
    symbols={line.split()[-1] for line in result.stdout.splitlines() if " U " in line}
    assert not symbols.intersection({"malloc","calloc","realloc","free","printf","puts","fopen","_sbrk"})


@pytest.mark.parametrize("requested_version", ["0.0", "0.1.1", "1.0"])
def test_incompatible_package_versions_are_rejected(built_profile, tmp_path, requested_version):
    prefix=tmp_path/"installed"
    run_checked(["cmake","--install",built_profile[1],"--prefix",prefix])
    source=tmp_path/"consumer"; source.mkdir()
    (source/"CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.20)
project(consumer LANGUAGES C)
find_package(manipulator {requested_version} CONFIG REQUIRED)
''')
    result=subprocess.run(["cmake","-S",str(source),"-B",str(tmp_path/"build"),
                           f"-DCMAKE_PREFIX_PATH={prefix}"],capture_output=True,text=True,timeout=30)
    assert result.returncode!=0, "An incompatible development API version was accepted"
    assert "0.2.0" in result.stderr
    assert "not accepted" in result.stderr


def test_invalid_profile_is_configuration_error(tmp_path):
    result=subprocess.run(["cmake","-S",str(ROOT),"-B",str(tmp_path/"bad"),"-DK_REAL_TYPE=half"],
                          capture_output=True,text=True,timeout=30)
    assert result.returncode!=0
    assert "must be float or double" in result.stderr


@pytest.mark.parametrize("flag", ["-ffast-math", "-ffinite-math-only"])
def test_fast_math_is_rejected(built_profile, tmp_path, flag):
    result=subprocess.run([CC,"-std=c11",flag,"-I",str(ROOT/"include"),
                           "-I",str(built_profile[1]/"generated"),"-c",str(ROOT/"src/transform.c"),
                           "-o",str(tmp_path/"bad.o")], capture_output=True,text=True,timeout=30)
    assert result.returncode!=0
    assert "violates the numerical contract" in result.stderr


def test_conflicting_config_is_rejected(built_profile, tmp_path):
    source=tmp_path/"main.c"; source.write_text(CONSUMER)
    override="0" if built_profile[0]=="float" else "1"
    result=subprocess.run([CC,f"-DK_USE_FLOAT={override}","-I",str(ROOT/"include"),
                           "-I",str(built_profile[1]/"generated"),"-fsyntax-only",str(source)],
                           capture_output=True,text=True,timeout=30)
    assert result.returncode!=0
    assert "conflicts with installed" in result.stderr


def test_configurable_driver_limit(tmp_path):
    build=tmp_path/"limited"
    run_checked(["cmake","-S",ROOT,"-B",build,"-DK_BUILD_TEST_DRIVER=ON",
                 "-DK_DRIVER_MAX_JOINTS=3","-DCMAKE_BUILD_TYPE=Release"])
    run_checked(["cmake","--build",build,"--parallel","2"])
    exe=Path((build/"driver-path-Release.txt").read_text().strip())
    from test_forward_stream import request_line
    result=subprocess.run([str(exe)],input=request_line([["R",0,0,0,0,0]]*4)+"\n",
                          capture_output=True,text=True,timeout=5)
    assert result.returncode==0 and result.stdout=="ERR REQUEST_TOO_LARGE\n"


def test_multiconfig_driver_metadata(tmp_path):
    build=tmp_path/"multiconfig"
    run_checked(["cmake","-G","Ninja Multi-Config","-S",ROOT,"-B",build,"-DK_BUILD_TEST_DRIVER=ON"])
    run_checked(["cmake","--build",build,"--config","Release","--parallel","2"])
    exe=Path((build/"driver-path-Release.txt").read_text().strip())
    assert exe.is_file()
    result=subprocess.run([str(exe)],input="",capture_output=True,text=True,timeout=5)
    assert result.returncode==0 and result.stdout==""
