import subprocess

from conftest import ROOT, run_checked
from test_build_package import CONSUMER, CC


def test_opposite_profile_fails_at_link_time(built_profile, tmp_path):
    other="float" if built_profile[0]=="double" else "double"
    build=tmp_path/"other"
    run_checked(["cmake","-S",ROOT,"-B",build,f"-DMANIPULATOR_REAL_TYPE={other}"])
    source=tmp_path/"consumer.c"; source.write_text(CONSUMER)
    library=next(built_profile[1].glob("libmanipulator_kinematics_*.a"))
    result=subprocess.run([CC,"-std=c11","-I",str(ROOT/"include"),"-I",str(build/"generated"),
                           str(source),str(library),"-lm","-o",str(tmp_path/"wrong")],
                          capture_output=True,text=True,timeout=30)
    assert result.returncode!=0
    wanted="manipulator_v1_f32_forward" if other=="float" else "manipulator_v1_f64_forward"
    assert wanted in result.stderr
