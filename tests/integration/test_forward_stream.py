import math
import queue
import subprocess
import threading

import pytest

IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def request_line(rows=None, base=None, tool=None):
    rows = rows if rows is not None else [["R", 1, 0, 0, 0, 0]]
    tokens = ["FK", len(rows)]
    for row in rows:
        tokens.extend(row)
    tokens.extend(base if base is not None else IDENTITY)
    tokens.extend(tool if tool is not None else IDENTITY)
    return " ".join(map(str, tokens))


def execute(driver, text):
    result = subprocess.run([str(driver[1])], input=text, capture_output=True, text=True, timeout=5)
    assert result.returncode == 0
    assert result.stderr == ""
    return result.stdout.splitlines()


def matrix(line):
    fields = line.split()
    assert fields[0] == "OK"
    assert len(fields) == 17
    result = [float(x) for x in fields[1:]]
    assert all(math.isfinite(x) for x in result)
    assert result[12:] == [0, 0, 0, 1]
    return result


@pytest.mark.parametrize("n", [1, 3, 4, 5, 6, 7, 16, 64])
def test_planar_independent_reference(driver, n):
    rows = [["R", 0.1 + i * 0.01, 0, 0, 0, (-1)**i * 0.3] for i in range(n)]
    out = matrix(execute(driver, request_line(rows) + "\n")[0])
    angle = x = y = 0
    for _, length, _, _, _, q in rows:
        angle += q
        x += length * math.cos(angle)
        y += length * math.sin(angle)
    scale = max(1, sum(abs(r[1]) for r in rows))
    atol = 5e-5 if driver[0] == "float" else 5e-12
    rtol = 5e-6 if driver[0] == "float" else 5e-13
    tol = atol*scale + rtol*max(abs(x), abs(y))
    assert out[3] == pytest.approx(x, abs=tol)
    assert out[7] == pytest.approx(y, abs=tol)
    rotation_tol = 2e-5 if driver[0] == "float" else 2e-12
    assert [out[0], out[1], out[4], out[5]] == pytest.approx(
        [math.cos(angle), -math.sin(angle), math.sin(angle), math.cos(angle)], abs=rotation_tol)


def test_mixed_base_and_local_tool(driver):
    base = [0, -1, 0, 10, 1, 0, 0, 20, 0, 0, 1, 30, 0, 0, 0, 1]
    tool = IDENTITY.copy()
    tool[3] = 2
    rows = [["R", 1, 0, 0, 0, math.pi/2], ["P", 0, 0, 2, 0, 3]]
    out = matrix(execute(driver, request_line(rows, base, tool) + "\n")[0])
    assert [out[3], out[7], out[11]] == pytest.approx([7, 20, 35], abs=2e-5)


@pytest.mark.parametrize("ending", ["\n", "\r\n", ""])
def test_eof_and_line_endings(driver, ending):
    assert matrix(execute(driver, request_line() + ending)[0])[3] == 1


@pytest.mark.parametrize("ending", ["\n", "\r\n", ""])
def test_exact_line_limit_excludes_terminator(driver, ending):
    line=request_line()
    line+=" "*(65536-len(line))
    matrix(execute(driver,line+ending)[0])


def test_whitespace_is_explicitly_ascii(driver):
    line=" \t"+request_line().replace(" ","\t")+" \t"
    matrix(execute(driver,line+"\n")[0])
    assert execute(driver,request_line().replace(" ","\v",1)+"\n")==["ERR PARSE_ERROR"]


@pytest.mark.parametrize("bad", ["", "FK", "FK 0", "FK 01", "FK +1", "FK 65", "FK 99999999999999999999999999"])
def test_invalid_then_valid_in_same_stream(driver, bad):
    lines = execute(driver, request_line()+"\n"+bad+"\n"+request_line()+"\n")
    assert len(lines) == 3
    matrix(lines[0]); matrix(lines[2])
    assert lines[1].startswith("ERR ")


@pytest.mark.parametrize("number", ["0x1p0", "1f", "1,2", "1e", ".", "+", "NAN", "nan(x)", "infinity"])
def test_number_grammar(driver, number):
    assert execute(driver, request_line([["R", number, 0, 0, 0, 0]])+"\n") == ["ERR PARSE_ERROR"]


@pytest.mark.parametrize("number", ["nan", "inf", "+inf", "-inf"])
def test_nonfinite_is_api_error(driver, number):
    assert execute(driver, request_line([["R", number, 0, 0, 0, 0]])+"\n") == ["ERR MK_NONFINITE_INPUT"]


@pytest.mark.parametrize("number", ["+1.", ".25", "-0.5", "2e-3"])
def test_allowed_numbers(driver, number):
    out = matrix(execute(driver, request_line([["R", number, 0, 0, 0, 0]])+"\n")[0])
    assert out[3] == pytest.approx(float(number), abs=1e-7)


@pytest.mark.parametrize("number", ["1e10000", "1e-10000"])
def test_range_errors(driver, number):
    assert execute(driver, request_line([["R", number, 0, 0, 0, 0]])+"\n") == ["ERR NUMBER_RANGE_ERROR"]


def test_profile_specific_range(driver):
    for number in ["1e100", "1e-100"]:
        line = execute(driver, request_line([["R", number, 0, 0, 0, 0]])+"\n")[0]
        if driver[0] == "float":
            assert line == "ERR NUMBER_RANGE_ERROR"
        else:
            assert matrix(line)[3] == pytest.approx(float(number))


@pytest.mark.parametrize("kind", ["nul", "nonascii", "long", "extra", "truncated", "carriage"])
def test_corruption_and_recovery(driver, kind):
    good = request_line()
    bad = {"nul":good+"\0", "nonascii":good+"\x80", "long":"x"*65537,
           "extra":good+" 9", "truncated":" ".join(good.split()[:-1]), "carriage":good+"\r"}[kind]
    if kind == "carriage": bad += " x"
    lines = execute(driver, bad+"\n"+good+"\n")
    assert lines[0] == ("ERR REQUEST_TOO_LARGE" if kind=="long" else "ERR PARSE_ERROR")
    matrix(lines[1])


def test_empty_eof(driver):
    assert execute(driver, "") == []


def test_invalid_transform(driver):
    bad = IDENTITY.copy(); bad[12] = 1e-10
    assert execute(driver, request_line(base=bad)+"\n") == ["ERR MK_INVALID_TRANSFORM"]


def test_numeric_error_and_recovery(driver):
    huge = "3e38" if driver[0]=="float" else "1e308"
    line = request_line([["P",0,0,huge,0,huge]])
    answers = execute(driver,line+"\n"+request_line()+"\n")
    assert answers[0] == "ERR MK_NUMERIC_ERROR"
    matrix(answers[1])


def test_fragmented_interactive_stream(driver):
    process = subprocess.Popen([str(driver[1])], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    events = queue.Queue(maxsize=64)
    def reader(stream, name):
        try:
            total=0
            while True:
                line=stream.readline(4097)
                if not line: break
                total+=len(line)
                if total>65536 or len(line)>4096: raise RuntimeError("Output limit exceeded")
                events.put((name,line),timeout=2)
        except Exception as exc:
            events.put(("error",repr(exc).encode()),timeout=2)
    threads=[threading.Thread(target=reader,args=(stream,name),daemon=True)
             for stream,name in [(process.stdout,"out"),(process.stderr,"err")]]
    for thread in threads: thread.start()
    try:
        payload=(request_line()+"\n").encode()
        for chunk in [payload[:5],payload[5:20],payload[20:]]:
            process.stdin.write(chunk); process.stdin.flush()
        name,line=events.get(timeout=5)
        assert name=="out"; matrix(line.decode())
        process.stdin.write(b"bad\n"); process.stdin.flush()
        assert events.get(timeout=5)==("out",b"ERR PARSE_ERROR\n")
        process.stdin.write(payload); process.stdin.flush()
        name,line=events.get(timeout=5); assert name=="out"; matrix(line.decode())
        process.stdin.close()
        assert process.wait(timeout=5)==0
    finally:
        if process.poll() is None:
            process.terminate()
            try: process.wait(timeout=2)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=2)
        for thread in threads: thread.join(timeout=2); assert not thread.is_alive()
        for stream in [process.stdin,process.stdout,process.stderr]: stream.close()
    assert events.empty()
