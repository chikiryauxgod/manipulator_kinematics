import math

import pytest

from test_forward_stream import IDENTITY, execute, matrix, request_line


def inverse_request(rows, target, request_id=1, iterations=500, trials=16,
                    tolerance=2e-5, damping=0.005, base=None, tool=None, length_scale=1):
    tokens = ["IK", len(rows), request_id]
    for row in rows:
        tokens.extend(row)
    tokens.extend(IDENTITY if base is None else base)
    tokens.extend(IDENTITY if tool is None else tool)
    tokens.extend([*target, tolerance, length_scale, damping, iterations, trials])
    return " ".join(map(str, tokens))


def result(line, count, request_id):
    fields = line.split()
    assert fields[:4] == ["OK", "IK", str(request_id), str(count)], line
    assert len(fields) == count + 7
    values = [float(value) for value in fields[4:4+count]]
    residual = float(fields[4+count])
    assert all(math.isfinite(value) for value in values)
    assert math.isfinite(residual)
    assert int(fields[5+count]) <= 500
    return values, residual


@pytest.mark.parametrize("count", [3, 4, 5, 6, 7, 16, 64])
def test_inverse_independent_spatial_family(driver, count):
    rows = [["R", 0, math.pi/2, 0.1, 0, 0.15, "B", -math.pi, math.pi, 1, 0.2]]
    rows.extend(["R", 0.15, 0, 0, 0, 0.15, "B", -math.pi, math.pi, 1, 0.2]
                for _ in range(count-1))
    angle = 0
    radial = 0
    height = 0.1
    for i in range(1, count):
        angle += 0.2 + 0.02*i
        radial += 0.15*math.cos(angle)
        height += 0.15*math.sin(angle)
    target = [math.cos(0.3)*radial, math.sin(0.3)*radial, height]
    line = execute(driver, inverse_request(rows, target, request_id=70000)+"\n")[0]
    positions, residual = result(line, count, 70000)
    assert residual <= 2e-5
    assert all(-math.pi <= value <= math.pi for value in positions)
    forward_rows = [row[:5]+[value] for row, value in zip(rows, positions)]
    pose = matrix(execute(driver, request_line(forward_rows)+"\n")[0])
    error = math.sqrt(sum((pose[4*r+3]-target[r])**2 for r in range(3)))
    assert error <= 3e-5


def test_fixed_prismatic_limits_and_transformed_tool(driver):
    rows = [["P", 0, 0, 0, 0, 0.2, "B", 0.2, 0.2, 1, 0.1],
            ["P", 0, 0, 0, 0, 0.1, "B", 0, 2, 1, 0.2]]
    base = IDENTITY.copy()
    base[11] = 3
    tool = IDENTITY.copy()
    tool[11] = 0.4
    line = execute(driver, inverse_request(rows, [0, 0, 4.1], base=base, tool=tool)+"\n")[0]
    positions, residual = result(line, 2, 1)
    assert positions[0] == pytest.approx(0.2, abs=1e-7)
    assert positions[1] == pytest.approx(0.5, abs=3e-5)
    assert residual <= 2e-5


@pytest.mark.parametrize("failure", ["limit", "missing", "options", "target", "stalled", "budget", "range"])
def test_inverse_failures_and_recovery(driver, failure):
    rows = [["R", 1, 0, 0, 0, 0, "B", -1, 1, 1, 0.1]]
    target = [0, 1, 0]
    kwargs = {"request_id":42}
    expected = "K_NO_CONVERGENCE"
    if failure == "limit":
        rows[0][5] = 2
        expected = "K_JOINT_LIMIT"
    elif failure == "options":
        kwargs["damping"] = 0
        expected = "K_INVALID_OPTIONS"
    elif failure == "target":
        target[0] = "nan"
        expected = "K_NONFINITE_INPUT"
    elif failure == "stalled":
        target = [2, 0, 0]
        expected = "K_STALLED"
    elif failure == "budget":
        kwargs["iterations"] = 1
    elif failure == "range":
        rows[0][7:9] = [1, -1]
        expected = "K_INVALID_MODEL"
    bad = inverse_request(rows, target, **kwargs)
    if failure == "missing":
        bad = " ".join(bad.split()[:-1])
        expected = "PARSE_ERROR"
    good_rows = [["P", 0, 0, 0, 0, 0, "U", 0, 0, 1, 0.2]]
    good = inverse_request(good_rows, [0, 0, 0.1], request_id=43)
    lines = execute(driver, bad+"\n"+good+"\n")
    assert lines[0] == f"ERR IK 42 {expected}"
    positions, _ = result(lines[1], 1, 43)
    assert positions[0] == pytest.approx(0.1, abs=2e-5)


def test_inverse_and_forward_requests_share_stream(driver):
    rows = [["P", 0, 0, 0, 0, 0, "U", 0, 0, 1, 0.2]]
    commands = [inverse_request(rows, [0, 0, 0.1], request_id=i) for i in range(3)]
    commands.insert(1, request_line())
    lines = execute(driver, "\n".join(commands))
    result(lines[0], 1, 0)
    matrix(lines[1])
    result(lines[2], 1, 1)
    result(lines[3], 1, 2)


def test_normalization_with_small_length_scale(driver):
    smallest = math.ldexp(1.0, -126 if driver[0] == "float" else -1022)
    rows = [["P", 0, 0, 0, 0, 0, "U", 0, 0, 1, 16]]
    request = inverse_request(rows, [0, 0, 8], length_scale=smallest)
    positions, residual = result(execute(driver, request+"\n")[0], 1, 1)
    assert positions[0] == pytest.approx(8, abs=2e-5)
    assert residual <= 2e-5


def test_normalization_with_large_joint_scale(driver):
    large = math.ldexp(1.0, 127 if driver[0] == "float" else 1023)
    rows = [["R", 4, 0, 0, 0, 0.1, "U", 0, 0, large, 0.2]]
    target = [4*math.cos(0.5), 4*math.sin(0.5), 0]
    request = inverse_request(rows, target, length_scale=large)
    positions, residual = result(execute(driver, request+"\n")[0], 1, 1)
    assert positions[0] == pytest.approx(0.5, abs=1e-5)
    assert residual <= 2e-5
