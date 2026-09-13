import pytest

from motor.motor_control import SERVOS, CONFIG, angles_to_steps, check_collision, move_to_angles, read_load, read_pos

START = [0, 20, 20, 0, 0, 0]


def test_check_collision_clamps_to_limits():
    angles = check_collision([500, -500, 90, -300, 95, -151])
    for angle, servo in zip(angles, SERVOS.values()):
        assert servo["min_angle"] <= angle <= servo["max_angle"]


def test_check_collision_rejects_wrong_length():
    with pytest.raises(ValueError):
        check_collision([0] * (len(SERVOS) - 1))


def test_angles_to_steps_matches_zero_positions():
    assert angles_to_steps([0] * len(SERVOS)) == {servo_ID: servo["null_pos"] for servo_ID, servo in SERVOS.items()}


def test_read_pos_in_degrees(fake_servos):
    pk = fake_servos(START)
    for servo_ID, angle in zip(SERVOS, START):
        # One step is about 0.09 degrees.
        assert read_pos(pk, servo_ID, degree=True) == pytest.approx(angle, abs=0.1)


def test_read_load_decodes_direction(fake_servos):
    assert read_load(fake_servos(START, load=450), 2) == 450
    assert read_load(fake_servos(START, load=(1 << 10) | 300), 2) == -300


def test_move_to_angles_reaches_target(fake_servos):
    pk = fake_servos(START)
    assert move_to_angles(pk, [10] * len(SERVOS)) is True
    assert all(speed == CONFIG["speed"] for _, _, speed, _ in pk.writes)


def test_move_to_angles_returns_false_on_timeout(fake_servos):
    pk = fake_servos(START, reach=False)
    assert move_to_angles(pk, [10] * len(SERVOS), timeout=0.05) is False


@pytest.mark.parametrize("load, moving", [(CONFIG["max_load_moving"] + 1, 1), (CONFIG["max_load_holding"] + 1, 0)])
def test_move_to_angles_stops_on_overload(fake_servos, load, moving):
    pk = fake_servos(START, load=load, moving=moving)
    with pytest.raises(RuntimeError):
        move_to_angles(pk, [10] * len(SERVOS))

    # stop() holds every servo at its current position.
    held = pk.writes[-len(SERVOS):]
    assert [(servo_ID, pos) for servo_ID, pos, _, _ in held] == list(pk.pos.items())


def test_move_to_angles_accepts_normal_load(fake_servos):
    pk = fake_servos(START, load=CONFIG["max_load_holding"] - 1)
    assert move_to_angles(pk, [10] * len(SERVOS)) is True
