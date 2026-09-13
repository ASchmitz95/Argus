import numpy as np
import pytest

from motor.motor_control import SERVOS
from arm.kinematics import (
    AXES,
    CONFIG,
    forward_kinematics,
    inverse_kinematics,
    jacobian,
    orientation_error,
    pose_from_rpy,
    rotation,
    rotation_error,
)

LIMITS = np.array([(servo["min_angle"], servo["max_angle"]) for servo in SERVOS.values()])


def random_angles(rng, count):
    return rng.uniform(LIMITS[:, 0], LIMITS[:, 1], (count, len(SERVOS)))


def test_zero_pose_is_tool_pose():
    pose = forward_kinematics([0] * len(SERVOS))
    assert np.allclose(pose[:3, 3], CONFIG["tool"]["position"])
    assert np.allclose(pose[:3, :3], pose_from_rpy(0, 0, 0, *CONFIG["tool"]["rpy"])[:3, :3])


def test_forward_kinematics_rejects_wrong_length():
    with pytest.raises(ValueError):
        forward_kinematics([0] * (len(SERVOS) + 1))


def test_jacobian_matches_numerical_differences():
    rng = np.random.default_rng(0)
    eps = 1e-4
    for angles in random_angles(rng, 10):
        jac = jacobian(angles)
        for i in range(len(SERVOS)):
            plus, minus = angles.copy(), angles.copy()
            plus[i] += np.degrees(eps)
            minus[i] -= np.degrees(eps)
            plus_pose, minus_pose = forward_kinematics(plus), forward_kinematics(minus)
            assert np.allclose(jac[:3, i], (plus_pose[:3, 3] - minus_pose[:3, 3]) / (2 * eps), atol=1e-4)
            assert np.allclose(jac[3:, i], rotation_error(plus_pose[:3, :3], minus_pose[:3, :3]) / (2 * eps), atol=1e-6)


def test_orientation_error_modes():
    current = forward_kinematics([10, 30, 40, 20, 30, 10])[:3, :3]

    # Rotating about the tool x-axis keeps its direction.
    spun = current @ rotation(AXES["x"], 1.0)
    assert np.allclose(orientation_error(spun, current, "direction"), 0, atol=1e-9)
    assert np.linalg.norm(orientation_error(spun, current, "full")) == pytest.approx(1.0)
    assert np.allclose(orientation_error(spun, current, "position"), 0)

    # The direction error turns the current tool x-axis onto the target one.
    tilted = rotation(AXES["y"], 0.4) @ current
    error = orientation_error(tilted, current, "direction")
    angle = np.linalg.norm(error)
    assert np.allclose(rotation(error / angle, angle) @ current[:, 0], tilted[:, 0], atol=1e-9)

    with pytest.raises(ValueError):
        orientation_error(current, current, "xyz")


@pytest.mark.parametrize("mode", ["full", "direction", "position"])
def test_inverse_kinematics_roundtrip(mode):
    rng = np.random.default_rng(1)
    found = 0
    for angles in random_angles(rng, 50):
        target = forward_kinematics(angles)
        start = np.clip(angles + rng.uniform(-20, 20, len(SERVOS)), LIMITS[:, 0], LIMITS[:, 1])
        result = inverse_kinematics(target, start, mode=mode)
        if result is None:
            continue

        found += 1
        pose = forward_kinematics(result)
        assert np.linalg.norm(pose[:3, 3] - target[:3, 3]) <= 0.5 + 1e-9
        assert np.degrees(np.linalg.norm(orientation_error(target[:3, :3], pose[:3, :3], mode))) <= 0.5 + 1e-6

    assert found >= 45


def test_inverse_kinematics_unreachable():
    target = pose_from_rpy(1000, 0, 0, 0, 0, 0)
    assert inverse_kinematics(target, [0] * len(SERVOS)) is None
    assert len(inverse_kinematics(target, [0] * len(SERVOS), best_effort=True)) == len(SERVOS)
