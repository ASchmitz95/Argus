import numpy as np
import pytest

from motor.motor_control import CONFIG
from arm.kinematics import CONFIG as ARM_CONFIG
from arm.kinematics import forward_kinematics, pose_from_rpy, orientation_error
from arm.arm_control import follow_path, move_linear, move_to_pose, plan_linear, plan_near_linear, read_angles, solve_pose

START = [0, 20, 20, 0, 0, 0]
MAX_JOINT_STEP = ARM_CONFIG["linear_motion"]["max_joint_step"]
UNREACHABLE = pose_from_rpy(1000, 0, 0, 0, 0, 0)


def distance_to_line(point, start, end):
    direction = (end - start) / np.linalg.norm(end - start)
    offset = point - start
    return np.linalg.norm(offset - (offset @ direction) * direction)


def test_solve_pose_reaches_target():
    target = forward_kinematics([10, 40, 50, 0, 20, 0])
    result = solve_pose(target, START)
    assert np.linalg.norm(forward_kinematics(result)[:3, 3] - target[:3, 3]) <= 0.5


def test_position_mode_reaches_more_than_full_pose():
    # With the tool pointing forward the wrist would be behind the base.
    target = pose_from_rpy(0, 0, 300, 0, 0, 0)
    assert solve_pose(target, START) is None
    assert solve_pose(target, START, mode="position") is not None


def test_plan_linear_follows_straight_line():
    start_pose = forward_kinematics(START)
    target = start_pose.copy()
    target[:3, 3] += (40, 0, -30)

    path, step_time = plan_linear(START, target)
    assert step_time > 0
    for angles in path:
        assert distance_to_line(forward_kinematics(angles)[:3, 3], start_pose[:3, 3], target[:3, 3]) <= 0.6
    assert np.linalg.norm(forward_kinematics(path[-1])[:3, 3] - target[:3, 3]) <= 0.5


def test_plan_linear_direction_mode_keeps_tool_axis():
    target = pose_from_rpy(200, 40, 150, 0, 0, 0)
    plan = plan_linear(START, target, mode="direction")
    assert plan is not None
    end = forward_kinematics(plan[0][-1])
    assert np.degrees(np.linalg.norm(orientation_error(target[:3, :3], end[:3, :3], "direction"))) <= 0.5


def test_plan_near_linear_reaches_target_without_jumps():
    start = [0.2, 20, 17.8, -0.2, -1.8, 0]
    target = pose_from_rpy(200, 100, 60, 0, 90, 0)
    assert plan_linear(start, target) is None

    path, _, pos_dev, _ = plan_near_linear(start, target)
    steps = np.abs(np.diff(np.vstack([start, path]), axis=0)).max()
    assert steps <= MAX_JOINT_STEP + 1e-9
    assert np.linalg.norm(forward_kinematics(path[-1])[:3, 3] - target[:3, 3]) <= 0.5
    assert pos_dev > 0


def test_unreachable_target_is_rejected(fake_servos):
    assert plan_near_linear(START, UNREACHABLE) is None

    pk = fake_servos(START)
    assert move_linear(pk, UNREACHABLE) is None
    assert move_to_pose(pk, UNREACHABLE) is None
    assert not pk.writes and not pk.sync_packets


def test_follow_path_sends_synchronized_waypoints(fake_servos):
    target = forward_kinematics(START)
    target[:3, 3] += (30, 0, 0)
    path, step_time = plan_linear(START, target)

    pk = fake_servos(START)
    follow_path(pk, path, step_time)

    assert len(pk.sync_packets) == len(path)
    speeds = [speed for packet in pk.sync_packets for _, speed in packet.values()]
    assert 1 <= min(speeds) and max(speeds) <= CONFIG["speed"]
    assert np.linalg.norm(forward_kinematics(read_angles(pk))[:3, 3] - target[:3, 3]) <= 0.5


def test_follow_path_stops_on_overload(fake_servos):
    target = forward_kinematics(START)
    target[:3, 3] += (30, 0, 0)
    path, step_time = plan_linear(START, target)

    pk = fake_servos(START, load=CONFIG["max_load_moving"] + 1)
    with pytest.raises(RuntimeError):
        follow_path(pk, path, step_time)
    assert len(pk.sync_packets) == 1
