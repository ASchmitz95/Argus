import numpy as np
import numpy.typing as npt
from motor.motor_control import CONFIG, SERVOS, read_pos, move_to_angles
from arm.kinematics import inverse_kinematics


def read_angles(pk) -> list[float]:
    """Return the current servo angles in degrees in the order of SERVOS."""
    return [read_pos(pk, servo_ID, degree=True) for servo_ID in SERVOS]


def solve_pose(
    target_pose: npt.NDArray[np.float64],
    start_angles: list[float],
    attempts: int = 10,
    max_angle_change: float = 90,
) -> list[float] | None:
    """Find servo angles for a target pose with as little joint motion as possible.

    Tries the inverse kinematics from start_angles first. If that fails or needs
    too much motion, retries from random angles within the servo limits and
    keeps the solution with the smallest joint motion.

    Args:
        target_pose: Target tool pose with shape (4, 4), position in mm.
        start_angles: Current servo angles in degrees.
        attempts: Number of random restarts.
        max_angle_change: Largest allowed change of a single servo angle in degrees.

    Returns:
        Servo angles in degrees in the order of SERVOS, or None if no solution
        within max_angle_change is found.
    """
    start = np.array(start_angles, dtype=float)

    result = inverse_kinematics(target_pose, start_angles)
    if result is not None and np.abs(np.array(result) - start).max() <= max_angle_change:
        return result

    limits = np.array([(servo["min_angle"], servo["max_angle"]) for servo in SERVOS.values()])
    rng = np.random.default_rng()
    best = None
    best_change = max_angle_change

    for _ in range(attempts):
        result = inverse_kinematics(target_pose, rng.uniform(limits[:, 0], limits[:, 1]))
        if result is None:
            continue

        # Motion time depends on the joint that moves the most.
        change = np.abs(np.array(result) - start).max()
        if change <= best_change:
            best = result
            best_change = change

    return best


def move_to_pose(pk, target_pose: npt.NDArray[np.float64], attempts: int = 10, max_angle_change: float = 90) -> list[float] | None:
    """Move the tool to a target pose.

    Starts the inverse kinematics from the current servo angles and waits
    until the motion completes. The arm does not move if no solution is found.

    Args:
        pk: Servo communication interface.
        target_pose: Target tool pose with shape (4, 4), position in mm.
        attempts: Number of random restarts, see solve_pose.
        max_angle_change: Largest allowed change of a single servo angle in degrees.

    Returns:
        The commanded servo angles in degrees, or None if the pose is not reachable.

    Raises:
        RuntimeError: If a servo is overloaded during the motion, see move_to_angles.
    """
    start = read_angles(pk)
    result = solve_pose(target_pose, start, attempts, max_angle_change)

    if result is None:
        print("Pose nicht erreichbar; Arm bleibt stehen.")
        return None

    # Wait for the joint with the longest travel, plus a margin for acceleration.
    change = np.abs(np.array(result) - start).max()
    timeout = change / 360 * CONFIG["steps_per_rev"] / CONFIG["speed"] + 1
    move_to_angles(pk, result, timeout=timeout)

    return result
