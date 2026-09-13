import numpy as np
import numpy.typing as npt
import time
from motor.motor_control import CONFIG, SERVOS, read_pos, move_to_angles, angles_to_steps, check_load, sync_write_pos
from arm.kinematics import CONFIG as ARM_CONFIG
from arm.kinematics import AXES, forward_kinematics, inverse_kinematics, orientation_error, rotation, rotation_error


def read_angles(pk) -> list[float]:
    """Return the current servo angles in degrees in the order of SERVOS."""
    return [read_pos(pk, servo_ID, degree=True) for servo_ID in SERVOS]


def solve_pose(
    target_pose: npt.NDArray[np.float64],
    start_angles: list[float],
    attempts: int = 10,
    max_angle_change: float = 90,
    mode: str = "full",
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
        mode: "full", "direction" or "position", see orientation_error.

    Returns:
        Servo angles in degrees in the order of SERVOS, or None if no solution
        within max_angle_change is found.
    """
    start = np.array(start_angles, dtype=float)

    result = inverse_kinematics(target_pose, start_angles, mode=mode)
    if result is not None and np.abs(np.array(result) - start).max() <= max_angle_change:
        return result

    limits = np.array([(servo["min_angle"], servo["max_angle"]) for servo in SERVOS.values()])
    rng = np.random.default_rng()
    best = None
    best_change = max_angle_change

    for _ in range(attempts):
        result = inverse_kinematics(target_pose, rng.uniform(limits[:, 0], limits[:, 1]), mode=mode)
        if result is None:
            continue

        # Motion time depends on the joint that moves the most.
        change = np.abs(np.array(result) - start).max()
        if change <= best_change:
            best = result
            best_change = change

    return best


def move_to_pose(pk, target_pose: npt.NDArray[np.float64], attempts: int = 10, max_angle_change: float = 90, mode: str = "full") -> list[float] | None:
    """Move the tool to a target pose.

    Starts the inverse kinematics from the current servo angles and waits
    until the motion completes. The arm does not move if no solution is found.

    Args:
        pk: Servo communication interface.
        target_pose: Target tool pose with shape (4, 4), position in mm.
        attempts: Number of random restarts, see solve_pose.
        max_angle_change: Largest allowed change of a single servo angle in degrees.
        mode: "full", "direction" or "position", see orientation_error.

    Returns:
        The commanded servo angles in degrees, or None if the pose is not reachable.

    Raises:
        RuntimeError: If a servo is overloaded during the motion, see move_to_angles.
    """
    start = read_angles(pk)
    result = solve_pose(target_pose, start, attempts, max_angle_change, mode)

    if result is None:
        print("Pose nicht erreichbar; Arm bleibt stehen.")
        return None

    # Wait for the joint with the longest travel, plus a margin for acceleration.
    change = np.abs(np.array(result) - start).max()
    timeout = change / 360 * CONFIG["steps_per_rev"] / CONFIG["speed"] + 1
    move_to_angles(pk, result, timeout=timeout)

    return result


def adapt_target(start_pose: npt.NDArray[np.float64], target_pose: npt.NDArray[np.float64], mode: str) -> npt.NDArray[np.float64]:
    """Return the target pose with the orientation the tool actually needs for mode.

    "full" keeps the target orientation, "direction" turns the start orientation
    by the shortest rotation onto the target tool x-axis, "position" keeps the
    start orientation. A straight path to the returned pose then contains no
    unnecessary rotation.

    Returns:
        Pose with shape (4, 4).
    """
    rot_vec = orientation_error(target_pose[:3, :3], start_pose[:3, :3], mode)
    rot_angle = np.linalg.norm(rot_vec)
    rot_axis = rot_vec / rot_angle if rot_angle > 1e-9 else AXES["z"]

    pose = target_pose.copy()
    pose[:3, :3] = rotation(rot_axis, rot_angle) @ start_pose[:3, :3]
    return pose


def interpolate_pose(start_pose: npt.NDArray[np.float64], target_pose: npt.NDArray[np.float64], fraction: float) -> npt.NDArray[np.float64]:
    """Return the tool pose at a fraction of the straight path between two poses.

    The position is interpolated linearly, the orientation rotates about a
    fixed axis at a constant rate.

    Args:
        start_pose: Start pose with shape (4, 4).
        target_pose: Target pose with shape (4, 4).
        fraction: Position on the path, 0 is start_pose and 1 is target_pose.

    Returns:
        Pose with shape (4, 4).
    """
    rot_vec = rotation_error(target_pose[:3, :3], start_pose[:3, :3])
    rot_angle = np.linalg.norm(rot_vec)
    rot_axis = rot_vec / rot_angle if rot_angle > 1e-9 else AXES["z"]

    pose = np.eye(4)
    pose[:3, 3] = start_pose[:3, 3] + fraction * (target_pose[:3, 3] - start_pose[:3, 3])
    pose[:3, :3] = rotation(rot_axis, fraction * rot_angle) @ start_pose[:3, :3]
    return pose


def segment_deviation(
    from_angles: npt.NDArray[np.float64],
    to_angles: npt.NDArray[np.float64],
    start_pose: npt.NDArray[np.float64],
    target_pose: npt.NDArray[np.float64],
    from_fraction: float,
    to_fraction: float,
    mode: str = "full",
) -> tuple[float, float]:
    """Return how far the tool leaves the straight path while the servos move
    linearly from from_angles to to_angles.

    The tool is checked at least every 2 degrees of joint motion against the
    path pose between from_fraction and to_fraction. The angle only counts
    the orientation that matters for mode, see orientation_error.

    Returns:
        A tuple (position, angle) with the largest deviation in mm and degrees.
    """
    checks = max(4, int(np.ceil(np.abs(to_angles - from_angles).max() / 2)))
    pos_dev = 0.0
    rot_dev = 0.0

    for k in range(1, checks + 1):
        reached = forward_kinematics(from_angles + k / checks * (to_angles - from_angles))
        expected = interpolate_pose(start_pose, target_pose, from_fraction + k / checks * (to_fraction - from_fraction))
        pos_dev = max(pos_dev, np.linalg.norm(reached[:3, 3] - expected[:3, 3]))
        rot_dev = max(rot_dev, np.degrees(np.linalg.norm(orientation_error(expected[:3, :3], reached[:3, :3], mode))))

    return pos_dev, rot_dev


def path_timing(start_pose: npt.NDArray[np.float64], target_pose: npt.NDArray[np.float64]) -> tuple[int, float]:
    """Return the number of waypoints and the duration of a straight path.

    Uses step_size, step_angle, speed and rot_speed from linear_motion in arm.json.

    Returns:
        A tuple (count, duration) with at least one waypoint and the duration in seconds.
    """
    motion = ARM_CONFIG["linear_motion"]
    distance = np.linalg.norm(target_pose[:3, 3] - start_pose[:3, 3])
    rot_angle = np.degrees(np.linalg.norm(rotation_error(target_pose[:3, :3], start_pose[:3, :3])))

    count = max(1, int(np.ceil(distance / motion["step_size"])), int(np.ceil(rot_angle / motion["step_angle"])))
    duration = max(distance / motion["speed"], rot_angle / motion["rot_speed"])
    return count, duration


def plan_linear(start_angles: list[float], target_pose: npt.NDArray[np.float64], mode: str = "full") -> tuple[list[list[float]], float] | None:
    """Plan a straight tool path from the pose at start_angles to a target pose.

    The position is interpolated linearly, the orientation rotates about a
    fixed axis at a constant rate. Each waypoint is solved with the inverse
    kinematics, starting from the previous waypoint. Settings are read from
    linear_motion in arm.json.

    Args:
        start_angles: Current servo angles in degrees.
        target_pose: Target tool pose with shape (4, 4), position in mm.
        mode: "full", "direction" or "position", see orientation_error.

    Returns:
        A tuple (path, step_time). path contains the servo angles in degrees
        for each waypoint, the last one reaches target_pose. step_time is the
        planned time between waypoints in seconds. Returns None if a waypoint
        is not reachable or the tool would leave the path by more than
        path_tolerance or path_angle_tolerance.
    """
    motion = ARM_CONFIG["linear_motion"]
    start_pose = forward_kinematics(start_angles)
    target_pose = adapt_target(start_pose, target_pose, mode)
    count, duration = path_timing(start_pose, target_pose)

    path = []
    previous = np.array(start_angles, dtype=float)

    for i in range(1, count + 1):
        result = inverse_kinematics(interpolate_pose(start_pose, target_pose, i / count), previous, mode=mode)
        if result is None:
            return None

        result = np.array(result)
        jump = np.abs(result - previous).max()

        # Near a singularity the IK may reorient the wrist by a large angle,
        # e.g. joints 5 and 7 turning against each other. Split such jumps into
        # small joint steps and accept them if the tool stays on the path.
        substeps = max(1, int(np.ceil(jump / motion["max_joint_step"])))
        last = previous

        for j in range(1, substeps + 1):
            fraction = (i - 1 + j / substeps) / count
            angles = previous + j / substeps * (result - previous)

            # Pull the interpolated angles back onto the path, unless the IK
            # jumps to another solution. The tool check below decides then.
            if j < substeps:
                corrected = inverse_kinematics(interpolate_pose(start_pose, target_pose, fraction), angles, mode=mode)
                if corrected is not None and np.abs(np.array(corrected) - last).max() <= 1.5 * motion["max_joint_step"]:
                    angles = np.array(corrected)

            # The servos move linearly between waypoints, check the tool in between.
            pos_dev, rot_dev = segment_deviation(last, angles, start_pose, target_pose, (i - 1 + (j - 1) / substeps) / count, fraction, mode)
            if pos_dev > motion["path_tolerance"] or rot_dev > motion["path_angle_tolerance"]:
                return None

            path.append(angles.tolist())
            last = angles

        previous = result

    return path, duration / count


def plan_near_linear(
    start_angles: list[float],
    target_pose: npt.NDArray[np.float64],
    max_angle_change: float = 360,
    mode: str = "full",
) -> tuple[list[list[float]], float, float, float] | None:
    """Plan a path that follows a straight tool line as closely as possible.

    Used when plan_linear fails. The target angles are chosen with solve_pose.
    Each waypoint starts from an even joint step toward these angles, then the
    inverse kinematics pulls the tool as close to the straight line as possible
    without large joint jumps. The last waypoint always reaches target_pose.

    Args:
        start_angles: Current servo angles in degrees.
        target_pose: Target tool pose with shape (4, 4), position in mm.
        max_angle_change: Largest allowed change of a single servo angle
            between start and target in degrees, see solve_pose.
        mode: "full", "direction" or "position", see orientation_error.

    Returns:
        A tuple (path, step_time, pos_dev, rot_dev). path and step_time as in
        plan_linear, pos_dev and rot_dev are the largest deviations from the
        straight line in mm and degrees. Returns None if target_pose is not reachable.
    """
    motion = ARM_CONFIG["linear_motion"]
    goal = solve_pose(target_pose, start_angles, max_angle_change=max_angle_change, mode=mode)
    if goal is None:
        return None

    start = np.array(start_angles, dtype=float)
    goal = np.array(goal)
    start_pose = forward_kinematics(start)
    target_pose = adapt_target(start_pose, target_pose, mode)

    count, duration = path_timing(start_pose, target_pose)
    count = max(count, int(np.ceil(np.abs(goal - start).max() / motion["max_joint_step"])))

    path = []
    previous = start
    pos_dev = 0.0
    rot_dev = 0.0

    for i in range(1, count + 1):
        remaining = count - i

        if remaining == 0:
            angles = goal
        else:
            # Even joint step toward the goal, then pull the tool toward the line.
            # A low rot_weight keeps the position closer to the line than the orientation.
            guess = previous + (goal - previous) / (remaining + 1)
            pose = interpolate_pose(start_pose, target_pose, i / count)
            angles = np.array(inverse_kinematics(pose, guess, max_iter=50, rot_weight=10.0, best_effort=True, mode=mode))

            # Keep the guess if the IK jumps or the goal could not be reached in time anymore.
            if (np.abs(angles - previous).max() > motion["max_joint_step"]
                    or np.abs(goal - angles).max() > remaining * motion["max_joint_step"]):
                angles = guess

        segment_pos, segment_rot = segment_deviation(previous, angles, start_pose, target_pose, (i - 1) / count, i / count, mode)
        pos_dev = max(pos_dev, segment_pos)
        rot_dev = max(rot_dev, segment_rot)

        path.append(angles.tolist())
        previous = angles

    return path, duration / count, pos_dev, rot_dev


def follow_path(pk, path: list[list[float]], step_time: float) -> list[float]:
    """Send planned waypoints to the servos one after another.

    All servos are scaled to reach each waypoint at the same time, no servo
    exceeds the configured speed and the load is checked while moving.

    Args:
        pk: Servo communication interface.
        path: Servo angles in degrees for each waypoint, see plan_linear.
        step_time: Planned time between waypoints in seconds.

    Returns:
        The servo angles in degrees of the last waypoint.

    Raises:
        RuntimeError: If a servo is overloaded during the motion. All servos are stopped.
    """
    steps = CONFIG["steps_per_rev"]
    previous = read_angles(pk)

    for angles in path:
        # Travel in steps, computed from angles to avoid wrap-around at 0 / steps_per_rev.
        travel = {servo_ID: abs(angle - last) / 360 * steps for servo_ID, angle, last in zip(SERVOS, angles, previous)}

        # Slow down if a servo would exceed the configured speed.
        duration = max(step_time, max(travel.values()) / CONFIG["speed"])
        speeds = {servo_ID: max(1, int(np.ceil(dist / duration))) for servo_ID, dist in travel.items()}

        sync_write_pos(pk, angles_to_steps(angles), speeds)

        end = time.time() + duration
        while True:
            for servo_ID in SERVOS:
                check_load(pk, servo_ID, moving=True)
            if time.time() >= end:
                break

        previous = angles

    # Settle at the target with the usual completion and load check.
    move_to_angles(pk, path[-1])

    return path[-1]


def move_linear(pk, target_pose: npt.NDArray[np.float64], mode: str = "full") -> list[float] | None:
    """Move the tool along a straight line to a target pose.

    Plans the whole path before moving. If the exact line is not feasible,
    the path follows the line as closely as possible, see plan_near_linear.
    The arm does not move if the target is not reachable.

    Args:
        pk: Servo communication interface.
        target_pose: Target tool pose with shape (4, 4), position in mm.
        mode: "full", "direction" or "position", see orientation_error.

    Returns:
        The servo angles in degrees at the target, or None if the target is not reachable.

    Raises:
        RuntimeError: If a servo is overloaded during the motion. All servos are stopped.
    """
    start = read_angles(pk)
    plan = plan_linear(start, target_pose, mode)

    if plan is None:
        near = plan_near_linear(start, target_pose, mode=mode)
        if near is None:
            print("Pose nicht erreichbar; Arm bleibt stehen.")
            return None

        path, step_time, pos_dev, rot_dev = near
        print(f"Gerade Linie nicht exakt fahrbar; Abweichung bis {pos_dev:.0f} mm / {rot_dev:.0f} Grad.")
        plan = path, step_time

    return follow_path(pk, *plan)
