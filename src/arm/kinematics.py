import numpy as np
import numpy.typing as npt
import json
from pathlib import Path
from motor.motor_control import SERVOS, check_collision

CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "arm.json"

with open(CONFIG_PATH, encoding="utf-8") as f:
    CONFIG = json.load(f)

AXES = {"x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0], "z": [0.0, 0.0, 1.0]}

JOINT_IDS = [joint["servo_id"] for joint in CONFIG["joints"]]
if JOINT_IDS != list(SERVOS):
    raise ValueError(f"Servo-IDs in arm.json {JOINT_IDS} passen nicht zu motor.json {list(SERVOS)}.")

# Joint axes and points on the axes in the zero pose, in the order of SERVOS.
JOINT_AXES = np.array([AXES[joint["axis"]] for joint in CONFIG["joints"]])
JOINT_POINTS = np.array([joint["position"] for joint in CONFIG["joints"]], dtype=float)
JOINT_DIRECTIONS = np.array([joint["direction"] for joint in CONFIG["joints"]], dtype=float)


def rotation(axis: npt.ArrayLike, angle: float) -> npt.NDArray[np.float64]:
    """Return the 3x3 rotation matrix for a rotation about a unit axis.

    Args:
        axis: Unit vector (x, y, z) of the rotation axis.
        angle: Rotation angle in radians, positive by the right-hand rule.

    Returns:
        Rotation matrix with shape (3, 3).
    """
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)
    # Rodrigues' rotation formula.
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def pose_from_rpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> npt.NDArray[np.float64]:
    """Return a pose from a position and roll, pitch and yaw angles.

    Args:
        x, y, z: Position in mm relative to the base origin.
        roll, pitch, yaw: Rotation in degrees about the fixed x, y and z axes,
            applied in this order.

    Returns:
        Homogeneous transformation matrix with shape (4, 4).
    """
    roll, pitch, yaw = np.radians([roll, pitch, yaw])
    pose = np.eye(4)
    pose[:3, :3] = rotation(AXES["z"], yaw) @ rotation(AXES["y"], pitch) @ rotation(AXES["x"], roll)
    pose[:3, 3] = [x, y, z]
    return pose


def joint_axes(angles: npt.ArrayLike) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64], npt.NDArray[np.float64]]:
    """Compute the joint axes and the tool pose for the given servo angles.

    Args:
        angles: Servo angles in degrees relative to the zero pose,
            one per servo in the order of SERVOS.

    Returns:
        A tuple (points, axes, tool_pose). points and axes have shape (N, 3)
        and contain a point on each joint axis and its unit direction in mm
        relative to the base origin. tool_pose has shape (4, 4).

    Raises:
        ValueError: If the number of angles does not match the number of joints.
    """
    angles = np.asarray(angles, dtype=float)
    if angles.shape != (len(JOINT_AXES),):
        raise ValueError(f"{len(JOINT_AXES)} Winkel erwartet, {angles.size} erhalten.")

    # Servo angles may turn against the right-hand rule of their axis.
    angles = np.radians(angles) * JOINT_DIRECTIONS

    transform = np.eye(4)
    points = np.empty_like(JOINT_POINTS)
    axes = np.empty_like(JOINT_AXES)

    for i, (axis, point, angle) in enumerate(zip(JOINT_AXES, JOINT_POINTS, angles)):
        # Each axis is moved by all joints before it.
        points[i] = transform[:3, :3] @ point + transform[:3, 3]
        axes[i] = transform[:3, :3] @ axis

        joint = np.eye(4)
        joint[:3, :3] = rotation(axis, angle)
        joint[:3, 3] = point - joint[:3, :3] @ point
        transform = transform @ joint

    tool = pose_from_rpy(*CONFIG["tool"]["position"], *CONFIG["tool"]["rpy"])
    return points, axes, transform @ tool


def forward_kinematics(angles: npt.ArrayLike) -> npt.NDArray[np.float64]:
    """Return the tool pose for the given servo angles in degrees.

    Returns:
        Homogeneous transformation matrix with shape (4, 4),
        position in mm relative to the base origin.
    """
    return joint_axes(angles)[2]


def jacobian(angles: npt.ArrayLike) -> npt.NDArray[np.float64]:
    """Return the geometric Jacobian of the tool for the given servo angles.

    Args:
        angles: Servo angles in degrees, one per servo in the order of SERVOS.

    Returns:
        Array with shape (6, N). Rows 0-2 contain the linear velocity in mm,
        rows 3-5 the angular velocity, each per radian of servo angle.
    """
    points, axes, pose = joint_axes(angles)
    tool_pos = pose[:3, 3]

    jac = np.empty((6, len(axes)))
    jac[:3] = np.cross(axes, tool_pos - points).T
    jac[3:] = axes.T

    return jac * JOINT_DIRECTIONS


def rotation_error(target: npt.NDArray[np.float64], current: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
    """Return the rotation from current to target as a rotation vector.

    Args:
        target: Target rotation matrix with shape (3, 3).
        current: Current rotation matrix with shape (3, 3).

    Returns:
        Rotation vector with shape (3,). Its direction is the rotation axis,
        its length the rotation angle in radians.
    """
    r = target @ current.T
    angle = np.arccos(np.clip((np.trace(r) - 1) / 2, -1, 1))
    vee = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])

    if angle < 1e-6:
        return vee / 2

    if np.pi - angle < 1e-3:
        # sin(angle) is close to zero, recover the axis from r + I = 2 * axis * axis^T.
        sym = r + np.eye(3)
        axis = sym[:, np.linalg.norm(sym, axis=0).argmax()]
        return angle * axis / np.linalg.norm(axis)

    return angle / (2 * np.sin(angle)) * vee


def orientation_error(target: npt.NDArray[np.float64], current: npt.NDArray[np.float64], mode: str = "full") -> npt.NDArray[np.float64]:
    """Return the orientation error that matters for the given IK mode.

    Args:
        target: Target rotation matrix with shape (3, 3).
        current: Current rotation matrix with shape (3, 3).
        mode: "full" for the complete orientation, "direction" for the
            direction of the tool x-axis only, "position" to ignore the orientation.

    Returns:
        Rotation vector with shape (3,), length in radians, see rotation_error.

    Raises:
        ValueError: If mode is unknown.
    """
    if mode == "full":
        return rotation_error(target, current)

    if mode == "direction":
        # Shortest rotation that turns the current tool x-axis into the target one.
        current_axis, target_axis = current[:, 0], target[:, 0]
        cross = np.cross(current_axis, target_axis)
        angle = np.arctan2(np.linalg.norm(cross), current_axis @ target_axis)
        if np.linalg.norm(cross) < 1e-9:
            if angle < np.pi / 2:
                return np.zeros(3)
            # Opposite direction, turn about any axis perpendicular to the tool axis.
            cross = np.cross(current_axis, AXES["z"] if abs(current_axis[2]) < 0.9 else AXES["y"])
        return angle * cross / np.linalg.norm(cross)

    if mode == "position":
        return np.zeros(3)

    raise ValueError(f"Unbekannter IK-Modus '{mode}', erlaubt: full, direction, position.")


def inverse_kinematics(
    target_pose: npt.NDArray[np.float64],
    start_angles: npt.ArrayLike,
    max_iter: int = 300,
    pos_tol: float = 0.5,
    rot_tol: float = 0.5,
    damping: float = 5.0,
    max_step: float = 10.0,
    rot_weight: float = 100.0,
    best_effort: bool = False,
    mode: str = "full",
) -> list[float] | None:
    """Find servo angles that move the tool to a target pose.

    Uses damped least squares, starting from start_angles. After each step
    the angles are clamped to the servo limits with check_collision.
    Depending on mode, the orientation is matched completely, only the
    direction of the tool x-axis, or not at all.

    Args:
        target_pose: Target tool pose with shape (4, 4), position in mm.
        start_angles: Initial servo angles in degrees, usually the current pose.
        max_iter: Maximum number of iterations.
        pos_tol: Allowed position error in mm.
        rot_tol: Allowed orientation error in degrees.
        damping: Damping factor in mm. Larger values are more stable near
            singularities but converge slower.
        max_step: Maximum change of a single servo angle per iteration in degrees.
        rot_weight: Length in mm that one radian of orientation error counts as.
        best_effort: If True, return the angles closest to the target instead
            of None when the tolerances are not reached.
        mode: "full", "direction" or "position", see orientation_error.

    Returns:
        Servo angles in degrees in the order of SERVOS, or None if the target
        is not reached within the tolerances and best_effort is False.

    Raises:
        ValueError: If mode is unknown.
    """
    angles = np.array(check_collision(list(start_angles)), dtype=float)
    best_angles = angles
    best_error = np.inf

    for _ in range(max_iter):
        pose = forward_kinematics(angles)
        pos_error = target_pose[:3, 3] - pose[:3, 3]
        rot_error = orientation_error(target_pose[:3, :3], pose[:3, :3], mode)

        if np.linalg.norm(pos_error) <= pos_tol and np.degrees(np.linalg.norm(rot_error)) <= rot_tol:
            return angles.tolist()

        # Weight orientation so that mm and radians are comparable.
        error = np.concatenate([pos_error, rot_weight * rot_error])
        if np.linalg.norm(error) < best_error:
            best_angles = angles
            best_error = np.linalg.norm(error)
        jac = jacobian(angles)
        jac[3:] *= rot_weight

        # Rotation about the tool x-axis does not change its direction, so it is free.
        if mode == "direction":
            tool_axis = pose[:3, 0]
            jac[3:] = (np.eye(3) - np.outer(tool_axis, tool_axis)) @ jac[3:]
        elif mode == "position":
            jac[3:] = 0

        step = jac.T @ np.linalg.solve(jac @ jac.T + damping**2 * np.eye(6), error)
        step = np.degrees(step)

        biggest = np.abs(step).max()
        if biggest > max_step:
            step *= max_step / biggest

        angles = np.array(check_collision((angles + step).tolist()))

    if best_effort:
        return best_angles.tolist()
    return None
