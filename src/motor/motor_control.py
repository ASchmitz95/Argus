from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS, SMS_STS_PRESENT_LOAD_L
import serial.tools.list_ports
import json
import sys
import time
from pathlib import Path

CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "motor.json"

with open(CONFIG_PATH, encoding="utf-8") as f:
    CONFIG = json.load(f)

# JSON keys are strings, servo IDs are used as integers.
SERVOS = {int(servo_ID): servo for servo_ID, servo in CONFIG["servos"].items()}


def read_pos(pk, servo_ID, degree=False):
    """Read a servo's current position in steps or degrees.

    Args:
        pk: Servo communication interface.
        servo_ID: ID of the servo to read.
        degree: If True, convert the position to degrees relative to the
            servo's calibrated zero position. Defaults to False.

    Returns:
        The absolute position in steps, or the signed angular offset
        from the servo's null_pos in degrees if degree is True.
    """
    pos, _, _, _ = pk.ReadPosSpeed(servo_ID)
    if degree:
        pos = ((pos - SERVOS[servo_ID]["null_pos"]) / CONFIG["steps_per_rev"]) * 360
        return pos
    return pos


def read_load(pk, servo_ID):
    """Read a servo's current load.

    Args:
        pk: Servo communication interface.
        servo_ID: ID of the servo to read.

    Returns:
        Signed load in 0.1 % of the maximum torque (-1000 to 1000).
        The sign indicates the direction of the load.
    """
    load, _, _ = pk.read2ByteTxRx(servo_ID, SMS_STS_PRESENT_LOAD_L)
    # Bit 10 encodes the direction.
    return pk.scs_tohost(load, 10)


def stop(pk):
    """Hold all configured servos at their current positions.

    Torque stays enabled, so the arm does not fall under gravity.
    """
    for servo_ID in SERVOS:
        pk.WritePosEx(servo_ID, read_pos(pk, servo_ID), CONFIG["speed"], CONFIG["acc"])


def connect():
    """Open the first serial port that supports the configured baud rate.

    Tries each detected serial port until opening it and setting the baud
    rate succeeds. This does not verify that a servo is connected or responding.

    Returns:
        A tuple containing the open PortHandler and its sms_sts interface.

    Raises:
        SystemExit: If no serial ports are detected or none can be opened
            and configured successfully.
    """
    ports = [p.device for p in serial.tools.list_ports.comports()]

    if not ports:
        sys.exit("Kein serieller Port gefunden.")

    print("Gefundene Ports:", ", ".join(ports))

    for dev in ports:
        ph = PortHandler(dev)

        if not ph.openPort():
            continue

        if not ph.setBaudRate(CONFIG["baud"]):
            ph.closePort()
            continue

        print(f"Verbunden mit {dev}")
        return ph, sms_sts(ph)

    sys.exit("Keine Verbindung möglich.")


def check_collision(angles):
    """Clamp target angles for all configured servos to their limits."""
    angles = [min(max(angle, servo["min_angle"]), servo["max_angle"]) for angle, servo in zip(angles, SERVOS.values(), strict=True)]
    return angles


def angles_to_steps(angles):
    """Convert servo angles in degrees relative to the zero positions to
    absolute positions in steps.

    Returns:
        Dict mapping each servo ID to its position in steps.
    """
    steps = CONFIG["steps_per_rev"]
    return {servo_ID: (servo["null_pos"] + int(angle / 360 * steps)) % steps for (servo_ID, servo), angle in zip(SERVOS.items(), angles, strict=True)}


def check_load(pk, servo_ID, moving):
    """Stop all servos and raise if a servo is overloaded.

    Args:
        pk: Servo communication interface.
        servo_ID: ID of the servo to check.
        moving: Whether the servo is moving. Moving servos need more load
            than holding ones, so max_load_moving or max_load_holding is used.

    Raises:
        RuntimeError: If the load exceeds the limit. All servos are stopped before raising.
    """
    load = read_load(pk, servo_ID)
    max_load = CONFIG["max_load_moving"] if moving else CONFIG["max_load_holding"]
    if abs(load) > max_load:
        stop(pk)
        raise RuntimeError(f"Überlast an Servo {servo_ID} ({load}); Arm gestoppt.")


def sync_write_pos(pk, positions, speeds):
    """Send goal positions and speeds to all given servos in one packet.

    Args:
        pk: Servo communication interface.
        positions: Dict mapping servo IDs to positions in steps.
        speeds: Dict mapping servo IDs to speeds in steps per second.
    """
    for servo_ID, pos in positions.items():
        pk.SyncWritePosEx(servo_ID, pos, speeds[servo_ID], CONFIG["acc"])
    pk.groupSyncWrite.txPacket()
    pk.groupSyncWrite.clearParam()


def move_to_angles(pk, angles, timeout=1):
    """Move all configured servos to angles in degrees relative to their
    calibrated zero positions, waiting until motion completes or the timeout expires.

    Returns:
        True if all servos reached their target within the tolerance,
        False if the timeout expired first. The servos keep moving toward
        the target in that case.

    Raises:
        RuntimeError: If a servo's load exceeds max_load_moving while it moves
            or max_load_holding while it holds its position.
            All servos are stopped before raising.
    """
    angles = check_collision(angles)
    target_pos = angles_to_steps(angles)
    start = time.time()
    for servo_ID, pos in target_pos.items():
        pk.WritePosEx(servo_ID, pos, CONFIG["speed"], CONFIG["acc"])
    while time.time() - start <= timeout:
        flag = True
        for servo_ID, pos in target_pos.items():
            moving, _, _ = pk.ReadMoving(servo_ID)
            # Stop on overload, e.g. when the arm hits an obstacle.
            check_load(pk, servo_ID, moving)
            if abs(read_pos(pk, servo_ID) - pos) > CONFIG["tolerance"]:
                flag = False
            if moving != 0:
                flag = False
        if flag:
            return True
    return False


def move_by_angles(pk, angle_offsets):
    """Move all configured servos by the given signed offsets in degrees
    relative to their current positions.

    Returns:
        True if the target was reached, False on timeout, see move_to_angles.
    """
    current_pos_List = [read_pos(pk, servo_ID, degree=True) for servo_ID in SERVOS]
    target_pos_List = [current + offset for current, offset in zip(current_pos_List, angle_offsets, strict=True)]
    return move_to_angles(pk, target_pos_List)


def go_home(pk):
    """Returns all servoes to their Null Position.

    Returns:
        True if the zero positions were reached, False on timeout, see move_to_angles.
    """
    return move_to_angles(pk, [0] * len(SERVOS))
