from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
import serial.tools.list_ports
import sys
import time

STEPS_PER_REV = 4096
BAUD      = 1_000_000
SPEED     = 2400
ACC       = 200
TOLERANZ  = 8
NULL_POS = {2 : 2051, 3 : 1050, 4 : 900, 5 : 2112, 6 : 2240, 7 : 1981}


def read_pos(pk, servo_ID, degree=False):
    """Read a servo's current position in steps or degrees.

    Args:
        pk: Servo communication interface.
        servo_ID: ID of the servo to read.
        degree: If True, convert the position to degrees relative to the
            servo's calibrated zero position. Defaults to False.

    Returns:
        The absolute position in steps, or the signed angular offset
        from NULL_POS[servo_ID] in degrees if degree is True.
    """
    pos, _, _, _ = pk.ReadPosSpeed(servo_ID)
    if degree:
        pos = ((pos - NULL_POS[servo_ID]) / STEPS_PER_REV) * 360
        return pos
    return pos


def connect():
    """Open the first serial port that supports the configured baud rate.

    Tries each detected serial port until opening it and setting BAUD
    succeeds. This does not verify that a servo is connected or responding.

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

        if not ph.setBaudRate(BAUD):
            ph.closePort()
            continue

        print(f"Verbunden mit {dev}")
        return ph, sms_sts(ph)

    sys.exit("Keine Verbindung möglich.")


def move_to_angles(pk, angles, timeout=1):
    """Move servos 2-7 to angles in degrees relative to their calibrated
    zero positions, waiting until motion completes or the timeout expires.
    """
    target_pos_List = [(NULL_POS[i] + int(angles[i-2] / 360 * STEPS_PER_REV)) % STEPS_PER_REV for i in range(2,8)]
    start = time.time()
    for i in range(2,8):
        pk.WritePosEx(i, target_pos_List[i-2], SPEED, ACC)
    while time.time() - start <= timeout:
        flag = True
        moving, _, _ = pk.ReadMoving(i)
        for i in range(2,8):
            if abs(read_pos(pk, i) - target_pos_List[i-2]) > TOLERANZ:
                flag = False
            if moving != 0:
                flag = False
        if flag:
            return
    #raise TimeoutError("Ziel nicht rechtzeitig erreicht; Haltebefehle gesendet.")


def move_by_angles(pk, angle_offsets):
    """Move servos 2-7 by the given signed offsets in degrees
    relative to their current positions.
    """
    current_pos_List = [(read_pos(pk, i, degree=True)) for i in range(2,8)]
    target_pos_List = [current_pos_List[i] + angle_offsets[i] for i in range(len(angle_offsets))]
    move_to_angles(pk, target_pos_List)


def go_home():
    """Returns all servoes to their Null Position.
    """
    move_to_angles([0,0,0,0,0,0])