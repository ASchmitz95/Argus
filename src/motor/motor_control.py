from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
import serial.tools.list_ports
import sys

STEPS_PER_REV = 4096
BAUD      = 1_000_000
SPEED     = 2400
ACC       = 200
TOLERANZ  = 6
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