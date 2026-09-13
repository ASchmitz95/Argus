import sys
from pathlib import Path

import pytest
from scservo_sdk import sms_sts

# Modules in src/ import each other as motor.* and arm.*.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from motor.motor_control import angles_to_steps


class FakeSyncWrite:
    """Collects synchronized writes like GroupSyncWrite."""

    def __init__(self, pk):
        self.pk = pk
        self.params = {}

    def txPacket(self):
        self.pk.sync_packets.append(dict(self.params))
        for servo_ID, (pos, _) in self.params.items():
            self.pk.pos[servo_ID] = pos

    def clearParam(self):
        self.params = {}


class FakeServos:
    """Servo interface without hardware. Servos reach their goal immediately.

    Args:
        angles: Initial servo angles in degrees in the order of SERVOS.
        load: Raw load register value returned for every servo.
        moving: Moving flag returned for every servo.
        reach: If False, written positions are ignored, so targets are never reached.
    """

    scs_tohost = sms_sts.scs_tohost

    def __init__(self, angles, load=0, moving=0, reach=True):
        self.pos = angles_to_steps(angles)
        self.load = load
        self.moving = moving
        self.reach = reach
        self.writes = []
        self.sync_packets = []
        self.groupSyncWrite = FakeSyncWrite(self)

    def WritePosEx(self, servo_ID, pos, speed, acc):
        self.writes.append((servo_ID, pos, speed, acc))
        if self.reach:
            self.pos[servo_ID] = pos

    def SyncWritePosEx(self, servo_ID, pos, speed, acc):
        self.groupSyncWrite.params[servo_ID] = (pos, speed)

    def ReadPosSpeed(self, servo_ID):
        return self.pos[servo_ID], 0, 0, 0

    def ReadMoving(self, servo_ID):
        return self.moving, 0, 0

    def read2ByteTxRx(self, servo_ID, address):
        return self.load, 0, 0


@pytest.fixture
def fake_servos():
    """Factory for FakeServos, see its arguments."""
    return FakeServos
