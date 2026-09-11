from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
import serial.tools.list_ports
import sys
import time
from motor_control import *

ph, pk = connect()

for servo_ID in range(2, 8):
    pk.write1ByteTxRx(servo_ID, 40, 1)

def test():
    move_to_angles(pk, [0,20,60,0,40,0])
    move_by_angles(pk, (20,20,20,20,20,20))
    move_by_angles(pk, (-40,-40,-40,-40,-40,0))
    move_by_angles(pk, (20,20,20,20,20,-20))


if __name__ == "__main__":
    test()