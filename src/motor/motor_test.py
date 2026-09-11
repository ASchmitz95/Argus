from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
import serial.tools.list_ports
import sys
import time
from motor_control import *

ph, pk = connect()

for servo_ID in range(2, 8):
    pk.write1ByteTxRx(servo_ID, 40, 1)

print(read_pos(pk, 3, degree=True))