from scservo_sdk import COMM_SUCCESS, SMS_STS_ID
import sys
from motor_control import connect

# Valid servo IDs; 254 is the broadcast ID.
MAX_ID = 253


def main():
    ph, pk = connect()

    # Ping every ID to make sure exactly one servo is connected.
    print("Suche Servos ...")
    found = [servo_ID for servo_ID in range(MAX_ID + 1) if pk.ping(servo_ID)[1] == COMM_SUCCESS]
    if len(found) != 1:
        ph.closePort()
        sys.exit(f"Genau ein Servo muss angeschlossen sein, gefunden: {len(found)}.")
    old_ID = found[0]
    print(f"Gefundener Servo hat ID {old_ID}.")

    new_ID = input(f"Neue ID (0-{MAX_ID}): ")
    if not new_ID.isdigit() or int(new_ID) > MAX_ID:
        ph.closePort()
        sys.exit("Ungültige ID.")
    new_ID = int(new_ID)

    # The ID is stored in the EEPROM, which must be unlocked for writing.
    pk.unLockEprom(old_ID)
    pk.write1ByteTxRx(old_ID, SMS_STS_ID, new_ID)
    pk.LockEprom(new_ID)
    ph.closePort()

    print(f"ID {old_ID} -> {new_ID} geändert.")


if __name__ == "__main__":
    main()
