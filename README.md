# Argus

Steuerung für einen mobilen Roboter mit 6-DoF-Roboterarm.

Der Arm sitzt auf einer Plattform mit zwei Rädern. Der Roboter fährt zu einer Position, zieht die Räder nach oben ein und steht dann fest, damit der Arm stabil arbeiten kann.

**Aktueller Stand:** Der 6-DoF-Arm ist gebaut und lässt sich über Gelenkwinkel, Zielposen und gerade Bahnen steuern. Fahrwerk, Radeinfahrmechanik und Greifer folgen später.

## Funktionen

- Servo-Ansteuerung über den seriellen Bus mit Winkelgrenzen pro Gelenk
- Lastüberwachung: Der Arm stoppt bei Überlast, z. B. wenn er gegen ein Hindernis fährt
- Vorwärts- und inverse Kinematik (Damped Least Squares)
- IK-Modi: volle Pose, nur Greiferrichtung oder nur Position
- Gerade Bahnen des Greifers mit synchronisierten Servos, bei Bedarf angenähert
- Farb- und Objekterkennung mit OpenCV
- Tests ohne Hardware

## Hardware

| Komponente | Details |
|---|---|
| Servos | 6× Feetech STS3215, IDs 2–7, serieller Bus mit 1 Mbit/s |
| Gelenke | 2: Z (Basis), 3: Y (Schulter), 4: Y (Ellbogen), 5: X, 6: Y, 7: X (Handgelenk) |
| Handgelenk | sphärisch, die Achsen von 5, 6 und 7 schneiden sich in einem Punkt |
| Kamera | aktuell Webcam über OpenCV, eigene Kamera für den Arm folgt |
| Rechner | aktuell Windows-PC per USB, später Raspberry Pi o. ä. |

## Projektstruktur

```
config/
  motor.json            Serielle Verbindung, Geschwindigkeit, Lastgrenzen, Servos
  arm.json              Kinematikmodell und Einstellungen für gerade Bahnen
  camera.json           Kamera und Farberkennung
src/
  motor/motor_control.py  Verbindung, Positionen lesen, Arm bewegen, Lastüberwachung
  arm/kinematics.py       Vorwärtskinematik, Jacobi-Matrix, inverse Kinematik
  arm/arm_control.py      Zielposen und gerade Bahnen anfahren
  cam/img_processing.py   Farb- und Objekterkennung
  cam/img_draw.py         Bounding Boxes und Mittelpunkte zeichnen
tests/                  Tests ohne Hardware
```

## Installation

Voraussetzung ist Python 3.14.

```bash
pip install -r requirements.txt
```

## Konfiguration

Alle Einstellungen liegen als JSON in `config/`. Werte für echte Hardware (Nullpositionen, Winkelgrenzen, Geschwindigkeit, Lastgrenzen) nur bewusst ändern.

### `motor.json`

| Wert | Bedeutung |
|---|---|
| `baud` | Baudrate des Servo-Busses |
| `speed`, `acc` | Geschwindigkeit in Steps/s und Beschleunigung der Servos |
| `tolerance` | erlaubte Abweichung vom Ziel in Steps |
| `max_load_moving`, `max_load_holding` | Lastgrenzen in 0,1 % des Maximalmoments beim Fahren bzw. Halten |
| `servos` | pro Servo-ID `null_pos` (Nullstellung in Steps) und `min_angle`/`max_angle` in Grad |

Die Lastgrenzen hängen von `speed` und `acc` ab und müssen nach einer Änderung neu gemessen werden.

### `arm.json`

Das Kinematikmodell beschreibt jedes Gelenk in der Nullstellung (alle Winkel 0):

- `axis`: Drehachse `x`, `y` oder `z`
- `position`: ein Punkt auf der Drehachse in mm
- `direction`: `1`, wenn ein positiver Servowinkel nach der Rechte-Hand-Regel um die Achse dreht, sonst `-1`
- `tool`: Position und Ausrichtung (`rpy` in Grad) der Greiferspitze

**Koordinatensystem:** rechtshändig, Ursprung im Mittelpunkt des Fußes. z zeigt nach oben, x in Richtung des Arms in der Nullstellung, y nach links.

`linear_motion` enthält Bahngeschwindigkeit (`speed` in mm/s, `rot_speed` in Grad/s), Schrittweite der Zwischenpunkte und die erlaubte Abweichung von der Bahn.

## Verwendung

Module in `src/` importieren sich gegenseitig als `motor.*` und `arm.*`, `src/` muss daher im Python-Pfad liegen.

```python
from motor.motor_control import connect, move_to_angles
from arm.kinematics import pose_from_rpy
from arm.arm_control import move_to_pose, move_linear

ph, pk = connect()

# Gelenkwinkel in Grad relativ zur Nullstellung, Reihenfolge der Servo-IDs 2–7
move_to_angles(pk, [0, 20, 20, 0, 0, 0], timeout=3)

# Greiferspitze nur positionieren, Ausrichtung frei
move_to_pose(pk, pose_from_rpy(150, 150, 150, 0, 0, 0), mode="position")

# Auf gerader Linie dorthin, Greifer zeigt nach unten (Pitch 90), Drehung um die Greiferachse frei
move_linear(pk, pose_from_rpy(120, 0, 100, 0, 90, 0), mode="direction")

ph.closePort()
```

### IK-Modi

| Modus | Vorgabe | Einsatz |
|---|---|---|
| `full` | Position und komplette Ausrichtung | Werkzeug exakt ausrichten |
| `direction` | Position und Richtung der Greiferachse | Greifen, z. B. von oben |
| `position` | nur Position | größte Reichweite |

Das Handgelenk sitzt 100 mm hinter der Greiferspitze. Mit `full` oder `direction` sind deshalb deutlich weniger Positionen erreichbar als mit `position`.

### Bewegungsfunktionen

| Funktion | Beschreibung |
|---|---|
| `move_to_angles(pk, angles, timeout)` | Gelenkwinkel anfahren, gibt `True` bei erreichtem Ziel und `False` bei Timeout zurück |
| `move_to_pose(pk, pose, mode=...)` | Zielpose über die Gelenke anfahren, mit möglichst wenig Gelenkbewegung |
| `move_linear(pk, pose, mode=...)` | Greiferspitze auf gerader Linie bewegen; ist die Linie nicht exakt fahrbar, wird sie angenähert |

Ist ein Ziel nicht erreichbar, bewegt sich der Arm nicht und die Funktion gibt `None` zurück. Bei Überlast halten alle Servos an und es wird ein `RuntimeError` ausgelöst.

## Tests

Die Tests laufen ohne Hardware mit simulierten Servos:

```bash
python -m pytest tests
```

## Sicherheit

- Der Code bewegt echte Hardware. Neue Bewegungen zuerst langsam und mit kleinen Wegen testen.
- Die Winkelgrenzen berücksichtigen noch nicht, wie sich die Gelenke gegenseitig einschränken. Kollisionen mit dem eigenen Arm oder der Umgebung sind möglich.
- Die Lastüberwachung ist nur aktiv, während eine Bewegungsfunktion läuft.

## Roadmap

- [ ] Kalibrierung am überarbeiteten Arm
- [ ] Kamera am Arm: Kamerakalibrierung, Hand-Auge-Kalibrierung, Objekte in mm anfahren
- [ ] Greifer
- [ ] Kollisionsprüfung über das Geometriemodell
- [ ] Fahrwerk mit einfahrbaren Rädern
- [ ] Umstieg auf Raspberry Pi
