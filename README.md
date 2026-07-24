# barebonesblimp

Minimal, self-contained control stack for a thrust-vectoring bicopter blimp on
XIAO ESP32-S3 boards. Keyboard ground station on the laptop, ESP-NOW link
through a USB transceiver, PID + open-loop manual control on the blimp.

Nothing here depends on the larger BlimpSwarm / SwarmBase libraries — the three
firmware sketches are standalone.

## Layout

```
barebonesblimp/
  BareBonesControl.py          keyboard ground station (main program)
  CheckLink.py                 link diagnostic (optional)
  user_parameters.py           blimp MAC + serial port config
  comm/
    __init__.py
    Serial.py                  SerialController: packs & sends the packet
  firmware/
    BareBonesDiff/             blimp flight firmware  -> flash to blimp XIAO
    BaseTranseiver/            USB<->ESP-NOW bridge    -> flash to transceiver XIAO
    ServoSweepTest/            servo isolation test    -> diagnostic only
```

## Hardware (blimp XIAO ESP32-S3)

| Function | Pin |
|----------|-----|
| Motor 1 (ESC signal) | D9  |
| Motor 2 (ESC signal) | D10 |
| Tilt servo (signal)  | D1  |
| I2C SDA (BNO085 + BMP390) | D4 |
| I2C SCL (BNO085 + BMP390) | D5 |

Motors/servo are powered from the LiPo (via ESC / BEC); the XIAO runs off USB.
The transceiver is a second XIAO on USB with `BaseTranseiver` flashed.

## One-time setup

**Arduino libraries** (Library Manager):
- ESP32Servo
- SparkFun BNO08x Arduino Library
- Adafruit BMP3XX Library (pulls in Adafruit BusIO + Unified Sensor)
- esp32 board package (select board: XIAO_ESP32S3)

**Python** (3.10+):
```
pip install pyserial pygame
```

**Config** — edit for your setup:
- `user_parameters.py` -> `robot_macs[0]` = the blimp's MAC (printed on its
  serial monitor at boot: "Blimp MAC: ..."), and `SERIAL_PORT` = the
  transceiver's USB port.
- `BareBonesControl.py` -> `EXPECTED_BLIMP_MAC` = same blimp MAC. The program
  refuses to start unless this matches `robot_macs[0]`, as a guard against
  sending to the wrong board.

## Run

1. Props OFF. Connect the LiPo, then plug in the blimp USB, then the transceiver USB.
2. Flash `firmware/BareBonesDiff` to the blimp, `firmware/BaseTranseiver` to the
   transceiver.
3. Open the blimp's Serial Monitor @ 115200; press RESET; confirm the "Blimp MAC"
   line matches your config. Close any monitor on the transceiver port.
4. `python BareBonesControl.py`
5. Click the pygame window, then SPACE to arm.

## Controls

| Key | PID / auto mode | Manual / bench mode |
|-----|-----------------|---------------------|
| SPACE | arm / disarm | arm / disarm |
| M   | switch to manual | switch to PID |
| W   | forward thrust | tilt servo forward (held) |
| A / D | yaw | yaw (motor split) |
| Q / E | height up / down | throttle up / down (latched) |
| J / L | — | jog servo angle (calibration) |
| ESC | quit | quit |

## Telemetry

The blimp prints CSV over serial, header repeated every 20 lines:

```
# armed,mode,fx,dyaw,yawSP,yaw,tauz,zSP,alt,fz/thr,theta,f1,f2,servo
```

`mode` is `P` (PID) or `M` (manual). `f1`/`f2` are the motor commands (0..1),
`servo` is the commanded servo angle in degrees.

## Servo calibration (manual mode, props off, disarmed)

1. Press M (manual). Do not arm.
2. Jog J/L until the props point straight UP. Record the servo value = U.
3. Jog J/L until the props point straight FORWARD. Record the value = F.
4. In `firmware/BareBonesDiff`, set `SERVO_CENTER_DEG = F` and
   `SERVO_SCALE = (U - F) / 90`, reflash.

## Diagnostics

- `CheckLink.py` — sends test packets straight to the transceiver and prints its
  replies; use to prove the radio link when the blimp shows `armed=0`.
- `firmware/ServoSweepTest` — sweeps the servo with no motors or radio, to prove
  the servo/pin/power in isolation.
