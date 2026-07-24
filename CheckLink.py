"""
CheckLink.py
---------------------------------------------------------------------------
Diagnostic for the base transceiver. Talks to it directly with pyserial and
prints EVERY byte it sends back -- unlike SerialController, which swallows
the replies inside wait_for_acknowledgement().

Answers three questions:
    1. Which serial ports exist, and is SERIAL_PORT one of them?
    2. Does the transceiver acknowledge adding the blimp as a peer?
    3. Does it acknowledge forwarding control packets?

Run with the blimp powered on and its own serial monitor open, so you can
watch both ends at once.
---------------------------------------------------------------------------
"""

import struct
import time

import serial
import serial.tools.list_ports

from user_parameters import ROBOT_MACS, SERIAL_PORT

TARGET_MAC = ROBOT_MACS[0]
N_PACKETS = 20


def list_ports():
    print("=" * 62)
    print("SERIAL PORTS")
    print("=" * 62)
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  (none found)")
    for p in ports:
        marker = "  <-- SERIAL_PORT" if p.device == SERIAL_PORT else ""
        print(f"  {p.device:<28} {p.description}{marker}")
    print()
    if SERIAL_PORT not in [p.device for p in ports]:
        print(f"  !! {SERIAL_PORT} is NOT in the list above.")
        print("     Fix SERIAL_PORT in user_parameters.py before continuing.\n")
        return False
    if len(ports) > 1:
        print("  NOTE: more than one port present. SERIAL_PORT must be the")
        print("        TRANSCEIVER, not the blimp. If unsure, unplug the blimp")
        print("        and re-run -- the port that disappears is the blimp.\n")
    return True


def drain(ser, label, settle=0.35):
    """Read everything currently waiting and print it."""
    time.sleep(settle)
    data = ser.read(ser.in_waiting or 1)
    text = data.decode(errors="replace").strip()
    if text:
        for line in text.splitlines():
            print(f"    [esp] {line}")
    else:
        print(f"    [esp] (silence -- no reply to {label})")
    return text


def main():
    if not list_ports():
        return

    print("=" * 62)
    print(f"OPENING {SERIAL_PORT}")
    print("=" * 62)
    try:
        ser = serial.Serial(SERIAL_PORT, 115200, timeout=0.4)
    except serial.SerialException as exc:
        print(f"  FAILED: {exc}")
        print("  If this says 'Resource busy', close the Arduino Serial Monitor")
        print("  that is open on THIS port.")
        return

    time.sleep(2.0)          # ESP32 resets when the port opens
    ser.reset_input_buffer()

    # Boot banner tells us which sketch is actually running.
    print("  Resetting and reading boot output...")
    boot = drain(ser, "boot", settle=1.5)
    if "Transmitter ESP Board" in boot:
        print("    --> Confirmed: this is the BaseTranseiver.")
    elif "BareBonesDiff" in boot:
        print("    !! This is the BLIMP, not the transceiver.")
        print("       SERIAL_PORT is pointing at the wrong device.")
        ser.close()
        return
    else:
        print("    ?? Unrecognized boot banner (may just have missed it).")
    print()

    # --- add peer ---
    print("=" * 62)
    print(f"ADD PEER  {TARGET_MAC}")
    print("=" * 62)
    mac_bytes = bytes(int(x, 16) for x in TARGET_MAC.split(":"))
    ser.write(b"A" + mac_bytes)
    reply = drain(ser, "'A'")
    if "Added peer successfully" in reply:
        print("    --> Peer accepted.")
    elif "Failed to add peer" in reply:
        print("    !! Transceiver REFUSED the peer.")
    print()

    # --- send armed packets ---
    print("=" * 62)
    print(f"SEND {N_PACKETS} ARMED PACKETS (armed=1, fx=1.0, dyaw=-1.0)")
    print("=" * 62)
    print("  Watch the BLIMP's serial monitor now -- it should print '# ARMED'.\n")

    params = [0.0] * 13
    params[0] = 1.0     # armed
    params[1] = 1.0     # fx forward
    params[4] = -1.0    # dyaw left

    acks = 0
    for i in range(N_PACKETS):
        payload = struct.pack("<6B13f", *mac_bytes, *params)
        ser.write(b"C" + payload)
        time.sleep(0.05)
        resp = ser.read(ser.in_waiting or 1).decode(errors="replace")
        if "Sent Controls" in resp:
            acks += 1
        if i == 0:
            for line in resp.strip().splitlines():
                print(f"    [esp] {line}")

    print(f"\n  {acks}/{N_PACKETS} packets acknowledged by the transceiver.")

    # --- disarm ---
    params = [0.0] * 13
    for _ in range(5):
        ser.write(b"C" + struct.pack("<6B13f", *mac_bytes, *params))
        time.sleep(0.03)
    ser.read(ser.in_waiting or 1)
    print("  Disarm sent.\n")

    ser.close()

    print("=" * 62)
    print("HOW TO READ THIS")
    print("=" * 62)
    print("  Transceiver ack'd, blimp printed '# ARMED'")
    print("      -> Link is fine. Problem is actuation: ESC power,")
    print("         ESC calibration range, or motor/servo pins.")
    print()
    print("  Transceiver ack'd, blimp printed NOTHING")
    print("      -> Packets are leaving but not landing. Check that the MAC")
    print("         above matches the 'Blimp MAC:' line in the blimp's boot")
    print("         output, character for character.")
    print()
    print("  Transceiver silent / refused peer")
    print("      -> Wrong port, or BaseTranseiver.ino isn't flashed on it.")


if __name__ == "__main__":
    main()
