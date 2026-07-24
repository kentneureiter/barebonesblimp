"""
BareBonesControl.py
---------------------------------------------------------------------------
Minimal keyboard ground station for BareBonesDiff.ino.

Deliberately flat: it imports only comm/Serial.py. No RobotMaster, no
SimpleGUI, no state machine, no ready-gate. Nothing to arm except SPACE.

CONTROLS (a pygame window must have focus)
    W      forward thrust
    A / D  yaw left / right
    Q / E  up / down
    SPACE  toggle ARMED
    ESC    disarm and quit

PACKET
    Same 13-float ControlInput the existing BaseTranseiver.ino forwards, so
    the transceiver firmware needs no changes.

        params[0] = armed (0/1)
        params[1] = fx     forward
        params[2] = dz     up/down
        params[4] = dyaw   turn
        (all others 0)
---------------------------------------------------------------------------
"""

import sys
import time

import pygame

from comm.Serial import SerialController
from user_parameters import ROBOT_MACS, SERIAL_PORT

# --- tuning -----------------------------------------------------------------
SEND_HZ = 20            # command rate
SERIAL_TIMEOUT = 0.1    # keep short so the loop never stalls on a read

# Manual-mode step sizes (per 1/SEND_HZ tick while the key is held)
THROTTLE_STEP = 0.010   # Q / E throttle ramp
SERVO_STEP    = 1.0     # J / L servo up-reference jog, degrees
SERVO_UP_INIT = 90.0    # starting guess for "props point up"

# Which robot to drive. Index into ROBOT_MACS, or hardcode a MAC string here.
TARGET_MAC = ROBOT_MACS[0]

# Paste the MAC the blimp prints at boot ("Blimp MAC: ...") here. The program
# refuses to start unless it matches TARGET_MAC -- this is the guard against
# sending packets to a blimp that isn't listening. Set to None to skip.
EXPECTED_BLIMP_MAC = "34:85:18:8f:36:b0" # 34:85:18:8f:36:b0 34:85:18:91:B7:4C


def build_params(armed, p1, p2, dyaw, mode=0.0, servo_up=0.0):
    """Pack the live values into the 13-float packet.

    PID mode    : p1 = fx,       p2 = dz
    Manual mode : p1 = forward,  p2 = throttle, mode = 1, servo_up = up-ref deg
    """
    p = [0.0] * 13
    p[0] = 1.0 if armed else 0.0
    p[1] = p1
    p[2] = p2
    p[3] = mode
    p[4] = dyaw
    p[5] = servo_up
    return tuple(p)


def main():
    print(f"Port   : {SERIAL_PORT}")
    print(f"Target : {TARGET_MAC}")

    if EXPECTED_BLIMP_MAC is not None:
        if TARGET_MAC.lower() != EXPECTED_BLIMP_MAC.lower():
            print("\n" + "!" * 60)
            print("  MAC MISMATCH -- refusing to start.")
            print(f"    sending to      : {TARGET_MAC}")
            print(f"    blimp expects   : {EXPECTED_BLIMP_MAC.lower()}")
            print("  Fix robot_macs[0] in user_parameters.py, or update")
            print("  EXPECTED_BLIMP_MAC to match the blimp's boot banner.")
            print("!" * 60)
            return
        print(f"Match  : OK (target == expected blimp)")

    serial_ctl = SerialController(SERIAL_PORT, timeout=SERIAL_TIMEOUT)

    # Register the blimp with the transceiver.
    serial_ctl.manage_peer("A", TARGET_MAC)

    pygame.init()
    screen = pygame.display.set_mode((520, 300))
    pygame.display.set_caption("BareBones Control - SPACE arm, M mode, ESC quit")
    font = pygame.font.SysFont("menlo,consolas,monospace", 16)

    armed = False
    manual = False              # False = PID/auto, True = manual/bench
    throttle = 0.0              # latched, manual mode
    servo_up = SERVO_UP_INIT    # jogged servo "up" reference, manual mode
    period = 1.0 / SEND_HZ

    print("\nReady. Click this window, then SPACE to arm.\n")

    try:
        while True:
            loop_start = time.time()

            # --- events (SPACE / M / ESC are edge-triggered) ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        armed = not armed
                        throttle = 0.0            # always re-arm at zero thrust
                        print("ARMED" if armed else "DISARMED")
                    elif event.key == pygame.K_m:
                        manual = not manual
                        throttle = 0.0
                        print("MANUAL mode" if manual else "PID mode")
                    elif event.key == pygame.K_ESCAPE:
                        raise KeyboardInterrupt

            keys = pygame.key.get_pressed()
            dyaw = (1.0 if keys[pygame.K_d] else 0.0) - (1.0 if keys[pygame.K_a] else 0.0)

            if manual:
                # Q/E ramp a latched throttle; J/L jog the servo up-reference.
                if armed:
                    throttle += THROTTLE_STEP * ((1 if keys[pygame.K_q] else 0)
                                                 - (1 if keys[pygame.K_e] else 0))
                    throttle = min(1.0, max(0.0, throttle))
                servo_up += SERVO_STEP * ((1 if keys[pygame.K_l] else 0)
                                          - (1 if keys[pygame.K_j] else 0))
                servo_up = min(180.0, max(0.0, servo_up))
                forward = 1.0 if keys[pygame.K_w] else 0.0
                if not armed:
                    dyaw = 0.0
                params = build_params(armed, forward, throttle, dyaw,
                                      mode=1.0, servo_up=servo_up)
            else:
                # PID mode: W = fx, Q/E = height rate.
                fx = 1.0 if keys[pygame.K_w] else 0.0
                dz = (1.0 if keys[pygame.K_q] else 0.0) - (1.0 if keys[pygame.K_e] else 0.0)
                if not armed:
                    fx = dyaw = dz = 0.0
                params = build_params(armed, fx, dz, dyaw, mode=0.0)

            serial_ctl.send_control_params(TARGET_MAC, params)

            # --- on-screen state ---
            screen.fill((18, 18, 20))
            if manual:
                lines = [
                    ("ARMED" if armed else "DISARMED",
                     (90, 220, 110) if armed else (220, 90, 90)),
                    ("MODE: MANUAL / bench", (120, 200, 255)),
                    (f"throttle (Q/E)  {throttle:.2f}", (220, 220, 220)),
                    (f"yaw      (A/D)  {dyaw:+.0f}", (220, 220, 220)),
                    (f"forward  (W)    {1 if keys[pygame.K_w] else 0}", (220, 220, 220)),
                    (f"servo    (J/L)  {servo_up:.0f} deg", (255, 210, 120)),
                    ("SPACE arm   M mode   ESC quit", (140, 140, 140)),
                ]
            else:
                fx = 1.0 if (armed and keys[pygame.K_w]) else 0.0
                dz = ((1.0 if keys[pygame.K_q] else 0.0)
                      - (1.0 if keys[pygame.K_e] else 0.0)) if armed else 0.0
                lines = [
                    ("ARMED" if armed else "DISARMED",
                     (90, 220, 110) if armed else (220, 90, 90)),
                    ("MODE: PID / auto", (120, 200, 255)),
                    (f"fx   (W)   {fx:+.2f}", (220, 220, 220)),
                    (f"dyaw (A/D) {dyaw:+.0f}", (220, 220, 220)),
                    (f"dz   (Q/E) {dz:+.2f}", (220, 220, 220)),
                    ("", (0, 0, 0)),
                    ("SPACE arm   M mode   ESC quit", (140, 140, 140)),
                ]
            for i, (text, color) in enumerate(lines):
                screen.blit(font.render(text, True, color), (20, 20 + i * 28))
            pygame.display.flip()

            # --- pace the loop ---
            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        print("\nStopping - sending disarm...")
    finally:
        # Best effort: several disarms in case one drops.
        try:
            for _ in range(5):
                serial_ctl.send_control_params(TARGET_MAC, build_params(False, 0, 0, 0))
                time.sleep(0.02)
            serial_ctl.close()
        except Exception as exc:
            print("Disarm on exit failed:", exc)
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    main()
