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

# Which robot to drive. Index into ROBOT_MACS, or hardcode a MAC string here.
TARGET_MAC = ROBOT_MACS[0]


def build_params(armed, fx, dz, dyaw):
    """Pack the four live values into the 13-float packet."""
    p = [0.0] * 13
    p[0] = 1.0 if armed else 0.0
    p[1] = fx
    p[2] = dz
    p[4] = dyaw
    return tuple(p)


def main():
    print(f"Port   : {SERIAL_PORT}")
    print(f"Target : {TARGET_MAC}")

    serial_ctl = SerialController(SERIAL_PORT, timeout=SERIAL_TIMEOUT)

    # Register the blimp with the transceiver.
    serial_ctl.manage_peer("A", TARGET_MAC)

    pygame.init()
    screen = pygame.display.set_mode((480, 220))
    pygame.display.set_caption("BareBones Control - SPACE to arm, ESC to quit")
    font = pygame.font.SysFont("menlo,consolas,monospace", 16)

    armed = False
    period = 1.0 / SEND_HZ

    print("\nReady. Click this window, then SPACE to arm.\n")

    try:
        while True:
            loop_start = time.time()

            # --- events (SPACE / ESC are edge-triggered) ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        armed = not armed
                        print("ARMED" if armed else "DISARMED")
                    elif event.key == pygame.K_ESCAPE:
                        raise KeyboardInterrupt

            # --- held keys (continuous control) ---
            keys = pygame.key.get_pressed()
            fx   = 1.0 if keys[pygame.K_w] else 0.0
            dyaw = (1.0 if keys[pygame.K_d] else 0.0) - (1.0 if keys[pygame.K_a] else 0.0)
            dz   = (1.0 if keys[pygame.K_q] else 0.0) - (1.0 if keys[pygame.K_e] else 0.0)

            if not armed:
                fx = dyaw = dz = 0.0

            params = build_params(armed, fx, dz, dyaw)
            serial_ctl.send_control_params(TARGET_MAC, params)

            # --- on-screen state ---
            screen.fill((18, 18, 20))
            lines = [
                ("ARMED" if armed else "DISARMED",
                 (90, 220, 110) if armed else (220, 90, 90)),
                (f"fx   (W)   {fx:+.2f}", (220, 220, 220)),
                (f"dyaw (A/D) {dyaw:+.2f}", (220, 220, 220)),
                (f"dz   (Q/E) {dz:+.2f}", (220, 220, 220)),
                ("", (0, 0, 0)),
                ("SPACE arm/disarm    ESC quit", (140, 140, 140)),
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
