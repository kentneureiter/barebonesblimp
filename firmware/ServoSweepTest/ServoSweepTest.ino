/**
 * ServoSweepTest.ino
 * ---------------------------------------------------------------------------
 * Dead-simple servo isolation test. NO motors, NO ESP-NOW, NO mixer.
 * Just sweeps one servo back and forth so you can confirm, on the scope and by
 * eye, that the servo + pin + power all work on their own.
 *
 * If the servo sweeps cleanly here but not in BareBonesDiff, the problem is an
 * interaction in the full sketch (PWM timer sharing with the motors, or the
 * motor current sagging the servo's power rail) -- not the servo itself.
 *
 * Change PIN_SERVO below to test D0 vs D1.
 * ---------------------------------------------------------------------------
 */

#include <ESP32Servo.h>

#define PIN_SERVO   D1        // <-- change to D0 to test the other header

Servo servo;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.print("Servo sweep test on ");
    Serial.println("D1");   // update the label if you change the pin

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    servo.setPeriodHertz(50);
    servo.attach(PIN_SERVO, 550, 2450);
    Serial.println("Sweeping 0 -> 180 -> 0 ...");
}

void loop() {
    for (int a = 0; a <= 180; a += 2) {
        servo.write(a);
        Serial.println(a);
        delay(25);
    }
    for (int a = 180; a >= 0; a -= 2) {
        servo.write(a);
        Serial.println(a);
        delay(25);
    }
}
