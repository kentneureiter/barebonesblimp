/**
 * BareBonesDiff.ino
 * ---------------------------------------------------------------------------
 * Minimal thrust-vectoring blimp test rig for a XIAO ESP32-S3.
 *
 * Deliberately standalone: this sketch includes NOTHING from BlimpSwarm/src.
 * Every line of the control path is visible in this one file so it can be
 * reasoned about without chasing the class hierarchy.
 *
 * HARDWARE (matches your PCB)
 *   MOTOR1 -> D9    MOTOR2 -> D10    SERVO (tilt) -> D0
 *   I2C    -> D4 (SDA) / D5 (SCL)    BNO085 @ 0x4A, BMP390 default addr
 *
 * CONTROL
 *   W = forward     A = yaw left     D = yaw right     Q = up     E = down
 *   Commands arrive over ESP-NOW as the same 13-float packet the existing
 *   BaseTranseiver.ino already forwards, so the transceiver needs NO changes.
 *
 * PID
 *   Yaw    : closed loop on BNO085 heading
 *   Height : closed loop on BMP390 altitude
 *   Forward: open loop (W maps straight to thrust)
 *
 * SAFETY
 *   THROTTLE_CAP limits every motor command. Start with props OFF.
 * ---------------------------------------------------------------------------
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <Adafruit_BMP3XX.h>

// ===========================================================================
//  TUNING  -- everything you should need to touch lives in this block
// ===========================================================================

// ---- Pins -----------------------------------------------------------------
#define PIN_MOTOR1  D9
#define PIN_MOTOR2  D10
#define PIN_SERVO   D0
#define PIN_SDA     D4
#define PIN_SCL     D5

// ---- ESC pulse range ------------------------------------------------------
// Must match what your ESCs were calibrated to. 1100/1927 mirrors the values
// BLMotor.cpp produces today. If motors never spin, suspect these first.
static const int   MOTOR_MIN_US   = 1100;
static const int   MOTOR_MAX_US   = 1927;

// Hard ceiling on any motor command (0..1). Bench testing: keep this low.
static const float THROTTLE_CAP   = 0.35f;

// ---- Servo ----------------------------------------------------------------
// servo_deg = SERVO_CENTER_DEG + theta_deg * SERVO_SCALE
//   theta =  0 deg -> thrust points FORWARD
//   theta = 90 deg -> thrust points UP
// Defaults reproduce your current geometry (theta 0 -> 90 deg, theta 90 -> 45 deg).
// Flip the sign of SERVO_SCALE if the servo tilts the wrong way.
static const float SERVO_CENTER_DEG = 90.0f;
static const float SERVO_SCALE      = -0.5f;
static const float SERVO_MIN_DEG    = 0.0f;
static const float SERVO_MAX_DEG    = 180.0f;

// ---- Geometry -------------------------------------------------------------
// Half the distance between the two motors, in meters.
static const float ARM_L = 0.10f;

// ---- Yaw PID --------------------------------------------------------------
static const float KP_YAW = 0.06f;
static const float KD_YAW = 0.02f;    // acts on yaw RATE (gyro)
static const float KI_YAW = 0.0f;
static const float YAW_I_LIMIT   = 0.05f;
static const float YAW_ERR_LIMIT = 0.6f;    // rad, caps how hard it chases
static const float TAUZ_LIMIT    = 0.10f;   // N*m

// ---- Height PID -----------------------------------------------------------
static const float KP_Z = 0.9f;
static const float KD_Z = 1.2f;       // acts on climb rate
static const float KI_Z = 0.0f;
static const float Z_I_LIMIT  = 0.15f;
static const float FZ_LIMIT   = 0.8f;

// ---- Command scaling ------------------------------------------------------
static const float YAW_RATE_CMD = 1.2f;   // rad/s of setpoint travel per key
static const float Z_RATE_CMD   = 0.30f;  // m/s of setpoint travel per key
static const float FX_GAIN      = 0.50f;  // W key -> forward thrust

// ---- Mixer guards ---------------------------------------------------------
// Yaw torque scales with cos(theta). Near theta = 90 deg (thrust straight up)
// yaw authority goes to zero and tauz/(l*cos) explodes. Floor the cosine and
// cap the split so the mixer degrades gracefully instead of saturating.
static const float COS_FLOOR = 0.30f;
static const float DIFF_MAX  = 0.60f;

// ---- Loop timing ----------------------------------------------------------
static const unsigned long TIME_STEP_US  = 5000;    // 200 Hz control
static const unsigned long PRINT_STEP_US = 50000;   // 20 Hz telemetry
static const unsigned long COMM_TIMEOUT_US = 750000; // disarm if link drops

// ===========================================================================
//  STATE
// ===========================================================================

typedef struct ControlInput {
    float params[13];
} ControlInput;

// params[0] = armed (0/1)
// params[1] = fx    forward   (W)
// params[2] = dz    up/down   (Q / E)
// params[4] = dyaw  turn      (A / D)
volatile ControlInput cmd;
volatile bool     newCmd      = false;
volatile unsigned long lastRxUs = 0;

Servo motor1, motor2, tiltServo;
BNO08x        imu;
Adafruit_BMP3XX bmp;

bool  imuOk  = false;
bool  bmpOk  = false;

float yaw = 0.0f, yawRate = 0.0f;
float roll = 0.0f, pitch = 0.0f;
float altitude = 0.0f, climbRate = 0.0f, groundLevel = 0.0f;

float yawSetpoint = 0.0f;
float zSetpoint   = 0.0f;
float yawIntegral = 0.0f;
float zIntegral   = 0.0f;

bool  wasArmed = false;

// Last values actually written to hardware, for telemetry.
float outF1 = 0.0f, outF2 = 0.0f, outServoDeg = SERVO_CENTER_DEG;
float dbgTauz = 0.0f, dbgFz = 0.0f, dbgThetaDeg = 0.0f;

unsigned long clockTime, printTime;

// ===========================================================================
//  HELPERS
// ===========================================================================

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Wrap an angle error into [-PI, PI] so 179 -> -179 is a 2 degree error,
// not a 358 degree one.
static float wrapPi(float a) {
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

static void writeMotor(Servo &m, float value) {
    value = clampf(value, 0.0f, THROTTLE_CAP);
    int us = MOTOR_MIN_US + (int)(value * (MOTOR_MAX_US - MOTOR_MIN_US));
    m.writeMicroseconds(us);
}

// ===========================================================================
//  ESP-NOW
// ===========================================================================

void onDataReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len == sizeof(ControlInput)) {
        memcpy((void *)&cmd, data, len);
        newCmd   = true;
        lastRxUs = micros();
    }
}

void initEspNow() {
    WiFi.mode(WIFI_STA);
    delay(200);
    Serial.print("Blimp MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println("^ put this in SwarmBase/user_parameters.py");

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED");
        return;
    }
    esp_now_register_recv_cb(onDataReceive);
    Serial.println("ESP-NOW ready");
}

// ===========================================================================
//  SENSORS
// ===========================================================================

void initIMU() {
    Wire.begin(PIN_SDA, PIN_SCL);
    delay(100);
    if (!imu.begin(0x4A, Wire)) {
        Serial.println("BNO085 NOT FOUND - yaw PID will be disabled");
        imuOk = false;
        return;
    }
    Wire.setClock(400000);
    imu.softReset();
    imu.modeOn();
    delay(50);
    if (imu.enableGyroIntegratedRotationVector()) {
        Serial.println("BNO085 ready");
        imuOk = true;
    } else {
        Serial.println("BNO085 report enable FAILED");
        imuOk = false;
    }
}

void initBaro() {
    if (!bmp.begin_I2C()) {
        Serial.println("BMP390 NOT FOUND - height PID will be disabled");
        bmpOk = false;
        return;
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ);

    // Average a few readings to establish "zero" at the bench/floor.
    float acc = 0.0f;
    int   n   = 0;
    for (int i = 0; i < 15; i++) {
        if (bmp.performReading()) {
            if (i >= 5) { acc += bmp.readAltitude(1013.25); n++; }
        }
        delay(50);
    }
    if (n > 0) {
        groundLevel = acc / n;
        bmpOk = true;
        Serial.print("BMP390 ready, ground level = ");
        Serial.println(groundLevel);
    }
}

void quatToEuler(float qw, float qx, float qy, float qz,
                 float &r, float &p, float &y) {
    r = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
    float sinp = 2.0f * (qw * qy - qz * qx);
    p = (fabsf(sinp) >= 1.0f) ? copysignf(PI / 2.0f, sinp) : asinf(sinp);
    y = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}

void readSensors(float dt) {
    if (imuOk) {
        while (imu.getSensorEvent()) {
            if (imu.getSensorEventID() == SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR) {
                quatToEuler(imu.getGyroIntegratedRVReal(),
                            imu.getGyroIntegratedRVI(),
                            imu.getGyroIntegratedRVJ(),
                            imu.getGyroIntegratedRVK(),
                            roll, pitch, yaw);
                yawRate = imu.getGyroIntegratedRVangVelZ();
                break;
            }
        }
    }
    if (bmpOk && bmp.performReading()) {
        float newAlt = bmp.readAltitude(1013.25) - groundLevel;
        float rawRate = (newAlt - altitude) / dt;
        climbRate = climbRate * 0.9f + rawRate * 0.1f;   // heavy smoothing
        altitude  = newAlt;
    }
}

// ===========================================================================
//  CONTROL
// ===========================================================================

void resetControllers() {
    yawSetpoint = yaw;
    zSetpoint   = altitude;
    yawIntegral = 0.0f;
    zIntegral   = 0.0f;
}

void disarm() {
    writeMotor(motor1, 0.0f);
    writeMotor(motor2, 0.0f);
    tiltServo.write((int)SERVO_CENTER_DEG);
    outF1 = 0.0f;
    outF2 = 0.0f;
    outServoDeg = SERVO_CENTER_DEG;
    dbgTauz = 0.0f;
    dbgFz = 0.0f;
    dbgThetaDeg = 0.0f;
}

void controlStep(float dt) {
    float fxCmd   = cmd.params[1];
    float dzCmd   = cmd.params[2];
    float dyawCmd = cmd.params[4];

    // ---- integrate key presses into setpoints ----
    yawSetpoint = wrapPi(yawSetpoint + dyawCmd * YAW_RATE_CMD * dt);
    zSetpoint   = zSetpoint + dzCmd * Z_RATE_CMD * dt;
    if (zSetpoint < 0.0f) zSetpoint = 0.0f;

    // ---- yaw PID -> tauz ----
    float tauz = 0.0f;
    if (imuOk) {
        float eYaw = clampf(wrapPi(yawSetpoint - yaw), -YAW_ERR_LIMIT, YAW_ERR_LIMIT);
        yawIntegral = clampf(yawIntegral + eYaw * dt * KI_YAW, -YAW_I_LIMIT, YAW_I_LIMIT);
        tauz = KP_YAW * eYaw - KD_YAW * yawRate + yawIntegral;
        tauz = clampf(tauz, -TAUZ_LIMIT, TAUZ_LIMIT);
    }

    // ---- height PID -> fz ----
    float fz = 0.0f;
    if (bmpOk) {
        float eZ = zSetpoint - altitude;
        zIntegral = clampf(zIntegral + eZ * dt * KI_Z, -Z_I_LIMIT, Z_I_LIMIT);
        fz = KP_Z * eZ - KD_Z * climbRate + zIntegral;
        fz = clampf(fz, -FZ_LIMIT, FZ_LIMIT);
    }

    // ---- forward thrust, open loop ----
    float fx = clampf(fxCmd * FX_GAIN, -0.6f, 0.6f);

    // ---- mixer ----
    // Thrust vector magnitude and angle. theta = 0 is forward, PI/2 is up.
    float fMag  = sqrtf(fx * fx + fz * fz);
    float theta = atan2f(fz, fx);

    // Yaw torque = ARM_L * cos(theta) * (f1 - f2).
    // Floor the cosine so the split stays finite when thrust points up.
    float cosT = cosf(theta);
    if (fabsf(cosT) < COS_FLOOR) cosT = (cosT < 0.0f) ? -COS_FLOOR : COS_FLOOR;

    float diff = clampf(tauz / (ARM_L * cosT), -DIFF_MAX, DIFF_MAX);

    float f1 = 0.5f * (fMag + diff);
    float f2 = 0.5f * (fMag - diff);

    f1 = clampf(f1, 0.0f, THROTTLE_CAP);
    f2 = clampf(f2, 0.0f, THROTTLE_CAP);

    // ---- servo ----
    float thetaDeg = theta * 180.0f / PI;
    float servoDeg = clampf(SERVO_CENTER_DEG + thetaDeg * SERVO_SCALE,
                            SERVO_MIN_DEG, SERVO_MAX_DEG);

    // ---- actuate ----
    writeMotor(motor1, f1);
    writeMotor(motor2, f2);
    tiltServo.write((int)servoDeg);

    outF1 = f1;
    outF2 = f2;
    outServoDeg = servoDeg;
    dbgTauz = tauz;
    dbgFz = fz;
    dbgThetaDeg = thetaDeg;
}

// ===========================================================================
//  TELEMETRY
// ===========================================================================

void printTelemetry(bool armed) {
    // Header repeated occasionally so the columns are never a mystery.
    static int lineCount = 0;
    if (lineCount % 20 == 0) {
        Serial.println(F("# armed,fx,dyaw,yawSP,yaw,tauz,zSP,alt,fz,theta,f1,f2,servo"));
    }
    lineCount++;

    Serial.print("D,");
    Serial.print(armed ? 1 : 0);            Serial.print(",");
    Serial.print(cmd.params[1], 2);         Serial.print(",");
    Serial.print(cmd.params[4], 2);         Serial.print(",");
    Serial.print(yawSetpoint, 3);           Serial.print(",");
    Serial.print(yaw, 3);                   Serial.print(",");
    Serial.print(dbgTauz, 4);               Serial.print(",");
    Serial.print(zSetpoint, 2);             Serial.print(",");
    Serial.print(altitude, 2);              Serial.print(",");
    Serial.print(dbgFz, 3);                 Serial.print(",");
    Serial.print(dbgThetaDeg, 1);           Serial.print(",");
    Serial.print(outF1, 3);                 Serial.print(",");
    Serial.print(outF2, 3);                 Serial.print(",");
    Serial.println(outServoDeg, 1);
}

// ===========================================================================
//  ARDUINO
// ===========================================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== BareBonesDiff starting ===");

    for (int i = 0; i < 13; i++) cmd.params[i] = 0.0f;

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    motor1.setPeriodHertz(50);
    motor2.setPeriodHertz(50);
    tiltServo.setPeriodHertz(50);

    motor1.attach(PIN_MOTOR1, MOTOR_MIN_US, MOTOR_MAX_US);
    motor2.attach(PIN_MOTOR2, MOTOR_MIN_US, MOTOR_MAX_US);
    tiltServo.attach(PIN_SERVO, 550, 2450);

    // Hold minimum throttle so the ESCs arm. No sweep, no integer math.
    Serial.println("Arming ESCs (min throttle, 3s)...");
    writeMotor(motor1, 0.0f);
    writeMotor(motor2, 0.0f);
    tiltServo.write((int)SERVO_CENTER_DEG);
    delay(3000);
    Serial.println("ESCs armed");

    initEspNow();
    initIMU();
    initBaro();

    resetControllers();

    clockTime = micros();
    printTime = micros();
    Serial.println("=== ready ===");
}

void loop() {
    unsigned long now = micros();
    float dt = (float)(now - clockTime) / 1000000.0f;
    if (dt < (float)TIME_STEP_US / 1000000.0f) return;
    clockTime = now;

    readSensors(dt);

    bool linkAlive = (micros() - lastRxUs) < COMM_TIMEOUT_US;
    bool armed     = linkAlive && (cmd.params[0] > 0.5f);

    // Re-zero the setpoints on the arming edge so the blimp holds where it is
    // instead of lunging toward a stale target.
    if (armed && !wasArmed) {
        resetControllers();
        Serial.println("# ARMED");
    }
    if (!armed && wasArmed) {
        Serial.println("# DISARMED");
    }
    wasArmed = armed;

    if (armed) controlStep(dt);
    else       disarm();

    if (micros() - printTime > PRINT_STEP_US) {
        printTelemetry(armed);
        printTime = micros();
    }
}
