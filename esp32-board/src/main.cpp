#include <Arduino.h>
#include <BLEGamepadClient.h>

// ── Motor driver pins (PWM, DIR) ──────────────────────────────────────────────
//   PWM=0  DIR=x  → OUT_A=LOW,  OUT_B=LOW   (brake)
//   PWM=1  DIR=0  → OUT_A=HIGH, OUT_B=LOW   (forward)
//   PWM=1  DIR=1  → OUT_A=LOW,  OUT_B=HIGH  (reverse)
//
//   ROBOT LAYOUT (top view):
//   [M1 LF]───[M2 RF]
//      │           │
//   [M3 LR]───[M4 RR]
//
//   LEFT SIDE  (left stick Y):  M1(32/33) + M3(23/22)
//   RIGHT SIDE (right stick Y): M2(25/26) + M4(17/16)

struct Motor { uint8_t chan, pwm, dir; };

static const Motor M[] = {
    {0, 32, 33},  // M1 — Left  Front
    {1, 25, 26},  // M2 — Right Front
    {2, 23, 22},  // M3 — Left  Rear
    {3, 17, 16},  // M4 — Right Rear
};

#define PWM_FREQ  1000
#define PWM_BITS  8

// ── Controller ────────────────────────────────────────────────────────────────
XboxController controller;

// ── Motor helper ──────────────────────────────────────────────────────────────
void drive(const Motor& m, float val) {
    if (fabsf(val) < 0.05f) { ledcWrite(m.chan, 0); return; }
    digitalWrite(m.dir, val > 0 ? LOW : HIGH);
    ledcWrite(m.chan, (uint8_t)(fabsf(val) * 255.0f));
}

void driveLeft(float v)  { drive(M[0], v); drive(M[2], v); }
void driveRight(float v) { drive(M[1], v); drive(M[3], v); }

void stopAll() {
    for (auto& m : M) ledcWrite(m.chan, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // Motor pins
    for (auto& m : M) {
        pinMode(m.dir, OUTPUT);
        digitalWrite(m.dir, LOW);
        ledcSetup(m.chan, PWM_FREQ, PWM_BITS);
        ledcAttachPin(m.pwm, m.chan);
        ledcWrite(m.chan, 0);
    }

    // BLE
    controller.begin();
    Serial.println("BLE: scanning for Xbox controller...");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    bool connected = controller.isConnected();

    if (connected) {
        XboxControlsState s;
        controller.read(&s);

        // Tank drive: left Y → left side, right Y → right side
        float ml = -s.leftStickY;
        float mr = -s.rightStickY;

        driveLeft(-ml);
        driveRight(mr);
    } else {
        stopAll();
    }
}
