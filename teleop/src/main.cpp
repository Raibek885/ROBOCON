// ═══════════════════════════════════════════════════════════════════════
//   Abu Robocon — Unified Teleop Controller
//   Single ESP32 + PS4 DualShock (Bluepad32)
//
//   Subsystems:
//     1. Diamond Omni Drive: 4× Cytron motors (holonomic)
//     2. BTS7960: 2× H-bridge motors (arm / mechanism)
//
//   ── Diamond Omni Layout (top view) ────────────────────────────────
//
//              [M2] ↕ fwd/bwd
//               |
//    [M1] ←→ ──●── ←→ [M3]   strafe left/right
//               |
//              [M4] ↕ fwd/bwd
//
//    M2 (25/26) + M4 (17/16) → forward / backward
//    M1 (32/33) + M3 (23/22) → left / right  (strafe)
//
//   ── Gamepad Mapping (PS4 DualShock) ───────────────────────────────
//     Right Stick Y  → Forward / Backward   (M2 + M4)
//     Right Stick X  → Strafe Left / Right  (M1 + M3)
//     Left  Stick X  → Rotate CW / CCW      (all motors)
//     L2 trigger     → BTS Motor 1 forward  (proportional)
//     L1 button      → BTS Motor 1 reverse  (full speed)
//     R2 trigger     → BTS Motor 2 forward  (proportional)
//     R1 button      → BTS Motor 2 reverse  (full speed)
//     ○ (Circle)     → EMERGENCY STOP — all motors off
//
//   ── Pin Map (12 GPIOs total) ──────────────────────────────────────
//     Cytron M1 (strafe):  PWM=32, DIR=33  (LEDC ch 0) ← SOLDERED
//     Cytron M2 (fwd/bwd): PWM=25, DIR=26  (LEDC ch 1) ← SOLDERED
//     Cytron M3 (strafe):  PWM=23, DIR=22  (LEDC ch 2) ← SOLDERED
//     Cytron M4 (fwd/bwd): PWM=17, DIR=16  (LEDC ch 3) ← SOLDERED
//     BTS Motor 1:  RPWM=4,  LPWM=27  (LEDC ch 4,5)
//     BTS Motor 2:  RPWM=14, LPWM=13  (LEDC ch 6,7)
//     BTS EN pins:  ALL wired to 3.3V  (not using GPIOs)
// ═══════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Bluepad32.h>

// ─────────────────────────────────────────────────────────────────────
//  Cytron Omni-Drive Motors (PWM + DIR)
//  PWM=0 DIR=x → brake  |  PWM>0 DIR=LOW → fwd  |  PWM>0 DIR=HIGH → rev
//  PINS ARE SOLDERED — DO NOT CHANGE
// ─────────────────────────────────────────────────────────────────────
struct DriveMotor { uint8_t ledcChan, pwmPin, dirPin; };

static const DriveMotor DM[] = {
    {0, 32, 33},  // M1 — Strafe axis (left/right)
    {1, 25, 26},  // M2 — Forward/backward axis
    {2, 23, 22},  // M3 — Strafe axis (left/right)
    {3, 17, 16},  // M4 — Forward/backward axis
};

#define DRIVE_PWM_FREQ  1000
#define DRIVE_PWM_BITS  8

// ─────────────────────────────────────────────────────────────────────
//  BTS7960 Motor 1 — EN pins hardwired to 3.3V
//  LEDC channels 4 + 5
// ─────────────────────────────────────────────────────────────────────
#define BTS1_RPWM_PIN  4
#define BTS1_LPWM_PIN  27
#define BTS1_LEDC_R    4
#define BTS1_LEDC_L    5

// ─────────────────────────────────────────────────────────────────────
//  BTS7960 Motor 2 — EN pins hardwired to 3.3V
//  LEDC channels 6 + 7
// ─────────────────────────────────────────────────────────────────────
#define BTS2_RPWM_PIN  14
#define BTS2_LPWM_PIN  13
#define BTS2_LEDC_R    6
#define BTS2_LEDC_L    7

#define BTS_PWM_FREQ   5000
#define BTS_PWM_RES    8
#define MOTOR_SPEED    255
#define MOTOR_RUN_MS   1000

// ─────────────────────────────────────────────────────────────────────
//  Bluepad32 — PS4 DualShock Controller
// ─────────────────────────────────────────────────────────────────────
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
#define JOYSTICK_DEADZONE 50
#define TRIGGER_DEADZONE  30

// ═══════════════════════════════════════════════════════════════════════
//  Omni-Drive motor helper
// ═══════════════════════════════════════════════════════════════════════

// val: -1.0 (reverse) .. 0 (brake) .. +1.0 (forward)
void driveMotor(const DriveMotor& m, float val) {
    if (fabsf(val) < 0.05f) { ledcWrite(m.ledcChan, 0); return; }
    digitalWrite(m.dirPin, val > 0 ? LOW : HIGH);
    ledcWrite(m.ledcChan, (uint8_t)(fabsf(val) * 255.0f));
}

void stopDriveAll() {
    for (auto& m : DM) ledcWrite(m.ledcChan, 0);
}

// ═══════════════════════════════════════════════════════════════════════
//  BTS7960 Motor helpers
// ═══════════════════════════════════════════════════════════════════════

void btsMotor1(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  if (forward) { ledcWrite(BTS1_LEDC_R, speed); ledcWrite(BTS1_LEDC_L, 0); }
  else         { ledcWrite(BTS1_LEDC_R, 0);     ledcWrite(BTS1_LEDC_L, speed); }
}
void stopBtsMotor1() { ledcWrite(BTS1_LEDC_R, 0); ledcWrite(BTS1_LEDC_L, 0); }

void btsMotor2(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  if (forward) { ledcWrite(BTS2_LEDC_R, speed); ledcWrite(BTS2_LEDC_L, 0); }
  else         { ledcWrite(BTS2_LEDC_R, 0);     ledcWrite(BTS2_LEDC_L, speed); }
}
void stopBtsMotor2() { ledcWrite(BTS2_LEDC_R, 0); ledcWrite(BTS2_LEDC_L, 0); }

void emergencyStop() {
  stopDriveAll();
  stopBtsMotor1();
  stopBtsMotor2();
  Serial.println("!!! EMERGENCY STOP — all motors off !!!");
}

// ─── Serial motor commands ─────────────────────────────────────────

void spinLeft()   { Serial.println("\n=== BTS Motor 1: fwd ===");
                    btsMotor1(MOTOR_SPEED, true);  delay(MOTOR_RUN_MS); stopBtsMotor1();
                    Serial.println("  Stopped.\n"); }
void spinRight()  { Serial.println("\n=== BTS Motor 1: rev ===");
                    btsMotor1(MOTOR_SPEED, false); delay(MOTOR_RUN_MS); stopBtsMotor1();
                    Serial.println("  Stopped.\n"); }
void spinLeft2()  { Serial.println("\n=== BTS Motor 2: fwd ===");
                    btsMotor2(MOTOR_SPEED, true);  delay(MOTOR_RUN_MS); stopBtsMotor2();
                    Serial.println("  Stopped.\n"); }
void spinRight2() { Serial.println("\n=== BTS Motor 2: rev ===");
                    btsMotor2(MOTOR_SPEED, false); delay(MOTOR_RUN_MS); stopBtsMotor2();
                    Serial.println("  Stopped.\n"); }

// ═══════════════════════════════════════════════════════════════════════
//  Serial help
// ═══════════════════════════════════════════════════════════════════════

void printHelp() {
  Serial.println("\n=== Teleop Control — Commands ===");
  Serial.println("  left      BTS Motor 1: spin forward");
  Serial.println("  right     BTS Motor 1: spin reverse");
  Serial.println("  left2     BTS Motor 2: spin forward");
  Serial.println("  right2    BTS Motor 2: spin reverse");
  Serial.println("  stop      Emergency stop all motors");
  Serial.println("  help      Show this help message");
  Serial.println();
  Serial.println("  PS4 Gamepad — Diamond Omni Drive:");
  Serial.println("    R-Stick Y  → Forward / Backward  (M2+M4)");
  Serial.println("    R-Stick X  → Strafe Left / Right  (M1+M3)");
  Serial.println("    L-Stick X  → Rotate CW / CCW     (all)");
  Serial.println("    L2 trigger → BTS Motor 1 fwd  (proportional)");
  Serial.println("    L1 button  → BTS Motor 1 rev  (full speed)");
  Serial.println("    R2 trigger → BTS Motor 2 fwd  (proportional)");
  Serial.println("    R1 button  → BTS Motor 2 rev  (full speed)");
  Serial.println("    Circle     → EMERGENCY STOP");
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════════════
//  Bluepad32 callbacks
// ═══════════════════════════════════════════════════════════════════════

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CONTROLLER: Connected, index=%d\n", i);
      myControllers[i] = ctl;
      return;
    }
  }
  Serial.println("CONTROLLER: Connected, but no empty slot!");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CONTROLLER: Disconnected, index=%d\n", i);
      myControllers[i] = nullptr;
      emergencyStop();
      return;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════
//  Apply joystick deadzone and normalize to -1.0 .. +1.0
// ═══════════════════════════════════════════════════════════════════════
float applyDeadzone(int raw, int deadzone) {
  if (abs(raw) <= deadzone) return 0.0f;
  float val = (float)(abs(raw) - deadzone) / (float)(512 - deadzone);
  val = constrain(val, 0.0f, 1.0f);
  return (raw < 0) ? -val : val;
}

// ═══════════════════════════════════════════════════════════════════════
//  Process PS4 Gamepad → Diamond Omni Drive + BTS Motors
//
//  Holonomic mixing for diamond omni wheels:
//
//    M1 = strafe + rotate       (strafe axis motor)
//    M2 = fwdBwd + rotate       (fwd/bwd axis motor)
//    M3 = strafe − rotate       (strafe axis, opposite rotation)
//    M4 = fwdBwd − rotate       (fwd/bwd axis, opposite rotation)
//
//  This ensures:
//    • Pure strafe:   M1 & M3 same direction, M2 & M4 = 0
//    • Pure fwd/bwd:  M2 & M4 same direction, M1 & M3 = 0
//    • Pure rotation: M1 & M2 one way, M3 & M4 opposite
//    • Combined:      all blended smoothly
//
//  !! If a direction is backwards, flip the sign in the mixing below !!
// ═══════════════════════════════════════════════════════════════════════

void processGamepad(ControllerPtr ctl) {

  // ── Read sticks ──
  float fwdBwd = -applyDeadzone(ctl->axisRY(), JOYSTICK_DEADZONE);  // R-stick Y: up = forward (+)
  float strafe =  applyDeadzone(ctl->axisRX(), JOYSTICK_DEADZONE);  // R-stick X: right = strafe right (+)
  float rotate =  applyDeadzone(ctl->axisX(),  JOYSTICK_DEADZONE);  // L-stick X: right = CW (+)

  // ── Diamond Omni Mixing ──
  // If a motor runs the wrong way, flip its sign below (e.g. change + to -)
  float m1 =  strafe + rotate;     // M1 — strafe axis
  float m2 =  fwdBwd + rotate;     // M2 — fwd/bwd axis
  float m3 =  strafe - rotate;     // M3 — strafe axis (opposite rotation)
  float m4 =  fwdBwd - rotate;     // M4 — fwd/bwd axis (opposite rotation)

  // ── Normalize: scale all if any motor exceeds ±1.0 ──
  float maxVal = fabsf(m1);
  if (fabsf(m2) > maxVal) maxVal = fabsf(m2);
  if (fabsf(m3) > maxVal) maxVal = fabsf(m3);
  if (fabsf(m4) > maxVal) maxVal = fabsf(m4);
  if (maxVal > 1.0f) {
    m1 /= maxVal;
    m2 /= maxVal;
    m3 /= maxVal;
    m4 /= maxVal;
  }

  // ── Drive motors ──
  driveMotor(DM[0], m1);   // M1 (32/33)
  driveMotor(DM[1], m2);   // M2 (25/26)
  driveMotor(DM[2], m3);   // M3 (23/22)
  driveMotor(DM[3], m4);   // M4 (17/16)

  // ── BTS Motor 1: L2 trigger (forward) / L1 button (reverse) ──
  int l2 = ctl->brake();
  bool l1 = ctl->l1();

  if (l1) {
    btsMotor1(MOTOR_SPEED, false);
  } else if (l2 > TRIGGER_DEADZONE) {
    int speed = map(l2, TRIGGER_DEADZONE, 1023, 0, 255);
    btsMotor1(constrain(speed, 0, 255), true);
  } else {
    stopBtsMotor1();
  }

  // ── BTS Motor 2: R2 trigger (forward) / R1 button (reverse) ──
  int r2 = ctl->throttle();
  bool r1 = ctl->r1();

  if (r1) {
    btsMotor2(MOTOR_SPEED, false);
  } else if (r2 > TRIGGER_DEADZONE) {
    int speed = map(r2, TRIGGER_DEADZONE, 1023, 0, 255);
    btsMotor2(constrain(speed, 0, 255), true);
  } else {
    stopBtsMotor2();
  }

  // ── Emergency Stop ──
  if (ctl->b()) {
    emergencyStop();
  }
}

// ═══════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("================================================");
  Serial.println("   Abu Robocon — Diamond Omni Teleop Controller");
  Serial.println("   PS4 DualShock  |  1 ESP32  |  All Systems");
  Serial.println("================================================");
  Serial.println();

  // ── Cytron Omni-Drive Motors (PWM + DIR) — SOLDERED ──
  for (auto& m : DM) {
    pinMode(m.dirPin, OUTPUT);
    digitalWrite(m.dirPin, LOW);
    ledcSetup(m.ledcChan, DRIVE_PWM_FREQ, DRIVE_PWM_BITS);
    ledcAttachPin(m.pwmPin, m.ledcChan);
    ledcWrite(m.ledcChan, 0);
  }
  Serial.println("Diamond omni drive initialized:");
  Serial.println("  M1(32/33) M2(25/26) M3(23/22) M4(17/16)");

  // ── BTS7960 Motor 1 (EN hardwired to 3.3V) ──
  pinMode(BTS1_RPWM_PIN, OUTPUT);
  pinMode(BTS1_LPWM_PIN, OUTPUT);
  ledcSetup(BTS1_LEDC_R, BTS_PWM_FREQ, BTS_PWM_RES);
  ledcSetup(BTS1_LEDC_L, BTS_PWM_FREQ, BTS_PWM_RES);
  ledcAttachPin(BTS1_RPWM_PIN, BTS1_LEDC_R);
  ledcAttachPin(BTS1_LPWM_PIN, BTS1_LEDC_L);
  stopBtsMotor1();
  Serial.println("BTS7960 Motor 1 initialized (RPWM=4, LPWM=27)");

  // ── BTS7960 Motor 2 (EN hardwired to 3.3V) ──
  pinMode(BTS2_RPWM_PIN, OUTPUT);
  pinMode(BTS2_LPWM_PIN, OUTPUT);
  ledcSetup(BTS2_LEDC_R, BTS_PWM_FREQ, BTS_PWM_RES);
  ledcSetup(BTS2_LEDC_L, BTS_PWM_FREQ, BTS_PWM_RES);
  ledcAttachPin(BTS2_RPWM_PIN, BTS2_LEDC_R);
  ledcAttachPin(BTS2_LPWM_PIN, BTS2_LEDC_L);
  stopBtsMotor2();
  Serial.println("BTS7960 Motor 2 initialized (RPWM=14, LPWM=13)");

  // ── Bluepad32 ──
  Serial.println("\nInitializing Bluepad32...");
  BP32.setup(&onConnectedController, &onDisconnectedController);
  // BP32.forgetBluetoothKeys();   // Uncomment to clear old pairings
  Serial.println("Bluepad32 ready! Hold SHARE+PS on your PS4 controller to pair.\n");

  printHelp();
}

// ═══════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════
void loop() {

  // ── Process PS4 controller ──
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      if (myControllers[i] && myControllers[i]->isConnected()
          && myControllers[i]->hasData()
          && myControllers[i]->isGamepad()) {
        processGamepad(myControllers[i]);
      }
    }
  }
  vTaskDelay(1);

  // ── Process Serial commands ──
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    if (input.equalsIgnoreCase("help"))   { printHelp();       return; }
    if (input.equalsIgnoreCase("left"))   { spinLeft();        return; }
    if (input.equalsIgnoreCase("right"))  { spinRight();       return; }
    if (input.equalsIgnoreCase("left2"))  { spinLeft2();       return; }
    if (input.equalsIgnoreCase("right2")) { spinRight2();      return; }
    if (input.equalsIgnoreCase("stop"))   { emergencyStop();   return; }

    Serial.println("Unknown command: '" + input + "'");
    Serial.println("Type 'help' for available commands.");
  }
}
