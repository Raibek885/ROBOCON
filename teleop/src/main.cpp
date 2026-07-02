// ═══════════════════════════════════════════════════════════════════════
//   Abu Robocon — Unified Teleop Controller
//   Single ESP32 + PS4 DualShock (Bluepad32)
//
//   Subsystems:
//     1. Tank-drive: 4× PWM+DIR motors (left stick Y / right stick Y)
//     2. BTS7960:    2× H-bridge motors (L2/R2 triggers + L1/R1 reverse)
//     3. PCA9685:    16-channel servo driver (grippers via buttons)
//
//   Gamepad mapping (PS4 DualShock):
//     Left  Stick Y  → Tank-drive left  side (M1 + M3)
//     Right Stick Y  → Tank-drive right side (M2 + M4)
//     L2 trigger     → BTS Motor 1 forward  (proportional)
//     L1 button      → BTS Motor 1 reverse  (full speed)
//     R2 trigger     → BTS Motor 2 forward  (proportional)
//     R1 button      → BTS Motor 2 reverse  (full speed)
//     ✕ (Cross)      → Grip SPEAR (servos 0 + 1)
//     □ (Square)     → Grip STUFF (servos 2 + 3)
//     ○ (Circle)     → EMERGENCY STOP — all motors off
//     △ (Triangle)   → Reset all servos to 90°
// ═══════════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Bluepad32.h>

// ─────────────────────────────────────────────────────────────────────
//  I2C pins (PCA9685 servo driver)
// ─────────────────────────────────────────────────────────────────────
#define I2C_SDA 21
#define I2C_SCL 22

// ─────────────────────────────────────────────────────────────────────
//  PCA9685 — 16-channel servo driver
// ─────────────────────────────────────────────────────────────────────
#define PCA9685_ADDR  0x40
Adafruit_PWMServoDriver servoPWM = Adafruit_PWMServoDriver(PCA9685_ADDR);

#define SERVOMIN      150     // Pulse length for 0°
#define SERVOMAX      600     // Pulse length for 180°
#define NUM_SERVOS    16
#define DEFAULT_ANGLE 90

int  currentAngle[NUM_SERVOS];
bool pca9685Connected = false;

// Per-servo angle limits (channels 0-3 are grippers, 4-15 full range)
int servoMinAngle[NUM_SERVOS] = {
   70,  70,  70,  70,          // Ch 0-3
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  // Ch 4-15
};
int servoMaxAngle[NUM_SERVOS] = {
  110, 110, 110, 110,          // Ch 0-3
  180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180  // Ch 4-15
};

// ─────────────────────────────────────────────────────────────────────
//  Tank-Drive Motors (PWM + DIR, small H-bridges)
//
//  PWM=0 DIR=x → brake
//  PWM>0 DIR=0 → forward
//  PWM>0 DIR=1 → reverse
//
//  ROBOT TOP VIEW:
//    [M1 LF]───[M2 RF]
//       │           │
//    [M3 LR]───[M4 RR]
//
//  *** IMPORTANT: Change these pins to match YOUR wiring ***
// ─────────────────────────────────────────────────────────────────────
struct DriveMotor { uint8_t ledcChan, pwmPin, dirPin; };

static const DriveMotor DM[] = {
    {4,  4,  2},   // M1 — Left  Front (LEDC ch 4, PWM=GPIO4,  DIR=GPIO2)
    {5, 16,  17},  // M2 — Right Front (LEDC ch 5, PWM=GPIO16, DIR=GPIO17)
    {6, 18,  19},  // M3 — Left  Rear  (LEDC ch 6, PWM=GPIO18, DIR=GPIO19)
    {7, 23,   5},  // M4 — Right Rear  (LEDC ch 7, PWM=GPIO23, DIR=GPIO5)
};

#define DRIVE_PWM_FREQ  1000
#define DRIVE_PWM_BITS  8

// ─────────────────────────────────────────────────────────────────────
//  BTS7960 Motor 1 — (arm / mechanism motor 1)
//  Uses LEDC channels 0 + 1
// ─────────────────────────────────────────────────────────────────────
#define RPWM_PIN   26
#define LPWM_PIN   27
#define R_EN_PIN   14
#define L_EN_PIN   12

// ─────────────────────────────────────────────────────────────────────
//  BTS7960 Motor 2 — (arm / mechanism motor 2)
//  Uses LEDC channels 2 + 3
// ─────────────────────────────────────────────────────────────────────
#define RPWM2_PIN  25
#define LPWM2_PIN  33
#define R_EN2_PIN  32
#define L_EN2_PIN  13

#define BTS_PWM_FREQ  5000
#define BTS_PWM_RES   8
#define MOTOR_SPEED   255     // Full speed for serial commands
#define MOTOR_RUN_MS  1000    // Run duration for serial commands

// ─────────────────────────────────────────────────────────────────────
//  Bluepad32 — PS4 DualShock Controller
// ─────────────────────────────────────────────────────────────────────
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
#define JOYSTICK_DEADZONE 50
#define TRIGGER_DEADZONE  30

// ═══════════════════════════════════════════════════════════════════════
//  Utility functions
// ═══════════════════════════════════════════════════════════════════════

int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

bool isValidInt(const String &str) {
  if (str.length() == 0) return false;
  int start = 0;
  if (str.charAt(0) == '-' || str.charAt(0) == '+') {
    if (str.length() == 1) return false;
    start = 1;
  }
  for (unsigned int i = start; i < str.length(); i++) {
    if (!isDigit(str.charAt(i))) return false;
  }
  return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  Tank-Drive helpers
// ═══════════════════════════════════════════════════════════════════════

void driveMotor(const DriveMotor& m, float val) {
    if (fabsf(val) < 0.05f) { ledcWrite(m.ledcChan, 0); return; }
    digitalWrite(m.dirPin, val > 0 ? LOW : HIGH);
    ledcWrite(m.ledcChan, (uint8_t)(fabsf(val) * 255.0f));
}

void driveLeft(float v)  { driveMotor(DM[0], v); driveMotor(DM[2], v); }
void driveRight(float v) { driveMotor(DM[1], v); driveMotor(DM[3], v); }

void stopDriveAll() {
    for (auto& m : DM) ledcWrite(m.ledcChan, 0);
}

// ═══════════════════════════════════════════════════════════════════════
//  BTS7960 Motor helpers
// ═══════════════════════════════════════════════════════════════════════

void btsMotor1(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  if (forward) { ledcWrite(0, speed); ledcWrite(1, 0); }
  else         { ledcWrite(0, 0);     ledcWrite(1, speed); }
}

void stopBtsMotor1() { ledcWrite(0, 0); ledcWrite(1, 0); }

void btsMotor2(int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  if (forward) { ledcWrite(2, speed); ledcWrite(3, 0); }
  else         { ledcWrite(2, 0);     ledcWrite(3, speed); }
}

void stopBtsMotor2() { ledcWrite(2, 0); ledcWrite(3, 0); }

// Stop EVERYTHING
void emergencyStop() {
  stopDriveAll();
  stopBtsMotor1();
  stopBtsMotor2();
  Serial.println("!!! EMERGENCY STOP — all motors off !!!");
}

// ═══════════════════════════════════════════════════════════════════════
//  PCA9685 Servo / Gripper functions
// ═══════════════════════════════════════════════════════════════════════

void scanI2CBus() {
  Serial.println("\n=== I2C Bus Scan ===");
  int deviceCount = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  Device found at address 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      if (addr == PCA9685_ADDR) {
        Serial.print("  <-- PCA9685 Servo Driver");
        pca9685Connected = true;
      }
      Serial.println();
      deviceCount++;
    }
  }
  if (deviceCount == 0) Serial.println("  No I2C devices found! Check wiring.");
  else { Serial.print("  Total devices found: "); Serial.println(deviceCount); }
  if (!pca9685Connected) {
    Serial.print("  WARNING: PCA9685 not found at 0x");
    Serial.println(PCA9685_ADDR, HEX);
  }
  Serial.println();
}

void scanServos() {
  if (!pca9685Connected) {
    Serial.println("ERROR: PCA9685 not detected.");
    return;
  }
  Serial.println("\n=== Servo Channel Scan ===");
  Serial.println("Each channel will wiggle +/-20 from 90.\n");
  delay(1000);
  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    Serial.print("  Channel ");
    if (ch < 10) Serial.print(" ");
    Serial.print(ch);
    Serial.print(": wiggling... ");
    servoPWM.setPWM(ch, 0, angleToPulse(DEFAULT_ANGLE));      delay(250);
    servoPWM.setPWM(ch, 0, angleToPulse(DEFAULT_ANGLE + 20)); delay(250);
    servoPWM.setPWM(ch, 0, angleToPulse(DEFAULT_ANGLE - 20)); delay(250);
    servoPWM.setPWM(ch, 0, angleToPulse(DEFAULT_ANGLE));
    currentAngle[ch] = DEFAULT_ANGLE;
    delay(150);
    Serial.println("done");
  }
  Serial.println("\n=== Scan Complete ===\n");
}

void resetAllServos() {
  Serial.print("Resetting all servos to ");
  Serial.print(DEFAULT_ANGLE);
  Serial.println("...");
  int pulse = angleToPulse(DEFAULT_ANGLE);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoPWM.setPWM(i, 0, pulse);
    currentAngle[i] = DEFAULT_ANGLE;
    delay(30);
  }
  Serial.println("All servos reset to default.\n");
}

void moveServo(int n, int m) {
  if (n < 0 || n >= NUM_SERVOS) {
    Serial.print("ERROR: Servo "); Serial.print(n); Serial.println(" out of range (0-15).");
    return;
  }
  if (m < servoMinAngle[n] || m > servoMaxAngle[n]) {
    Serial.print("ERROR: Angle "); Serial.print(m);
    Serial.print(" out of range for servo "); Serial.print(n);
    Serial.print(". Valid: "); Serial.print(servoMinAngle[n]);
    Serial.print("-"); Serial.print(servoMaxAngle[n]); Serial.println();
    return;
  }
  int prev = currentAngle[n];
  servoPWM.setPWM(n, 0, angleToPulse(m));
  currentAngle[n] = m;
  Serial.print("Servo "); Serial.print(n);
  Serial.print(": "); Serial.print(prev);
  Serial.print(" -> "); Serial.print(m); Serial.println();
}

void controlServo(int n, int m) {
  if (n < 0 || n >= NUM_SERVOS) {
    Serial.print("ERROR: Servo "); Serial.print(n); Serial.println(" out of range (0-15).");
    return;
  }
  if (m < servoMinAngle[n] || m > servoMaxAngle[n]) {
    Serial.print("ERROR: Angle "); Serial.print(m);
    Serial.print(" out of range for servo "); Serial.print(n);
    Serial.print(". Valid: "); Serial.print(servoMinAngle[n]);
    Serial.print("-"); Serial.print(servoMaxAngle[n]); Serial.println();
    return;
  }
  resetAllServos();
  delay(300);
  servoPWM.setPWM(n, 0, angleToPulse(m));
  currentAngle[n] = m;
  Serial.print("Servo "); Serial.print(n);
  Serial.print(" moved to "); Serial.print(m); Serial.println("\n");
}

void showStatus() {
  Serial.println("\n=== Servo Status ===");
  Serial.println("  Channel | Angle  | Limits");
  Serial.println("  --------|--------|--------");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print("     ");
    if (i < 10) Serial.print(" ");
    Serial.print(i);
    Serial.print("    | ");
    if (currentAngle[i] < 110) Serial.print(" ");
    if (currentAngle[i] < 10)  Serial.print(" ");
    Serial.print(currentAngle[i]);
    Serial.print("  | ");
    Serial.print(servoMinAngle[i]);
    Serial.print("-");
    Serial.print(servoMaxAngle[i]);
    Serial.println();
  }
  Serial.println();
}

// ─── Gripper macros ─────────────────────────────────────────────────

void gripSpear() {
  Serial.println("\n=== SPEAR Gripper (Ch 0 + Ch 1) ===");
  Serial.println("  Releasing...");
  servoPWM.setPWM(0, 0, angleToPulse(110));
  servoPWM.setPWM(1, 0, angleToPulse(70));
  currentAngle[0] = 110;
  currentAngle[1] = 70;
  delay(2000);
  Serial.println("  Holding...");
  servoPWM.setPWM(0, 0, angleToPulse(70));
  servoPWM.setPWM(1, 0, angleToPulse(110));
  currentAngle[0] = 70;
  currentAngle[1] = 110;
  Serial.println("  Gripping. Done.\n");
}

void gripStuff() {
  Serial.println("\n=== STUFF Gripper (Ch 2 + Ch 3) ===");
  Serial.println("  Releasing...");
  servoPWM.setPWM(3, 0, angleToPulse(110));
  servoPWM.setPWM(2, 0, angleToPulse(70));
  currentAngle[3] = 110;
  currentAngle[2] = 70;
  delay(2000);
  Serial.println("  Holding...");
  servoPWM.setPWM(3, 0, angleToPulse(70));
  servoPWM.setPWM(2, 0, angleToPulse(110));
  currentAngle[3] = 70;
  currentAngle[2] = 110;
  Serial.println("  Gripping. Done.\n");
}

// ─── Serial motor commands (timed spin) ────────────────────────────

void spinLeft()   { Serial.println("\n=== BTS Motor 1: LEFT (fwd) ===");
                    btsMotor1(MOTOR_SPEED, true);  delay(MOTOR_RUN_MS); stopBtsMotor1();
                    Serial.println("  Stopped.\n"); }
void spinRight()  { Serial.println("\n=== BTS Motor 1: RIGHT (rev) ===");
                    btsMotor1(MOTOR_SPEED, false); delay(MOTOR_RUN_MS); stopBtsMotor1();
                    Serial.println("  Stopped.\n"); }
void spinLeft2()  { Serial.println("\n=== BTS Motor 2: LEFT2 (fwd) ===");
                    btsMotor2(MOTOR_SPEED, true);  delay(MOTOR_RUN_MS); stopBtsMotor2();
                    Serial.println("  Stopped.\n"); }
void spinRight2() { Serial.println("\n=== BTS Motor 2: RIGHT2 (rev) ===");
                    btsMotor2(MOTOR_SPEED, false); delay(MOTOR_RUN_MS); stopBtsMotor2();
                    Serial.println("  Stopped.\n"); }

// ═══════════════════════════════════════════════════════════════════════
//  Serial command parser
// ═══════════════════════════════════════════════════════════════════════

bool parseServoCommand(const String &input, int &n, int &m) {
  int commaIndex = input.indexOf(',');
  if (commaIndex <= 0 || commaIndex == (int)(input.length() - 1)) return false;
  String partN = input.substring(0, commaIndex);
  String partM = input.substring(commaIndex + 1);
  partN.trim(); partM.trim();
  if (!isValidInt(partN) || !isValidInt(partM)) return false;
  n = partN.toInt();
  m = partM.toInt();
  return true;
}

void printHelp() {
  Serial.println("\n=== Teleop Control — Commands ===");
  Serial.println("  n,m       Reset all to 90, then move servo n to m");
  Serial.println("  move n,m  Move only servo n to m (no reset)");
  Serial.println("  spear     Gripper 0+1: release then hold");
  Serial.println("  stuff     Gripper 2+3: release then hold");
  Serial.println("  left      BTS Motor 1: spin forward");
  Serial.println("  right     BTS Motor 1: spin reverse");
  Serial.println("  left2     BTS Motor 2: spin forward");
  Serial.println("  right2    BTS Motor 2: spin reverse");
  Serial.println("  reset     Reset all servos to 90");
  Serial.println("  scan      I2C bus scan + wiggle-test");
  Serial.println("  status    Show servo angles & limits");
  Serial.println("  stop      Emergency stop all motors");
  Serial.println("  help      Show this help message");
  Serial.println();
  Serial.println("  PS4 Gamepad:");
  Serial.println("    L-Stick Y  → Tank drive left  (M1+M3)");
  Serial.println("    R-Stick Y  → Tank drive right (M2+M4)");
  Serial.println("    L2 trigger → BTS Motor 1 fwd  (proportional)");
  Serial.println("    L1 button  → BTS Motor 1 rev  (full speed)");
  Serial.println("    R2 trigger → BTS Motor 2 fwd  (proportional)");
  Serial.println("    R1 button  → BTS Motor 2 rev  (full speed)");
  Serial.println("    Cross      → Grip SPEAR");
  Serial.println("    Square     → Grip STUFF");
  Serial.println("    Circle     → EMERGENCY STOP");
  Serial.println("    Triangle   → Reset servos");
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
//  Process PS4 Gamepad → All subsystems
// ═══════════════════════════════════════════════════════════════════════

void processGamepad(ControllerPtr ctl) {

  // ── Tank Drive: Left Stick Y → left side, Right Stick Y → right side ──
  int leftY  = ctl->axisY();      // -511 (up) to 512 (down)
  int rightY = ctl->axisRY();

  if (abs(leftY) > JOYSTICK_DEADZONE) {
    float speed = (float)(abs(leftY) - JOYSTICK_DEADZONE)
                / (float)(512 - JOYSTICK_DEADZONE);
    speed = constrain(speed, 0.0f, 1.0f);
    if (leftY < 0) speed = -speed;    // stick UP = forward (negative val)
    driveLeft(-speed);                 // invert: stick up → positive drive
  } else {
    driveLeft(0);
  }

  if (abs(rightY) > JOYSTICK_DEADZONE) {
    float speed = (float)(abs(rightY) - JOYSTICK_DEADZONE)
                / (float)(512 - JOYSTICK_DEADZONE);
    speed = constrain(speed, 0.0f, 1.0f);
    if (rightY < 0) speed = -speed;
    driveRight(speed);
  } else {
    driveRight(0);
  }

  // ── BTS Motor 1: L2 trigger (forward) / L1 button (reverse) ──
  int l2 = ctl->brake();      // 0-1023
  bool l1 = ctl->l1();

  if (l1) {
    btsMotor1(MOTOR_SPEED, false);              // L1 → full reverse
  } else if (l2 > TRIGGER_DEADZONE) {
    int speed = map(l2, TRIGGER_DEADZONE, 1023, 0, 255);
    speed = constrain(speed, 0, 255);
    btsMotor1(speed, true);                      // L2 → proportional forward
  } else {
    stopBtsMotor1();
  }

  // ── BTS Motor 2: R2 trigger (forward) / R1 button (reverse) ──
  int r2 = ctl->throttle();   // 0-1023
  bool r1 = ctl->r1();

  if (r1) {
    btsMotor2(MOTOR_SPEED, false);              // R1 → full reverse
  } else if (r2 > TRIGGER_DEADZONE) {
    int speed = map(r2, TRIGGER_DEADZONE, 1023, 0, 255);
    speed = constrain(speed, 0, 255);
    btsMotor2(speed, true);                      // R2 → proportional forward
  } else {
    stopBtsMotor2();
  }

  // ── Button Actions ──
  if (ctl->a()) {          // ✕ Cross → Grip SPEAR
    gripSpear();
  }
  if (ctl->x()) {          // □ Square → Grip STUFF
    gripStuff();
  }
  if (ctl->b()) {          // ○ Circle → EMERGENCY STOP
    emergencyStop();
  }
  if (ctl->y()) {          // △ Triangle → Reset servos
    resetAllServos();
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
  Serial.println("   Abu Robocon — Unified Teleop Controller");
  Serial.println("   PS4 DualShock  |  1 ESP32  |  All Systems");
  Serial.println("================================================");
  Serial.println();

  // ── Servo angle tracking ──
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i] = DEFAULT_ANGLE;
  }

  // ── I2C + PCA9685 ──
  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("FATAL: I2C init failed! Check SDA/SCL wiring.");
    while (true) { delay(1000); }
  }
  Serial.println("I2C initialized (SDA=" + String(I2C_SDA) + ", SCL=" + String(I2C_SCL) + ")");

  scanI2CBus();

  if (!pca9685Connected) {
    Serial.println("WARNING: PCA9685 not found. Servo commands may have no effect.\n");
  }

  servoPWM.begin();
  servoPWM.setPWMFreq(50);      // 50 Hz for standard servos
  Serial.println("PCA9685 initialized at 50 Hz.");

  // ── Tank-Drive Motors (PWM + DIR) ──
  for (auto& m : DM) {
    pinMode(m.dirPin, OUTPUT);
    digitalWrite(m.dirPin, LOW);
    ledcSetup(m.ledcChan, DRIVE_PWM_FREQ, DRIVE_PWM_BITS);
    ledcAttachPin(m.pwmPin, m.ledcChan);
    ledcWrite(m.ledcChan, 0);
  }
  Serial.println("Tank-drive motors initialized (4x PWM+DIR).");

  // ── BTS7960 Motor 1 ──
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  ledcSetup(0, BTS_PWM_FREQ, BTS_PWM_RES);   // Ch 0 → RPWM
  ledcSetup(1, BTS_PWM_FREQ, BTS_PWM_RES);   // Ch 1 → LPWM
  ledcAttachPin(RPWM_PIN, 0);
  ledcAttachPin(LPWM_PIN, 1);

  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  stopBtsMotor1();
  Serial.println("BTS7960 Motor 1 initialized (GPIO 26/27/14/12).");

  // ── BTS7960 Motor 2 ──
  pinMode(RPWM2_PIN, OUTPUT);
  pinMode(LPWM2_PIN, OUTPUT);
  pinMode(R_EN2_PIN, OUTPUT);
  pinMode(L_EN2_PIN, OUTPUT);

  ledcSetup(2, BTS_PWM_FREQ, BTS_PWM_RES);   // Ch 2 → RPWM2
  ledcSetup(3, BTS_PWM_FREQ, BTS_PWM_RES);   // Ch 3 → LPWM2
  ledcAttachPin(RPWM2_PIN, 2);
  ledcAttachPin(LPWM2_PIN, 3);

  digitalWrite(R_EN2_PIN, HIGH);
  digitalWrite(L_EN2_PIN, HIGH);
  stopBtsMotor2();
  Serial.println("BTS7960 Motor 2 initialized (GPIO 25/33/32/13).");

  // ── Reset servos ──
  resetAllServos();

  // ── Bluepad32 ──
  Serial.println("Initializing Bluepad32...");
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
    if (input.equalsIgnoreCase("scan"))   { scanI2CBus(); scanServos(); return; }
    if (input.equalsIgnoreCase("reset"))  { resetAllServos();  return; }
    if (input.equalsIgnoreCase("status")) { showStatus();      return; }
    if (input.equalsIgnoreCase("spear"))  { gripSpear();       return; }
    if (input.equalsIgnoreCase("stuff"))  { gripStuff();       return; }
    if (input.equalsIgnoreCase("left"))   { spinLeft();        return; }
    if (input.equalsIgnoreCase("right"))  { spinRight();       return; }
    if (input.equalsIgnoreCase("left2"))  { spinLeft2();       return; }
    if (input.equalsIgnoreCase("right2")) { spinRight2();      return; }
    if (input.equalsIgnoreCase("stop"))   { emergencyStop();   return; }

    if (input.startsWith("move ") || input.startsWith("MOVE ")) {
      String args = input.substring(5);
      args.trim();
      int n, m;
      if (parseServoCommand(args, n, m)) { moveServo(n, m); }
      else { Serial.println("ERROR: Invalid format. Use: move n,m"); }
      return;
    }

    {
      int n, m;
      if (parseServoCommand(input, n, m)) { controlServo(n, m); }
      else {
        Serial.println("Unknown command: '" + input + "'");
        Serial.println("Type 'help' for available commands.");
      }
    }
  }
}
