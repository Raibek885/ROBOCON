#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Bluepad32.h>

#define I2C_SDA 21
#define I2C_SCL 22

// ─── PCA9685 Configuration ──────────────────────────────────────────
#define PCA9685_ADDR 0x40   // Default I2C address (change if A0-A5 bridges are soldered)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// ─── Servo Parameters ───────────────────────────────────────────────
#define SERVOMIN  150   // Minimum pulse length for 0 degrees
#define SERVOMAX  600   // Maximum pulse length for 180 degrees
#define NUM_SERVOS 16   // PCA9685 supports up to 16 channels
#define DEFAULT_ANGLE 90 // Default position in degrees

// ─── BTS7960 Motor 1 Configuration ─────────────────────────────────
#define RPWM_PIN   26   // Motor 1: Right PWM (Forward)
#define LPWM_PIN   27   // Motor 1: Left PWM (Reverse)
#define R_EN_PIN   14   // Motor 1: Right Enable
#define L_EN_PIN   12   // Motor 1: Left Enable

// ─── BTS7960 Motor 2 Configuration ─────────────────────────────────
#define RPWM2_PIN  25   // Motor 2: Right PWM (Forward)
#define LPWM2_PIN  33   // Motor 2: Left PWM (Reverse)
#define R_EN2_PIN  32   // Motor 2: Right Enable
#define L_EN2_PIN  13   // Motor 2: Left Enable

// ─── Motor Common Settings ─────────────────────────────────────────
#define PWM_FREQ  5000  // 5 kHz PWM frequency
#define PWM_RES   8     // 8-bit resolution (0-255)
#define MOTOR_SPEED 255 // Full speed for left/right commands
#define MOTOR_RUN_MS 1000 // Run duration in milliseconds

// ─── State Tracking ─────────────────────────────────────────────────
int currentAngle[NUM_SERVOS];      // Current angle of each servo
bool pca9685Connected = false;     // Whether PCA9685 was detected on I2C

// ─── Bluepad32 Controller ──────────────────────────────────────────
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
#define JOYSTICK_DEADZONE 50       // Ignore small stick movements

// ─── Per-Servo Angle Limits ─────────────────────────────────────────
// Limits for servos 0-3 (gripper motors), servos 4-15 use full range
int servoMinAngle[NUM_SERVOS] = {
   70,   // Channel 0:  70-110
   70,   // Channel 1:  70-110
   70,   // Channel 2:  70-110
   70,   // Channel 3:  70-110
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  // Channels 4-15: 0-180
};
int servoMaxAngle[NUM_SERVOS] = {
  110,   // Channel 0:  70-110
  110,   // Channel 1:  70-110
  110,   // Channel 2:  70-110
  110,   // Channel 3:  70-110
  180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180  // Channels 4-15: 0-180
};

// ─────────────────────────────────────────────────────────────────────
// Utility: Convert degrees to PWM pulse length
// ─────────────────────────────────────────────────────────────────────
int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// ─────────────────────────────────────────────────────────────────────
// Utility: Check if a string is a valid integer
// ─────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────
// I2C Bus Scan: Detect all devices on the I2C bus
// ─────────────────────────────────────────────────────────────────────
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

      // Identify known devices
      if (addr == PCA9685_ADDR) {
        Serial.print("  <-- PCA9685 Servo Driver");
        pca9685Connected = true;
      }
      Serial.println();
      deviceCount++;
    }
  }

  if (deviceCount == 0) {
    Serial.println("  No I2C devices found! Check wiring.");
  } else {
    Serial.print("  Total devices found: ");
    Serial.println(deviceCount);
  }

  if (!pca9685Connected) {
    Serial.print("  WARNING: PCA9685 not found at address 0x");
    if (PCA9685_ADDR < 16) Serial.print("0");
    Serial.println(PCA9685_ADDR, HEX);
  }
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────
// Servo Scan: Wiggle-test each channel so you can visually identify
// which channels have servos connected.
// ─────────────────────────────────────────────────────────────────────
void scanServos() {
  if (!pca9685Connected) {
    Serial.println("ERROR: PCA9685 not detected. Run I2C scan first.");
    return;
  }

  Serial.println("\n=== Servo Channel Scan ===");
  Serial.println("Each channel will wiggle ±20° from 90°.");
  Serial.println("Watch your servos to see which ones move.\n");
  delay(1000);

  for (int ch = 0; ch < NUM_SERVOS; ch++) {
    Serial.print("  Channel ");
    if (ch < 10) Serial.print(" ");
    Serial.print(ch);
    Serial.print(": wiggling... ");

    // Move to center
    pwm.setPWM(ch, 0, angleToPulse(DEFAULT_ANGLE));
    delay(250);

    // Wiggle: +20°
    pwm.setPWM(ch, 0, angleToPulse(DEFAULT_ANGLE + 20));
    delay(250);

    // Wiggle: -20°
    pwm.setPWM(ch, 0, angleToPulse(DEFAULT_ANGLE - 20));
    delay(250);

    // Return to center and record position
    pwm.setPWM(ch, 0, angleToPulse(DEFAULT_ANGLE));
    currentAngle[ch] = DEFAULT_ANGLE;
    delay(150);

    Serial.println("done");
  }

  Serial.println("\n=== Scan Complete ===");
  Serial.println("Note which servos physically moved — those are your connected channels.\n");
}

// ─────────────────────────────────────────────────────────────────────
// Reset all servos to default angle (90°)
// ─────────────────────────────────────────────────────────────────────
void resetAllServos() {
  Serial.print("Resetting all servos to ");
  Serial.print(DEFAULT_ANGLE);
  Serial.println("°...");

  int pulse = angleToPulse(DEFAULT_ANGLE);
  for (int i = 0; i < NUM_SERVOS; i++) {
    pwm.setPWM(i, 0, pulse);
    currentAngle[i] = DEFAULT_ANGLE;
    delay(30);
  }

  Serial.println("All servos reset to default.\n");
}

// ─────────────────────────────────────────────────────────────────────
// Move a single servo to a target angle (no reset)
// ─────────────────────────────────────────────────────────────────────
void moveServo(int n, int m) {
  if (n < 0 || n >= NUM_SERVOS) {
    Serial.print("ERROR: Servo number ");
    Serial.print(n);
    Serial.println(" out of range. Valid: 0-15.");
    return;
  }
  if (m < servoMinAngle[n] || m > servoMaxAngle[n]) {
    Serial.print("ERROR: Angle ");
    Serial.print(m);
    Serial.print("° out of range for servo ");
    Serial.print(n);
    Serial.print(". Valid: ");
    Serial.print(servoMinAngle[n]);
    Serial.print("-");
    Serial.print(servoMaxAngle[n]);
    Serial.println("°");
    return;
  }

  int prevAngle = currentAngle[n];
  pwm.setPWM(n, 0, angleToPulse(m));
  currentAngle[n] = m;

  Serial.print("Servo ");
  Serial.print(n);
  Serial.print(": ");
  Serial.print(prevAngle);
  Serial.print("° -> ");
  Serial.print(m);
  Serial.println("°");
}

// ─────────────────────────────────────────────────────────────────────
// Control servo: reset all to default, then move target servo
// ─────────────────────────────────────────────────────────────────────
void controlServo(int n, int m) {
  if (n < 0 || n >= NUM_SERVOS) {
    Serial.print("ERROR: Servo number ");
    Serial.print(n);
    Serial.println(" out of range. Valid: 0-15.");
    return;
  }
  if (m < servoMinAngle[n] || m > servoMaxAngle[n]) {
    Serial.print("ERROR: Angle ");
    Serial.print(m);
    Serial.print("° out of range for servo ");
    Serial.print(n);
    Serial.print(". Valid: ");
    Serial.print(servoMinAngle[n]);
    Serial.print("-");
    Serial.print(servoMaxAngle[n]);
    Serial.println("°");
    return;
  }

  // Reset all servos first
  resetAllServos();
  delay(300);

  // Then move the target servo
  pwm.setPWM(n, 0, angleToPulse(m));
  currentAngle[n] = m;

  Serial.print("Servo ");
  Serial.print(n);
  Serial.print(" moved to ");
  Serial.print(m);
  Serial.println("°\n");
}

// ─────────────────────────────────────────────────────────────────────
// Show current position of all servos
// ─────────────────────────────────────────────────────────────────────
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
    if (currentAngle[i] < 10) Serial.print(" ");
    Serial.print(currentAngle[i]);
    Serial.print("°  | ");
    Serial.print(servoMinAngle[i]);
    Serial.print("-");
    Serial.print(servoMaxAngle[i]);
    Serial.println("°");
  }
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────
// Gripper: SPEAR (servos 0 + 1)
//   Release: servo 0 → 120°, servo 1 → 70°   (open)
//   Hold:    servo 0 → 80°,  servo 1 → 110°   (close & grip)
// ─────────────────────────────────────────────────────────────────────
void gripSpear() {
  Serial.println("\n=== SPEAR Gripper (Ch 0 + Ch 1) ===");

  // ── Release (open) ──
  Serial.println("  Releasing...");
  pwm.setPWM(0, 0, angleToPulse(110));
  pwm.setPWM(1, 0, angleToPulse(70));
  currentAngle[0] = 110;
  currentAngle[1] = 70;
  delay(2000);

  // ── Hold (close & stay gripping) ──
  Serial.println("  Holding...");
  pwm.setPWM(0, 0, angleToPulse(70));
  pwm.setPWM(1, 0, angleToPulse(110));
  currentAngle[0] = 70;
  currentAngle[1] = 110;

  Serial.println("  Gripping. Done.\n");
}

// ─────────────────────────────────────────────────────────────────────
// Gripper: STUFF (servos 2 + 3)
//   Release: servo 3 → 120°, servo 2 → 60°    (open)
//   Hold:    servo 3 → 80°,  servo 2 → 100°   (close & grip)
// ─────────────────────────────────────────────────────────────────────
void gripStuff() {
  Serial.println("\n=== STUFF Gripper (Ch 2 + Ch 3) ===");

  // ── Release (open) ──
  Serial.println("  Releasing...");
  pwm.setPWM(3, 0, angleToPulse(110));
  pwm.setPWM(2, 0, angleToPulse(70));
  currentAngle[3] = 110;
  currentAngle[2] = 70;
  delay(2000);

  // ── Hold (close & stay gripping) ──
  Serial.println("  Holding...");
  pwm.setPWM(3, 0, angleToPulse(70));
  pwm.setPWM(2, 0, angleToPulse(110));
  currentAngle[3] = 70;
  currentAngle[2] = 110;

  Serial.println("  Gripping. Done.\n");
}

// ─────────────────────────────────────────────────────────────────────
// BTS7960 Motor 1: spin in a direction at given speed
// Uses LEDC channels 0 + 1
// ─────────────────────────────────────────────────────────────────────
void motorControl(int speed, bool forward) {
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;

  if (forward) {
    ledcWrite(0, speed);  // RPWM
    ledcWrite(1, 0);      // LPWM
  } else {
    ledcWrite(0, 0);      // RPWM
    ledcWrite(1, speed);  // LPWM
  }
}

void stopMotor() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

// ─────────────────────────────────────────────────────────────────────
// BTS7960 Motor 2: spin in a direction at given speed
// Uses LEDC channels 2 + 3
// ─────────────────────────────────────────────────────────────────────
void motor2Control(int speed, bool forward) {
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;

  if (forward) {
    ledcWrite(2, speed);  // RPWM2
    ledcWrite(3, 0);      // LPWM2
  } else {
    ledcWrite(2, 0);      // RPWM2
    ledcWrite(3, speed);  // LPWM2
  }
}

void stopMotor2() {
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

// ─────────────────────────────────────────────────────────────────────
// Motor 1 Commands: LEFT / RIGHT
// ─────────────────────────────────────────────────────────────────────
void spinLeft() {
  Serial.println("\n=== Motor 1: LEFT (forward 2s) ===");
  Serial.println("  Spinning...");
  motorControl(MOTOR_SPEED, true);
  delay(MOTOR_RUN_MS);
  stopMotor();
  Serial.println("  Stopped.\n");
}

void spinRight() {
  Serial.println("\n=== Motor 1: RIGHT (reverse 2s) ===");
  Serial.println("  Spinning...");
  motorControl(MOTOR_SPEED, false);
  delay(MOTOR_RUN_MS);
  stopMotor();
  Serial.println("  Stopped.\n");
}

// ─────────────────────────────────────────────────────────────────────
// Motor 2 Commands: LEFT2 / RIGHT2
// ─────────────────────────────────────────────────────────────────────
void spinLeft2() {
  Serial.println("\n=== Motor 2: LEFT2 (forward 2s) ===");
  Serial.println("  Spinning...");
  motor2Control(MOTOR_SPEED, true);
  delay(MOTOR_RUN_MS);
  stopMotor2();
  Serial.println("  Stopped.\n");
}

void spinRight2() {
  Serial.println("\n=== Motor 2: RIGHT2 (reverse 2s) ===");
  Serial.println("  Spinning...");
  motor2Control(MOTOR_SPEED, false);
  delay(MOTOR_RUN_MS);
  stopMotor2();
  Serial.println("  Stopped.\n");
}

// ─────────────────────────────────────────────────────────────────────
// Print help text
// ─────────────────────────────────────────────────────────────────────
void printHelp() {
  Serial.println("\n=== Gripper Control — Commands ===");
  Serial.println("  n,m       Reset all to 90°, then move servo n to m°");
  Serial.println("  move n,m  Move only servo n to m° (no reset)");
  Serial.println("  spear     Gripper 0+1: release then hold");
  Serial.println("  stuff     Gripper 2+3: release then hold");
  Serial.println("  left      Motor 1: spin forward for 2s");
  Serial.println("  right     Motor 1: spin reverse for 2s");
  Serial.println("  left2     Motor 2: spin forward for 2s");
  Serial.println("  right2    Motor 2: spin reverse for 2s");
  Serial.println("  reset     Reset all servos to 90°");
  Serial.println("  scan      I2C bus scan + wiggle-test all channels");
  Serial.println("  status    Show current angle of all servos & limits");
  Serial.println("  help      Show this help message");
  Serial.println();
  Serial.println("  n = servo channel (0-15)");
  Serial.println("  Angle limits per servo:");
  Serial.println("    Ch 0: 70-120°  |  Ch 1: 70-110°");
  Serial.println("    Ch 2: 70-100°  |  Ch 3: 70-120°");
  Serial.println("    Ch 4-15: 0-180° (full range)");
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────
// Parse "n,m" from a string. Returns true on success.
// ─────────────────────────────────────────────────────────────────────
bool parseServoCommand(const String &input, int &n, int &m) {
  int commaIndex = input.indexOf(',');
  if (commaIndex <= 0 || commaIndex == (int)(input.length() - 1)) {
    return false;
  }

  String partN = input.substring(0, commaIndex);
  String partM = input.substring(commaIndex + 1);
  partN.trim();
  partM.trim();

  if (!isValidInt(partN) || !isValidInt(partM)) {
    return false;
  }

  n = partN.toInt();
  m = partM.toInt();
  return true;
}

// ─────────────────────────────────────────────────────────────────────
// Bluepad32 Callbacks
// ─────────────────────────────────────────────────────────────────────
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
      stopMotor();
      stopMotor2();
      return;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────
// Process PS4 Gamepad → Motor Control
//   Left stick Y  → Motor 1 (forward/reverse, proportional speed)
//   Right stick Y → Motor 2 (forward/reverse, proportional speed)
//   ✕ (Cross)     → Grip SPEAR
//   □ (Square)    → Grip STUFF
//   ○ (Circle)    → Emergency stop all motors
//   △ (Triangle)  → Reset all servos
// ─────────────────────────────────────────────────────────────────────
void processGamepad(ControllerPtr ctl) {
  // ── Left Stick Y → Motor 1 ──
  int leftY = ctl->axisY();  // -511 (up) to 512 (down)

  if (abs(leftY) > JOYSTICK_DEADZONE) {
    int speed = map(abs(leftY), JOYSTICK_DEADZONE, 512, 0, 255);
    speed = constrain(speed, 0, 255);
    bool forward = (leftY < 0);  // stick UP = forward
    motorControl(speed, forward);
  } else {
    stopMotor();
  }

  // ── Right Stick Y → Motor 2 ──
  int rightY = ctl->axisRY();

  if (abs(rightY) > JOYSTICK_DEADZONE) {
    int speed = map(abs(rightY), JOYSTICK_DEADZONE, 512, 0, 255);
    speed = constrain(speed, 0, 255);
    bool forward = (rightY < 0);
    motor2Control(speed, forward);
  } else {
    stopMotor2();
  }

  // ── Button Actions ──
  if (ctl->a()) {          // ✕ Cross → Grip SPEAR
    gripSpear();
  }
  if (ctl->x()) {          // □ Square → Grip STUFF
    gripStuff();
  }
  if (ctl->b()) {          // ○ Circle → STOP all motors
    stopMotor();
    stopMotor2();
    Serial.println("EMERGENCY STOP");
  }
  if (ctl->y()) {          // △ Triangle → Reset servos
    resetAllServos();
  }
}

// ═════════════════════════════════════════════════════════════════════
// SETUP
// ═════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   Abu Robocon Gripper Control System   ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();

  // Initialize angle tracking
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i] = DEFAULT_ANGLE;
  }

  // Initialize I2C
  if (!Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("FATAL: I2C initialization failed! Check SDA/SCL wiring.");
    while (true) { delay(1000); }
  }
  Serial.println("I2C initialized (SDA=" + String(I2C_SDA) + ", SCL=" + String(I2C_SCL) + ")");

  // Scan I2C bus to detect PCA9685
  scanI2CBus();

  if (!pca9685Connected) {
    Serial.println("WARNING: PCA9685 not found. Commands will be sent but may have no effect.");
    Serial.println("Check wiring and I2C address.\n");
  }

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Hz for standard servos
  Serial.println("PCA9685 driver initialized at 50 Hz.");

  // Initialize BTS7960 motor driver
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  ledcSetup(0, PWM_FREQ, PWM_RES);  // Channel 0 for Motor 1 RPWM
  ledcSetup(1, PWM_FREQ, PWM_RES);  // Channel 1 for Motor 1 LPWM
  ledcAttachPin(RPWM_PIN, 0);
  ledcAttachPin(LPWM_PIN, 1);

  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  stopMotor();
  Serial.println("BTS7960 Motor 1 initialized (GPIO 26/27/14/12).");

  // Initialize BTS7960 Motor 2
  pinMode(RPWM2_PIN, OUTPUT);
  pinMode(LPWM2_PIN, OUTPUT);
  pinMode(R_EN2_PIN, OUTPUT);
  pinMode(L_EN2_PIN, OUTPUT);

  ledcSetup(2, PWM_FREQ, PWM_RES);  // Channel 2 for Motor 2 RPWM
  ledcSetup(3, PWM_FREQ, PWM_RES);  // Channel 3 for Motor 2 LPWM
  ledcAttachPin(RPWM2_PIN, 2);
  ledcAttachPin(LPWM2_PIN, 3);

  digitalWrite(R_EN2_PIN, HIGH);
  digitalWrite(L_EN2_PIN, HIGH);
  stopMotor2();
  Serial.println("BTS7960 Motor 2 initialized (GPIO 25/33/32/13).");

  // Set all servos to default position
  resetAllServos();

  // Show help
  printHelp();

  // ─── Initialize Bluepad32 ──────────────────────────────────────────
  Serial.println("Initializing Bluepad32...");
  BP32.setup(&onConnectedController, &onDisconnectedController);
  // BP32.forgetBluetoothKeys();  // Uncomment to clear old pairings
  Serial.println("Bluepad32 ready! Hold SHARE+PS on your PS4 controller to pair.\n");
}

// ═════════════════════════════════════════════════════════════════════
// LOOP
// ═════════════════════════════════════════════════════════════════════
void loop() {
  // ── Process Bluepad32 controllers ──
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

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    // ── Command: help ──
    if (input.equalsIgnoreCase("help")) {
      printHelp();
      return;
    }

    // ── Command: scan ──
    if (input.equalsIgnoreCase("scan")) {
      scanI2CBus();
      scanServos();
      return;
    }

    // ── Command: reset ──
    if (input.equalsIgnoreCase("reset")) {
      resetAllServos();
      return;
    }

    // ── Command: status ──
    if (input.equalsIgnoreCase("status")) {
      showStatus();
      return;
    }

    // ── Command: spear (gripper 0+1) ──
    if (input.equalsIgnoreCase("spear")) {
      gripSpear();
      return;
    }

    // ── Command: stuff (gripper 2+3) ──
    if (input.equalsIgnoreCase("stuff")) {
      gripStuff();
      return;
    }

    // ── Command: left (Motor 1 forward 2s) ──
    if (input.equalsIgnoreCase("left")) {
      spinLeft();
      return;
    }

    // ── Command: right (Motor 1 reverse 2s) ──
    if (input.equalsIgnoreCase("right")) {
      spinRight();
      return;
    }

    // ── Command: left2 (Motor 2 forward 2s) ──
    if (input.equalsIgnoreCase("left2")) {
      spinLeft2();
      return;
    }

    // ── Command: right2 (Motor 2 reverse 2s) ──
    if (input.equalsIgnoreCase("right2")) {
      spinRight2();
      return;
    }

    // ── Command: move n,m (single servo, no reset) ──
    if (input.startsWith("move ") || input.startsWith("MOVE ")) {
      String args = input.substring(5);
      args.trim();
      int n, m;
      if (parseServoCommand(args, n, m)) {
        moveServo(n, m);
      } else {
        Serial.println("ERROR: Invalid format. Use: move n,m (example: move 1,45)");
      }
      return;
    }

    // ── Command: n,m (reset all, then move) ──
    {
      int n, m;
      if (parseServoCommand(input, n, m)) {
        controlServo(n, m);
      } else {
        Serial.println("Unknown command: '" + input + "'");
        Serial.println("Type 'help' for available commands.");
      }
    }
  }
}