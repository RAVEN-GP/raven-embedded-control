#include <Arduino_LSM6DSOX.h>
#include <Servo.h>
#include <math.h>

// ===================== USER CONFIG =====================
const float WHEEL_DIAMETER_M = 0.06055f;
const float WHEEL_CIRC_M     = 3.1415926f * WHEEL_DIAMETER_M;

// Encoder
const int ENC_A = 14;   // CHANGE to your interrupt-capable pin
const int ENC_B = 15;   // CHANGE to your pin
const long ENCODER_CPR = 2230; // <<< CHANGE (counts per wheel revolution)

// Motor driver (L298)
const int IN1 = 7;
const int IN2 = 8;
const int EN  = 9;   // PWM pin

// Steering servo
const int SERVO_PIN = 11;
const int SERVO_CENTER = 90;
const int SERVO_MIN = 45;
const int SERVO_MAX = 135;

// --- REVERSALS (you asked for both) ---
const bool STEER_REVERSED = true;
const bool MOTOR_REVERSED = true;

// Control loop
const int CONTROL_HZ = 50;  // decision rate

// Speed control
float KP_SPEED = 120.0f;      // tune
int PWM_FF = 90;              // feedforward baseline (tune)
int PWM_MIN = 0;
int PWM_MAX = 255;

// Heading control
float KP_HEADING = 2.2f;      // tune (deg -> servo degrees)
float TURN_KP = 2.0f;         // tune for turn accuracy
float TURN_RATE_LIMIT_PWM = 160; // keep turning not too fast

// Motion targets
const float TARGET_SPEED_MS = 0.35f;  // tune

// Debug (low-rate printing to avoid performance hit)
const bool DEBUG_ON = true;
const unsigned long DEBUG_PERIOD_MS = 200; // 5 Hz prints

// ===================== IMU CALIB KNOB =====================
// If real 90° turn reads as 100° on serial -> set to 0.90
// If real 90° turn reads as 80° on serial  -> set to 1.125
float YAW_SCALE = 1.20f;
// =======================================================

// ===================== GLOBALS =====================
Servo steer;

volatile long encTicks = 0;
volatile int lastA = 0;

long lastEncTicks = 0;

float gyroBiasZ = 0.0f;
float yawDeg = 0.0f;

// REAL dt for yaw integration
unsigned long lastYawUs = 0;

unsigned long lastLoopMs = 0;
unsigned long lastDebugMs = 0;

// Debug snapshots (updated in loop, printed low-rate)
float dbg_vMeas = 0;
int   dbg_pwm = 0;
float dbg_headingTarget = 0;
float dbg_headingErr = 0;
float dbg_traveled = 0;
float dbg_remaining = 0;

// =======================================================
// =============== SERIAL COMMAND LAYER ===================
// =======================================================

// Old commands (unchanged):
// #speed:X;;   -> -50..50
// #steer:A;;   -> -25..25
// #brake:0;;   -> stop immediately
// #imu:1/0;;   -> enable/disable IMU telemetry

// New commands:
// #drive:M;;   -> drive straight M meters (AUTO)
// #turn:D;;    -> turn relative D degrees (AUTO)
// #auto:0;;    -> cancel auto

volatile int  cmdSpeed = 0;         // -50..50
volatile int  cmdSteer = 0;         // -25..25
volatile bool cmdBrake = false;     // true => immediate stop
volatile bool imuEnabled = false;   // telemetry + yaw updates

String rxLine = "";                 // accumulates serial data until ";;"

// Map speed command to m/s target (keeps your working speed controller)
float speedCmdToTargetMs(int s) {
  const float MAX_MS = 0.50f; // tune
  float mag = fabs((float)s) / 50.0f;
  return mag * MAX_MS;
}

void sendImuTelemetryMaybe() {
  if (!imuEnabled) return;
  Serial.print("@imu:yaw=");
  Serial.print(yawDeg, 2);
  Serial.print(",biasZ=");
  Serial.print(gyroBiasZ, 6);
  Serial.println(";;");
}

// =======================================================
// ===================== AUTO MODE ========================
// =======================================================

enum AutoState {
  AUTO_NONE = 0,
  AUTO_DRIVE_STRAIGHT,
  AUTO_TURN_REL
};

volatile AutoState autoState = AUTO_NONE;

// Drive-straight state
float auto_driveMeters = 0.0f;
long  auto_startTicks = 0;
float auto_headingHold = 0.0f;

// Turn state
float auto_turnTargetYaw = 0.0f;
int   auto_turnSteerCmd = SERVO_CENTER;

void autoCancel() {
  autoState = AUTO_NONE;
  stopMotor();
  setSteerDeg(SERVO_CENTER);
}

// Start driving straight for meters holding current yaw
void autoStartDrive(float meters) {
  if (meters <= 0.0f) return;

  auto_headingHold = yawDeg;

  noInterrupts();
  auto_startTicks = encTicks;
  interrupts();

  auto_driveMeters = meters;
  autoState = AUTO_DRIVE_STRAIGHT;
}

// Start turning relative degrees
void autoStartTurn(float deltaDeg) {
  auto_turnTargetYaw = wrapAngleDeg(yawDeg + deltaDeg);
  auto_turnSteerCmd = (deltaDeg >= 0) ? SERVO_MAX : SERVO_MIN;
  setSteerDeg(auto_turnSteerCmd);
  autoState = AUTO_TURN_REL;
}
// ===================== MOTOR & STEER =====================
void setMotor(int pwm, int dir /*+1 forward, -1 backward*/) {
  pwm = clampi(pwm, 0, 255);

  // Reverse motor direction if requested
  if (MOTOR_REVERSED) dir = -dir;

  if (dir >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(EN, pwm);
}

// One control tick for auto mode (NON-BLOCKING)
void autoTick() {
  // Safety: brake cancels auto
  if (cmdBrake) {
    cmdBrake = false;
    autoCancel();
    return;
  }

  // Update yaw only if IMU enabled (your choice). Turning NEEDS yaw.
  // If you want turning to work regardless of #imu, change to: updateYaw();
  if (imuEnabled) updateYaw();

  if (autoState == AUTO_DRIVE_STRAIGHT) {
    // Distance traveled
    long currentTicks;
    noInterrupts();
    currentTicks = encTicks;
    interrupts();

    float traveled = ticksToMeters(currentTicks - auto_startTicks);
    float remaining = auto_driveMeters - traveled;

    if (remaining <= 0.0f) {
      autoCancel();
      return;
    }

    // Slow down near end (same idea as your working function)
    float vTarget = TARGET_SPEED_MS;
    if (remaining < 0.25f) vTarget = 0.20f;
    if (remaining < 0.10f) vTarget = 0.12f;
    if (remaining < 0.04f) vTarget = 0.08f;

    float vMeas = measuredSpeedMs();
    int pwm = speedPwmFromTarget(vTarget, vMeas);

    // Hold heading
    float headingErr = wrapAngleDeg(auto_headingHold - yawDeg);
    int steerCmd = steerFromHeadingTarget(auto_headingHold);

    setSteerDeg(steerCmd);
    setMotor(pwm, +1);

    dbg_vMeas = vMeas;
    dbg_pwm = pwm;
    dbg_headingTarget = auto_headingHold;
    dbg_headingErr = headingErr;
    dbg_traveled = traveled;
    dbg_remaining = remaining;
    maybeDebugPrint();
    sendImuTelemetryMaybe();
    return;
  }

  if (autoState == AUTO_TURN_REL) {
    float err = wrapAngleDeg(auto_turnTargetYaw - yawDeg);

    if (fabs(err) < 2.0f) {
      autoCancel();
      return;
    }

    // Keep steering hard in chosen direction
    setSteerDeg(auto_turnSteerCmd);

    int pwm = (int)(TURN_KP * fabs(err) + 60);
    pwm = clampi(pwm, 60, (int)TURN_RATE_LIMIT_PWM);

    setMotor(pwm, +1);

    dbg_vMeas = measuredSpeedMs();
    dbg_pwm = pwm;
    dbg_headingTarget = auto_turnTargetYaw;
    dbg_headingErr = err;
    dbg_traveled = 0;
    dbg_remaining = 0;
    maybeDebugPrint();
    sendImuTelemetryMaybe();
    return;
  }

  // If none, do nothing here
}

// =======================================================
// ===================== PARSER ============================
// =======================================================

void handleCommand(const String &cmd) {
  if (!cmd.startsWith("#")) return;

  int colon = cmd.indexOf(':');
  if (colon < 0) return;

  String key = cmd.substring(1, colon);
  String valStr = cmd.substring(colon + 1);
  valStr.trim();

  // IMPORTANT: keep old behavior for existing commands
  int   valInt   = valStr.toInt();
  float valFloat = valStr.toFloat();

  if (key == "speed") {
    cmdSpeed = clampi(valInt, -50, 50);
    if (cmdSpeed == 0) cmdBrake = true; // per your table
  }
  else if (key == "steer") {
    cmdSteer = clampi(valInt, -25, 25);
  }
  else if (key == "brake") {
    cmdBrake = true;
  }
  else if (key == "imu") {
    imuEnabled = (valInt != 0);
  }

  // ===== NEW AUTO COMMANDS =====
  else if (key == "drive") {
    // meters, ex: #drive:0.50;;
    autoStartDrive(valFloat);
  }
  else if (key == "turn") {
    // degrees, ex: #turn:90;;
    autoStartTurn(valFloat);
  }
  else if (key == "auto") {
    // #auto:0;; cancels
    if (valInt == 0) autoCancel();
  }
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    rxLine += c;

    int idx = rxLine.indexOf(";;");
    while (idx >= 0) {
      String one = rxLine.substring(0, idx);
      rxLine = rxLine.substring(idx + 2);

      one.trim();
      if (one.length() > 0) handleCommand(one);

      idx = rxLine.indexOf(";;");
    }

    if (rxLine.length() > 200) rxLine = "";
  }
}

// ===================== ENCODER ISR (quadrature) =====================
void ISR_encA() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  if (a != lastA) {
    if (a == b) encTicks++;
    else encTicks--;
    lastA = a;
  }
}

// ===================== UTILS =====================
float ticksToMeters(long ticks) {
  return ( (float)ticks / (float)ENCODER_CPR ) * WHEEL_CIRC_M;
}

int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float wrapAngleDeg(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

// ===================== IMU =====================
void calibrateGyroBias() {
  Serial.println("Calibrating gyro bias... KEEP STILL (3s)");

  const int N = 600; // 600 * 5ms = 3s
  float sum = 0;
  int count = 0;

  for (int i = 0; i < N; i++) {
    float gx, gy, gz;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);

      // reject spikes / vibration bumps
      if (fabs(gz) < 5.0f) {
        sum += gz;
        count++;
      }
    }
    delay(5);
  }

  gyroBiasZ = (count > 0) ? (sum / count) : 0.0f;
}

void updateYaw() {
  if (!IMU.gyroscopeAvailable()) return;

  unsigned long nowUs = micros();
  if (lastYawUs == 0) lastYawUs = nowUs;
  float dt = (nowUs - lastYawUs) * 1e-6f; // seconds
  lastYawUs = nowUs;

  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);

  float wz = gz - gyroBiasZ; // deg/s
  yawDeg += (wz * YAW_SCALE) * dt;
  yawDeg = wrapAngleDeg(yawDeg);
}


void stopMotor() {
  analogWrite(EN, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void setSteerDeg(int deg) {
  deg = clampi(deg, SERVO_MIN, SERVO_MAX);

  // Reverse steering around center if requested
  if (STEER_REVERSED) {
    deg = SERVO_CENTER - (deg - SERVO_CENTER);
  }

  deg = clampi(deg, SERVO_MIN, SERVO_MAX);
  steer.write(deg);
}

// ===================== CONTROL LOOPS =====================
float measuredSpeedMs() {
  long nowTicks;
  noInterrupts();
  nowTicks = encTicks;
  interrupts();

  long dTicks = nowTicks - lastEncTicks;
  lastEncTicks = nowTicks;

  float ticksPerSec = (float)dTicks * CONTROL_HZ;
  float ms = (ticksPerSec / (float)ENCODER_CPR) * WHEEL_CIRC_M;
  return ms;
}

int speedPwmFromTarget(float vTarget, float vMeas) {
  float err = vTarget - vMeas;
  int pwm = (int)(PWM_FF + KP_SPEED * err);
  return clampi(pwm, PWM_MIN, PWM_MAX);
}

int steerFromHeadingTarget(float headingTargetDeg) {
  float err = wrapAngleDeg(headingTargetDeg - yawDeg);
  int steerCmd = (int)(SERVO_CENTER + KP_HEADING * err);
  return clampi(steerCmd, SERVO_MIN, SERVO_MAX);
}

// ===================== DEBUG PRINT (LOW RATE) =====================
void maybeDebugPrint() {
  if (!DEBUG_ON) return;
  unsigned long now = millis();
  if (now - lastDebugMs < DEBUG_PERIOD_MS) return;
  lastDebugMs = now;

  Serial.print("yaw=");
  Serial.print(yawDeg, 2);
  Serial.print(" deg | herr=");
  Serial.print(dbg_headingErr, 2);
  Serial.print(" deg | v=");
  Serial.print(dbg_vMeas, 3);
  Serial.print(" m/s | pwm=");
  Serial.print(dbg_pwm);
  Serial.print(" | dist=");
  Serial.print(dbg_traveled, 3);
  Serial.print(" m | rem=");
  Serial.print(dbg_remaining, 3);
  Serial.println(" m");
}

// ===================== MOTION PRIMITIVES (UNCHANGED) =====================
// (kept exactly as your working code; still available if you want them)

void driveStraightMeters(float meters) {
  float headingHold = yawDeg;

  long startTicks;
  noInterrupts();
  startTicks = encTicks;
  interrupts();

  while (true) {
    unsigned long now = millis();
    if (now - lastLoopMs < (1000 / CONTROL_HZ)) continue;
    lastLoopMs = now;

    updateYaw();

    long currentTicks;
    noInterrupts();
    currentTicks = encTicks;
    interrupts();

    float traveled = ticksToMeters(currentTicks - startTicks);
    float remaining = meters - traveled;

    if (remaining <= 0.0f) break;

    float vTarget = TARGET_SPEED_MS;
    if (remaining < 0.25f) vTarget = 0.20f;
    if (remaining < 0.10f) vTarget = 0.12f;
    if (remaining < 0.04f) vTarget = 0.08f;

    float vMeas = measuredSpeedMs();
    int pwm = speedPwmFromTarget(vTarget, vMeas);

    float headingErr = wrapAngleDeg(headingHold - yawDeg);
    int steerCmd = steerFromHeadingTarget(headingHold);

    setSteerDeg(steerCmd);
    setMotor(pwm, +1);

    dbg_vMeas = vMeas;
    dbg_pwm = pwm;
    dbg_headingTarget = headingHold;
    dbg_headingErr = headingErr;
    dbg_traveled = traveled;
    dbg_remaining = remaining;
    maybeDebugPrint();
  }

  stopMotor();
  setSteerDeg(SERVO_CENTER);
  delay(200);
}

void turnRelativeDeg(float deltaDeg) {
  float target = wrapAngleDeg(yawDeg + deltaDeg);

  int steerDir = (deltaDeg >= 0) ? SERVO_MAX : SERVO_MIN;
  setSteerDeg(steerDir);

  while (true) {
    unsigned long now = millis();
    if (now - lastLoopMs < (1000 / CONTROL_HZ)) continue;
    lastLoopMs = now;

    updateYaw();

    float err = wrapAngleDeg(target - yawDeg);
    if (fabs(err) < 2.0f) break;

    int pwm = (int)(TURN_KP * fabs(err) + 60);
    pwm = clampi(pwm, 60, (int)TURN_RATE_LIMIT_PWM);

    setMotor(pwm, +1);

    dbg_vMeas = measuredSpeedMs();
    dbg_pwm = pwm;
    dbg_headingTarget = target;
    dbg_headingErr = err;
    dbg_traveled = 0;
    dbg_remaining = 0;
    maybeDebugPrint();
  }

  stopMotor();
  setSteerDeg(SERVO_CENTER);
  delay(250);
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  lastA = digitalRead(ENC_A);
  attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_encA, CHANGE);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);

  steer.attach(SERVO_PIN);
  setSteerDeg(SERVO_CENTER);

  if (!IMU.begin()) {
    while (1) {
      Serial.println("IMU init failed");
      delay(500);
    }
  }

  calibrateGyroBias();
  Serial.print("gyroBiasZ = ");
  Serial.println(gyroBiasZ, 6);

  lastYawUs = micros();

  lastLoopMs = millis();
  lastDebugMs = millis();

  Serial.print("YAW_SCALE = ");
  Serial.println(YAW_SCALE, 4);

  stopMotor();
  setSteerDeg(SERVO_CENTER);

  Serial.println("READY:");
  Serial.println("Teleop: #speed:X;;  #steer:A;;  #brake:0;;  #imu:0/1;;");
  Serial.println("Auto:   #drive:meters;;   #turn:deg;;   #auto:0;;");
}

void loop() {
  // 1) Always read serial (non-blocking)
  readSerialCommands();

  // 2) 50 Hz tick
  unsigned long now = millis();
  if (now - lastLoopMs < (1000 / CONTROL_HZ)) return;
  lastLoopMs = now;

  // 3) If auto is active -> run auto tick (ignores teleop speed/steer until done)
  if (autoState != AUTO_NONE) {
    autoTick();
    return;
  }

  // 4) TELEOP MODE (your already-working behavior)
  if (imuEnabled) updateYaw();

  if (cmdBrake) {
    stopMotor();
    setSteerDeg(SERVO_CENTER);
    cmdBrake = false;
    sendImuTelemetryMaybe();
    return;
  }

  // Steering from command (-25..25)
  int steerAbsDeg = SERVO_CENTER + cmdSteer;
  setSteerDeg(steerAbsDeg);

  // Speed from command (-50..50) using your speed loop
  int s = cmdSpeed;
  int dir = (s >= 0) ? +1 : -1;

  if (s == 0) {
    stopMotor();
  } else {
    float vTarget = speedCmdToTargetMs(s);
    float vMeas = measuredSpeedMs();
    int pwm = speedPwmFromTarget(vTarget, vMeas);
    setMotor(pwm, dir);

    dbg_vMeas = vMeas;
    dbg_pwm = pwm;
    dbg_headingTarget = 0;
    dbg_headingErr = 0;
    dbg_traveled = 0;
    dbg_remaining = 0;
    maybeDebugPrint();
  }

  sendImuTelemetryMaybe();
}