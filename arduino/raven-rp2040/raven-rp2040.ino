/*
 * RAVEN - Arduino Nano RP2040 Connect Firmware
 * Compatible with raven-brain-stack serial protocol (Mbed replacement)
 *
 * Protocol:
 * - Command: #KEY:VALUE;;
 * - Response: @KEY:VALUE;;\r\n
 *
 * Hardware:
 * - Arduino Nano RP2040 Connect
 * - H-Bridge Motor Driver (IN1=7, IN2=8, EN=9)
 * - Servo for Steering on Pin 11
 * - Built-in IMU (LSM6DSOX)
 */

#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Servo.h>
#include <WiFiNINA.h>

// PIN CONFIGURATION
const int IN1 = 7;
const int IN2 = 8;
const int PIN_MOTOR_EN = 9; // PWM Speed Control
const int SERVO_PIN = 11;   // Steering Servo Signal
const int LED_BUILTIN_PIN = LED_BUILTIN;

// Wi-Fi Configuration
const char *WIFI_SSID = "AREA51";
const char *WIFI_PASS = "LeaveMyWifiAlonePlease";

// Protocol Constants
const char MSG_START = '#';
const char MSG_SEP = ':';
const char MSG_END_1 = ';';
const char MSG_END_2 = ';';

// State Variables
Servo steeringServo;
bool imuActive = true;
unsigned long lastImuTime = 0;
const int IMU_INTERVAL = 100; // 10Hz

// Forward Declarations
void initMotors();
void stopMotor();
void setMotorSpeed(float speed);
void setSteering(float angle);
void handleSerial();
void handleIMU();
void processMessage(String msg);
void printFloat(float val);

void initMotors() {
  // Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PIN_MOTOR_EN, OUTPUT);

  // Servo Pin
  steeringServo.attach(SERVO_PIN);

  // Initial State
  stopMotor();
  steeringServo.write(90); // Center
}

void setup() {
  Serial.begin(115200);

  // Init Pins
  pinMode(LED_BUILTIN_PIN, OUTPUT);

  // Init Motors
  initMotors();

  // Init IMU
  if (!IMU.begin()) {
    while (1) { // Error blink
      digitalWrite(LED_BUILTIN_PIN, !digitalRead(LED_BUILTIN_PIN));
      delay(100);
    }
  }

  // Connect to Wi-Fi
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
  } else {
    int status = WL_IDLE_STATUS;
    if (status != WL_CONNECTED) {
      status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }

  // Send Startup Message
  Serial.print("\r\n\r\n");
  Serial.print("#################\r\n");
  Serial.print("#               #\r\n");
  Serial.print("#   I'm alive   #\r\n");
  Serial.print("#   (Arduino)   #\r\n");
  Serial.print("#               #\r\n");
  Serial.print("#################\r\n");
  Serial.print("\r\n");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("# WiFi: Connected to ");
    Serial.print(WIFI_SSID);
    Serial.print("\r\n");
    IPAddress ip = WiFi.localIP();
    Serial.print("# IP: ");
    Serial.print(ip);
    Serial.print("\r\n");
  }
}

void loop() {
  handleSerial();
  handleIMU();
}

// ============================================
// SERIAL PROTOCOL HANDLER
// ============================================
String inputBuffer = "";

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == MSG_START) {
      inputBuffer = ""; // Reset on start char
    } else if (c == '\n' || c == '\r') {
      // Ignore
    } else {
      inputBuffer += c;
    }

    if (inputBuffer.endsWith(";;")) {
      processMessage(inputBuffer.substring(0, inputBuffer.length() - 2));
      inputBuffer = "";
    }
  }
}

void processMessage(String msg) {
  int splitIdx = msg.indexOf(MSG_SEP);
  if (splitIdx == -1)
    return;

  String key = msg.substring(0, splitIdx);
  String value = msg.substring(splitIdx + 1);
  String response = "";

  if (key == "speed") {
    float speedVal = value.toFloat();
    setMotorSpeed(speedVal);
    response = String(speedVal);
  } else if (key == "steer") {
    float steerVal = value.toFloat();
    setSteering(steerVal);
    response = String(steerVal);
  } else if (key == "brake") {
    setMotorSpeed(0);
    response = "0";
  } else if (key == "imu") {
    imuActive = (value.toInt() == 1);
    response = imuActive ? "1" : "0";
  }

  if (response.length() > 0) {
    Serial.print("@");
    Serial.print(key);
    Serial.print(":");
    Serial.print(response);
    Serial.print(";;\r\n");
  }
}

// ============================================
// MOTOR & SERVO CONTROL
// ============================================

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(PIN_MOTOR_EN, 0);
}

void setMotorSpeed(float speed) {
  // Speed comes in -500 to 500 from Brain
  // Dashboard sends -30 to 30 (cm/s)
  // We need to map this to PWM 0-255 with a deadband offset.

  // Logic: Map 0-50 input to 70-255 output.
  // This ensures that even small commands (speed=2) give enough juice (~77 PWM)
  // to move. And max dashboard speed (30) gives strong response (~180 PWM).

  int pwm = 0;
  int absSpeed = (int)abs(speed);

  // Deadband compensation
  if (absSpeed > 0) {
    // Map 0 to 50 input -> 70 to 255 output
    // If input > 50, it will clamp to 255
    pwm = map(absSpeed, 0, 50, 70, 255);
    pwm = constrain(pwm, 0, 255);
  }

  if (speed > 0) {
    // Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    // Stop
    stopMotor();
    return;
  }

  analogWrite(PIN_MOTOR_EN, pwm);

  // Debug
  // Serial.print("@debug:Speed="); Serial.print(speed); Serial.print(",PWM=");
  // Serial.println(pwm);
}

void setSteering(float angle) {
  // Angle comes in -25.0 to 25.0 (degrees relative to center)
  // Map to Servo Angle 0-180 (Center 90)

  // Assuming -25 is Left and 25 is Right
  // 90 is center.
  // -25 -> 65
  // +25 -> 115
  int servoAngle = 90 + (int)angle;

  // Constrain to safe physical limits
  servoAngle = constrain(servoAngle, 50, 130);

  steeringServo.write(servoAngle);
}

// ============================================
// IMU HANDLER
// ============================================
void handleIMU() {
  if (!imuActive || (millis() - lastImuTime < IMU_INTERVAL))
    return;

  float ax, ay, az, gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // Simplistic Euler estimation
    float roll = ax * 90.0;
    float pitch = ay * 90.0;
    float yaw = 0;

    Serial.print("@imu:");
    printFloat(roll);
    Serial.print(";");
    printFloat(pitch);
    Serial.print(";");
    printFloat(yaw);
    Serial.print(";");
    printFloat(ax);
    Serial.print(";");
    printFloat(ay);
    Serial.print(";");
    printFloat(az);
    Serial.print(";;\r\n");

    lastImuTime = millis();
  }
}

void printFloat(float val) { Serial.print(val, 3); }
