#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ============================================================
// TEENSY 4.1 SERIAL CONNECTIONS
// ============================================================

// ESP32-S3 receiver TX -> Teensy pin 0 (RX1)
// ESP32-S3 GND -> Teensy GND
#define ESPSerial Serial1

// Teensy pin 8 (TX2) -> Hoverboard USART3 RX
// Teensy pin 7 (RX2) <- Hoverboard USART3 TX
#define HoverSerial Serial2

#define ESP_BAUD 115200
#define HOVER_BAUD 115200
#define USB_BAUD 115200

#define START_FRAME 0xABCD

const unsigned long COMMAND_PERIOD_MS = 20;
const unsigned long PRINT_PERIOD_MS = 200;
const unsigned long ESP_TIMEOUT_MS = 200;

// ============================================================
// MOTOR SPEED & RAMPING SETTINGS
// ============================================================

// Must match N_MOT_MAX in STM32 firmware config.h
const float MAX_RPM = 150.0f;

// Joystick speed limits
const float DRIVE_RPM = 10.0f;
const float TURN_RPM = 5.0f;

// --- Ramping / Slew-Rate Limiting ---
// Step rate: 0.25 RPM every 20ms = 12.5 RPM per second.
// From 0 to DRIVE_RPM (10 RPM) takes ~0.8 seconds.
// Tune this value: smaller = smoother/slower, larger = faster response.
const float RAMP_RPM_PER_STEP = 0.25f;
const unsigned long RAMP_PERIOD_MS = 20;

// Joystick deadzone, percentage
const float JOYSTICK_DEADZONE = 8.0f;

// Reverse these if joystick directions are incorrect
const int JOYSTICK_X_SIGN = +1;
const int JOYSTICK_Y_SIGN = +1;

// ============================================================
// ROBOT GEOMETRY
// ============================================================

const float LEFT_WHEEL_DIAMETER_M = 0.1651f;
const float RIGHT_WHEEL_DIAMETER_M = 0.1651f;

// Placeholder values, require calibration
const float LEFT_TICKS_PER_REV = 90.0f;
const float RIGHT_TICKS_PER_REV = 90.0f;

const float TRACK_WIDTH_M = 0.500f;

// ============================================================
// MOTOR AND ENCODER DIRECTION
// ============================================================

const int LEFT_CMD_SIGN = +1;
const int RIGHT_CMD_SIGN = +1;

const int LEFT_ODOM_SIGN = +1;
const int RIGHT_ODOM_SIGN = +1;

// ============================================================
// HOVERBOARD SERIAL PACKETS
// ============================================================

struct __attribute__((packed)) SerialCommand {
  uint16_t start;
  int16_t left;
  int16_t right;
  uint16_t checksum;
};

struct __attribute__((packed)) SerialFeedback {
  uint16_t start;

  int16_t cmd1;
  int16_t cmd2;

  int16_t speedR_meas;
  int16_t speedL_meas;

  int16_t wheelR_cnt;
  int16_t wheelL_cnt;

  int16_t batVoltage;
  int16_t boardTemp;

  uint16_t cmdLed;
  uint16_t checksum;
};

SerialCommand Command = {};
SerialFeedback Feedback = {};
SerialFeedback NewFeedback = {};

// ============================================================
// JOYSTICK VARIABLES
// ============================================================

int joystickX = 0;
int joystickY = 0;

unsigned long lastESPMessageTime = 0;
bool espConnected = false;

// Incoming text buffer
char espBuffer[40];
uint8_t espBufferIndex = 0;

// ============================================================
// MOTOR TARGETS & CURRENT SPEEDS (RAMPING)
// ============================================================

float targetLeftRPM = 0.0f;
float targetRightRPM = 0.0f;

float currentLeftRPM = 0.0f;
float currentRightRPM = 0.0f;

// ============================================================
// FEEDBACK VARIABLES
// ============================================================

uint8_t feedbackBuffer[sizeof(SerialFeedback)];
uint8_t feedbackIndex = 0;

unsigned long validFeedbackPackets = 0;

// ============================================================
// ODOMETRY VARIABLES
// ============================================================

int16_t previousLeftRaw = 0;
int16_t previousRightRaw = 0;

bool odometryInitialized = false;

int32_t totalLeftTicks = 0;
int32_t totalRightTicks = 0;

float leftDistanceM = 0.0f;
float rightDistanceM = 0.0f;

float robotX = 0.0f;
float robotY = 0.0f;
float robotTheta = 0.0f;

// ============================================================
// TIMING
// ============================================================

unsigned long lastCommandTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastRampTime = 0;

// ============================================================
// MOTOR CONTROL & RAMPING
// ============================================================

void setWheelRPM(float leftRPM, float rightRPM) {
  targetLeftRPM = constrain(leftRPM, -MAX_RPM, MAX_RPM);
  targetRightRPM = constrain(rightRPM, -MAX_RPM, MAX_RPM);
}

// Gradually steps the current wheel speeds toward target speeds
void updateRamping() {
  unsigned long now = millis();
  if (now - lastRampTime < RAMP_PERIOD_MS) {
    return;
  }
  lastRampTime = now;

  // Left Wheel Ramp
  if (currentLeftRPM < targetLeftRPM) {
    currentLeftRPM += RAMP_RPM_PER_STEP;
    if (currentLeftRPM > targetLeftRPM) currentLeftRPM = targetLeftRPM;
  } else if (currentLeftRPM > targetLeftRPM) {
    currentLeftRPM -= RAMP_RPM_PER_STEP;
    if (currentLeftRPM < targetLeftRPM) currentLeftRPM = targetLeftRPM;
  }

  // Right Wheel Ramp
  if (currentRightRPM < targetRightRPM) {
    currentRightRPM += RAMP_RPM_PER_STEP;
    if (currentRightRPM > targetRightRPM) currentRightRPM = targetRightRPM;
  } else if (currentRightRPM > targetRightRPM) {
    currentRightRPM -= RAMP_RPM_PER_STEP;
    if (currentRightRPM < targetRightRPM) currentRightRPM = targetRightRPM;
  }
}

int16_t rpmToCommand(float rpm) {
  rpm = constrain(rpm, -MAX_RPM, MAX_RPM);
  float command = (rpm / MAX_RPM) * 1000.0f;
  return (int16_t)roundf(command);
}

void sendWheelCommand() {
  Command.start = START_FRAME;

  // Sends the smoothed current RPM instead of the abrupt target RPM
  Command.left = rpmToCommand(currentLeftRPM) * LEFT_CMD_SIGN;
  Command.right = rpmToCommand(currentRightRPM) * RIGHT_CMD_SIGN;

  Command.checksum =
      Command.start ^
      (uint16_t)Command.left ^
      (uint16_t)Command.right;

  HoverSerial.write(
      (const uint8_t *)&Command,
      sizeof(Command)
  );
}

// ============================================================
// JOYSTICK PROCESSING
// ============================================================

float applyDeadzone(float value) {
  value = constrain(value, -100.0f, 100.0f);
  float magnitude = fabsf(value);

  if (magnitude <= JOYSTICK_DEADZONE) {
    return 0.0f;
  }

  float scaled =
      (magnitude - JOYSTICK_DEADZONE) /
      (100.0f - JOYSTICK_DEADZONE);

  return (value < 0.0f) ? -scaled : scaled;
}

void applyJoystick(int x, int y) {
  float forward =
      applyDeadzone((float)(y * JOYSTICK_Y_SIGN))
      * DRIVE_RPM;

  float turn =
      applyDeadzone((float)(x * JOYSTICK_X_SIGN))
      * TURN_RPM;

  float leftRPM = forward + turn;
  float rightRPM = forward - turn;

  float largest = max(fabsf(leftRPM), fabsf(rightRPM));

  if (largest > DRIVE_RPM) {
    float scale = DRIVE_RPM / largest;
    leftRPM *= scale;
    rightRPM *= scale;
  }

  setWheelRPM(leftRPM, rightRPM);
}

// ============================================================
// READ ESP32 UART MESSAGES
// ============================================================

void processESPMessage(const char *message) {
  int x, y;
  int consumed = 0;

  if (sscanf(message, "X:%d,Y:%d%n", &x, &y, &consumed) != 2) {
    return;
  }

  if (message[consumed] != '\0') {
    return;
  }

  if (x < -100 || x > 100 || y < -100 || y > 100) {
    return;
  }

  joystickX = x;
  joystickY = y;

  lastESPMessageTime = millis();
  espConnected = true;

  applyJoystick(joystickX, joystickY);
}

void readESPSerial() {
  while (ESPSerial.available()) {
    char c = (char)ESPSerial.read();

    if (c == '\n') {
      espBuffer[espBufferIndex] = '\0';
      processESPMessage(espBuffer);
      espBufferIndex = 0;
    } else if (c == '\r') {
      // Ignore carriage return
    } else {
      if (espBufferIndex < sizeof(espBuffer) - 1) {
        espBuffer[espBufferIndex++] = c;
      } else {
        espBufferIndex = 0;
      }
    }
  }
}

// ============================================================
// COMMUNICATION TIMEOUT
// ============================================================

void checkESPTimeout() {
  if (!espConnected) {
    setWheelRPM(0, 0);
    currentLeftRPM = 0;
    currentRightRPM = 0;
    return;
  }

  if (millis() - lastESPMessageTime > ESP_TIMEOUT_MS) {
    espConnected = false;
    joystickX = 0;
    joystickY = 0;

    // Hard emergency stop on timeout
    setWheelRPM(0, 0);
    currentLeftRPM = 0;
    currentRightRPM = 0;

    Serial.println("ESP32 TIMEOUT - MOTORS STOPPED");
  }
}

// ============================================================
// FEEDBACK VALIDATION
// ============================================================

bool validFeedback(const SerialFeedback &f) {
  uint16_t checksum =
      f.start ^
      (uint16_t)f.cmd1 ^
      (uint16_t)f.cmd2 ^
      (uint16_t)f.speedR_meas ^
      (uint16_t)f.speedL_meas ^
      (uint16_t)f.wheelR_cnt ^
      (uint16_t)f.wheelL_cnt ^
      (uint16_t)f.batVoltage ^
      (uint16_t)f.boardTemp ^
      f.cmdLed;

  return (f.start == START_FRAME && checksum == f.checksum);
}

// ============================================================
// ODOMETRY
// ============================================================

void updateOdometry(int16_t newLeftRaw, int16_t newRightRaw) {
  if (!odometryInitialized) {
    previousLeftRaw = newLeftRaw;
    previousRightRaw = newRightRaw;
    odometryInitialized = true;
    return;
  }

  int16_t deltaLeftRaw =
      (int16_t)((uint16_t)newLeftRaw - (uint16_t)previousLeftRaw);

  int16_t deltaRightRaw =
      (int16_t)((uint16_t)newRightRaw - (uint16_t)previousRightRaw);

  previousLeftRaw = newLeftRaw;
  previousRightRaw = newRightRaw;

  int32_t deltaLeftTicks = (int32_t)deltaLeftRaw * LEFT_ODOM_SIGN;
  int32_t deltaRightTicks = (int32_t)deltaRightRaw * RIGHT_ODOM_SIGN;

  totalLeftTicks += deltaLeftTicks;
  totalRightTicks += deltaRightTicks;

  float leftMetersPerTick = (PI * LEFT_WHEEL_DIAMETER_M) / LEFT_TICKS_PER_REV;
  float rightMetersPerTick = (PI * RIGHT_WHEEL_DIAMETER_M) / RIGHT_TICKS_PER_REV;

  float deltaLeftM = deltaLeftTicks * leftMetersPerTick;
  float deltaRightM = deltaRightTicks * rightMetersPerTick;

  leftDistanceM += deltaLeftM;
  rightDistanceM += deltaRightM;

  float deltaDistance = (deltaLeftM + deltaRightM) / 2.0f;
  float deltaTheta = (deltaRightM - deltaLeftM) / TRACK_WIDTH_M;

  float thetaMid = robotTheta + deltaTheta / 2.0f;

  robotX += deltaDistance * cosf(thetaMid);
  robotY += deltaDistance * sinf(thetaMid);

  robotTheta += deltaTheta;

  while (robotTheta > PI) robotTheta -= 2.0f * PI;
  while (robotTheta < -PI) robotTheta += 2.0f * PI;
}

// ============================================================
// RECEIVE HOVERBOARD FEEDBACK
// ============================================================

void receiveFeedback() {
  while (HoverSerial.available()) {
    uint8_t incoming = (uint8_t)HoverSerial.read();

    if (feedbackIndex == 0) {
      if (incoming == 0xCD) {
        feedbackBuffer[feedbackIndex++] = incoming;
      }
      continue;
    }

    if (feedbackIndex == 1) {
      if (incoming == 0xAB) {
        feedbackBuffer[feedbackIndex++] = incoming;
      } else if (incoming == 0xCD) {
        feedbackBuffer[0] = incoming;
        feedbackIndex = 1;
      } else {
        feedbackIndex = 0;
      }
      continue;
    }

    feedbackBuffer[feedbackIndex++] = incoming;

    if (feedbackIndex == sizeof(SerialFeedback)) {
      memcpy(&NewFeedback, feedbackBuffer, sizeof(SerialFeedback));

      if (validFeedback(NewFeedback)) {
        Feedback = NewFeedback;
        validFeedbackPackets++;
        updateOdometry(Feedback.wheelL_cnt, Feedback.wheelR_cnt);
      }
      feedbackIndex = 0;
    }
  }
}

// ============================================================
// RESET ODOMETRY
// ============================================================

void resetOdometry() {
  totalLeftTicks = 0;
  totalRightTicks = 0;

  leftDistanceM = 0.0f;
  rightDistanceM = 0.0f;

  robotX = 0.0f;
  robotY = 0.0f;
  robotTheta = 0.0f;

  previousLeftRaw = Feedback.wheelL_cnt;
  previousRightRaw = Feedback.wheelR_cnt;

  odometryInitialized = (validFeedbackPackets > 0);

  Serial.println("ODOMETRY RESET");
}

// ============================================================
// USB SERIAL MONITOR COMMANDS
// ============================================================

void readUSBCommands() {
  while (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 's':
      case 'S':
        setWheelRPM(0, 0);
        currentLeftRPM = 0;
        currentRightRPM = 0;
        break;

      case 'z':
      case 'Z':
        resetOdometry();
        break;
    }
  }
}

// ============================================================
// PRINT TELEMETRY
// ============================================================

void printStatus() {
  Serial.println();
  Serial.println("--------------------------------");

  Serial.print("ESP connected: ");
  Serial.println(espConnected ? "YES" : "NO");

  Serial.print("Joystick X: ");
  Serial.print(joystickX);
  Serial.print("  Y: ");
  Serial.println(joystickY);

  Serial.print("Target L: ");
  Serial.print(targetLeftRPM, 2);
  Serial.print(" (Smoothed: ");
  Serial.print(currentLeftRPM, 2);
  Serial.print(") | Target R: ");
  Serial.print(targetRightRPM, 2);
  Serial.print(" (Smoothed: ");
  Serial.print(currentRightRPM, 2);
  Serial.println(")");

  Serial.print("Measured L: ");
  Serial.print(Feedback.speedL_meas);
  Serial.print("  Measured R: ");
  Serial.println(Feedback.speedR_meas);

  Serial.print("L ticks: ");
  Serial.print(totalLeftTicks);
  Serial.print("  R ticks: ");
  Serial.println(totalRightTicks);

  Serial.print("L distance: ");
  Serial.print(leftDistanceM, 3);
  Serial.print(" m  R distance: ");
  Serial.print(rightDistanceM, 3);
  Serial.println(" m");

  Serial.print("X: ");
  Serial.print(robotX, 3);
  Serial.print(" m  Y: ");
  Serial.print(robotY, 3);
  Serial.println(" m");

  Serial.print("Heading: ");
  Serial.print(robotTheta * 180.0f / PI, 2);
  Serial.println(" deg");

  Serial.print("Battery raw: ");
  Serial.println(Feedback.batVoltage);

  Serial.print("Board temperature raw: ");
  Serial.println(Feedback.boardTemp);

  Serial.print("Valid feedback packets: ");
  Serial.println(validFeedbackPackets);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(USB_BAUD);

  ESPSerial.begin(ESP_BAUD);
  HoverSerial.begin(HOVER_BAUD);

  setWheelRPM(0, 0);

  delay(500);

  Serial.println();
  Serial.println("TEENSY 4.1 JOYSTICK HOVERBOARD CONTROL");
  Serial.println("=====================================");
  Serial.println("ESP32 UART: Serial1 (RX1 = pin 0, TX1 = pin 1)");
  Serial.println("Hoverboard UART: Serial2 (RX2 = pin 7, TX2 = pin 8)");
  Serial.println();
  Serial.print("Max forward RPM: ");
  Serial.println(DRIVE_RPM);
  Serial.print("Max turning RPM: ");
  Serial.println(TURN_RPM);
  Serial.print("Joystick deadzone: ");
  Serial.println(JOYSTICK_DEADZONE);
  Serial.println();
  Serial.println("Waiting for ESP32 joystick data...");
  Serial.println("USB commands: s = stop, z = reset odometry");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  readESPSerial();
  checkESPTimeout();
  receiveFeedback();
  readUSBCommands();

  // Smooth the target speeds toward desired values
  updateRamping();

  unsigned long now = millis();

  // Send motor commands every 20 ms
  if (now - lastCommandTime >= COMMAND_PERIOD_MS) {
    lastCommandTime = now;
    sendWheelCommand();
  }

  // Print telemetry every 200 ms
  if (now - lastPrintTime >= PRINT_PERIOD_MS) {
    lastPrintTime = now;
    printStatus();
  }
}
