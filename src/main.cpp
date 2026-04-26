#include <Arduino.h>
#include <Wire.h>
#include <SD.h>

#include "config.h"
#include "bmp.h"
#include "imu.h"
#include "sd_driver.h"

// ===================== H-bridge motor pins =====================
//
// Your wiring:
//
// Teensy pin 2 -> H-bridge IN1
// Teensy pin 3 -> H-bridge IN2
//
// Teensy pin 5 -> H-bridge IN3
// Teensy pin 6 -> H-bridge IN4
//
// Motor A uses IN1 and IN2.
// Motor B uses IN3 and IN4.

// Pin and timing configuration is defined in lib/include/config.h.

// ===================== State machine =====================

enum class RocketState {
  ACTIVE,
  FLIGHT,
  DEPLOYMENT
};

RocketState currentState = RocketState::ACTIVE;

// ===================== Flight parameters =====================

constexpr float LIFTOFF_ALTITUDE_DELTA_M = 0.50f; //will be 5 meter,s 10 cm for testing
constexpr uint32_t PARACHUTE_TIMER_MS = 3000;

// ===================== Altitude and time reference =====================

float referenceAltitudeM = 0.0f;
uint32_t flightStartTimeMs = 0;

// ===================== Altitude history =====================

constexpr int ALTITUDE_HISTORY_SIZE = 20;
float altitudeHistory[ALTITUDE_HISTORY_SIZE];

int altitudeHistoryCount = 0;
int altitudeHistoryIndex = 0;

constexpr float ALTITUDE_DECREASE_TOLERANCE_M = 0.10f;

// ===================== Deployment status =====================

bool deploymentMotorsActive = false;
uint32_t deploymentStartTimeMs = 0;

void setPowerLed(bool on) {
  digitalWrite(LED_POWER_PIN, on ? HIGH : LOW);
}

void setStatusLed(bool on) {
  digitalWrite(LED_STATUS_PIN, on ? HIGH : LOW);
}

void blinkStatusLedFor(uint32_t durationMs) {
  const uint32_t blinkIntervalMs = LED_STATUS_BLINK_INTERVAL_MS;
  uint32_t startMs = millis();
  bool ledOn = false;

  while (millis() - startMs < durationMs) {
    ledOn = !ledOn;
    digitalWrite(LED_STATUS_PIN, ledOn ? HIGH : LOW);
    delay(blinkIntervalMs);
  }

  digitalWrite(LED_STATUS_PIN, LOW);
}

// ===================== Helper functions =====================

const char* stateToString(RocketState state) {
  switch (state) {
    case RocketState::ACTIVE:
      return "ACTIVE";

    case RocketState::FLIGHT:
      return "FLIGHT";

    case RocketState::DEPLOYMENT:
      return "DEPLOYMENT";

    default:
      return "UNKNOWN";
  }
}

void logEvent(const char* eventName) {
  char line[120];

  snprintf(
    line,
    sizeof(line),
    "%lu,EVENT_%s,,,",
    static_cast<unsigned long>(millis()),
    eventName
  );

  sd::writeLine(line);
  sd::flush();

  Serial.print("EVENT: ");
  Serial.println(eventName);
}

void resetAltitudeHistory() {
  altitudeHistoryCount = 0;
  altitudeHistoryIndex = 0;

  for (int i = 0; i < ALTITUDE_HISTORY_SIZE; i++) {
    altitudeHistory[i] = 0.0f;
  }
}

int currentRunNumber = 0;

int findNextRunNumber() {
  char path[32];
  for (int run = 1; run <= 999; ++run) {
    snprintf(path, sizeof(path), "/logs/flight%03d.csv", run);
    if (!sd::exists(path)) {
      return run;
    }
  }
  return -1;
}

bool openRunLogFile() {
  if (!sd::mkdirs("/logs")) {
    Serial.println("ERROR: Could not create /logs directory.");
    return false;
  }

  currentRunNumber = findNextRunNumber();
  if (currentRunNumber < 1) {
    Serial.println("ERROR: Too many run log files.");
    return false;
  }

  char path[32];
  snprintf(path, sizeof(path), "/logs/flight%03d.csv", currentRunNumber);

  Serial.print("Opening log file: ");
  Serial.println(path);

  if (!sd::open(path, false)) {
    Serial.print("ERROR: Could not open run log file: ");
    Serial.println(path);
    return false;
  }

  return true;
}

constexpr int SERIAL_COMMAND_BUFFER_SIZE = 48;
char serialCommandBuffer[SERIAL_COMMAND_BUFFER_SIZE];
int serialCommandLength = 0;

int findLastRunNumber();
bool readLastRunFile();

void processSerialCommandLine() {
  serialCommandBuffer[serialCommandLength] = '\0';
  String cmd = String(serialCommandBuffer);
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "readlast" || cmd == "read_last") {
    readLastRunFile();
  } else if (cmd == "help") {
    Serial.println("Serial commands: readlast, help");
  } else if (serialCommandLength > 0) {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
    Serial.println("Type help for available commands.");
  }

  serialCommandLength = 0;
}

void processSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialCommandLength > 0) {
        processSerialCommandLine();
      }
      serialCommandLength = 0;
      continue;
    }

    if (serialCommandLength < SERIAL_COMMAND_BUFFER_SIZE - 1) {
      serialCommandBuffer[serialCommandLength++] = c;
    }
  }
}

int findLastRunNumber() {
  char path[32];
  for (int run = 999; run >= 1; --run) {
    snprintf(path, sizeof(path), "/logs/flight%03d.csv", run);
    if (sd::exists(path)) {
      return run;
    }
  }
  return 0;
}

bool readLastRunFile() {
  int lastRun = findLastRunNumber();
  if (lastRun == 0) {
    Serial.println("No log files found.");
    return false;
  }

  char path[32];
  snprintf(path, sizeof(path), "/logs/flight%03d.csv", lastRun);

  Serial.print("Reading last log file: ");
  Serial.println(path);

  File file = SD.open(path, FILE_READ);
  if (!file) {
    Serial.print("Failed to open ");
    Serial.println(path);
    return false;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    Serial.println(line);
  }
  file.close();

  return true;
}

void logFlightData(uint32_t now, float altitudeM, float accelX, float accelY, float accelZ) {
  char line[140];

  snprintf(
    line,
    sizeof(line),
    "%lu,%s,%.2f,%.3f,%.3f,%.3f",
    static_cast<unsigned long>(now),
    stateToString(currentState),
    altitudeM,
    accelX,
    accelY,
    accelZ
  );

  sd::writeLine(line);

  static uint32_t lastFlushMs = 0;

  if (now - lastFlushMs >= 1000) {
    sd::flush();
    lastFlushMs = now;
  }
}

void addAltitudeMeasurement(float altitudeM) {
  altitudeHistory[altitudeHistoryIndex] = altitudeM;

  altitudeHistoryIndex++;

  if (altitudeHistoryIndex >= ALTITUDE_HISTORY_SIZE) {
    altitudeHistoryIndex = 0;
  }

  if (altitudeHistoryCount < ALTITUDE_HISTORY_SIZE) {
    altitudeHistoryCount++;
  }
}

bool altitudeIsDecreasing() {
  if (altitudeHistoryCount < ALTITUDE_HISTORY_SIZE) {
    return false;
  }

  for (int i = 0; i < ALTITUDE_HISTORY_SIZE - 1; i++) {
    int currentIndex = (altitudeHistoryIndex + i) % ALTITUDE_HISTORY_SIZE;
    int nextIndex = (altitudeHistoryIndex + i + 1) % ALTITUDE_HISTORY_SIZE;

    float currentAltitude = altitudeHistory[currentIndex];
    float nextAltitude = altitudeHistory[nextIndex];

    if (nextAltitude >= currentAltitude - ALTITUDE_DECREASE_TOLERANCE_M) {
      return false;
    }
  }

  return true;
}

bool flightTimerExpired(uint32_t now) {
  return now - flightStartTimeMs >= PARACHUTE_TIMER_MS;
}

// ===================== Motor control functions =====================

void stopDeploymentMotors() {
  if (DEPLOYMENT_MOTOR_OUTPUT_ENABLED) {
    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);

    digitalWrite(MOTOR_B_IN3_PIN, LOW);
    digitalWrite(MOTOR_B_IN4_PIN, LOW);
  }

  deploymentMotorsActive = false;

  Serial.println("Deployment motors stopped.");
  logEvent("DEPLOYMENT_MOTORS_OFF");
}

void activateDeploymentMotors(uint32_t now) {
  if (DEPLOYMENT_MOTOR_OUTPUT_ENABLED) {
    // Motor A forward
    digitalWrite(MOTOR_A_IN1_PIN, HIGH);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);

    // Motor B forward
    digitalWrite(MOTOR_B_IN3_PIN, HIGH);
    digitalWrite(MOTOR_B_IN4_PIN, LOW);
  }

  deploymentStartTimeMs = now;
  deploymentMotorsActive = true;

  Serial.println("Deployment motors activated.");
  logEvent("DEPLOYMENT_MOTORS_ON");
}

// ===================== State handlers =====================

void handleActiveState(uint32_t now, float relativeAltitudeM) {
  if (relativeAltitudeM >= LIFTOFF_ALTITUDE_DELTA_M) {
    currentState = RocketState::FLIGHT;

    flightStartTimeMs = now;
    resetAltitudeHistory();

    Serial.println("Entered FLIGHT state.");
    Serial.print("Flight start time saved: ");
    Serial.print(flightStartTimeMs);
    Serial.println(" ms");

    logEvent("ENTER_FLIGHT");
  }
}

void handleFlightState(uint32_t now, float altitudeM) {
  addAltitudeMeasurement(altitudeM);

  if (altitudeIsDecreasing() || flightTimerExpired(now)) {
    currentState = RocketState::DEPLOYMENT;

    Serial.println("Entered DEPLOYMENT state.");
    logEvent("ENTER_DEPLOYMENT");

    activateDeploymentMotors(now);
  }
}

void handleDeploymentState(uint32_t now) {
  if (!deploymentMotorsActive) {
    return;
  }

  if (now - deploymentStartTimeMs >= DEPLOY_MOTOR_ON_TIME_MS) {
    stopDeploymentMotors();
  }
}

// ===================== Setup =====================

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Rocket avionics starting...");

  Wire.begin();

  // LED pins
  pinMode(LED_POWER_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  setPowerLed(true);
  setStatusLed(false);

  // H-bridge motor control pins
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);

  pinMode(MOTOR_B_IN3_PIN, OUTPUT);
  pinMode(MOTOR_B_IN4_PIN, OUTPUT);

  // Make sure motors are off at startup
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);

  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);

  // Initialize BMP altitude sensor
  if (bmp::setup() != 0) {
    Serial.println("ERROR: BMP setup failed.");
    while (true) {
      blinkStatusLedFor(1000);
    }
  }

  // Initialize IMU
  if (imu::setup() != 0) {
    Serial.println("ERROR: IMU setup failed.");
    while (true) {
      blinkStatusLedFor(1000);
    }
  }

  // Initialize SD card
  if (sd::setup() != 0) {
    Serial.println("ERROR: SD setup failed.");
    while (true) {
      blinkStatusLedFor(1000);
    }
  }

  // Open a new numbered run log file
  if (!openRunLogFile()) {
    while (true) {
      delay(1000);
    }
  }

  // Write CSV header
  sd::writeLine("time_ms,state,altitude_m,accel_x_m_s2,accel_y_m_s2,accel_z_m_s2");
  sd::flush();

  // Start in ACTIVE state
  currentState = RocketState::ACTIVE;

  Serial.println("Entering ACTIVE state...");
  Serial.println("Saving reference altitude...");

  bmp::Reading altitudeReading;
  const int MAX_STARTUP_ATTEMPTS = 10;
  const float STARTUP_ALTITUDE_LIMIT_M = 1000.0f;
  int attempt = 0;

  do {
    altitudeReading = bmp::read();

    if (!altitudeReading.valid) {
      Serial.println("ERROR: Could not read startup altitude.");
      while (true) {
        delay(1000);
      }
    }

    Serial.print("startup altitude=");
    Serial.print(altitudeReading.altitude, 3);
    Serial.print(" pressure=");
    Serial.print(altitudeReading.pressure, 1);
    Serial.print(" temp=");
    Serial.println(altitudeReading.temperature, 2);

    if (altitudeReading.altitude <= STARTUP_ALTITUDE_LIMIT_M) {
      break;
    }

    Serial.println("Startup altitude too high, retrying...");
    delay(200);
    attempt++;
  } while (attempt < MAX_STARTUP_ATTEMPTS);

  if (altitudeReading.altitude > STARTUP_ALTITUDE_LIMIT_M) {
    Serial.println("ERROR: startup altitude did not settle to a valid value.");
    while (true) {
      delay(1000);
    }
  }

  referenceAltitudeM = altitudeReading.altitude;
  resetAltitudeHistory();

  Serial.print("Reference altitude saved: ");
  Serial.print(referenceAltitudeM);
  Serial.println(" m");

  logEvent("ENTER_ACTIVE");

  setStatusLed(true);
  Serial.println("Setup complete.");
  Serial.println("Type 'readlast' and press Enter to display the latest run file.");
}

// ===================== Main loop ====

void loop() {
  processSerialInput();
  const uint32_t now = millis();

  handleDeploymentState(now);

  static uint32_t lastBmpSampleMs = 0;
  if (now - lastBmpSampleMs < BMP_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastBmpSampleMs = now;

  bmp::Reading altitudeReading = bmp::read();

  if (!altitudeReading.valid) {
    Serial.println("WARNING: Invalid BMP reading.");
    return;
  }

  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t magEvent;
  sensors_event_t tempEvent;
  imu::read(accelEvent, gyroEvent, magEvent, tempEvent);

  const float altitudeM = altitudeReading.altitude;
  const float relativeAltitudeM = altitudeM - referenceAltitudeM;

  logFlightData(now,
                altitudeM,
                accelEvent.acceleration.x,
                accelEvent.acceleration.y,
                accelEvent.acceleration.z);

  switch (currentState) {
    case RocketState::ACTIVE:
      handleActiveState(now, relativeAltitudeM);
      break;

    case RocketState::FLIGHT:
      handleFlightState(now, altitudeM);
      break;

    case RocketState::DEPLOYMENT:
      handleDeploymentState(now);
      break;
  }
}