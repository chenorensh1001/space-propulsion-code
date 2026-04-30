#include <Arduino.h>
#include <Wire.h>
#include <SD.h>

#include "config.h"
#include "bmp.h"
#include "sd_driver.h"

#ifndef DEPLOY_MOTOR_PWM
#define DEPLOY_MOTOR_PWM 150
#endif

#ifndef RETRACT_MOTOR_ON_TIME_MS
#define RETRACT_MOTOR_ON_TIME_MS 300
#endif

// ===================== State machine =====================

enum class RocketState {
  ACTIVE,
  FLIGHT,
  DEPLOYMENT
};

RocketState currentState = RocketState::ACTIVE;

// ===================== Flight parameters =====================

constexpr float LIFTOFF_ALTITUDE_DELTA_M = 0.30f; // Testing value
constexpr uint32_t PARACHUTE_TIMER_MS = 2700;

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

// ===================== LEDs =====================

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
    "%lu,EVENT_%s,",
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

// ===================== SD log file handling =====================

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

// ===================== Serial commands =====================

constexpr int SERIAL_COMMAND_BUFFER_SIZE = 48;
char serialCommandBuffer[SERIAL_COMMAND_BUFFER_SIZE];
int serialCommandLength = 0;

void processSerialCommandLine() {
  serialCommandBuffer[serialCommandLength] = '\0';

  String cmd = String(serialCommandBuffer);
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "readlast" || cmd == "read_last") {
    readLastRunFile();

  } else if (cmd == "help") {
    Serial.println("Serial commands:");
    Serial.println("  readlast  - print latest SD log file");
    Serial.println("  help      - show this help");

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

// ===================== Logging =====================

void logFlightData(uint32_t now, float altitudeM) {
  char line[100];

  snprintf(
    line,
    sizeof(line),
    "%lu,%s,%.2f",
    static_cast<unsigned long>(now),
    stateToString(currentState),
    altitudeM
  );

  sd::writeLine(line);

  static uint32_t lastFlushMs = 0;

  if (now - lastFlushMs >= 1000) {
    sd::flush();
    lastFlushMs = now;
  }
}

// ===================== Altitude logic =====================

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

void writeMotorOutputsOff() {
  analogWrite(MOTOR_A_IN1_PIN, 0);
  analogWrite(MOTOR_A_IN2_PIN, 0);
  analogWrite(MOTOR_B_IN3_PIN, 0);
  analogWrite(MOTOR_B_IN4_PIN, 0);

  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);
  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);
}

void stopDeploymentMotors() {
  writeMotorOutputsOff();

  if (deploymentMotorsActive) {
    deploymentMotorsActive = false;

    Serial.println("Deployment motors stopped.");
    logEvent("DEPLOYMENT_MOTORS_OFF");
  }
}

void activateDeploymentMotors(uint32_t now) {
  if (deploymentMotorsActive) {
    return;
  }

  deploymentStartTimeMs = now;
  deploymentMotorsActive = true;

  if (DEPLOYMENT_MOTOR_OUTPUT_ENABLED) {
    // Motor A forward
    analogWrite(MOTOR_A_IN1_PIN, DEPLOY_MOTOR_PWM);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);

    // Motor B forward
    analogWrite(MOTOR_B_IN3_PIN, DEPLOY_MOTOR_PWM);
    digitalWrite(MOTOR_B_IN4_PIN, LOW);

    Serial.println("Deployment motors activated: output enabled.");
  } else {
    Serial.println("Deployment motors activated: output disabled in config.");
  }

  logEvent("DEPLOYMENT_MOTORS_ON");
}

void retractDeploymentMotorsAtStartup() {
  Serial.println("Startup motor retraction requested.");

  if (!DEPLOYMENT_MOTOR_OUTPUT_ENABLED) {
    Serial.println("Startup retraction skipped: motor output disabled in config.");
    writeMotorOutputsOff();
    return;
  }

  // Motor A reverse
  digitalWrite(MOTOR_A_IN1_PIN, LOW);
  analogWrite(MOTOR_A_IN2_PIN, DEPLOY_MOTOR_PWM);

  // Motor B reverse
  digitalWrite(MOTOR_B_IN3_PIN, LOW);
  analogWrite(MOTOR_B_IN4_PIN, DEPLOY_MOTOR_PWM);

  delay(RETRACT_MOTOR_ON_TIME_MS);

  writeMotorOutputsOff();

  Serial.println("Startup motor retraction complete.");
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

  // Teensy PWM resolution: 0 to 255
  analogWriteResolution(8);

  // Make sure motors are off at startup
  writeMotorOutputsOff();
  deploymentMotorsActive = false;

  // Pull motors back at startup
  retractDeploymentMotorsAtStartup();

  // Make sure motors are off after startup retraction
  writeMotorOutputsOff();
  deploymentMotorsActive = false;

  // Initialize BMP altitude sensor
  if (bmp::setup() != 0) {
    Serial.println("ERROR: BMP setup failed.");

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
  sd::writeLine("time_ms,state,altitude_m");
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

// ===================== Main loop =====================

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

  const float altitudeM = altitudeReading.altitude;
  const float relativeAltitudeM = altitudeM - referenceAltitudeM;

  logFlightData(now, altitudeM);

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