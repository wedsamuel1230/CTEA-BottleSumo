/**
 * Super Simple BottleSumo Tracker / 超簡單追蹤器
 * ------------------------------------------------
 * The original file used a full finite-state machine and several helper
 * structs.  That is powerful, but it can feel scary when you are just
 * getting started.  This version keeps the exact same hardware setup but
 * boils the behavior down to five easy-to-read steps:
 *
 * 1. Press the start button to wake the robot.
 * 2. Check the IR sensors; if we see the white edge, back up and turn.
 * 3. Read the ToF distance sensors to find the closest opponent.
 * 4. If no one is seen, slowly spin to search.
 * 5. If we do see the opponent, drive forward and steer a tiny bit toward it.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "Ads1115Sampler.h"
#include "Car.h"
#include "ToFArray.h"

// -----------------------------------------------------------------------------
// Hardware pins (same as the advanced sketch) / 硬體腳位
// -----------------------------------------------------------------------------
constexpr uint8_t LEFT_MOTOR_PWM = 11;    // GP11
constexpr uint8_t LEFT_MOTOR_DIR = 12;    // GP12
constexpr uint8_t RIGHT_MOTOR_PWM = 14;   // GP14
constexpr uint8_t RIGHT_MOTOR_DIR = 15;   // GP15
constexpr uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz 靜音 PWM

constexpr uint8_t TOF_NUM = 5;
constexpr uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8, 7, 6, 5, 4};
constexpr uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x30, 0x31, 0x32, 0x33, 0x34};
constexpr const char* TOF_NAMES[TOF_N] = {"R45", "R23", "M0", "L23", "L45"};
constexpr uint8_t I2C_SDA = 2;
constexpr uint8_t I2C_SCL = 3;

constexpr uint8_t ADS_I2C_ADDR = 0x48;
constexpr adsGain_t ADS_GAIN = GAIN_ONE;      // ±4.096 V
constexpr uint8_t ADS_RATE = RATE_ADS1115_860SPS;
constexpr uint8_t IR_CHANNELS = 4;            // A0~A3
constexpr float IR_FRONT_THRESHOLD_VOLTS = 2.5f;  // A1, A2
constexpr float IR_BACK_THRESHOLD_VOLTS = 3.0f;   // A0, A3

constexpr uint8_t START_BUTTON_PIN = 28;      // 接地觸發
constexpr bool BUTTON_ACTIVE_LEVEL = LOW;     // 內建上拉，按下=LOW

// -----------------------------------------------------------------------------
// Friendly tuning knobs / 友善調整參數
// -----------------------------------------------------------------------------
constexpr uint16_t DETECT_MIN_MM = 60;
constexpr uint16_t DETECT_MAX_MM = 1200;
constexpr float TURN_GAIN = 50.0f;            // 小孩懂：偏左就往左轉一點

// Friendly speed scale helper: 1 = fastest, 100 = stop
constexpr float FRIENDLY_SLOPE = -100.0f / 99.0f;
constexpr float FRIENDLY_INTERCEPT = 100.0f - FRIENDLY_SLOPE;  // ≈101.01
constexpr float friendlyFromActual(float actual) {
  return (actual - FRIENDLY_INTERCEPT) / FRIENDLY_SLOPE;
}

constexpr float MAX_FORWARD_SPEED = friendlyFromActual(90.0f);   // ≈10.9 -> very fast
constexpr float MIN_FORWARD_SPEED = friendlyFromActual(35.0f);   // ≈65.3 -> cruise/slow
constexpr float SEARCH_SPIN_SPEED = friendlyFromActual(55.0f);   // ≈45.5 -> medium spin
constexpr float EMERGENCY_SPEED = friendlyFromActual(60.0f);     // ≈40.6 -> safe retreat
constexpr float EDGE_ESCAPE_SPEED = friendlyFromActual(50.0f);   // Match car_ir edge maneuvers
constexpr float BACK_ESCAPE_SPEED = friendlyFromActual(1.0f);    // Max forward when rear is off edge
constexpr float FULL_SPEED_FRIENDLY = friendlyFromActual(100.0f); // Full throttle in friendly units
constexpr float SEARCH_ALIGN_GAIN = 35.0f;
constexpr float CHASE_TURN_GAIN = 15.0f;
constexpr float CHASE_TURN_LIMIT = 25.0f;
constexpr uint16_t CHASE_THRESHOLD_MM = 200;  // 20 cm triggers chase
constexpr uint16_t CHASE_EXIT_MM = 260;       // Leave chase when farther than 26 cm
constexpr uint32_t TOF_READ_INTERVAL_MS = 80;
constexpr uint32_t IR_READ_INTERVAL_MS = 25;
constexpr uint32_t SEARCH_PATTERN_PERIOD_MS = 2500;
constexpr uint32_t EMERGENCY_BACKUP_MS = 500;
constexpr uint32_t EMERGENCY_TURN_MS = 300;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
constexpr uint32_t EDGE_VERIFY_DELAY_MS = 100;

// -----------------------------------------------------------------------------
// Simple data containers / 簡單資料結構
// -----------------------------------------------------------------------------
struct TargetInfo {
  bool seen = false;
  uint16_t distanceMm = 0;
  float bias = 0.0f;  // -1.0 = far right, +1.0 = far left
};

Car gCar;
ToFArray gTof(&Wire1, nullptr);
Ads1115Sampler gIr;

ToFSample gSamples[TOF_NUM];
float gIrVolts[IR_CHANNELS];
TargetInfo gTarget;

bool gStartLatched = false;
bool gEdgeDetected = false;
bool gFrontEdgeDetected = false;
bool gBackEdgeDetected = false;
uint8_t gIrPattern = 0;
bool gEdgeVerifyMode = false;
unsigned long gEdgeStopMs = 0;
bool gBackEscapeMode = false;
enum class TrackingMode : uint8_t { Searching = 0, Chasing = 1 };
TrackingMode gTrackingMode = TrackingMode::Searching;
unsigned long gLastModeSwitchMs = 0;

unsigned long gLastTofRead = 0;
unsigned long gLastIrRead = 0;
unsigned long gLastSpinSwap = 0;
int gSpinDirection = 1;

// Forward declarations --------------------------------------------------------
void updateStartButton(unsigned long now);
void readIrSensors(unsigned long now);
void readTofSensors(unsigned long now);
void searchForOpponent(unsigned long now);
void chaseOpponent();
TargetInfo pickClosestTarget(const ToFSample* samples);
float mapDistanceToSpeed(uint16_t distanceMm);
void driveDifferential(float forward, float turn);
void logSensors(const char* stateLabel);
float friendlyToActual(float friendly);
bool handleEdgeBehavior(unsigned long now);
void executeEdgeEscape(uint8_t pattern);
void updateTrackingMode(unsigned long now);
const char* trackingModeToString(TrackingMode mode);

// -----------------------------------------------------------------------------
// Arduino lifecycle / Arduino 生命週期
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("========================================");
  Serial.println(" Simple BottleSumo Tracker");
  Serial.println(" Press button to toggle START");
  Serial.println("========================================");

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin();
  Serial.println("[SETUP] Wire1 ready (GP2/GP3)");

  if (!gCar.initializeMotors(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR,
                             RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR,
                             MOTOR_PWM_FREQ)) {
    Serial.println("[SETUP] ✗ Motor init failed");
    while (true) {
      delay(500);
    }
  }
  gCar.stop();
  Serial.println("[SETUP] ✓ Motors ready");

  if (!gTof.configure(TOF_NUM, TOF_XSHUT_PINS, TOF_I2C_ADDR)) {
    Serial.println("[SETUP] ✗ ToF configure failed");
    while (true) {
      delay(500);
    }
  }
  gTof.setTiming(33000, 14, 10);
  uint8_t online = gTof.beginAll();
  Serial.printf("[SETUP] ✓ %u/%u ToF online\n", online, TOF_NUM);

  if (!gIr.begin(ADS_I2C_ADDR, &Wire1, ADS_GAIN, ADS_RATE)) {
    Serial.println("[SETUP] ✗ ADS1115 init failed");
    while (true) {
      delay(500);
    }
  }
  Serial.println("[SETUP] ✓ IR sampler ready");

  gLastTofRead = millis();
  gLastIrRead = millis();
  gLastSpinSwap = millis();
}

void loop() {
  unsigned long now = millis();

  // 1) Wait for the human referee -------------------------------------------
  updateStartButton(now);
  readIrSensors(now);
  readTofSensors(now);

  if (!gStartLatched) {
    logSensors("Idle");
    gCar.stop();
    return;  // Nothing happens until the button is pressed
  }

  // 2) Watch the floor -------------------------------------------------------
  if (handleEdgeBehavior(now)) {
    return;  // Edge logic took control this loop
  }

  updateTrackingMode(now);

  // 3) Look for / chase the opponent ----------------------------------------
  if (gTrackingMode == TrackingMode::Searching) {
    logSensors(gTarget.seen ? "SearchFar" : "SearchSpin");
    searchForOpponent(now);
  } else {
    logSensors("Chase");
    chaseOpponent();
  }
}

// -----------------------------------------------------------------------------
// Button helper: toggle start/pause with a debounce
// -----------------------------------------------------------------------------
void updateStartButton(unsigned long now) {
  static bool lastReading = !BUTTON_ACTIVE_LEVEL;
  static bool stableState = !BUTTON_ACTIVE_LEVEL;
  static unsigned long lastChange = 0;

  bool reading = digitalRead(START_BUTTON_PIN);
  if (reading != lastReading) {
    lastReading = reading;
    lastChange = now;
  }

  if ((now - lastChange) < BUTTON_DEBOUNCE_MS) {
    return;
  }

  if (stableState != reading) {
    stableState = reading;
    if (stableState == BUTTON_ACTIVE_LEVEL) {
      gStartLatched = !gStartLatched;
      if (gStartLatched) {
        Serial.println("[BUTTON] Game on!");
      } else {
        gCar.stop();
        Serial.println("[BUTTON] Paused.");
      }
    }
  }
}

// -----------------------------------------------------------------------------
// IR helper: high voltage means the white edge is under a sensor
// -----------------------------------------------------------------------------
void readIrSensors(unsigned long now) {
  if (now - gLastIrRead < IR_READ_INTERVAL_MS) {
    return;
  }
  gLastIrRead = now;

  int16_t raw[IR_CHANNELS];
  gIr.readAll(raw, gIrVolts, IR_CHANNELS);

  gFrontEdgeDetected = false;
  gBackEdgeDetected = false;
  uint8_t pattern = 0;

  for (uint8_t i = 0; i < IR_CHANNELS; i++) {
    const float reading = gIrVolts[i];

    bool isFrontSensor = (i == 1) || (i == 2);
    float threshold = isFrontSensor ? IR_FRONT_THRESHOLD_VOLTS : IR_BACK_THRESHOLD_VOLTS;
    if (reading >= threshold) {
      if (isFrontSensor) {
        gFrontEdgeDetected = true;
      } else {
        gBackEdgeDetected = true;
      }
      pattern |= (1 << i);
    }
  }
  gIrPattern = pattern;
  gEdgeDetected = (pattern != 0);
}

// -----------------------------------------------------------------------------
// ToF helper: read sensors and remember the closest valid hit
// -----------------------------------------------------------------------------
void readTofSensors(unsigned long now) {
  if (now - gLastTofRead < TOF_READ_INTERVAL_MS) {
    return;
  }
  gLastTofRead = now;

  gTof.readAll(gSamples, DETECT_MIN_MM, DETECT_MAX_MM, 4);
  gTarget = pickClosestTarget(gSamples);
}

void updateTrackingMode(unsigned long now) {
  TrackingMode previous = gTrackingMode;
  bool targetClose = gTarget.seen && gTarget.distanceMm <= CHASE_THRESHOLD_MM;
  bool lostOrFar = (!gTarget.seen) || (gTarget.distanceMm >= CHASE_EXIT_MM);

  if (gTrackingMode == TrackingMode::Chasing) {
    if (lostOrFar) {
      gTrackingMode = TrackingMode::Searching;
    }
  } else {
    if (targetClose) {
      gTrackingMode = TrackingMode::Chasing;
    }
  }

  if (previous != gTrackingMode) {
    gLastModeSwitchMs = now;
    if (gTarget.seen) {
      Serial.printf("[MODE] -> %s (target %u mm bias %.2f)\n",
                    trackingModeToString(gTrackingMode),
                    gTarget.distanceMm, gTarget.bias);
    } else {
      Serial.printf("[MODE] -> %s (no target)\n",
                    trackingModeToString(gTrackingMode));
    }
  }
}

const char* trackingModeToString(TrackingMode mode) {
  return (mode == TrackingMode::Chasing) ? "Chase" : "Search";
}

// -----------------------------------------------------------------------------
// Behavior helpers
// -----------------------------------------------------------------------------
bool handleEdgeBehavior(unsigned long now) {
  const bool backEdgeActive = (gIrPattern & 0b0001) || (gIrPattern & 0b1000);

  if (backEdgeActive) {
    if (!gBackEscapeMode) {
      Serial.printf("BACK EDGE! Pattern 0b%04b → MAX FORWARD!\n", gIrPattern);
      gBackEscapeMode = true;
      gEdgeVerifyMode = false;
    }
    logSensors("BackEscape");
    gCar.forward(friendlyToActual(BACK_ESCAPE_SPEED));
    return true;
  }

  if (gBackEscapeMode && !backEdgeActive) {
    gBackEscapeMode = false;
    Serial.println("Back sensors safe again — resuming normal logic");
  }

  if (gEdgeVerifyMode) {
    if (now - gEdgeStopMs < EDGE_VERIFY_DELAY_MS) {
      logSensors("EdgeVerify");
      return true;
    }
    if (gIrPattern != 0) {
      Serial.printf("VERIFIED! Pattern 0b%04b → ESCAPE\n", gIrPattern);
      gEdgeVerifyMode = false;
      logSensors("EdgeEscape");
      executeEdgeEscape(gIrPattern);
      return true;
    } else {
      Serial.println("False alarm - edge cleared");
      gEdgeVerifyMode = false;
      return false;
    }
  }

  if (gIrPattern != 0) {
    gEdgeVerifyMode = true;
    gEdgeStopMs = now;
    gCar.stop();
    Serial.printf("EDGE DETECTED 0b%04b! STOPPING...\n", gIrPattern);
    logSensors("EdgeStop");
    return true;
  }

  return false;
}

void searchForOpponent(unsigned long now) {
  if (gTarget.seen) {
    float forward = mapDistanceToSpeed(gTarget.distanceMm);
    float turn = constrain(gTarget.bias * SEARCH_ALIGN_GAIN,
                           -SEARCH_SPIN_SPEED, SEARCH_SPIN_SPEED);
    driveDifferential(forward, turn);
    return;
  }

  if (now - gLastSpinSwap >= SEARCH_PATTERN_PERIOD_MS) {
    gSpinDirection = -gSpinDirection;
    gLastSpinSwap = now;
  }

  float turn = gSpinDirection * SEARCH_SPIN_SPEED;
  driveDifferential(0.0f, turn);
}

void chaseOpponent() {
  if (!gTarget.seen) {
    gTrackingMode = TrackingMode::Searching;
    return;
  }

  float forward = FULL_SPEED_FRIENDLY;
  float turn = constrain(gTarget.bias * CHASE_TURN_GAIN,
                         -CHASE_TURN_LIMIT, CHASE_TURN_LIMIT);
  if (fabsf(gTarget.bias) < 0.05f) {
    turn = 0.0f;
  }
  driveDifferential(forward, turn);
}

void executeEdgeEscape(uint8_t pattern) {
  const float escapeSpeedActual = friendlyToActual(EDGE_ESCAPE_SPEED);
  const float backEscapeActual = friendlyToActual(BACK_ESCAPE_SPEED);

  switch (pattern) {
    case 0b0001:
    case 0b0010:
      Serial.println("Back + Turn Right");
      gCar.backward(escapeSpeedActual);
      delay(500);
      gCar.turnRight(escapeSpeedActual);
      delay(1000);
      break;
    case 0b0100:
    case 0b1000:
      Serial.println("Back + Turn Left");
      gCar.backward(escapeSpeedActual);
      delay(500);
      gCar.turnLeft(escapeSpeedActual);
      delay(1000);
      break;
    case 0b0011:
      Serial.println("Strong Back + Right");
      gCar.backward(escapeSpeedActual);
      delay(700);
      gCar.turnRight(escapeSpeedActual);
      delay(800);
      break;
    case 0b1100:
      Serial.println("Strong Back + Left");
      gCar.backward(escapeSpeedActual);
      delay(700);
      gCar.turnLeft(escapeSpeedActual);
      delay(800);
      break;
    case 0b0110:
      Serial.println("Front edge → Back + Right");
      gCar.backward(escapeSpeedActual);
      delay(800);
      gCar.turnRight(escapeSpeedActual);
      delay(700);
      break;
    case 0b1001:
      Serial.println("Rear edge → Forward + Right");
      gCar.forward(backEscapeActual);
      delay(800);
      gCar.turnRight(escapeSpeedActual);
      delay(700);
      break;
    case 0b0111: case 0b1011: case 0b1101: case 0b1110: case 0b1111:
      Serial.println("CRITICAL! Back + Turn Right");
      gCar.backward(escapeSpeedActual);
      delay(800);
      gCar.turnRight(escapeSpeedActual);
      delay(700);
      break;
    case 0b0101:
      Serial.println("Diagonal (A0+A2) → Back + Right");
      gCar.backward(escapeSpeedActual);
      delay(800);
      gCar.turnRight(escapeSpeedActual);
      delay(700);
      break;
    case 0b1010:
      Serial.println("Diagonal (A1+A3) → Back + Left");
      gCar.backward(escapeSpeedActual);
      delay(800);
      gCar.turnLeft(escapeSpeedActual);
      delay(700);
      break;
    default:
      Serial.println("Unknown pattern → Backward");
      gCar.backward(escapeSpeedActual);
      delay(600);
      break;
  }

  gCar.stop();
  gEdgeDetected = false;
  gIrPattern = 0;
}

void logSensors(const char* stateLabel) {
  Serial.print("[STATE=");
  Serial.print(stateLabel);
  Serial.print("][MODE=");
  Serial.print(trackingModeToString(gTrackingMode));
  Serial.print("] IR[");
  for (uint8_t i = 0; i < IR_CHANNELS; i++) {
    bool isFront = (i == 1) || (i == 2);
    float threshold = isFront ? IR_FRONT_THRESHOLD_VOLTS : IR_BACK_THRESHOLD_VOLTS;
    bool triggered = gIrVolts[i] >= threshold;
    Serial.print(i);
    Serial.print(":");
    Serial.print(gIrVolts[i], 3);
    Serial.print(triggered ? "!" : " ");
    if (i < IR_CHANNELS - 1) {
      Serial.print(" | ");
    }
  }
  Serial.print("] ToF[");
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    const ToFSample& sample = gSamples[i];
    Serial.print(TOF_NAMES[i]);
    Serial.print(":");
    if (sample.valid) {
      Serial.print(sample.distanceMm);
      Serial.print("mm");
      Serial.print("(✔)");
    } else {
      Serial.print("---mm(X)");
    }
    if (i < TOF_NUM - 1) {
      Serial.print(" | ");
    }
  }
  Serial.print("] Target=");
  if (gTarget.seen) {
    Serial.print(gTarget.distanceMm);
    Serial.print("mm bias=");
    Serial.print(gTarget.bias, 2);
  } else {
    Serial.print("none");
  }
  Serial.println();
}

TargetInfo pickClosestTarget(const ToFSample* samples) {
  static constexpr float kWeights[TOF_NUM] = {-1.0f, -0.5f, 0.0f, +0.5f, +1.0f};
  TargetInfo info;

  uint16_t best = 0xFFFF;
  int8_t bestIndex = -1;
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    const ToFSample& sample = samples[i];
    if (!sample.valid) {
      continue;
    }
    if (sample.distanceMm < DETECT_MIN_MM || sample.distanceMm > DETECT_MAX_MM) {
      continue;
    }
    if (sample.distanceMm < best) {
      best = sample.distanceMm;
      bestIndex = i;
    }
  }

  if (bestIndex >= 0) {
    info.seen = true;
    info.distanceMm = best;
    info.bias = kWeights[bestIndex];
  }

  return info;
}

float mapDistanceToSpeed(uint16_t distanceMm) {
  if (distanceMm <= DETECT_MIN_MM) {
    return MAX_FORWARD_SPEED;
  }
  if (distanceMm >= DETECT_MAX_MM) {
    return MIN_FORWARD_SPEED;
  }
  float normalized = (float)(distanceMm - DETECT_MIN_MM) /
                     (float)(DETECT_MAX_MM - DETECT_MIN_MM);
  float speed = MAX_FORWARD_SPEED +
                normalized * (MIN_FORWARD_SPEED - MAX_FORWARD_SPEED);
  if (speed < MAX_FORWARD_SPEED) {
    speed = MAX_FORWARD_SPEED;
  } else if (speed > MIN_FORWARD_SPEED) {
    speed = MIN_FORWARD_SPEED;
  }
  return speed;
}

void driveDifferential(float forward, float turn) {
  float leftFriendly = constrain(forward + turn, -100.0f, 100.0f);
  float rightFriendly = constrain(forward - turn, -100.0f, 100.0f);
  float leftDuty = friendlyToActual(leftFriendly);
  float rightDuty = friendlyToActual(rightFriendly);
  gCar.setMotors(leftDuty, -rightDuty);  // Right motor is mirrored
}

float friendlyToActual(float friendly) {
  if (friendly == 0.0f) {
    return 0.0f;
  }
  float sign = (friendly < 0.0f) ? -1.0f : 1.0f;
  float magnitude = fabsf(friendly);
  if (magnitude < 1.0f) {
    magnitude = 1.0f;
  }
  if (magnitude > 100.0f) {
    magnitude = 100.0f;
  }
  float actual = FRIENDLY_SLOPE * magnitude + FRIENDLY_INTERCEPT;
  actual = constrain(actual, 0.0f, 100.0f);
  return sign * actual;
}
