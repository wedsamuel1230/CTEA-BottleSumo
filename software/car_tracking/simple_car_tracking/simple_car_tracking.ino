/**
 * BottleSumo Car Tracking State Machine / 瓶子相撲車追蹤狀態機
 * ---------------------------------------------------------------
 * This single-core RP2040 sketch refactors the tracking logic into an
 * object-oriented finite state machine that coordinates ToF tracking,
 * IR boundary avoidance, and button-controlled start sequencing.
 *
 * 本程式以物件導向有限狀態機重新打造追蹤流程，結合 ToF 追蹤、
 * 紅外線白線偵測與按鈕啟動控制，確保 BottleSumo 車可靠運作。
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "Ads1115Sampler.h"
#include "Car.h"
#include "ToFArray.h"

// -----------------------------------------------------------------------------
// Hardware configuration / 硬體腳位設定
// -----------------------------------------------------------------------------
constexpr uint8_t LEFT_MOTOR_PWM = 11;    // GP11
constexpr uint8_t LEFT_MOTOR_DIR = 12;    // GP12
constexpr uint8_t RIGHT_MOTOR_PWM = 14;   // GP14
constexpr uint8_t RIGHT_MOTOR_DIR = 15;   // GP15
constexpr uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz 靜音 PWM

constexpr uint8_t TOF_NUM = 5;
constexpr uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8, 7, 6, 5, 4};
constexpr uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x30, 0x31, 0x32, 0x33, 0x34};
constexpr const char* TOF_NAMES[TOF_NUM] = {"R45", "R23", "M0", "L23", "L45"};
constexpr uint8_t I2C_SDA = 2;
constexpr uint8_t I2C_SCL = 3;

constexpr uint8_t ADS_I2C_ADDR = 0x48;
constexpr adsGain_t ADS_GAIN = GAIN_ONE;      // ±4.096 V
constexpr uint8_t ADS_RATE = RATE_ADS1115_250SPS;
constexpr uint8_t IR_CHANNELS = 4;            // A0~A3
constexpr float IR_THRESHOLD_VOLTS = 2.6f;    // >此電壓視為邊界

constexpr uint8_t START_BUTTON_PIN = 18;      // 假設使用 GP16，接地觸發
constexpr bool BUTTON_ACTIVE_LEVEL = LOW;     // 內建上拉，按下=LOW

// -----------------------------------------------------------------------------
// Tracking parameters / 追蹤參數
// -----------------------------------------------------------------------------
constexpr uint16_t DETECT_MIN_MM = 60;
constexpr uint16_t DETECT_MAX_MM = 1200;
constexpr float BIAS_DEADZONE = 0.1f;          // 偏移死區
constexpr float TURN_GAIN = 45.0f;             // 轉向增益
constexpr float MAX_FORWARD_SPEED = 1.0f;     // 馬達 duty (0~100)
constexpr float MIN_FORWARD_SPEED = 30.0f;
constexpr float SEARCH_SPIN_SPEED = 50.0f;
constexpr uint32_t TOF_READ_INTERVAL_MS = 120;
constexpr uint32_t IR_READ_INTERVAL_MS = 20;
constexpr uint8_t LOCK_CONSECUTIVE = 3;        // 連續偵測次數
constexpr uint8_t LOST_CONSECUTIVE = 10;       // 連續遺失次數

constexpr uint32_t SEARCH_PATTERN_PERIOD_MS = 2500;
constexpr uint32_t EMERGENCY_BACKUP_MS = 600;
constexpr uint32_t EMERGENCY_TURN_MS = 400;
constexpr uint32_t EDGE_CLEAR_HOLD_MS = 400;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 20;

// -----------------------------------------------------------------------------
// Data structures / 資料結構
// -----------------------------------------------------------------------------
struct TargetResult {
  bool hasTarget;
  uint16_t distance;
  float bias;
};

struct ButtonTracker {
  bool debounced;
  bool lastRaw;
  bool pressedEvent;
  unsigned long lastChangeMs;
};

struct SensorFrame {
  ToFSample tof[TOF_NUM];
  TargetResult target;
  float irVolts[IR_CHANNELS];
  uint8_t irEdgeMask;
};

enum class RobotState : uint8_t {
  Idle = 0,
  Search = 1,
  Track = 2,
  EmergencyStop = 3
};

struct RobotContext {
  Car car;
  ToFArray tof;
  Ads1115Sampler ir;
  SensorFrame sensors;
  ButtonTracker button;
  RobotState currentState;
  RobotState requestedState;
  unsigned long lastTofReadMs;
  unsigned long lastIrReadMs;
  unsigned long stateEnterMs;
  unsigned long lastSearchSwapMs;
  int8_t searchDirection;
  uint8_t lockCounter;
  uint8_t lostCounter;
  bool edgeLatched;

  RobotContext() 
    : tof(&Wire1, nullptr),
      currentState(RobotState::Idle),
      requestedState(RobotState::Idle),
      lastTofReadMs(0), lastIrReadMs(0), stateEnterMs(0),
      lastSearchSwapMs(0), searchDirection(1),
      lockCounter(0), lostCounter(0), edgeLatched(false) {
    sensors.target = {false, 0, 0.0f};
    sensors.irEdgeMask = 0;
    button = {false, !BUTTON_ACTIVE_LEVEL, false, 0};
  }
};

// Forward declarations
struct StateHandler;
void requestState(RobotState next);
StateHandler* getHandler(RobotState state);
void pollSensors(unsigned long now);
void updateButton(unsigned long now);
void updateTof(unsigned long now);
void updateIr(unsigned long now);
void driveDifferential(float forward, float turn);
float mapDistanceToSpeed(uint16_t distanceMm);
TargetResult analyzeSamples(const ToFSample* samples);
const char* rangeStatusToString(uint8_t status);

// Global context / 全域狀態
RobotContext gCtx;

// -----------------------------------------------------------------------------
// State handler base class / 狀態處理基底類別
// -----------------------------------------------------------------------------
struct StateHandler {
  StateHandler(const char* en, const char* zh) : labelEn(en), labelZh(zh) {}
  virtual ~StateHandler() = default;
  virtual void onEnter() {}
  virtual void handle() = 0;
  const char* labelEn;
  const char* labelZh;
};

// Utility: log state transitions / 狀態切換紀錄
void logStateChange(RobotState state, const StateHandler* handler) {
  Serial.printf("[FSM] -> %s / %s (state=%d)\n", handler->labelEn, handler->labelZh, static_cast<int>(state));
}

// -----------------------------------------------------------------------------
// Idle state / 就緒狀態
// -----------------------------------------------------------------------------
class IdleState : public StateHandler {
public:
  IdleState() : StateHandler("Idle", "就緒") {}

  void onEnter() override {
    gCtx.car.stop();
  }

  void handle() override {
    if (gCtx.edgeLatched) {
      requestState(RobotState::EmergencyStop);
      return;
    }
    if (gCtx.button.pressedEvent) {
      gCtx.button.pressedEvent = false;
      requestState(RobotState::Search);
    }
  }
};

// -----------------------------------------------------------------------------
// Search state / 搜尋狀態
// -----------------------------------------------------------------------------
class SearchState : public StateHandler {
public:
  SearchState() : StateHandler("Search", "搜尋") {}

  void onEnter() override {
    gCtx.lockCounter = 0;
    gCtx.lostCounter = 0;
    gCtx.lastSearchSwapMs = millis();
    gCtx.searchDirection = 1;
  }

  void handle() override {
    if (gCtx.edgeLatched) {
      requestState(RobotState::EmergencyStop);
      return;
    }

    if (gCtx.sensors.target.hasTarget) {
      gCtx.lockCounter++;
      if (gCtx.lockCounter >= LOCK_CONSECUTIVE) {
        requestState(RobotState::Track);
        return;
      }
    } else {
      gCtx.lockCounter = 0;
    }

    unsigned long now = millis();
    if (now - gCtx.lastSearchSwapMs >= SEARCH_PATTERN_PERIOD_MS) {
      gCtx.searchDirection = -gCtx.searchDirection;
      gCtx.lastSearchSwapMs = now;
    }
    driveDifferential(0.0f, gCtx.searchDirection * SEARCH_SPIN_SPEED);
  }
};

// -----------------------------------------------------------------------------
// Track state / 追蹤狀態
// -----------------------------------------------------------------------------
class TrackState : public StateHandler {
public:
  TrackState() : StateHandler("Track", "追擊") {}

  void onEnter() override {
    gCtx.lostCounter = 0;
  }

  void handle() override {
    if (gCtx.edgeLatched) {
      requestState(RobotState::EmergencyStop);
      return;
    }

    if (gCtx.sensors.target.hasTarget) {
      gCtx.lostCounter = 0;
      float forward = mapDistanceToSpeed(gCtx.sensors.target.distance);
      float turn = gCtx.sensors.target.bias * TURN_GAIN;
      driveDifferential(forward, turn);
    } else {
      gCtx.lostCounter++;
      gCtx.car.stop();
      if (gCtx.lostCounter >= LOST_CONSECUTIVE) {
        requestState(RobotState::Search);
      }
    }
  }
};

// -----------------------------------------------------------------------------
// Emergency stop state / 緊急停止狀態
// -----------------------------------------------------------------------------
class EmergencyState : public StateHandler {
public:
  EmergencyState() : StateHandler("Emergency", "緊急停止"), phaseStart(0) {}

  void onEnter() override {
    phaseStart = millis();
    safeSince = 0;
    gCtx.car.stop();
  }

  void handle() override {
    unsigned long now = millis();
    if (gCtx.sensors.irEdgeMask) {
      if (now - phaseStart <= EMERGENCY_BACKUP_MS) {
        gCtx.car.backward(60.0f);
      } else if (now - phaseStart <= (EMERGENCY_BACKUP_MS + EMERGENCY_TURN_MS)) {
        gCtx.car.turnRight(60.0f);
      } else {
        gCtx.car.stop();
      }
      safeSince = now;
    } else {
      if (safeSince == 0) {
        safeSince = now;
      }
      if (now - safeSince >= EDGE_CLEAR_HOLD_MS) {
        gCtx.edgeLatched = false;
        requestState(RobotState::Idle);
      } else {
        gCtx.car.stop();
      }
    }
  }

private:
  unsigned long phaseStart;
  unsigned long safeSince = 0;
};

// Instantiate handlers / 建立狀態物件
IdleState gIdleState;
SearchState gSearchState;
TrackState gTrackState;
EmergencyState gEmergencyState;

StateHandler* getHandler(RobotState state) {
  switch (state) {
    case RobotState::Idle: return &gIdleState;
    case RobotState::Search: return &gSearchState;
    case RobotState::Track: return &gTrackState;
    case RobotState::EmergencyStop: return &gEmergencyState;
    default: return &gIdleState;
  }
}

void requestState(RobotState next) {
  if (gCtx.requestedState == next) {
    return;
  }
  gCtx.requestedState = next;
}

void applyStateTransitionIfNeeded() {
  if (gCtx.currentState == gCtx.requestedState) {
    return;
  }
  gCtx.currentState = gCtx.requestedState;
  gCtx.stateEnterMs = millis();
  StateHandler* handler = getHandler(gCtx.currentState);
  handler->onEnter();
  logStateChange(gCtx.currentState, handler);
}

// -----------------------------------------------------------------------------
// Arduino lifecycle / Arduino 生命週期
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("========================================");
  Serial.println(" BottleSumo FSM / 瓶子相撲狀態機");
  Serial.println("========================================");

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin();
  Serial.println("[SETUP] Wire1 ready (GP2/GP3)");

  if (!gCtx.car.initializeMotors(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR,
                                 RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR,
                                 MOTOR_PWM_FREQ)) {
    Serial.println("[SETUP] ✗ Motor init failed");
    while (true) {
      delay(500);
    }
  }
  gCtx.car.stop();
  Serial.println("[SETUP] ✓ Motors ready");

  if (!gCtx.tof.configure(TOF_NUM, TOF_XSHUT_PINS, TOF_I2C_ADDR)) {
    Serial.println("[SETUP] ✗ ToF configure failed");
    while (true) {
      delay(500);
    }
  }
  gCtx.tof.setTiming(33000, 14, 10);
  uint8_t online = gCtx.tof.beginAll();
  Serial.printf("[SETUP] ✓ %u/%u ToF online\n", online, TOF_NUM);

  if (!gCtx.ir.begin(ADS_I2C_ADDR, &Wire1, ADS_GAIN, ADS_RATE)) {
    Serial.println("[SETUP] ✗ ADS1115 init failed");
    while (true) {
      delay(500);
    }
  }
  Serial.println("[SETUP] ✓ IR sampler ready");

  gCtx.lastTofReadMs = millis();
  gCtx.lastIrReadMs = millis();
  gCtx.stateEnterMs = millis();
  getHandler(gCtx.currentState)->onEnter();
  logStateChange(gCtx.currentState, getHandler(gCtx.currentState));
}

void loop() {
  unsigned long now = millis();
  updateButton(now);
  pollSensors(now);

  if (gCtx.sensors.irEdgeMask != 0) {
    gCtx.edgeLatched = true;
    requestState(RobotState::EmergencyStop);
  }

  getHandler(gCtx.currentState)->handle();
  applyStateTransitionIfNeeded();
}

// -----------------------------------------------------------------------------
// Sensor + control helpers / 感測器與控制工具
// -----------------------------------------------------------------------------
void pollSensors(unsigned long now) {
  updateIr(now);
  updateTof(now);
}

void updateButton(unsigned long now) {
  bool raw = (digitalRead(START_BUTTON_PIN) == BUTTON_ACTIVE_LEVEL);
  if (raw != gCtx.button.lastRaw) {
    gCtx.button.lastChangeMs = now;
    gCtx.button.lastRaw = raw;
  }
  if ((now - gCtx.button.lastChangeMs) >= BUTTON_DEBOUNCE_MS) {
    if (raw != gCtx.button.debounced) {
      gCtx.button.debounced = raw;
      if (raw && gCtx.currentState == RobotState::Idle) {
        gCtx.button.pressedEvent = true;
      }
    }
  }
}

void updateTof(unsigned long now) {
  if (now - gCtx.lastTofReadMs < TOF_READ_INTERVAL_MS) {
    return;
  }
  gCtx.lastTofReadMs = now;
  gCtx.tof.readAll(gCtx.sensors.tof, DETECT_MIN_MM, DETECT_MAX_MM, 4);
  gCtx.sensors.target = analyzeSamples(gCtx.sensors.tof);
}

void updateIr(unsigned long now) {
  if (now - gCtx.lastIrReadMs < IR_READ_INTERVAL_MS) {
    return;
  }
  gCtx.lastIrReadMs = now;
  int16_t raw[IR_CHANNELS];
  gCtx.ir.readAll(raw, gCtx.sensors.irVolts, IR_CHANNELS);
  uint8_t mask = 0;
  for (uint8_t i = 0; i < IR_CHANNELS; i++) {
    if (gCtx.sensors.irVolts[i] >= IR_THRESHOLD_VOLTS) {
      mask |= (1 << i);
    }
  }
  gCtx.sensors.irEdgeMask = mask;
}

TargetResult analyzeSamples(const ToFSample* samples) {
  static constexpr float weights[TOF_NUM] = {-1.0f, -0.5f, 0.0f, +0.5f, +1.0f};
  TargetResult result = {false, 0, 0.0f};
  uint16_t closest = 0xFFFF;
  int8_t closestIndex = -1;

  for (uint8_t i = 0; i < TOF_NUM; i++) {
    const ToFSample& sample = samples[i];
    if (!sample.valid) {
      continue;
    }
    if (sample.distanceMm < DETECT_MIN_MM || sample.distanceMm > DETECT_MAX_MM) {
      continue;
    }
    if (sample.distanceMm < closest) {
      closest = sample.distanceMm;
      closestIndex = i;
    }
  }

  if (closestIndex >= 0) {
    result.hasTarget = true;
    result.distance = closest;
    result.bias = weights[closestIndex];
    if (fabsf(result.bias) < BIAS_DEADZONE) {
      result.bias = 0.0f;
    }
  }
  return result;
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
  float speed = MAX_FORWARD_SPEED - normalized * (MAX_FORWARD_SPEED - MIN_FORWARD_SPEED);
  return constrain(speed, MIN_FORWARD_SPEED, MAX_FORWARD_SPEED);
}

void driveDifferential(float forward, float turn) {
  float left = constrain(forward + turn, -100.0f, 100.0f);
  float right = constrain(forward - turn, -100.0f, 100.0f);
  float leftDuty = left;
  float rightDuty = -right;  // Right motor wired opposite
  gCtx.car.setMotors(leftDuty, rightDuty);
}

const char* rangeStatusToString(uint8_t status) {
  switch (status) {
    case 0: return "RangeValid";
    case 1: return "SigmaFail";
    case 2: return "SignalFail";
    case 3: return "MinClip";
    case 4: return "PhaseFail";
    case 5: return "Hardware";
    case 6: return "NoUpdate";
    case 7: return "Wrapped";
    default: return "Unknown";
  }
}
