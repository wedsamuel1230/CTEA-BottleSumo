#include "TofArray.h"
#include "Car.h"

const uint8_t LEFT_MOTOR_PWM = 11;   // GP11
const uint8_t LEFT_MOTOR_DIR = 12;   // GP12
const uint8_t RIGHT_MOTOR_PWM = 14;  // GP14
const uint8_t RIGHT_MOTOR_DIR = 15;  // GP15
const uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz (silent operation)

// ToF Sensor Configuration
const uint8_t TOF_NUM = 5;
const uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8, 7, 6, 5, 4};  // GP8-GP4
const uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x30, 0x31, 0x32, 0x33, 0x34};
const char* TOF_NAMES[TOF_NUM] = {"R45", "R23", "M0", "L23", "L45"};
const uint16_t SENSOR_ACTION_MAX_MM[TOF_NUM] = {450, 400, 600, 400, 450};
const uint32_t timingBudgetUs = 33000;
const uint8_t preRange = 10;
const uint8_t finalRange = 14;

// I2C Configuration
const uint8_t I2C_SDA = 2;  // GP2
const uint8_t I2C_SCL = 3;  // GP3

ToFArray tof(&Wire1, nullptr);
ToFSample tofData[TOF_NUM];

Car car;

float calculateDirectionBias(const ToFSample* samples) {
  // Sensor indices: R45=0, R23=1, M0=2, L23=3, L45=4
  // Sensor weights: negative=right, zero=center, positive=left
  const float weights[TOF_NUM] = {
    -1.0f,  // R45 (index 0) - far right
    -0.5f,  // R23 (index 1) - near right
     0.0f,  // M0  (index 2) - center
    +0.5f,  // L23 (index 3) - near left
    +1.0f   // L45 (index 4) - far left
  };
  
  // Find the closest valid sensor
  uint16_t closestDist = activeDetectMax + 1;
  int8_t closestIndex = -1;
  
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    if (isActionableTarget(i, samples[i]) && samples[i].distanceMm < closestDist) {
      closestDist = samples[i].distanceMm;
      closestIndex = i;
    }
  }
  
  // No valid object detected
  if (closestIndex < 0) {
    return 0.0f;
  }
  
  // Return the direction bias based on which sensor detected the closest object
  return weights[closestIndex];
}

void reportToFStatus(const ToFSample* samples){
  for (uint8_t i = 0; i < TOF_NUM; ++i) {
    bool online = tof.isOnline(i);
    Serial.printf("ToF %s: ", TOF_NAMES[i]);
    if (!online) {
      Serial.println("Offline");
      continue;
    }
    Serial.printf("Distance: %d mm\n", samples[i].distanceMm);
  }
}

void processTracking(const ToFSample* samples) {
  // Calculate steering direction from side sensors (actionable-only)
  float directionBias = calculateDirectionBias(samples);

  bool hasValidEcho = false;
  for (uint8_t i = 0; i < TOF_NUM; i++) {
    if (isValidTarget(samples[i])) {
      hasValidEcho = true;
      break;
    }
  }

  uint16_t leftClusterDistance = 0;
  uint16_t rightClusterDistance = 0;
  bool leftCluster = detectSideCluster(true, samples, leftClusterDistance);
  bool rightCluster = detectSideCluster(false, samples, rightClusterDistance);
  bool frontClose = isActionableTarget(2, samples[2]);
  bool actionableTarget = frontClose || leftCluster || rightCluster;

  if (actionableTarget) {
    lastDetection = millis();
    if (searchExpansionLevel > 0) {
      resetSearchEnvelope(true);
    }

    // Find closest actionable object for display
    uint16_t closestDist = activeDetectMax + 1;
    const char* closestSensor = "---";
    for (uint8_t i = 0; i < TOF_NUM; i++) {
      if (isActionableTarget(i, samples[i]) && samples[i].distanceMm < closestDist) {
        closestDist = samples[i].distanceMm;
        closestSensor = TOF_NAMES[i];
      }
    }

    uint16_t frontDistance = 0;
    bool frontAligned = detectFrontAlignment(samples, frontDistance);

    int8_t desiredDirection = 0;
    if (leftCluster && !rightCluster) {
      desiredDirection = 1;
    } else if (rightCluster && !leftCluster) {
      desiredDirection = -1;
    } else if (leftCluster && rightCluster) {
      desiredDirection = (leftClusterDistance <= rightClusterDistance) ? 1 : -1;
    } else if (!frontClose) {
      if (directionBias > BIAS_DEADZONE) {
        desiredDirection = 1;
      } else if (directionBias < -BIAS_DEADZONE) {
        desiredDirection = -1;
      }
    }

    int8_t filtered = updateDirectionFilter(frontAligned ? 0 : desiredDirection);
    bool validatingTurn = (!frontAligned && desiredDirection != 0 && filtered == 0);

    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;
    const char* direction = "✓ CENTERED";
    if (filtered > 0) {
      leftSpeed = -ROTATION_SPEED;
      rightSpeed = -ROTATION_SPEED;
      direction = "← TURN LEFT";
    } else if (filtered < 0) {
      leftSpeed = ROTATION_SPEED;
      rightSpeed = ROTATION_SPEED;
      direction = "TURN RIGHT →";
    } else if (frontAligned) {
      direction = "◎ FRONT HOLD";
      leftSpeed = 0.0f;
      rightSpeed = 0.0f;
      resetDirectionFilter();
    } else if (frontClose) {
      direction = "◎ FRONT DETECT";
      leftSpeed = 0.0f;
      rightSpeed = 0.0f;
      resetDirectionFilter();
    } else if (validatingTurn) {
      direction = "... VALIDATING";
    }
  }
  Serial.printf("[CORE0] ");
  Serial.printf("%s | dirBias=%.2f | dir=%s | closest=%s(%dmm) | leftCluster=%s(%dmm) | rightCluster=%s(%dmm) | frontClose=%s\n",
                actionableTarget ? "ACTIONABLE" : "NO ACTION",
                directionBias,
                direction,
                closestSensor, closestDist,
                leftCluster ? "Y" : "N", leftClusterDistance,
                rightCluster ? "Y" : "N", rightClusterDistance,
                frontClose ? "Y" : "N");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  // Initialize ToF sensors
  tof.begin(TOF_NUM, TOF_XSHUT_PINS, TOF_I2C_ADDR);
  tof.setTimeout(500);
  tof.startContinuous();

  // Initialize car motors
  car.begin(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR, RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR, MOTOR_PWM_FREQ);

  tof.setTiming(timingBudgetUs, preRange, finalRange);
}

void loop() {
  // put your main code here, to run repeatedly:
  tof.readAll(tofData, DETECT_MIN_MM, activeDetectMax, activeStatusMax);
  reportToFStatus(tofData);

  processTracking(tofData);

  delay(10);

}
