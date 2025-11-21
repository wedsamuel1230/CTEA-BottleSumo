/**
 * Dual-Core BottleSumo Tracker (Simplified)
 * ----------------------------------------
 * Core 0 continuously reads the five VL53L0X sensors, decides how the car
 * should move, and drops differential motor commands into a shared struct.
 * Core 1 wakes up every 20 ms, grabs the latest command, and applies it to the
 * two motors. The tracking rule is intentionally tiny: face the closest valid
 * target, drive forward fast, and spin in place while searching.
 *
 * Sensors:  R45, R23, M0, L23, L45  (indexed 0..4)
 * Bias:     Right sensors produce negative values, left sensors positive.
 * Motors:   Left (PWM=GP11, DIR=GP12), Right (PWM=GP14, DIR=GP15)
 *
 * 中文说明：
 *   - Core0 負責感測 + 決策，Core1 專心驅動馬達，兩核心透過共享結構體同步。
 *   - 追蹤策略：找到最近的對手→加速接近；找不到→原地旋轉搜尋；剛失去→短暫保持。
 */

#include <Wire.h>
#include <math.h>
#include <pico/mutex.h>

#include "Car.h"
#include "ToFArray.h"
#include "Ads1115Sampler.h"

// -----------------------------------------------------------------------------
// Hardware configuration
// 中文：以下常數描述馬達腳位、ToF 感測器 I2C 位址與 XSHUT 控制腳
// -----------------------------------------------------------------------------
constexpr uint8_t LEFT_MOTOR_PWM = 11;
constexpr uint8_t LEFT_MOTOR_DIR = 12;
constexpr uint8_t RIGHT_MOTOR_PWM = 14;
constexpr uint8_t RIGHT_MOTOR_DIR = 15;
constexpr uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz silent PWM

constexpr uint8_t TOF_NUM = 5;
// 把 XSHUT 腳位改成交錯順序（最重要！減少相鄰感測器間的串擾）
// 特別將 R45 和 L45 分開，避免兩側 45 度感測器互相干擾
constexpr uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8, 6, 5, 7, 4};  // R45, L23, R23, L45, MID
constexpr uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x30, 0x31, 0x32, 0x33, 0x34};
// 對應名稱也要改（方便除錯）
constexpr const char* TOF_NAMES[TOF_NUM] = {"R45", "L23", "R23", "L45", "MID"};
constexpr uint8_t I2C_SDA = 2;
constexpr uint8_t I2C_SCL = 3;

// Button control
constexpr uint8_t START_BUTTON_PIN = 28;
constexpr bool BUTTON_ACTIVE_LEVEL = LOW; // Pull-up, pressed = LOW
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;
constexpr uint8_t PICO_LED_PIN = LED_BUILTIN; // Built-in LED (usually pin 25)

// IR Sensors for edge detection (from car_ir)
constexpr uint8_t IR_SENSOR_CHANNELS = 4;
constexpr uint8_t IR_SENSOR_PINS[IR_SENSOR_CHANNELS] = {0, 1, 2, 3}; // A0 to A3
constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;

// -----------------------------------------------------------------------------
// Tracking parameters
// 中文：調整感測區間、偏差死區、轉向增益、搜尋速度等行為參數
// -----------------------------------------------------------------------------
constexpr uint16_t DETECT_MIN_MM = 70;
constexpr uint16_t DETECT_MAX_MM = 1000;
constexpr float BIAS_DEADZONE = 0.1f;
constexpr float TURN_GAIN = 30.0f;
constexpr float TURN_CLAMP = 25.0f;
constexpr float SEARCH_SPIN_SPEED = 40.0f;  // Lower = faster spin when searching
constexpr float LOST_HOLD_MS = 2000.0f;
constexpr uint32_t SENSOR_INTERVAL_MS = 35;  // Faster sensor reading (was 60ms)
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
constexpr float ALIGN_SPIN_SPEED = 50.0f;  // Lower = faster alignment
constexpr float ALIGN_FINE_SPIN_SPEED = 50.0f;  // Lower = faster fine alignment
constexpr float CENTER_PUSH_SPEED = 5.0f;  // Very low = very fast push!
constexpr float LEFT_MOTOR_COMPENSATION = 1.5f;  // Left motor is faster, reduce by 15%

// -----------------------------------------------------------------------------
// Edge detection parameters (from car_ir)
// 中文：IR 感測器邊緣偵測參數，用於避免掉出場地
// -----------------------------------------------------------------------------
// Dynamic threshold variables
float ir_threshold_front = 1.5F; // Voltage threshold for A1, A2 (front sensors)
float ir_threshold_back = 3.0F;  // Voltage threshold for A0, A3 (back sensors)
float threshold_min = 0.0F; // Calibration: min voltage seen
float threshold_max = 5.0F; // Calibration: max voltage seen
bool auto_threshold_enabled = false;
unsigned long calibration_start = 0;
const unsigned long CALIBRATION_DURATION_MS = 3000UL; // 3 seconds to calibrate

// Edge detection confirmation
uint8_t last_edge_pattern = 0b0000;
bool edge_verification_mode = false; // waiting to verify edge after stopping
unsigned long edge_stop_time = 0;
const unsigned long EDGE_VERIFY_DELAY_MS = 100UL; // 100ms to verify after stop

// Emergency escape mode
bool emergency_mode = false;
unsigned long emergency_start = 0;
const unsigned long EMERGENCY_DURATION_MS = 1500UL; // 1.5 seconds escape
const float ESCAPE_SPEED = 30.0F; // Lower = faster escape

// Back sensor emergency (drive forward until safe)
bool back_escape_mode = false;
const float BACK_ESCAPE_SPEED = 5.0F; // Very low = very fast forward when rear is off edge

// IR sensor reading interval
constexpr uint32_t IR_SENSOR_INTERVAL_MS = 10;

// Robot state control
bool gRobotActive = false; // Robot starts in idle mode
bool gButtonUsed = false; // Track if button has been pressed (one-time use)
unsigned long gLastButtonPress = 0;
unsigned long gStartDelayEndTime = 0; // Time when 2s forward delay ends
bool gInStartDelay = false; // Flag for 2s forward movement after button press
constexpr unsigned long START_FORWARD_DELAY_MS = 3000UL; // 3 seconds forward after start

// -----------------------------------------------------------------------------
// Shared motor command (Core 0 -> Core 1)
// 中文：Core0 將最新左右輪速度寫入 gSharedCommand，由 mutex 保護
// -----------------------------------------------------------------------------
struct MotorCommand {
	float leftSpeed = 0.0f;
	float rightSpeed = 0.0f;
	uint32_t timestamp = 0;
	bool valid = false;
	bool emergencyStop = false;
};

static MotorCommand gSharedCommand;
static mutex_t gMotorMutex;

// -----------------------------------------------------------------------------
// Globals
// 中文：主要物件與時間戳，全域共享於兩核心
// -----------------------------------------------------------------------------
Car gCar;
ToFArray gTof(&Wire1, nullptr);
Ads1115Sampler gAdcSampler;
ToFSample gSamples[TOF_NUM];
unsigned long gLastSensorRead = 0;
unsigned long gLastTargetSeen = 0;
unsigned long gLastIrRead = 0;

enum class TrackerMode { Searching, Holding, Aligning, Centered };
TrackerMode gMode = TrackerMode::Searching;
uint8_t gLastSensorHit = 0xFF;

struct TargetInfo {
	bool seen = false;
	uint16_t distance = 0;
	float bias = 0.0f;  // -1.0 → right, +1.0 → left
	uint8_t sensorIndex = 0xFF;
};

// -----------------------------------------------------------------------------
// Utility helpers
// 中文：封裝馬達命令、取樣解析與模式輸出的小工具函式
// -----------------------------------------------------------------------------
const char* directionLabel(float leftCmd, float rightCmd) {
	constexpr float kStopEps = 1.0f;
	constexpr float kBalanceEps = 4.0f;

	// Right motor is mirrored, so a positive command actually drives the car backward.
	// Convert into chassis-centric velocities where positive = forward for both wheels.
	float vL = leftCmd;
	float vR = -rightCmd;
	float absL = fabsf(vL);
	float absR = fabsf(vR);
	bool leftZero = absL < kStopEps;
	bool rightZero = absR < kStopEps;
	bool leftForward = vL > 0.0f;
	bool rightForward = vR > 0.0f;

	if (leftZero && rightZero) {
		return "stop";
	}
	if (leftZero) {
		return rightForward ? "pivot-left-forward" : "pivot-left-reverse";
	}
	if (rightZero) {
		return leftForward ? "pivot-right-forward" : "pivot-right-reverse";
	}

	bool sameSign = (leftForward && rightForward) || (!leftForward && !rightForward);
	if (sameSign) {
		float diff = fabsf(vL - vR);
		if (leftForward && rightForward) {
			if (diff <= kBalanceEps) {
				return "forward";
			}
			return (vL < vR) ? "forward-left" : "forward-right";
		} else {
			if (diff <= kBalanceEps) {
				return "reverse";
			}
			return (vL > vR) ? "reverse-left" : "reverse-right";
		}
	}

	float magDiff = fabsf(absL - absR);
	bool nearlyPureSpin = (magDiff <= kBalanceEps);
	bool turnLeft = (vR - vL) > 0.0f;  // positive angular velocity ⇒ left spin
	if (nearlyPureSpin) {
		return turnLeft ? "spin-left" : "spin-right";
	}
	return turnLeft ? "turn-left" : "turn-right";
}

// 中文：傳遞馬達指令到共享結構，並在序列埠上標記原因 + 移動方向
void sendMotorCommand(float left, float right, bool emergencyStop = false,
					 const char* reason = nullptr) {
	// Apply left motor compensation (left motor is faster, so increase value to slow it down)
	float compensatedLeft = left * LEFT_MOTOR_COMPENSATION;
	
	mutex_enter_blocking(&gMotorMutex);
	gSharedCommand.leftSpeed = compensatedLeft;
	gSharedCommand.rightSpeed = right;
	gSharedCommand.emergencyStop = emergencyStop;
	gSharedCommand.timestamp = millis();
	gSharedCommand.valid = true;
	mutex_exit(&gMotorMutex);

	Serial.printf("[CMD] t=%lu L=%.1f(%.1f) R=%.1f dir=%s stop=%s",
			 millis(), compensatedLeft, left, right, directionLabel(compensatedLeft, right),
			 emergencyStop ? "yes" : "no");
	if (reason && reason[0]) {
		Serial.printf(" reason=%s", reason);
	}
	Serial.println();
}

// 中文：Core1 讀取共享命令，若逾時則回傳 false 讓馬達進入安全狀態
bool readMotorCommand(MotorCommand& out) {
	mutex_enter_blocking(&gMotorMutex);
	out = gSharedCommand;
	mutex_exit(&gMotorMutex);

	if (!out.valid) {
		return false;
	}
	if (millis() - out.timestamp > WATCHDOG_TIMEOUT_MS) {
		return false;
	}
	return true;
}

// 中文：掃描五顆 ToF，挑選落在距離範圍內且最近的目標，並以權重換算左右偏差
TargetInfo findClosestTarget(const ToFSample* samples) {
	static constexpr float kWeights[TOF_NUM] = {-1.0f, -0.5f, 0.0f, +0.5f, +1.0f};
	TargetInfo info;
	uint16_t closest = 0xFFFF;
	for (uint8_t i = 0; i < TOF_NUM; ++i) {
		const ToFSample& sample = samples[i];
		if (!sample.valid) {
			continue;
		}
		if (sample.distanceMm < DETECT_MIN_MM || sample.distanceMm > DETECT_MAX_MM) {
			continue;
		}
		if (sample.distanceMm < closest) {
			closest = sample.distanceMm;
			info.seen = true;
			info.distance = sample.distanceMm;
			info.bias = kWeights[i];
			info.sensorIndex = i;
		}
	}
	if (info.seen && fabsf(info.bias) < BIAS_DEADZONE) {
		info.bias = 0.0f;
	}
	return info;
}

const char* modeToString(TrackerMode mode) {
	switch (mode) {
		case TrackerMode::Holding: return "Hold";
		case TrackerMode::Aligning: return "Align";
		case TrackerMode::Centered: return "Center";
		default: return "Search";
	}
}

const char* sensorName(uint8_t idx) {
	return (idx < TOF_NUM) ? TOF_NAMES[idx] : "none";
}

// 中文：將目前模式、追蹤目標與所有感測器距離列印到序列監視器
void printSamples(const TargetInfo& target) {
	Serial.printf("[MODE=%s] target=%s", modeToString(gMode),
								target.seen ? TOF_NAMES[target.sensorIndex] : "none");
	if (target.seen) {
		Serial.printf("(%umm bias=%.2f)", target.distance, target.bias);
	}
	Serial.print(" | ");
	for (uint8_t i = 0; i < TOF_NUM; ++i) {
		if (gSamples[i].valid) {
			Serial.printf("%s:%4u ", TOF_NAMES[i], gSamples[i].distanceMm);
		} else {
			Serial.printf("%s:---- ", TOF_NAMES[i]);
		}
	}
	if (gLastSensorHit != 0xFF) {
		Serial.printf("| last=%s", sensorName(gLastSensorHit));
	}
        Serial.println();
}

// -----------------------------------------------------------------------------
// Edge detection functions (from car_ir)
// 中文：邊緣偵測相關函式，防止車子掉出場地
// -----------------------------------------------------------------------------

// Execute escape maneuver based on edge pattern
void executeEscape(uint8_t pattern) {
    switch (pattern) {
        // Single sensor edge detection - Back away then turn
        case 0b0001: // A0 (bottom-left) → Back + Turn right
            Serial.println("[EDGE] Back + Turn Right");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-A0");
            delay(500);
            sendMotorCommand(ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-turn-right");
            delay(1000);
            break;
        case 0b0010: // A1 (top-left) → Back + Turn right
            Serial.println("[EDGE] Back + Turn Right");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-A1");
            delay(500);
            sendMotorCommand(ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-turn-right");
            delay(1000);
            break;
        case 0b0100: // A2 (top-right) → Back + Turn left
            Serial.println("[EDGE] Back + Turn Left");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-A2");
            delay(500);
            sendMotorCommand(-ESCAPE_SPEED, -ESCAPE_SPEED, false, "escape-turn-left");
            delay(1000);
            break;
        case 0b1000: // A3 (bottom-right) → Back + Turn left
            Serial.println("[EDGE] Back + Turn Left");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-A3");
            delay(500);
            sendMotorCommand(-ESCAPE_SPEED, -ESCAPE_SPEED, false, "escape-turn-left");
            delay(1000);
            break;
            
        // Two sensors on same side - Strong back + turn
        case 0b0011: // A0+A1 (left side) → Strong turn right
            Serial.println("[EDGE] Strong Back + Right");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-left-side");
            delay(700);
            sendMotorCommand(ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-turn-right");
            delay(800);
            break;
        case 0b1100: // A2+A3 (right side) → Strong turn left
            Serial.println("[EDGE] Strong Back + Left");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-right-side");
            delay(700);
            sendMotorCommand(-ESCAPE_SPEED, -ESCAPE_SPEED, false, "escape-turn-left");
            delay(800);
            break;
        case 0b0110: // A1+A2 (front) → Backward + Turn
            Serial.println("[EDGE] Back + Turn Right");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-front");
            delay(800);
            sendMotorCommand(ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-turn-right");
            delay(700);
            break;
        case 0b1001: // A0+A3 (back) → Forward until safe
            Serial.println("[EDGE] BACK SENSORS! Driving forward to safety");
            // Don't do escape maneuver, let checkEdge() handle it with back_escape_mode
            break;
            
        // 3+ sensors = CRITICAL
        case 0b0111: case 0b1011: case 0b1101: case 0b1110: case 0b1111:
            Serial.println("[EDGE] CRITICAL! Back + Turn");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-critical");
            delay(800);
            sendMotorCommand(ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-turn-right");
            delay(700);
            break;
            
        // Diagonal patterns
        case 0b0101: // A0+A2 (diagonal)
            Serial.println("[EDGE] Back + Turn Right");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-diag-A0A2");
            delay(800);
            sendMotorCommand(ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-turn-right");
            delay(700);
            break;
        case 0b1010: // A1+A3 (diagonal)
            Serial.println("[EDGE] Back + Turn Left");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-diag-A1A3");
            delay(800);
            sendMotorCommand(-ESCAPE_SPEED, -ESCAPE_SPEED, false, "escape-turn-left");
            delay(700);
            break;
            
        default:
            Serial.println("[EDGE] Unknown pattern → Backward");
            sendMotorCommand(-ESCAPE_SPEED, ESCAPE_SPEED, false, "escape-unknown");
            break;
    }
}

// Dynamic threshold functions
void startCalibration() {
    Serial.println("\n=== STARTING IR CALIBRATION ===");
    Serial.println("Move car over white and black surfaces for 3 seconds...");
    calibration_start = millis();
    threshold_min = 4.10F; // Reset to max
    threshold_max = 0.0F; // Reset to min
    auto_threshold_enabled = true;
}

void updateCalibration(float voltValues[]) {
    if (!auto_threshold_enabled) return;
    
    // Track min/max values
    for (int i = 0; i < IR_SENSOR_CHANNELS; i++) {
        if (voltValues[i] < threshold_min) threshold_min = voltValues[i];
        if (voltValues[i] > threshold_max) threshold_max = voltValues[i];
    }
    
    // Check if calibration time is up
    if (millis() - calibration_start >= CALIBRATION_DURATION_MS) {
        auto_threshold_enabled = false;
        // Set threshold to midpoint
        ir_threshold_front = (threshold_min + threshold_max) / 2.0F;
        ir_threshold_back = (threshold_min + threshold_max) / 2.0F;
        Serial.println("\n=== IR CALIBRATION COMPLETE ===");
        Serial.printf("Min: %.3fV, Max: %.3fV\n", threshold_min, threshold_max);
        Serial.printf("Front Threshold (A1,A2): %.3fV\n", ir_threshold_front);
        Serial.printf("Back Threshold (A0,A3): %.3fV\n", ir_threshold_back);
        Serial.println("================================\n");
    } else {
        // Show progress
        if ((millis() - calibration_start) % 500 < 50) {
            Serial.printf("Calibrating... %.1fs remaining | Min: %.3fV Max: %.3fV\n", 
                         (CALIBRATION_DURATION_MS - (millis() - calibration_start)) / 1000.0F,
                         threshold_min, threshold_max);
        }
    }
}

void processSerialCommands() {
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.equalsIgnoreCase("cal") || cmd.equalsIgnoreCase("calibrate")) {
            startCalibration();
        }
        else if (cmd.startsWith("f+")) { // Front threshold +
            ir_threshold_front += 0.1F;
            Serial.printf("Front threshold increased to: %.3fV\n", ir_threshold_front);
        }
        else if (cmd.startsWith("f-")) { // Front threshold -
            ir_threshold_front -= 0.1F;
            Serial.printf("Front threshold decreased to: %.3fV\n", ir_threshold_front);
        }
        else if (cmd.startsWith("f=")) { // Front threshold set
            float newThreshold = cmd.substring(2).toFloat();
            if (newThreshold > 0 && newThreshold < threshold_max) {
                ir_threshold_front = newThreshold;
                Serial.printf("Front threshold set to: %.3fV\n", ir_threshold_front);
            } else {
                Serial.println("Invalid threshold value");
            }
        }
        else if (cmd.startsWith("b+")) { // Back threshold +
            ir_threshold_back += 0.1F;
            Serial.printf("Back threshold increased to: %.3fV\n", ir_threshold_back);
        }
        else if (cmd.startsWith("b-")) { // Back threshold -
            ir_threshold_back -= 0.1F;
            Serial.printf("Back threshold decreased to: %.3fV\n", ir_threshold_back);
        }
        else if (cmd.startsWith("b=")) { // Back threshold set
            float newThreshold = cmd.substring(2).toFloat();
            if (newThreshold > 0 && newThreshold < threshold_max) {
                ir_threshold_back = newThreshold;
                Serial.printf("Back threshold set to: %.3fV\n", ir_threshold_back);
            } else {
                Serial.println("Invalid threshold value");
            }
        }
        else if (cmd.startsWith("t+")) { // Both thresholds +
            ir_threshold_front += 0.1F;
            ir_threshold_back += 0.1F;
            Serial.printf("Both thresholds increased - Front: %.3fV, Back: %.3fV\n", 
                         ir_threshold_front, ir_threshold_back);
        }
        else if (cmd.startsWith("t-")) { // Both thresholds -
            ir_threshold_front -= 0.1F;
            ir_threshold_back -= 0.1F;
            Serial.printf("Both thresholds decreased - Front: %.3fV, Back: %.3fV\n", 
                         ir_threshold_front, ir_threshold_back);
        }
        else if (cmd.startsWith("t=")) { // Both thresholds set
            float newThreshold = cmd.substring(2).toFloat();
            if (newThreshold > 0 && newThreshold < threshold_max) {
                ir_threshold_front = newThreshold;
                ir_threshold_back = newThreshold;
                Serial.printf("Both thresholds set to: %.3fV\n", ir_threshold_front);
            } else {
                Serial.println("Invalid threshold value");
            }
        }
        else if (cmd.equalsIgnoreCase("help") || cmd.equals("?")) {
            Serial.println("\n=== IR THRESHOLD COMMANDS ===");
            Serial.println("cal       - Start auto-calibration (3 seconds)");
            Serial.println("f+        - Increase FRONT threshold by 0.1V (A1,A2)");
            Serial.println("f-        - Decrease FRONT threshold by 0.1V (A1,A2)");
            Serial.println("f=X.XXX   - Set FRONT threshold to specific value");
            Serial.println("b+        - Increase BACK threshold by 0.1V (A0,A3)");
            Serial.println("b-        - Decrease BACK threshold by 0.1V (A0,A3)");
            Serial.println("b=X.XXX   - Set BACK threshold to specific value");
            Serial.println("t+        - Increase BOTH thresholds by 0.1V");
            Serial.println("t-        - Decrease BOTH thresholds by 0.1V");
            Serial.println("t=X.XXX   - Set BOTH thresholds to specific value");
            Serial.println("help or ? - Show this menu");
            Serial.printf("\nCurrent thresholds:\n");
            Serial.printf("  Front (A1,A2): %.3fV\n", ir_threshold_front);
            Serial.printf("  Back (A0,A3): %.3fV\n", ir_threshold_back);
            Serial.println("==============================\n");
        }
    }
}

// Check for edge detection - returns true if edge handling is in progress
bool checkEdge() {
    unsigned long now = millis();
    
    // Don't read sensors too frequently
    if (now - gLastIrRead < IR_SENSOR_INTERVAL_MS) {
        return emergency_mode || edge_verification_mode || back_escape_mode;
    }
    gLastIrRead = now;
    
    // Read IR sensors
    int16_t rawValues[IR_SENSOR_CHANNELS];
    float voltValues[IR_SENSOR_CHANNELS];
    gAdcSampler.readAll(rawValues, voltValues, IR_SENSOR_CHANNELS);
    
    // Update calibration if active
    updateCalibration(voltValues);
    
    // Don't process sensor logic during calibration
    if (auto_threshold_enabled) {
        sendMotorCommand(0.0f, 0.0f, false, "calibrating");
        return true;
    }
    
    // Determine which sensors detect EDGE (voltage > threshold = out of area)
    bool edge_detected[IR_SENSOR_CHANNELS];
    edge_detected[0] = (voltValues[0] > ir_threshold_back);  // A0 - back-left
    edge_detected[1] = (voltValues[1] > ir_threshold_front); // A1 - front-left
    edge_detected[2] = (voltValues[2] > ir_threshold_front); // A2 - front-right
    edge_detected[3] = (voltValues[3] > ir_threshold_back);  // A3 - back-right
    
    // Create bit pattern: [A3][A2][A1][A0]
    uint8_t pattern = (edge_detected[3] << 3) | (edge_detected[2] << 2) | 
                      (edge_detected[1] << 1) | edge_detected[0];
    
    // Print sensor readings periodically
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 200) {
        lastPrint = now;
        Serial.printf("[IR] A0:%.2f A1:%.2f A2:%.2f A3:%.2f | Thr F:%.2f B:%.2f | Pat:0b%04b\n",
                     voltValues[0], voltValues[1], voltValues[2], voltValues[3],
                     ir_threshold_front, ir_threshold_back, pattern);
    }
    
    // Check for back edge emergency (highest priority)
    // Pattern 0b1001 means both back sensors (A0 and A3) detect edge
    const bool back_edge_active = edge_detected[0] || edge_detected[3];
    if (back_edge_active) {
        if (!back_escape_mode) {
            back_escape_mode = true;
            edge_verification_mode = false;
            emergency_mode = false;
            Serial.printf("[EDGE] BACK EDGE! Pattern 0b%04b → DRIVING FORWARD TO SAFETY!\n", pattern);
        }
        // Drive forward until back sensors are safe again
        sendMotorCommand(BACK_ESCAPE_SPEED, -BACK_ESCAPE_SPEED, false, "back-edge-forward");
        return true;
    } else if (back_escape_mode) {
        back_escape_mode = false;
        Serial.println("[EDGE] Back sensors safe again — car back in safe zone");
    }
    
    // Check if in emergency escape mode
    if (emergency_mode) {
        if (now - emergency_start < EMERGENCY_DURATION_MS) {
            return true; // Still escaping
        } else {
            emergency_mode = false;
            Serial.println("[EDGE] Emergency escape complete!");
            return false;
        }
    }
    
    // Check for edge detection
    if (pattern != 0b0000) {
        if (!edge_verification_mode) {
            // First detection - STOP IMMEDIATELY
            sendMotorCommand(0.0f, 0.0f, true, "edge-stop");
            edge_verification_mode = true;
            edge_stop_time = now;
            last_edge_pattern = pattern;
            Serial.printf("[EDGE] DETECTED 0b%04b! STOPPING...\n", pattern);
            return true;
        }
    }
    
    // Check if in verification mode
    if (edge_verification_mode) {
        if (now - edge_stop_time < EDGE_VERIFY_DELAY_MS) {
            // Still waiting for verification delay
            return true;
        }
        
        // Verification delay complete - check if still out of safe zone
        if (pattern != 0b0000) {
            // Still detecting edge - start escape!
            Serial.printf("[EDGE] VERIFIED! Pattern 0b%04b → ESCAPE\n", pattern);
            edge_verification_mode = false;
            emergency_mode = true;
            emergency_start = now;
            
            // Execute escape based on pattern
            executeEscape(pattern);
            return true;
        } else {
            // False alarm - back to safe zone
            Serial.println("[EDGE] False alarm - safe zone restored");
            edge_verification_mode = false;
            return false;
        }
    }
    
    return false;
}

// Button handling - one-time activation (requires reset to use again)
void updateStartButton() {
    // If button already used, ignore all further presses
    if (gButtonUsed) {
        return;
    }
    
    unsigned long now = millis();
    if (now - gLastButtonPress < BUTTON_DEBOUNCE_MS) {
        return; // Debounce
    }
    
    bool buttonPressed = (digitalRead(START_BUTTON_PIN) == BUTTON_ACTIVE_LEVEL);
    if (buttonPressed) {
        gLastButtonPress = now;
        gButtonUsed = true; // Mark button as used - won't work again until reset
        gRobotActive = true; // Activate robot
        
        Serial.println("\n[BUTTON] === ROBOT ACTIVATED (ONE-TIME USE) ===");
        Serial.println("[BUTTON] Button disabled - reset Pico to reuse");
        Serial.println("[BUTTON] Moving forward for 3 seconds...");
        digitalWrite(PICO_LED_PIN, LOW);  // LED OFF when active
        gLastTargetSeen = millis(); // Reset target timer
        gInStartDelay = true; // Start the 2-second forward movement
        gStartDelayEndTime = millis() + START_FORWARD_DELAY_MS;
    }
}

// -----------------------------------------------------------------------------
// Core 0: sensor reading + decision
// 中文：核心0 週期性讀取感測器並依結果決定馬達指令
// -----------------------------------------------------------------------------
void processTracking() {
	unsigned long now = millis();
	TargetInfo target = findClosestTarget(gSamples);

	if (target.seen) {
		gLastTargetSeen = now;
		gLastSensorHit = target.sensorIndex;
	}

	if (target.seen) {
		float bias = target.bias;
		float absBias = fabsf(bias);
		if (absBias <= BIAS_DEADZONE) {
			gMode = TrackerMode::Centered;
			// FAST push when centered - edge detection will stop us
			// Lower value = faster speed
			sendMotorCommand(CENTER_PUSH_SPEED, -CENTER_PUSH_SPEED, false, "center-push");
		} else {
			float spin = (absBias > 0.5f) ? ALIGN_SPIN_SPEED : ALIGN_FINE_SPIN_SPEED;
			if (bias > 0) {
				gMode = TrackerMode::Aligning;
				sendMotorCommand(-spin, -spin, false, "align-left");
			} else {
				gMode = TrackerMode::Aligning;
				sendMotorCommand(spin, spin, false, "align-right");
			}
		}
	} else if (now - gLastTargetSeen < LOST_HOLD_MS) {
		gMode = TrackerMode::Holding;
		sendMotorCommand(0.0f, 0.0f, false, "hold-wait");
	} else {
		gMode = TrackerMode::Searching;
		sendMotorCommand(SEARCH_SPIN_SPEED, SEARCH_SPIN_SPEED, false, "search-spin");
	}

	printSamples(target);
}

// 中文：Core0 初始化序列埠、互斥鎖與 ToF 陣列
void setup() {
        Serial.begin(115200);
        delay(500);
        Serial.println("\n=== Dual-Core BottleSumo Tracker with Edge Detection ===\n");

        mutex_init(&gMotorMutex);
        sendMotorCommand(100.0f, 100.0f, true, "init");

        Wire1.setSDA(I2C_SDA);
        Wire1.setSCL(I2C_SCL);
        Wire1.begin();
        // Initialize start button
        pinMode(START_BUTTON_PIN, INPUT_PULLUP);
        Serial.println("[CORE0] Button initialized on pin 28 (pull-up)");
        Serial.println("[CORE0] === ROBOT IN IDLE MODE - PRESS BUTTON TO START ===");
        Serial.println("[CORE0] === BUTTON WORKS ONLY ONCE (RESET REQUIRED) ===\n");

        // Initialize ADS1115 for IR sensors
        if (!gAdcSampler.begin(ADS1115_I2C_ADDRESS, &Wire1, GAIN_ONE, 128)) {
                Serial.println("[CORE0] Failed to initialize ADS1115!");
                while (true) {
                        delay(1000);
                }
        }
        Serial.println("[CORE0] ADS1115 initialized successfully");

        if (!gTof.configure(TOF_NUM, TOF_XSHUT_PINS, TOF_I2C_ADDR)) {
                Serial.println("[CORE0] ToF configure failed!");
                while (true) {
                        delay(1000);
                }
        }
        // 延長 timing budget（讓每顆感測器發射完到下一顆有足夠間隔）
        gTof.setTiming(33000, 15, 8);    // 33ms budget for faster response (was 50ms)
        uint8_t online = gTof.beginAll();
        Serial.printf("[CORE0] Sensors online: %u/%u\n", online, TOF_NUM);

        // Print I2C address for each sensor
        for (uint8_t i = 0; i < TOF_NUM; i++) {
                Serial.print("  - ");
                Serial.print(TOF_NAMES[i]);
                Serial.print(" @ 0x");
                Serial.println(TOF_I2C_ADDR[i], HEX);
        }

        Serial.println("\n=== IR Edge Detection Control ===");
        Serial.println("Type 'help' or '?' for IR calibration commands");
        Serial.printf("Front threshold (A1,A2): %.3fV\n", ir_threshold_front);
        Serial.printf("Back threshold (A0,A3): %.3fV\n", ir_threshold_back);
        Serial.println("=================================\n");

        gLastTargetSeen = millis();

        // Initialize LED for status indication
        pinMode(PICO_LED_PIN, OUTPUT);
        digitalWrite(PICO_LED_PIN, HIGH); // LED ON in idle mode
        Serial.println("[CORE0] LED initialized (ON = idle, OFF = active)");
}

// 中文：依 SENSOR_INTERVAL_MS 週期觸發讀取 + 決策，保持主迴圈輕量
void loop() {
        // Check button state
        updateStartButton();
        
        // Process serial commands for IR threshold tuning
        processSerialCommands();
        
        // If robot is not active, read and display sensors only
        if (!gRobotActive) {
                // Read and display sensors every 500ms in idle mode
                static unsigned long lastIdlePrint = 0;
                unsigned long now = millis();
                if (now - lastIdlePrint >= 500) {
                        lastIdlePrint = now;
                        
                        // Read IR sensors
                        int16_t rawIR[IR_SENSOR_CHANNELS];
                        float voltIR[IR_SENSOR_CHANNELS];
                        gAdcSampler.readAll(rawIR, voltIR, IR_SENSOR_CHANNELS);
                        
                        // Read ToF sensors
                        gTof.readAll(gSamples, DETECT_MIN_MM, DETECT_MAX_MM, 3);
                        
                        // Print IR values
                        Serial.print("[IDLE] IR: ");
                        for (int i = 0; i < IR_SENSOR_CHANNELS; i++) {
                                Serial.printf("A%d:%.2fV ", i, voltIR[i]);
                        }
                        
                        // Print ToF values
                        Serial.print("| ToF: ");
                        for (uint8_t i = 0; i < TOF_NUM; i++) {
                                if (gSamples[i].valid) {
                                        Serial.printf("%s:%4umm ", TOF_NAMES[i], gSamples[i].distanceMm);
                                } else {
                                        Serial.printf("%s:---- ", TOF_NAMES[i]);
                                }
                        }
                        Serial.println();
                }
                
                delay(50);
                return;
        }
        
        // Check if in 3-second forward delay after button press
        if (gInStartDelay) {
                unsigned long now = millis();
                if (now < gStartDelayEndTime) {
                        // Still in delay period - drive forward
                        sendMotorCommand(CENTER_PUSH_SPEED, -CENTER_PUSH_SPEED, false, "start-forward");
                        delay(50);
                        return;
                } else {
                        // Delay complete
                        gInStartDelay = false;
                        Serial.println("[BUTTON] Start delay complete - beginning normal operation");
                }
        }
        
        // Check for edge detection (highest priority)
        if (checkEdge()) {
                // Edge handling in progress, skip tracking
                return;
        }
        
        // Normal tracking operation
        unsigned long now = millis();
        if (now - gLastSensorRead >= SENSOR_INTERVAL_MS) {
                gLastSensorRead = now;
                gTof.readAll(gSamples, DETECT_MIN_MM, DETECT_MAX_MM, 3);
                processTracking();
        }
        delay(5);
}// -----------------------------------------------------------------------------
// Core 1: motor application loop
// 中文：核心1 讀取共享命令並實際設定馬達輸出
// -----------------------------------------------------------------------------
void setup1() {
	delay(500);
	Serial.println("[CORE1] Motor core starting");
	if (!gCar.initializeMotors(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR,
														 RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR,
														 MOTOR_PWM_FREQ)) {
		Serial.println("[CORE1] Motor init failed");
		while (true) {
			delay(1000);
		}
	}
	gCar.stop();
	Serial.println("[CORE1] Motors ready\n");
}

// 中文：每 20ms 檢查命令，若逾時或緊急則停止馬達
void loop1() {
	MotorCommand cmd;
	if (readMotorCommand(cmd)) {
		if (cmd.emergencyStop) {
			gCar.stop();
		} else {
			gCar.setMotors(cmd.leftSpeed, cmd.rightSpeed);
		}
	} else {
		gCar.stop();
	}
	delay(10);  // Faster motor response (was 20ms)
}