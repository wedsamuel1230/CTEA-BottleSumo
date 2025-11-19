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
constexpr uint8_t TOF_XSHUT_PINS[TOF_NUM] = {8, 7, 6, 5, 4};
constexpr uint8_t TOF_I2C_ADDR[TOF_NUM] = {0x30, 0x31, 0x32, 0x33, 0x34};
constexpr const char* TOF_NAMES[TOF_NUM] = {"R45", "R23", "M0", "L23", "L45"};
constexpr uint8_t I2C_SDA = 2;
constexpr uint8_t I2C_SCL = 3;

// -----------------------------------------------------------------------------
// Tracking parameters
// 中文：調整感測區間、偏差死區、轉向增益、搜尋速度等行為參數
// -----------------------------------------------------------------------------
constexpr uint16_t DETECT_MIN_MM = 70;
constexpr uint16_t DETECT_MAX_MM = 1000;
constexpr float BIAS_DEADZONE = 0.1f;
constexpr float TURN_GAIN = 30.0f;
constexpr float TURN_CLAMP = 25.0f;
constexpr float SEARCH_SPIN_SPEED = 35.0f;
constexpr float LOST_HOLD_MS = 2000.0f;
constexpr uint32_t SENSOR_INTERVAL_MS = 60;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
constexpr float ALIGN_SPIN_SPEED = 32.0f;
constexpr float ALIGN_FINE_SPIN_SPEED = 18.0f;

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
ToFSample gSamples[TOF_NUM];
unsigned long gLastSensorRead = 0;
unsigned long gLastTargetSeen = 0;

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
	mutex_enter_blocking(&gMotorMutex);
	gSharedCommand.leftSpeed = left;
	gSharedCommand.rightSpeed = right;
	gSharedCommand.emergencyStop = emergencyStop;
	gSharedCommand.timestamp = millis();
	gSharedCommand.valid = true;
	mutex_exit(&gMotorMutex);

	Serial.printf("[CMD] t=%lu L=%.1f R=%.1f dir=%s stop=%s",
			 millis(), left, right, directionLabel(left, right),
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
			sendMotorCommand(100.0f, -100.0f, false, "center-push");
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
	Serial.println("\n=== Dual-Core BottleSumo Tracker (Simple) ===\n");

	mutex_init(&gMotorMutex);
	sendMotorCommand(100.0f, 100.0f, true, "init");

	Wire1.setSDA(I2C_SDA);
	Wire1.setSCL(I2C_SCL);
	Wire1.begin();

	if (!gTof.configure(TOF_NUM, TOF_XSHUT_PINS, TOF_I2C_ADDR)) {
		Serial.println("[CORE0] ToF configure failed!");
		while (true) {
			delay(1000);
		}
	}
	gTof.setTiming(33000, 14, 10);
	uint8_t online = gTof.beginAll();
	Serial.printf("[CORE0] Sensors online: %u/%u\n", online, TOF_NUM);
	gLastTargetSeen = millis();
}

// 中文：依 SENSOR_INTERVAL_MS 週期觸發讀取 + 決策，保持主迴圈輕量
void loop() {
	unsigned long now = millis();
	if (now - gLastSensorRead >= SENSOR_INTERVAL_MS) {
		gLastSensorRead = now;
		gTof.readAll(gSamples, DETECT_MIN_MM, DETECT_MAX_MM, 3);
		processTracking();
	}
	delay(5);
}

// -----------------------------------------------------------------------------
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
	delay(20);
}
