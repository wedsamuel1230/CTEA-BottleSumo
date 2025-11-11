//this is a car ir sensor control project

#include <Wire.h>
#include "Car.h"
#include "Ads1115Sampler.h"

constexpr uint8_t LEFT_MOTOR_PWM_PIN = 11;
constexpr uint8_t LEFT_MOTOR_DIR_PIN = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 14;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 15;
constexpr uint32_t MOTOR_FREQ = 20000; // 20 kHz
constexpr uint8_t IR_SENSOR_CHANNELS = 4;
constexpr uint8_t IR_SENSOR_PINS[IR_SENSOR_CHANNELS] = {0, 1, 2, 3}; // A0 to A3
constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;
float ir_threshold = 2.5F; // Voltage threshold for IR detection
/*
A3 (Channel 3) - bottom-right IR Sensor
A2 (Channel 2) - top-right IR Sensor
A1 (Channel 1) - top-left IR Sensor
A0 (Channel 0) - bottom-left IR Sensor

------------------------------------------------
| IR Sensor Arrangement on the Car:            |
|                                              |
|                    Front of Car              |
|       [Top-Left (A1)]   [Top-Right (A2)]     |
|               |                 |            |
|               |                 |            |
|               |                 |            |
|       [Bottom-Left (A0)] [Bottom-Right (A3)] |
------------------------------------------------
top: A1, A2
bottom: A0, A3

Behavior Logic:
- If all sensors read low (above threshold), the path is clear: move forward.
- If three or more sensors read high (below threshold), stop to avoid obstacles.
-> 0b1111, 0b1110, 0b1101, 0b1011, 0b0111
- If only one or two sensors read high, adjust direction to stay on track.
A1,A2 high -> backward -> 0b1100
A0,A3 high -> forward -> 0b1001
A1,A0 high -> turn right -> 0b0011
A2,A3 high -> turn left -> 0b1100
A1 only -> slight right -> 0b0010
A0 only -> slight right -> 0b0001
A2 only -> slight left -> 0b0100
A3 only -> slight left -> 0b1000
*/

Car car;
Ads1115Sampler adcSampler;

// Non-blocking searching mode state
bool searchingActive = false;
uint8_t searchStep = 0; // 0: forward, 1: left, 2: right, 3: stop
unsigned long searchStepStart = 0;
const unsigned long SEARCH_STEP_DURATION_MS = 500UL;
const unsigned long SEARCH_STOP_DURATION_MS = 200UL;

void startSearchingMode() {
    if (!searchingActive) {
        searchingActive = true;
        searchStep = 0;
        searchStepStart = millis();
    }
}

void stopSearchingMode() {
    if (searchingActive) {
        searchingActive = false;
    }
    car.stop();
}

// Call this frequently from loop() to run the non-blocking search sequence
void updateSearchingMode() {
    if (!searchingActive) return;
    unsigned long now = millis();
    unsigned long elapsed = now - searchStepStart;

    switch (searchStep) {
        case 0: // forward
            if (elapsed < SEARCH_STEP_DURATION_MS) {
                car.forward(50.0F);
            } else {
                searchStep = 1;
                searchStepStart = now;
            }
            break;
        case 1: // turn left
            if (elapsed < SEARCH_STEP_DURATION_MS) {
                car.turnLeft(50.0F);
            } else {
                searchStep = 2;
                searchStepStart = now;
            }
            break;
        case 2: // turn right
            if (elapsed < SEARCH_STEP_DURATION_MS) {
                car.turnRight(50.0F);
            } else {
                searchStep = 3;
                searchStepStart = now;
            }
            break;
        case 3: // short stop then loop back
            if (elapsed < SEARCH_STOP_DURATION_MS) {
                car.stop();
            } else {
                // loop search sequence
                searchStep = 0;
                searchStepStart = now;
            }
            break;
        default:
            // Reset to be safe
            searchStep = 0;
            searchStepStart = now;
            break;
    }
}

void setup(){
  Serial.begin(115200);
  while (!Serial)
  {
    delay(1);
  }
  Serial.println("Car IR Sensor Control Project");
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  if (!adcSampler.begin(0x48, &Wire1, GAIN_ONE, 128)) {
      Serial.println("Failed to initialize ADS1115");
      while (1);
  }
  Serial.println("ADS1115 initialized successfully");
  car.initializeMotors(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN,
                       RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN,
                       MOTOR_FREQ);
  if (!car.isInitialized()) {
      Serial.println("Failed to initialize motors");
      while (1);
  }
  Serial.println("Motors initialized successfully");
  car.stop();
}

void searching_mode(){
    // Deprecated blocking function kept for compatibility
    // Use startSearchingMode() / updateSearchingMode() instead.
    startSearchingMode();
}

void loop(){
    // Read IR sensor values with non-blocking call
    if (millis() % 100 < 20) { // Read every 100 ms
        int16_t rawValues[IR_SENSOR_CHANNELS];
        float voltValues[IR_SENSOR_CHANNELS];
        adcSampler.readAll(rawValues, voltValues, IR_SENSOR_CHANNELS);
        Serial.print("IR Sensor Voltages: ");
        for (int i = 0; i < IR_SENSOR_CHANNELS; i++) {
            Serial.print(voltValues[i], 4);
            Serial.print(" V ");
        }
        Serial.println();
        // advanced no falling out of the table logic using switch-case
        bool on_line[IR_SENSOR_CHANNELS];
        for (int i = 0; i < IR_SENSOR_CHANNELS; i++) {
            on_line[i] = (voltValues[i] < ir_threshold);
        }
        switch ((on_line[0] << 3) | (on_line[1] << 2) | (on_line[2] << 1) | on_line[3]) {
            case 0b0000:
                stopSearchingMode();
                Serial.println("Path clear, moving forward");
                car.forward(70.0F);
                break;
            // 3 or more sensors active -> stop
            case 0b1111:
            case 0b1110:
            case 0b1101:
            case 0b1011:
            case 0b0111:
                stopSearchingMode();
                Serial.println("Obstacles detected on 3 or more sensors, stopping");
                car.stop();
                break;
            case 0b0001: //A0 only
                stopSearchingMode();
                Serial.println("Slight left adjustment");
                car.turnLeft(30.0F);
                break;
            case 0b0010: //A1 only
                stopSearchingMode();
                Serial.println("Slight right adjustment");
                car.turnRight(30.0F);
                break;
            case 0b0100: //A2 only
                stopSearchingMode();
                Serial.println("Slight left adjustment");
                car.turnLeft(30.0F);
                break;
            case 0b1000: //A3 only
                stopSearchingMode();
                Serial.println("Slight right adjustment");
                car.turnRight(30.0F);
                break;
            case 0b0011: //A0,A1 high
                stopSearchingMode();
                Serial.println("Turning right");
                car.turnRight(50.0F);
                break;
            case 0b1100: //A2,A3 high
                stopSearchingMode();
                Serial.println("Turning left");
                car.turnLeft(50.0F);
                break;
            case 0b1001: //A0,A3 high
                stopSearchingMode();
                Serial.println("Moving forward");
                car.forward(70.0F);
                break;
            case 0b0110: //A1,A2 high
                stopSearchingMode();
                Serial.println("Moving backward");
                car.backward(70.0F);
                break;
            default:
                // start the non-blocking searching behavior
                Serial.println("Unrecognized sensor pattern, entering searching mode");
                startSearchingMode();
                break;
        }
        // Always update search mode state (non-blocking) so it can run while loop continues
        updateSearchingMode();
    }
}