#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Hardware Pin Definitions
#define XSHUT_1 11 // GP11 - Right sensor
#define XSHUT_2 12 // GP12 - Front sensor  
#define XSHUT_3 13 // GP13 - Left sensor

// Sensor Configuration Constants
#define TIMING_BUDGET_US 20000          // 20ms per reading for ultra-fast mode
#define VCSEL_PRE_RANGE 18              // Pre-range pulse period (speed optimized)
#define VCSEL_FINAL_RANGE 14            // Final-range pulse period (speed optimized)
#define DETECTION_THRESHOLD_MM 1600     // 160cm - sumo ring detection distance
#define MAX_VALID_RANGE_MM 2000         // Maximum sensor range
#define MIN_VALID_RANGE_MM 30           // Minimum sensor range (avoid false positives)
#define LOOP_DELAY_MS 30                // 30ms = ~33Hz update rate
#define MAX_INIT_RETRIES 3              // Maximum sensor initialization attempts

Adafruit_VL53L0X lox1, lox2, lox3;

struct ToF_Reading {
  uint16_t right_distance;
  uint16_t front_distance;
  uint16_t left_distance;
  bool right_valid;
  bool front_valid;
  bool left_valid;
  uint8_t right_status;
  uint8_t front_status;
  uint8_t left_status;
};

<<<<<<< Updated upstream
=======
// ULTRA FAST initialization with retry mechanism
>>>>>>> Stashed changes
bool init_tof_sensors() {
  for (int attempt = 1; attempt <= MAX_INIT_RETRIES; attempt++) {
    Serial.print("Initialization attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(MAX_INIT_RETRIES);
    
    // Reset all sensors
    pinMode(XSHUT_1, OUTPUT);
    pinMode(XSHUT_2, OUTPUT);
    pinMode(XSHUT_3, OUTPUT);
    digitalWrite(XSHUT_1, LOW);
    digitalWrite(XSHUT_2, LOW);
    digitalWrite(XSHUT_3, LOW);
    delay(50); // Full reset delay
    
    bool all_sensors_ok = true;

<<<<<<< Updated upstream
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(50); // Faster startup

  // Right sensor - FAST + 1.5m range balance
  digitalWrite(XSHUT_1, HIGH);
  delay(30);
  if (!lox1.begin(0x30, false, &Wire1)) {
    Serial.println("Failed to init RIGHT sensor");
    return false;
  }
  
  // Optimal timing: 50ms = fast + good 1.5m range
  lox1.setMeasurementTimingBudgetMicroSeconds(100000); // 50ms
  lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  Serial.println("Right sensor: FAST mode (1.5m target)");

  // Front sensor - same config
  digitalWrite(XSHUT_2, HIGH);
  delay(30);
  if (!lox2.begin(0x31, false, &Wire1)) {
    Serial.println("Failed to init FRONT sensor");
    return false;
  }
  lox2.setMeasurementTimingBudgetMicroSeconds(100000);
  lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  Serial.println("Front sensor: FAST mode (1.5m target)");

  // Left sensor - same config
  digitalWrite(XSHUT_3, HIGH);
  delay(30);
  if (!lox3.begin(0x32, false, &Wire1)) {
    Serial.println("Failed to init LEFT sensor");
    return false;
  }
  lox3.setMeasurementTimingBudgetMicroSeconds(100000);
  lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
  Serial.println("Left sensor: FAST mode (1.5m target)");

  return true;
=======
    // Right sensor - ULTRA FAST timing
    digitalWrite(XSHUT_1, HIGH);
    delay(20); // Minimal delay
    if (!lox1.begin(0x30, false, &Wire1)) {
      Serial.println("Failed to init RIGHT sensor");
      all_sensors_ok = false;
      delay(100); // Wait before retry
      continue;
    }
    
    // ULTRA FAST: 20ms timing budget
    lox1.setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
    lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, VCSEL_PRE_RANGE);
    lox1.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, VCSEL_FINAL_RANGE);
    Serial.println("Right sensor: ULTRA FAST mode (20ms)");

    // Front sensor
    digitalWrite(XSHUT_2, HIGH);
    delay(20);
    if (!lox2.begin(0x31, false, &Wire1)) {
      Serial.println("Failed to init FRONT sensor");
      all_sensors_ok = false;
      delay(100);
      continue;
    }
    
    lox2.setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
    lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, VCSEL_PRE_RANGE);
    lox2.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, VCSEL_FINAL_RANGE);
    Serial.println("Front sensor: ULTRA FAST mode (20ms)");

    // Left sensor
    digitalWrite(XSHUT_3, HIGH);
    delay(20);
    if (!lox3.begin(0x32, false, &Wire1)) {
      Serial.println("Failed to init LEFT sensor");
      all_sensors_ok = false;
      delay(100);
      continue;
    }
    
    lox3.setMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
    lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, VCSEL_PRE_RANGE);
    lox3.setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, VCSEL_FINAL_RANGE);
    Serial.println("Left sensor: ULTRA FAST mode (20ms)");

    if (all_sensors_ok) {
      Serial.println("All sensors initialized successfully!");
      return true;
    }
  }
  
  Serial.println("CRITICAL: Failed to initialize all sensors after multiple retries!");
  return false;
>>>>>>> Stashed changes
}

ToF_Reading read_tof_sensors() {
  ToF_Reading reading;
  VL53L0X_RangingMeasurementData_t data1, data2, data3;

  lox1.rangingTest(&data1, false);
  lox2.rangingTest(&data2, false);
  lox3.rangingTest(&data3, false);

  reading.right_status = data1.RangeStatus;
  reading.front_status = data2.RangeStatus;
  reading.left_status = data3.RangeStatus;

<<<<<<< Updated upstream
  // Accept status 0-2 for 1.5m range
  reading.right_valid = (data1.RangeStatus <= 2 && data1.RangeMilliMeter < 2000 && data1.RangeMilliMeter > 10);
  reading.right_distance = reading.right_valid ? data1.RangeMilliMeter : 0;

  reading.front_valid = (data2.RangeStatus <= 2 && data2.RangeMilliMeter < 2000 && data2.RangeMilliMeter > 10);
  reading.front_distance = reading.front_valid ? data2.RangeMilliMeter : 0;

  reading.left_valid = (data3.RangeStatus <= 2 && data3.RangeMilliMeter < 2000 && data3.RangeMilliMeter > 10);
=======
  // Accept status 0-2 for reliable readings (0=valid, 1=sigma fail, 2=signal fail)
  // Status 4 (phase fail) often means no object - can cause false positives
  reading.right_valid = (data1.RangeStatus <= 2 && 
                         data1.RangeMilliMeter >= MIN_VALID_RANGE_MM && 
                         data1.RangeMilliMeter < MAX_VALID_RANGE_MM);
  reading.right_distance = reading.right_valid ? data1.RangeMilliMeter : 0;

  reading.front_valid = (data2.RangeStatus <= 2 && 
                         data2.RangeMilliMeter >= MIN_VALID_RANGE_MM && 
                         data2.RangeMilliMeter < MAX_VALID_RANGE_MM);
  reading.front_distance = reading.front_valid ? data2.RangeMilliMeter : 0;

  reading.left_valid = (data3.RangeStatus <= 2 && 
                        data3.RangeMilliMeter >= MIN_VALID_RANGE_MM && 
                        data3.RangeMilliMeter < MAX_VALID_RANGE_MM);
>>>>>>> Stashed changes
  reading.left_distance = reading.left_valid ? data3.RangeMilliMeter : 0;

  return reading;
}

<<<<<<< Updated upstream
=======
String get_object_direction(ToF_Reading &reading, uint16_t detection_threshold = DETECTION_THRESHOLD_MM) {
  String directions = "";

  if (reading.front_valid && reading.front_distance < detection_threshold) {
    directions += "FRONT ";
  }
  if (reading.right_valid && reading.right_distance < detection_threshold) {
    directions += "RIGHT ";
  }
  if (reading.left_valid && reading.left_distance < detection_threshold) {
    directions += "LEFT ";
  }

  if (directions.length() == 0) {
    return "CLEAR";
  }

  directions.trim();
  return directions;
}

// Compact printing for speed
>>>>>>> Stashed changes
void print_tof_readings(ToF_Reading &reading) {
  Serial.print("R:");
  if (reading.right_valid) {
    Serial.print(reading.right_distance);
  } else {
    Serial.print("ERR("); Serial.print(reading.right_status); Serial.print(")");
  }
  Serial.print(" F:");
  if (reading.front_valid) {
    Serial.print(reading.front_distance);
  } else {
    Serial.print("ERR("); Serial.print(reading.front_status); Serial.print(")");
  }
  Serial.print(" L:");
  if (reading.left_valid) {
    Serial.print(reading.left_distance);
  } else {
    Serial.print("ERR("); Serial.print(reading.left_status); Serial.print(")");
  }
  Serial.println();
}

String get_shortest_direction(const ToF_Reading &reading) {
  uint16_t min_distance = 0xFFFF;
  String dir = "CLEAR";
  
<<<<<<< Updated upstream
  if (reading.right_valid && reading.right_distance < min_distance) {
    min_distance = reading.right_distance;
    dir = "RIGHT";
  }
  if (reading.front_valid && reading.front_distance < min_distance) {
    min_distance = reading.front_distance;
    dir = "FRONT";
  }
  if (reading.left_valid && reading.left_distance < min_distance) {
    min_distance = reading.left_distance;
    dir = "LEFT";
  }
  return dir;
=======
  Serial.print(" -> ");
  Serial.println(get_object_direction(reading, DETECTION_THRESHOLD_MM));
>>>>>>> Stashed changes
}

void scanI2C() {
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) Serial.println("No I2C devices found");
  else Serial.println("done");
}

void setup() {
  Serial.begin(115200);
  // Wait for Serial with timeout (3 seconds) for GUI debugging
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000));

  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();

  Serial.println("=== FAST MODE (Target: 1.5m) ===");
  if (!init_tof_sensors()) {
    Serial.println("ToF sensor initialization failed!");
    while (1);
  }
  
<<<<<<< Updated upstream
  Serial.println("ToF sensors ready! Fast readings with 1.5m range");
  scanI2C();
  Serial.println("Starting FAST readings...");
=======
  Serial.println("ToF sensors ready! ULTRA FAST mode");
  scanI2C(); // Debug: Verify all 3 sensors have unique I2C addresses (0x30, 0x31, 0x32)
  Serial.println("Starting ULTRA FAST readings...");
>>>>>>> Stashed changes
}

void loop() {
  ToF_Reading tof_data = read_tof_sensors();
  print_tof_readings(tof_data);
  
<<<<<<< Updated upstream
  String shortest_dir = get_shortest_direction(tof_data);
  Serial.print("SHORTEST: ");
  Serial.print(shortest_dir);
  
  // Show the actual shortest distance value
  uint16_t min_dist = 0xFFFF;
  if (tof_data.right_valid && tof_data.right_distance < min_dist) min_dist = tof_data.right_distance;
  if (tof_data.front_valid && tof_data.front_distance < min_dist) min_dist = tof_data.front_distance;
  if (tof_data.left_valid && tof_data.left_distance < min_dist) min_dist = tof_data.left_distance;
  
  if (min_dist < 0xFFFF) {
    Serial.print(" @ ");
    Serial.print(min_dist);
    Serial.println("mm");
  } else {
    Serial.println();
  }

  delay(100); // FAST: ~10 readings per second
=======
  String direction = get_object_direction(tof_data, DETECTION_THRESHOLD_MM);
  if (direction != "CLEAR") {
    Serial.print("DETECTED: ");
    Serial.println(direction);
  }
  
  delay(LOOP_DELAY_MS); // TARGET: ~33 readings per second!
>>>>>>> Stashed changes
}