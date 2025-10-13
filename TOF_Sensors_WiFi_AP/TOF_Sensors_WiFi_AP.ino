#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Hardware Pin Definitions - 5 sensors
#define XSHUT_1 11 // GP11 - Right 45°
#define XSHUT_2 12 // GP12 - Right 22.5°
#define XSHUT_3 13 // GP13 - Front 0°
#define XSHUT_4 14 // GP14 - Left 22.5°
#define XSHUT_5 15 // GP15 - Left 45°

#define NUM_SENSORS 5
const uint8_t xshut_pins[NUM_SENSORS] = {XSHUT_1, XSHUT_2, XSHUT_3, XSHUT_4, XSHUT_5};
const uint8_t sensor_addresses[NUM_SENSORS] = {0x30, 0x31, 0x32, 0x33, 0x34};

Adafruit_VL53L0X sensors[NUM_SENSORS];

struct ToF_Reading {
  uint16_t dist[NUM_SENSORS];
  bool valid[NUM_SENSORS];
  uint8_t status[NUM_SENSORS];
};

// Direction labels
const char* directions[NUM_SENSORS] = {
  "RIGHT", "RIGHT_22", "FRONT", "LEFT_22", "LEFT"
};

bool init_tof_sensors() {
  Serial.println("Initializing 5 ToF sensors for MAXIMUM RANGE...");
  
  // Reset all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshut_pins[i], OUTPUT);
    digitalWrite(xshut_pins[i], LOW);
  }
  delay(100);

  // Power up each sensor one by one
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(xshut_pins[i], HIGH);
    delay(50);
    
    if (!sensors[i].begin(sensor_addresses[i], false, &Wire1)) {
      Serial.print("Failed to init sensor ");
      Serial.print(directions[i]);
      Serial.println("!");
      return false;
    }
    
    // MAXIMUM RANGE mode - 200ms timing budget
    sensors[i].setMeasurementTimingBudgetMicroSeconds(200000); // 200ms
    
    // VCSEL settings for absolute maximum range
    sensors[i].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    sensors[i].setVcselPulsePeriod(VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
    
    Serial.print(directions[i]);
    Serial.println(" sensor: MAXIMUM RANGE mode");
  }

  Serial.println("All 5 sensors initialized successfully!");
  return true;
}

ToF_Reading read_tof_sensors() {
  ToF_Reading reading;
  VL53L0X_RangingMeasurementData_t data;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors[i].rangingTest(&data, false);
    
    reading.status[i] = data.RangeStatus;
    
    // ACCEPT MORE STATUS CODES for maximum range
    // 0 = Good, 1 = Sigma fail, 2 = Signal fail, 3 = Min range fail
    // For 2m range, we need to be lenient with status 2 (weak signal)
    reading.valid[i] = (data.RangeStatus <= 3 && 
                        data.RangeMilliMeter < 2500 && 
                        data.RangeMilliMeter > 10);
    reading.dist[i] = reading.valid[i] ? data.RangeMilliMeter : 0;
  }

  return reading;
}

void print_tof_readings(ToF_Reading &reading) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(directions[i]);
    Serial.print(":");
    
    if (reading.valid[i]) {
      Serial.print(reading.dist[i]);
      Serial.print("mm");
    } else {
      Serial.print("ERR(");
      Serial.print(reading.status[i]);
      Serial.print(")");
    }
    Serial.print(" ");
  }
  Serial.println();
}

String get_shortest_direction(const ToF_Reading &reading) {
  uint16_t min_distance = 0xFFFF;
  String dir = "CLEAR";
  int min_sensor = -1;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (reading.valid[i] && reading.dist[i] < min_distance) {
      min_distance = reading.dist[i];
      dir = String(directions[i]);
      min_sensor = i;
    }
  }
  
  return dir;
}

uint16_t get_shortest_distance(const ToF_Reading &reading) {
  uint16_t min_dist = 0xFFFF;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (reading.valid[i] && reading.dist[i] < min_dist) {
      min_dist = reading.dist[i];
    }
  }
  
  return (min_dist < 0xFFFF) ? min_dist : 0;
}

void scanI2C() {
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  
  for (address = 1; address < 127; address++) {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify ToF sensors
      if (address >= 0x30 && address <= 0x34) {
        Serial.print(" (ToF #");
        Serial.print(address - 0x30 + 1);
        Serial.print(" - ");
        Serial.print(directions[address - 0x30]);
        Serial.print(")");
      }
      Serial.println();
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" devices");
  }
}

void print_statistics(ToF_Reading &reading) {
  // Count valid sensors
  int valid_count = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (reading.valid[i]) valid_count++;
  }
  
  Serial.print("Valid sensors: ");
  Serial.print(valid_count);
  Serial.print("/");
  Serial.println(NUM_SENSORS);
  
  if (valid_count > 0) {
    String shortest_dir = get_shortest_direction(reading);
    uint16_t shortest_dist = get_shortest_distance(reading);
    
    Serial.print("SHORTEST: ");
    Serial.print(shortest_dir);
    Serial.print(" @ ");
    Serial.print(shortest_dist);
    Serial.println("mm");
  } else {
    Serial.println("SHORTEST: CLEAR (no valid readings)");
  }
}

void setup() {
  Serial.begin(115200);
  
  // Wait for Serial with timeout
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000));

  Serial.println("\n\n=== 5 SENSORS - MAXIMUM RANGE MODE (Target: 2m) ===");
  
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();

  if (!init_tof_sensors()) {
    Serial.println("CRITICAL: ToF sensor initialization failed!");
    while (1) delay(1000);
  }
  
  Serial.println("\n5 ToF sensors ready! Testing maximum range...");
  Serial.println("TIP: Use white/reflective targets for best range\n");
  
  scanI2C();
  
  Serial.println("\n=== Starting readings ===");
  Serial.println("Format: RIGHT | RIGHT_22 | FRONT | LEFT_22 | LEFT\n");
}

void loop() {
  static unsigned long reading_count = 0;
  
  ToF_Reading tof_data = read_tof_sensors();
  
  // Print reading number
  Serial.print("[");
  Serial.print(reading_count++);
  Serial.print("] ");
  
  // Print all sensor data
  print_tof_readings(tof_data);
  
  // Print statistics
  print_statistics(tof_data);
  
  Serial.println("─────────────────────────────────────────");
  
  delay(500); // Slower for maximum accuracy (200ms per sensor + processing)
}