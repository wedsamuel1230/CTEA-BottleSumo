#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include "ToFArray.h"

// Pin definitions
const uint8_t TOF_XSHUT_PINS[5] = {8, 7, 6, 5, 4};
const uint8_t TOF_ADDRESSES[5] = {0x30, 0x31, 0x32, 0x33, 0x34};

ToFArray tof_array(&Wire1, nullptr);

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Initialize I2C
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  Wire1.setClock(400000);
  
  // Configure ToF sensors
  tof_array.configure(5, TOF_XSHUT_PINS, TOF_ADDRESSES);
  tof_array.setTiming(100000, 14, 10); // 50ms per sensor
  
  uint8_t online = tof_array.beginAll();
  Serial.printf("ToF sensors online: %d/5\n", online);
  
  // Print which sensors are online and their I2C addresses
  Serial.println("\n=== Sensor Status ===");
  for (int i = 0; i < 5; i++) {
    Serial.printf("Sensor %d (I2C: 0x%02X): %s\n", 
                  i, 
                  TOF_ADDRESSES[i],
                  tof_array.isOnline(i) ? "ONLINE" : "OFFLINE");
  }

  byte error, address;
  int nDevices;
 
  Serial.println("\n=== I2C Bus Scan ===");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.printf("I2C scan complete - %d devices found\n", nDevices);
 
  delay(2000);           // wait 2 seconds before starting loop
}

void loop() {
  ToFSample samples[5];
  tof_array.readAll(samples, 30, 1500, 2);
  
  Serial.println("\n=== ToF Sensor Readings ===");
  Serial.println("Sensor | I2C Addr | Distance | Status | Valid");
  Serial.println("-------|----------|----------|--------|------");
  
  for (int i = 0; i < 5; i++) {
    Serial.printf("   %d   |  0x%02X    | ", i, TOF_ADDRESSES[i]);
    
    if (samples[i].valid) {
      Serial.printf("%4d mm  |   %d    | YES\n", 
                    samples[i].distanceMm, 
                    samples[i].status);
    } else {
      Serial.printf("  --    |  0x%02X  | NO   (", samples[i].status);
      
      // Decode status codes
      switch(samples[i].status) {
        case 0xFF:
          Serial.print("OFFLINE");
          break;
        case 1:
          Serial.print("SIGMA FAIL");
          break;
        case 2:
          Serial.print("SIGNAL FAIL");
          break;
        case 4:
          Serial.print("OUT OF RANGE");
          break;
        default:
          Serial.printf("STATUS_%d", samples[i].status);
          break;
      }
      Serial.println(")");
    }
  }
  
  Serial.printf("\nValid readings: %d/5\n", 
                samples[0].valid + samples[1].valid + samples[2].valid + 
                samples[3].valid + samples[4].valid);
  
  delay(500); // Read every 500ms
}