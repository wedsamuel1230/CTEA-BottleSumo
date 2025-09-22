/**************************************************************************
 ADS1115-Specific I2C Scanner for Raspberry Pi Pico
 
 This scanner extends the existing I2C_SCANNER functionality to specifically
 identify and test ADS1115 devices on the I2C bus.
 
 Features:
 - Scans entire I2C address range (like original scanner)
 - Specifically identifies ADS1115 devices
 - Tests ADS1115 register access
 - Displays device information for known addresses
 - Compatible with existing I2C setup (Wire1, SDA=26, SCL=27)
 **************************************************************************/

#include <Wire.h>

// Known I2C addresses for devices used in this repository
struct DeviceInfo {
  uint8_t address;
  const char* name;
  const char* description;
};

const DeviceInfo knownDevices[] = {
  {0x3C, "SSD1306", "OLED Display"},
  {0x29, "VL53L0X", "ToF Distance Sensor (default)"},
  {0x30, "VL53L1X", "ToF Distance Sensor (custom)"},
  {0x32, "VL53L1X", "ToF Distance Sensor (custom)"},
  {0x34, "VL53L1X", "ToF Distance Sensor (custom)"},
  {0x48, "ADS1115", "4-Channel 16-bit ADC (default)"},
  {0x49, "ADS1115", "4-Channel 16-bit ADC (ADDR=VDD)"},
  {0x4A, "ADS1115", "4-Channel 16-bit ADC (ADDR=SDA)"},
  {0x4B, "ADS1115", "4-Channel 16-bit ADC (ADDR=SCL)"}
};

const int numKnownDevices = sizeof(knownDevices) / sizeof(knownDevices[0]);

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  while (!Serial); // Wait for serial monitor
  
  // Initialize I2C using the same pattern as other sketches in this repo
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  
  Serial.println("ADS1115-Enhanced I2C Scanner");
  Serial.println("============================");
  Serial.println("I2C initialized: SDA=GP26, SCL=GP27");
  Serial.println("Scanning for I2C devices...\n");
}

void loop() {
  byte error, address;
  int nDevices = 0;
  int adsDevices = 0;
  
  Serial.println("Scanning I2C bus...");
  Serial.println();
  
  // Scan all possible I2C addresses
  for (address = 1; address < 127; address++) {
    // Test if device responds at this address
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();
    
    if (error == 0) {
      // Device found - print address
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Check if this is a known device
      const char* deviceName = getDeviceName(address);
      const char* deviceDesc = getDeviceDescription(address);
      
      if (deviceName) {
        Serial.print(" - ");
        Serial.print(deviceName);
        if (deviceDesc) {
          Serial.print(" (");
          Serial.print(deviceDesc);
          Serial.print(")");
        }
      } else {
        Serial.print(" - Unknown device");
      }
      
      Serial.println();
      
      // Special handling for ADS1115 devices
      if (isADS1115Address(address)) {
        testADS1115(address);
        adsDevices++;
      }
      
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
    }
  }
  
  // Summary
  Serial.println();
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.print(" device(s)");
    if (adsDevices > 0) {
      Serial.print(", including ");
      Serial.print(adsDevices);
      Serial.print(" ADS1115 device(s)");
    }
    Serial.println();
  }
  
  if (adsDevices > 0) {
    Serial.println("\nADS1115 devices are ready for use!");
    Serial.println("Try the ADS1115_test example to verify functionality.");
  }
  
  Serial.println("\n" + String('=', 50));
  delay(5000); // Wait 5 seconds before next scan
}

// Function to get device name from address
const char* getDeviceName(uint8_t address) {
  for (int i = 0; i < numKnownDevices; i++) {
    if (knownDevices[i].address == address) {
      return knownDevices[i].name;
    }
  }
  return nullptr;
}

// Function to get device description from address
const char* getDeviceDescription(uint8_t address) {
  for (int i = 0; i < numKnownDevices; i++) {
    if (knownDevices[i].address == address) {
      return knownDevices[i].description;
    }
  }
  return nullptr;
}

// Function to check if address could be an ADS1115
bool isADS1115Address(uint8_t address) {
  return (address >= 0x48 && address <= 0x4B);
}

// Function to test ADS1115 functionality
void testADS1115(uint8_t address) {
  Serial.print("  Testing ADS1115 at 0x");
  if (address < 16) Serial.print("0");
  Serial.print(address, HEX);
  Serial.println("...");
  
  // Try to read the config register (register 0x01)
  Wire1.beginTransmission(address);
  Wire1.write(0x01); // Config register
  byte error = Wire1.endTransmission();
  
  if (error == 0) {
    // Request 2 bytes from config register
    Wire1.requestFrom(address, 2);
    if (Wire1.available() >= 2) {
      uint8_t high = Wire1.read();
      uint8_t low = Wire1.read();
      uint16_t config = (high << 8) | low;
      
      Serial.print("  Config register: 0x");
      Serial.print(config, HEX);
      Serial.println(" - ADS1115 responding correctly!");
      
      // Test a simple conversion
      testADS1115Conversion(address);
    } else {
      Serial.println("  Warning: Could not read config register");
    }
  } else {
    Serial.println("  Error: Could not access ADS1115 registers");
  }
}

// Function to test ADS1115 conversion
void testADS1115Conversion(uint8_t address) {
  // Write config to start single conversion on channel 0
  uint16_t config = 0x8583; // Single shot, AIN0, +-4.096V, 128SPS
  
  Wire1.beginTransmission(address);
  Wire1.write(0x01); // Config register
  Wire1.write((config >> 8) & 0xFF); // High byte
  Wire1.write(config & 0xFF);        // Low byte
  Wire1.endTransmission();
  
  delay(10); // Wait for conversion
  
  // Read conversion result
  Wire1.beginTransmission(address);
  Wire1.write(0x00); // Conversion register
  Wire1.endTransmission();
  
  Wire1.requestFrom(address, 2);
  if (Wire1.available() >= 2) {
    uint8_t high = Wire1.read();
    uint8_t low = Wire1.read();
    int16_t result = (high << 8) | low;
    
    // Convert to voltage (assuming +-4.096V range)
    float voltage = result * 0.000125; // 4.096V / 32768
    
    Serial.print("  Test conversion (Ch0): ");
    Serial.print(result);
    Serial.print(" (");
    Serial.print(voltage, 3);
    Serial.println("V)");
  } else {
    Serial.println("  Warning: Could not read conversion result");
  }
}