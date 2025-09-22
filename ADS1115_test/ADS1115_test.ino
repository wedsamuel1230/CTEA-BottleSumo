/**************************************************************************
 ADS1115 4-Channel 16-bit ADC Test for Raspberry Pi Pico
 
 This sketch demonstrates how to use the ADS1115 4-channel ADC with
 Raspberry Pi Pico via I2C communication using Arduino IDE.
 
 ADS1115 Features:
 - 4-channel 16-bit ADC
 - I2C interface
 - Programmable gain amplifier (PGA)
 - Internal reference
 - Default I2C address: 0x48
 
 Connections:
 - VDD -> 3.3V or 5V
 - GND -> GND
 - SDA -> GP26 (Pin 31)
 - SCL -> GP27 (Pin 32)
 - A0-A3 -> Analog inputs (0-5V max depending on PGA setting)
 
 Compatible with existing I2C setup used in this repository.
 **************************************************************************/

#include <Wire.h>

// ADS1115 I2C address (default)
#define ADS1115_ADDRESS 0x48

// ADS1115 Register addresses
#define ADS1115_REG_CONVERSION   0x00
#define ADS1115_REG_CONFIG       0x01
#define ADS1115_REG_LO_THRESH    0x02
#define ADS1115_REG_HI_THRESH    0x03

// Configuration register bits
#define ADS1115_OS_SINGLE        0x8000  // Start single conversion
#define ADS1115_MUX_DIFF_0_1     0x0000  // Differential P = AIN0, N = AIN1
#define ADS1115_MUX_DIFF_0_3     0x1000  // Differential P = AIN0, N = AIN3
#define ADS1115_MUX_DIFF_1_3     0x2000  // Differential P = AIN1, N = AIN3
#define ADS1115_MUX_DIFF_2_3     0x3000  // Differential P = AIN2, N = AIN3
#define ADS1115_MUX_SINGLE_0     0x4000  // Single-ended AIN0
#define ADS1115_MUX_SINGLE_1     0x5000  // Single-ended AIN1
#define ADS1115_MUX_SINGLE_2     0x6000  // Single-ended AIN2
#define ADS1115_MUX_SINGLE_3     0x7000  // Single-ended AIN3

// Programmable Gain Amplifier (PGA) settings
#define ADS1115_PGA_6_144V       0x0000  // +/-6.144V range = Gain 2/3
#define ADS1115_PGA_4_096V       0x0200  // +/-4.096V range = Gain 1
#define ADS1115_PGA_2_048V       0x0400  // +/-2.048V range = Gain 2 (default)
#define ADS1115_PGA_1_024V       0x0600  // +/-1.024V range = Gain 4
#define ADS1115_PGA_0_512V       0x0800  // +/-0.512V range = Gain 8
#define ADS1115_PGA_0_256V       0x0A00  // +/-0.256V range = Gain 16

// Data rate settings
#define ADS1115_DR_8SPS          0x0000  // 8 samples per second
#define ADS1115_DR_16SPS         0x0020  // 16 samples per second
#define ADS1115_DR_32SPS         0x0040  // 32 samples per second
#define ADS1115_DR_64SPS         0x0060  // 64 samples per second
#define ADS1115_DR_128SPS        0x0080  // 128 samples per second (default)
#define ADS1115_DR_250SPS        0x00A0  // 250 samples per second
#define ADS1115_DR_475SPS        0x00C0  // 475 samples per second
#define ADS1115_DR_860SPS        0x00E0  // 860 samples per second

// Mode settings
#define ADS1115_MODE_CONTIN      0x0000  // Continuous conversion mode
#define ADS1115_MODE_SINGLE      0x0100  // Power-down single-shot mode (default)

// Comparator settings
#define ADS1115_CMODE_TRAD       0x0000  // Traditional comparator with hysteresis (default)
#define ADS1115_CMODE_WINDOW     0x0010  // Window comparator
#define ADS1115_CPOL_ACTVLOW     0x0000  // ALERT/RDY pin is low when active (default)
#define ADS1115_CPOL_ACTVHI      0x0008  // ALERT/RDY pin is high when active
#define ADS1115_CLAT_NONLAT      0x0000  // Non-latching comparator (default)
#define ADS1115_CLAT_LATCH       0x0004  // Latching comparator
#define ADS1115_CQUE_1CONV       0x0000  // Assert ALERT/RDY after one conversions
#define ADS1115_CQUE_2CONV       0x0001  // Assert ALERT/RDY after two conversions
#define ADS1115_CQUE_4CONV       0x0002  // Assert ALERT/RDY after four conversions
#define ADS1115_CQUE_NONE        0x0003  // Disable the comparator and put ALERT/RDY in high state (default)

// Default configuration
uint16_t config = ADS1115_OS_SINGLE |     // Start single conversion
                  ADS1115_MUX_SINGLE_0 |  // Single-ended AIN0
                  ADS1115_PGA_2_048V |    // +/-2.048V range
                  ADS1115_MODE_SINGLE |   // Single-shot mode
                  ADS1115_DR_128SPS |     // 128 samples per second
                  ADS1115_CMODE_TRAD |    // Traditional comparator
                  ADS1115_CPOL_ACTVLOW |  // Alert/Rdy active low
                  ADS1115_CLAT_NONLAT |   // Non-latching comparator
                  ADS1115_CQUE_NONE;      // Disable comparator

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for serial monitor
  
  // Initialize I2C using the same pattern as other sketches in this repo
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  
  Serial.println("ADS1115 4-Channel ADC Test");
  Serial.println("I2C initialized: SDA=GP26, SCL=GP27");
  
  // Check if ADS1115 is connected
  if (checkADS1115()) {
    Serial.println("ADS1115 found!");
  } else {
    Serial.println("ADS1115 not found. Check connections.");
    while(1); // Stop here if ADS1115 not found
  }
  
  Serial.println("\nReading all 4 channels...");
  Serial.println("Channel | Raw Value | Voltage (V)");
  Serial.println("--------|-----------|------------");
}

void loop() {
  // Read all 4 channels
  for (int channel = 0; channel < 4; channel++) {
    int16_t rawValue = readADS1115(channel);
    float voltage = convertToVoltage(rawValue);
    
    Serial.print("   ");
    Serial.print(channel);
    Serial.print("    |   ");
    Serial.print(rawValue);
    Serial.print("   | ");
    Serial.print(voltage, 3);
    Serial.println(" V");
  }
  
  Serial.println();
  delay(1000); // Wait 1 second before next reading
}

// Function to check if ADS1115 is connected
bool checkADS1115() {
  Wire1.beginTransmission(ADS1115_ADDRESS);
  return (Wire1.endTransmission() == 0);
}

// Function to read a specific channel (0-3)
int16_t readADS1115(uint8_t channel) {
  uint16_t mux;
  
  // Set the multiplexer for the specified channel
  switch (channel) {
    case 0: mux = ADS1115_MUX_SINGLE_0; break;
    case 1: mux = ADS1115_MUX_SINGLE_1; break;
    case 2: mux = ADS1115_MUX_SINGLE_2; break;
    case 3: mux = ADS1115_MUX_SINGLE_3; break;
    default: mux = ADS1115_MUX_SINGLE_0; break;
  }
  
  // Update configuration with the selected channel
  uint16_t channelConfig = (config & 0x8FFF) | mux;
  
  // Write configuration to start conversion
  writeRegister(ADS1115_REG_CONFIG, channelConfig);
  
  // Wait for conversion to complete
  delay(10); // Wait time depends on data rate setting
  
  // Read conversion result
  return readRegister(ADS1115_REG_CONVERSION);
}

// Function to convert raw ADC value to voltage
float convertToVoltage(int16_t rawValue) {
  // For PGA setting of +/-2.048V, each bit represents 2.048V/32767 = 0.0000625V
  // This gives us a range of +/-2.048V with 16-bit resolution
  return rawValue * 0.0000625;
}

// Function to write to ADS1115 register
void writeRegister(uint8_t reg, uint16_t value) {
  Wire1.beginTransmission(ADS1115_ADDRESS);
  Wire1.write(reg);
  Wire1.write((value >> 8) & 0xFF); // High byte
  Wire1.write(value & 0xFF);        // Low byte
  Wire1.endTransmission();
}

// Function to read from ADS1115 register
uint16_t readRegister(uint8_t reg) {
  Wire1.beginTransmission(ADS1115_ADDRESS);
  Wire1.write(reg);
  Wire1.endTransmission();
  
  Wire1.requestFrom(ADS1115_ADDRESS, 2);
  if (Wire1.available() >= 2) {
    uint8_t high = Wire1.read();
    uint8_t low = Wire1.read();
    return (high << 8) | low;
  }
  return 0;
}