# ADS1115 4-Channel ADC Support for Raspberry Pi Pico

This repository now includes comprehensive support for the ADS1115 4-channel 16-bit ADC using I2C communication with the Raspberry Pi Pico in Arduino IDE.

## Overview

The ADS1115 provides significant advantages over the built-in Raspberry Pi Pico ADC:

- **16-bit resolution** (65,536 levels) vs 12-bit (4,096 levels)
- **4 independent channels** on a single I2C device
- **Programmable Gain Amplifier (PGA)** for optimal signal range
- **Differential measurement capability** for noise reduction
- **I2C interface** reduces pin usage compared to parallel ADC connections

## Hardware Connections

### Standard I2C Setup (Compatible with existing code)
```
ADS1115 Module    Raspberry Pi Pico
--------------    ------------------
VDD           ->  3.3V or 5V
GND           ->  GND
SDA           ->  GP26 (Pin 31)
SCL           ->  GP27 (Pin 32)
A0            ->  Sensor Channel 0
A1            ->  Sensor Channel 1
A2            ->  Sensor Channel 2
A3            ->  Sensor Channel 3
```

### I2C Address
- Default address: **0x48**
- Can be changed to 0x49, 0x4A, or 0x4B using ADDR pin

## Available Examples

### 1. Basic ADS1115 Test (`ADS1115_test/`)
- **Purpose**: Basic functionality test and register-level control
- **Features**: 
  - Direct register manipulation
  - All 4 channels reading
  - Voltage conversion
  - Connection verification
- **Use when**: Learning ADS1115 fundamentals or maximum control needed

### 2. Enhanced Line Tracker (`ADS1115_LineTracker/`)
- **Purpose**: Drop-in replacement for existing line tracking system
- **Features**:
  - Compatible with existing `state_combine.ino` logic
  - Enhanced 16-bit resolution
  - 4-sensor boundary detection
  - Same result encoding (binary flags)
- **Use when**: Upgrading existing line tracking for better precision

### 3. Library-Based Example (`ADS1115_Library_Example/`)
- **Purpose**: Simplified integration using Adafruit ADS1X15 library
- **Features**:
  - Easy-to-use API
  - Automatic voltage conversion
  - Differential reading support
  - Continuous reading mode
- **Use when**: Rapid prototyping or when library features are needed

## Integration with Existing Code

### Upgrading Line Tracking Systems

Replace existing analog reading code:
```cpp
// OLD: Using built-in ADC
int sensorValue = analogRead(A0);
float voltage = (sensorValue / 4095.0) * 5.0;

// NEW: Using ADS1115
int16_t rawValue = readADS1115Channel(0);
float voltage = convertToVoltage(rawValue);
```

### Maintaining Compatibility

The ADS1115 examples maintain the same function signatures and result encoding as existing code:

```cpp
// Same function signature as existing opti_linetracker_func.ino
int checkLineTrackerADS1115(float thresholdV);

// Same result encoding as state_combine.ino
// 0b0001 = Left sensor
// 0b0010 = Left-center sensor  
// 0b0100 = Right-center sensor
// 0b1000 = Right sensor
```

### I2C Bus Sharing

The ADS1115 uses the same I2C bus (Wire1) as existing devices:

```cpp
// Standard initialization (same as OLED, ToF sensors)
Wire1.setSDA(26);
Wire1.setSCL(27);
Wire1.begin();
```

Multiple I2C devices can coexist:
- OLED Display: 0x3C
- ADS1115 ADC: 0x48
- ToF Sensors: 0x29, 0x30, 0x32, 0x34

## Library Installation (for Library Example)

### Method 1: Arduino IDE Library Manager
1. Open Arduino IDE
2. Go to **Tools → Manage Libraries**
3. Search for "**Adafruit ADS1X15**"
4. Install the library by Adafruit

### Method 2: Manual Installation
1. Download from: https://github.com/adafruit/Adafruit_ADS1X15
2. Extract to Arduino libraries folder
3. Restart Arduino IDE

## Voltage Ranges and Gain Settings

| Gain Setting | Voltage Range | Resolution | Use Case |
|--------------|---------------|------------|----------|
| GAIN_TWOTHIRDS | ±6.144V | 3mV | 5V sensors |
| GAIN_ONE | ±4.096V | 2mV | 3.3V sensors (default) |
| GAIN_TWO | ±2.048V | 1mV | Low voltage sensors |
| GAIN_FOUR | ±1.024V | 0.5mV | Precision measurements |
| GAIN_EIGHT | ±0.512V | 0.25mV | Very low voltage |
| GAIN_SIXTEEN | ±0.256V | 0.125mV | Maximum precision |

## Troubleshooting

### Common Issues

1. **"ADS1115 not found" error**
   - Check I2C connections (SDA=26, SCL=27)
   - Verify I2C address (default 0x48)
   - Use I2C scanner to detect devices

2. **Incorrect voltage readings**
   - Check gain setting matches sensor voltage range
   - Verify reference voltage configuration
   - Ensure proper grounding

3. **Slow readings**
   - Adjust data rate setting (default 128 SPS)
   - Use continuous mode for faster sampling
   - Consider reading multiple channels in sequence

### I2C Scanner Integration

Use the existing `I2C_SCANNER/I2C_SCANNER.ino` to verify ADS1115 connection:
```
Expected output:
I2C device found at address 0x48  !
```

## Performance Comparison

| Feature | Internal ADC | ADS1115 |
|---------|--------------|---------|
| Resolution | 12-bit (4,096) | 16-bit (65,536) |
| Channels | 3 available | 4 channels |
| Interface | Direct GPIO | I2C |
| Noise immunity | Low | High |
| Sampling rate | Very fast | 8-860 SPS |
| Pin usage | 1 per channel | 2 pins total |

## Next Steps

1. **Test basic functionality**: Start with `ADS1115_test` example
2. **Verify I2C communication**: Use existing I2C scanner
3. **Integrate with line tracking**: Use `ADS1115_LineTracker` example
4. **Optimize for your sensors**: Adjust gain and threshold settings
5. **Add to main robot code**: Replace existing analogRead() calls

## Additional Resources

- [ADS1115 Datasheet](https://www.ti.com/lit/ds/symlink/ads1115.pdf)
- [Adafruit ADS1X15 Library Documentation](https://adafruit.github.io/Adafruit_ADS1X15/)
- [Raspberry Pi Pico I2C Guide](https://docs.arduino.cc/tutorials/nano-rp2040-connect/rp2040-openmv-setup)

## Examples Summary

| Example | Purpose | Complexity | Best For |
|---------|---------|------------|----------|
| `ADS1115_test` | Basic functionality | Low | Learning, testing |
| `ADS1115_LineTracker` | Line tracking upgrade | Medium | Drop-in replacement |
| `ADS1115_Library_Example` | Library-based approach | Low | Rapid development |

Choose the example that best fits your needs and experience level.