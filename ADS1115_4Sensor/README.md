# ADS1115 4-Sensor ADC for Raspberry Pi Pico

This directory contains Arduino IDE code for using an ADS1115 ADC module with Raspberry Pi Pico to read 4 analog sensors via I2C.

## Hardware Requirements

- Raspberry Pi Pico
- ADS1115 16-bit ADC module
- 4 analog sensors (any voltage-output sensors)
- Jumper wires

## Wiring

### I2C Connection
- **Pico GP26** → **ADS1115 SDA**
- **Pico GP27** → **ADS1115 SCL**
- **Pico 3.3V** → **ADS1115 VDD**
- **Pico GND** → **ADS1115 GND**

### Sensor Connections
- **Sensor 1** → **ADS1115 A0**
- **Sensor 2** → **ADS1115 A1**
- **Sensor 3** → **ADS1115 A2**
- **Sensor 4** → **ADS1115 A3**

## Library Requirements

Install these libraries in Arduino IDE:
1. **Adafruit ADS1X15** library
2. **Wire** library (built-in)

## Files

### ADS1115_4Sensor.ino
Full-featured implementation with:
- Comprehensive sensor reading functions
- Voltage conversion and raw value display
- Sensor state calculation with thresholds
- Status indicators (HIGH/MID/LOW)
- Utility functions for individual sensor access

### ADS1115_Simple.ino
Simplified version following repository patterns:
- Basic 4-sensor reading
- Voltage threshold checking
- Binary result calculation
- Matches existing line tracker code style

## Usage

1. Open Arduino IDE
2. Install required libraries
3. Select "Raspberry Pi Pico" board
4. Load either sketch file
5. Upload to Pico
6. Open Serial Monitor (9600 baud)

## ADS1115 Configuration

- **I2C Address**: 0x48 (default)
- **Voltage Range**: ±4.096V (GAIN_ONE)
- **Resolution**: 16-bit (15-bit + sign)
- **Sample Rate**: Configurable via library

## Output Format

The code outputs sensor readings in this format:
```
Sensor1 (A0): 2.345V (Raw: 15234) [MID]
Sensor2 (A1): 0.123V (Raw: 800) [LOW]
Sensor3 (A2): 3.876V (Raw: 25200) [HIGH]
Sensor4 (A3): 1.567V (Raw: 10200) [MID]
Combined State (Binary): 1100 (Decimal: 12)
```

## Customization

- Adjust `threshold` value to change sensor activation levels
- Modify `GAIN` setting for different voltage ranges
- Change delay values for different reading frequencies
- Customize sensor names and channel assignments

## Troubleshooting

- Ensure correct I2C wiring (SDA/SCL)
- Check ADS1115 I2C address (use I2C scanner if needed)
- Verify 3.3V power supply to ADS1115
- Make sure sensors are properly connected to ADS1115 channels