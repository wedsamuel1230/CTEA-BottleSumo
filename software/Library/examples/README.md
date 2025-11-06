# Non-Blocking Sensor Examples

This folder contains example sketches demonstrating non-blocking sensor reading patterns for ToFArray and Ads1115Sampler.

## 📁 Examples Overview

### 1. **Minimal_SensorTest.ino** ⚡
- **Purpose:** Quick hardware validation
- **Complexity:** Beginner
- **Features:**
  - Basic non-blocking reads every 100ms
  - Simple serial output
  - ~50 lines of code
- **Use when:** Testing hardware setup

### 2. **SensorArray_NonBlocking.ino** 📊
- **Purpose:** Comprehensive sensor monitoring
- **Complexity:** Intermediate
- **Features:**
  - Time-sliced execution (ToF: 10 Hz, ADC: 20 Hz)
  - Formatted data display
  - Sensor fusion example
  - Status monitoring functions
- **Use when:** Learning non-blocking patterns

### 3. **RobotControl_Autonomous.ino** 🤖
- **Purpose:** Complete autonomous robot control
- **Complexity:** Advanced
- **Features:**
  - Integrated motor control (Car class)
  - Obstacle avoidance algorithm
  - Edge detection (line sensors)
  - Battery monitoring
  - State machine architecture
- **Use when:** Building autonomous robots

---

## 🔌 Hardware Requirements

### Common Components
- **Microcontroller:** RP2040 (Raspberry Pi Pico)
- **I2C Bus:** Wire (SDA/SCL)

### ToFArray (VL53L0X Sensors)
- **Quantity:** 1-5 sensors
- **Connections:**
  - I2C: SDA/SCL (shared bus)
  - XSHUT pins: Individual GPIO per sensor
- **I2C Addresses:** Configurable (default: 0x30-0x34)

### Ads1115Sampler (ADS1115 ADC)
- **Quantity:** 1 ADC
- **Connections:**
  - I2C: SDA/SCL (shared bus)
  - Analog inputs: A0-A3 (4 channels)
- **I2C Address:** 0x48 (default)

### Motor Control (for RobotControl_Autonomous.ino)
- **Motors:** 2 DC motors with H-bridge
- **Pins:** PWM and DIR for each motor

---

## ⚙️ Pin Configuration Examples

### Minimal Setup (3 ToF + ADS1115)
```
RP2040 Pinout:
├─ I2C0 (Wire)
│  ├─ SDA: GP4
│  └─ SCL: GP5
├─ ToF XSHUT Pins
│  ├─ Sensor 0: GP16
│  ├─ Sensor 1: GP17
│  └─ Sensor 2: GP18
└─ ADS1115: I2C address 0x48
```

### Full Robot Setup
```
RP2040 Pinout:
├─ I2C0 (Wire)
│  ├─ SDA: GP4
│  └─ SCL: GP5
├─ Motors
│  ├─ Left:  PWM=GP14, DIR=GP15
│  └─ Right: PWM=GP11, DIR=GP12
├─ ToF XSHUT Pins: GP16, GP17, GP18
└─ ADS1115 Channels
   ├─ A0: Battery voltage divider
   ├─ A1: Left line sensor
   ├─ A2: Center line sensor
   └─ A3: Right line sensor
```

---

## 🚀 Quick Start Guide

### 1. Install Required Libraries
```cpp
// Arduino Library Manager:
- Adafruit VL53L0X
- Adafruit ADS1X15
```

### 2. Copy Project Files
```
Library/
├── ToFArray.h
├── ToFArray.cpp
├── Ads1115Sampler.h
├── Ads1115Sampler.cpp
├── Car.h (if using motor control)
├── Car.cpp
├── Motor.h
└── Motor.cpp
```

### 3. Hardware Setup
1. Connect I2C devices to shared SDA/SCL
2. Connect ToF XSHUT pins to individual GPIOs
3. Power all devices (3.3V for I2C devices)
4. Add pull-up resistors on I2C lines (4.7kΩ recommended)

### 4. Test Sequence
1. Start with `Minimal_SensorTest.ino`
2. Verify all sensors detected
3. Check I2C addresses with scanner
4. Proceed to more complex examples

---

## 📖 Non-Blocking Pattern Explained

### Traditional Blocking Approach (❌ BAD)
```cpp
void loop() {
  readToFSensors();    // Blocks for 20-33ms
  readADC();           // Blocks for 30ms
  updateMotors();      // Blocks for 1ms
  delay(100);          // Blocks for 100ms
  // Total: ~150ms blocked = 6.6 Hz loop rate
}
```

### Non-Blocking Approach (✅ GOOD)
```cpp
void loop() {
  unsigned long now = millis();
  
  // Time-sliced execution
  if (now - lastToFRead >= 50) {
    lastToFRead = now;
    readToFSensors();  // Only when needed
  }
  
  if (now - lastADCRead >= 20) {
    lastADCRead = now;
    readADC();         // Only when needed
  }
  
  updateMotors();      // Runs every loop (~1000 Hz)
  delay(1);
}
```

### Benefits
- ✅ Higher control loop frequency
- ✅ Responsive motor control
- ✅ Independent sensor update rates
- ✅ Easy to add new tasks
- ✅ Better real-time performance

---

## 🎛️ Configuration Options

### ToF Timing Modes
```cpp
// Accurate mode (slower)
tof.setTiming(33000, 14, 10);  // 33ms budget

// Fast mode (less accurate)
tof.setTiming(20000, 12, 8);   // 20ms budget

// Ultra-fast mode (reduced range)
tof.setTiming(15000, 10, 6);   // 15ms budget
```

### ADC Sample Rates
```cpp
// Available rates (samples per second)
RATE_ADS1115_8SPS    // 8 SPS   (125ms per sample)
RATE_ADS1115_16SPS   // 16 SPS  (62.5ms per sample)
RATE_ADS1115_32SPS   // 32 SPS  (31.25ms per sample)
RATE_ADS1115_64SPS   // 64 SPS  (15.6ms per sample)
RATE_ADS1115_128SPS  // 128 SPS (7.8ms per sample) ← Default
RATE_ADS1115_250SPS  // 250 SPS (4ms per sample)
RATE_ADS1115_475SPS  // 475 SPS (2.1ms per sample)
RATE_ADS1115_860SPS  // 860 SPS (1.16ms per sample)
```

### ADC Gain Settings
```cpp
// Gain options (measurement range)
GAIN_TWOTHIRDS  // ±6.144V (default)
GAIN_ONE        // ±4.096V
GAIN_TWO        // ±2.048V
GAIN_FOUR       // ±1.024V
GAIN_EIGHT      // ±0.512V
GAIN_SIXTEEN    // ±0.256V
```

---

## 🐛 Troubleshooting

### I2C Errors
**Problem:** Sensors not detected
```
Solutions:
1. Check I2C wiring (SDA/SCL not swapped)
2. Add 4.7kΩ pull-up resistors
3. Reduce I2C speed: Wire.setClock(100000)
4. Scan I2C bus with scanner sketch
5. Check power supply (3.3V stable)
```

### ToF Sensors Offline
**Problem:** beginAll() returns 0
```
Solutions:
1. Verify XSHUT pins are unique per sensor
2. Check VL53L0X power (2.8V typical)
3. Add 10µF capacitor on VL53L0X VIN
4. Increase post-reset delay in ToFArray
5. Try sensors individually
```

### ADC Reading Errors
**Problem:** Readings stuck at 0 or max
```
Solutions:
1. Verify ADS1115 address (0x48 default)
2. Check ADDR pin connection (GND=0x48)
3. Reduce input voltage to gain range
4. Try different gain setting
5. Check analog signal connections
```

### Performance Issues
**Problem:** Slow loop rate
```
Solutions:
1. Reduce ToF timing budget
2. Increase ADC sample rate
3. Read fewer ADC channels
4. Reduce serial print frequency
5. Profile with micros() timing
```

---

## 📊 Performance Benchmarks

### Typical Loop Rates
| Configuration | Loop Rate | Notes |
|--------------|-----------|-------|
| 3 ToF + 4 ADC channels | ~800 Hz | Balanced |
| 5 ToF + 4 ADC channels | ~600 Hz | Max sensors |
| 1 ToF + 2 ADC channels | ~950 Hz | Minimal |

### Sensor Read Times
| Operation | Time | Blocking |
|-----------|------|----------|
| ToF readAll() (fast mode) | ~20ms | Yes |
| ToF readAll() (accurate) | ~33ms | Yes |
| ADC readAll() (4 ch @ 128SPS) | ~32ms | Yes |
| ADC readAll() (4 ch @ 860SPS) | ~5ms | Yes |

---

## 💡 Tips & Best Practices

### 1. **Start Simple**
Begin with Minimal_SensorTest.ino to verify hardware before adding complexity.

### 2. **Tune Timing**
Adjust read intervals based on your application:
- Fast obstacle detection: 20-50ms (20-50 Hz)
- Battery monitoring: 1000ms (1 Hz)
- Line following: 10-20ms (50-100 Hz)

### 3. **Monitor Performance**
Add timing diagnostics:
```cpp
unsigned long start = micros();
tof.readAll(data);
Serial.printf("ToF read: %lu us\n", micros() - start);
```

### 4. **Handle Edge Cases**
Always check sensor validity:
```cpp
if (tofData[i].valid) {
  // Use distance
} else {
  // Sensor out of range or error
}
```

### 5. **Optimize I2C**
- Use 400 kHz I2C speed: `Wire.setClock(400000)`
- Keep I2C wires short (<15cm ideal)
- Use shielded cable for long runs

---

## 📚 API Reference

### ToFArray Methods
```cpp
bool configure(count, xshutPins[], addresses[])  // Setup sensors
void setTiming(budget_us, preRange, finalRange)  // Adjust speed/accuracy
uint8_t beginAll()                               // Initialize all sensors
void readAll(ToFSample* out)                     // Read all sensors
bool isOnline(uint8_t idx)                       // Check sensor status
uint8_t size()                                   // Get sensor count
```

### Ads1115Sampler Methods
```cpp
bool begin(address, wire, gain, rate)            // Initialize ADC
void readAll(int16_t* raw, float* volts, count) // Read channels
bool isReady()                                   // Check initialization
```

### ToFSample Structure
```cpp
struct ToFSample {
  uint16_t distanceMm;  // Distance in millimeters
  uint8_t status;       // VL53L0X status code
  bool valid;           // true if reading is valid
};
```

---

## 🤝 Contributing

Found a bug or have an improvement? Please:
1. Test on hardware first
2. Document the change
3. Follow existing code style
4. Add comments for clarity

---

## 📄 License

These examples are provided as-is for educational purposes.

---

## 🔗 Related Documentation

- [ToFArray.h](../ToFArray.h) - Time-of-Flight sensor array driver
- [Ads1115Sampler.h](../Ads1115Sampler.h) - ADS1115 ADC wrapper
- [Car.h](../Car.h) - Differential drive robot control
- [Motor.h](../Motor.h) - PWM motor driver

---

**Questions?** Check the main Library.ino for additional examples.
