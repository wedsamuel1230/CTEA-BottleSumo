// ButtonManager: Debounced button sampling with time budget enforcement
#pragma once

#ifdef ARDUINO
#include <Arduino.h>
#else
typedef unsigned long uint32_t;
typedef unsigned char uint8_t;
void pinMode(uint8_t, uint8_t) {}
int digitalRead(uint8_t) { return 0; }
unsigned long millis() { return 0; }
#define INPUT_PULLUP 0
#define HIGH 1
#define LOW 0
#endif

enum ButtonMode {
  MODE_UNKNOWN = 0,
  MODE_TEST = 1,
  MODE_RUN = 2
};

class ButtonManager {
public:
  ButtonManager(uint8_t testPin, uint8_t runPin, uint32_t debounceMs = 20)
    : _testPin(testPin), _runPin(runPin), _debounceMs(debounceMs),
      _lastTestState(HIGH), _lastRunState(HIGH),
      _testPressed(false), _runPressed(false),
      _lastDebounceTime(0), _currentMode(MODE_UNKNOWN) {}

  void begin() {
    pinMode(_testPin, INPUT_PULLUP);
    pinMode(_runPin, INPUT_PULLUP);
    _lastTestState = digitalRead(_testPin);
    _lastRunState = digitalRead(_runPin);
  }

  // Sample buttons (5ms budget)
  // Returns true if sampling completed within budget
  bool sample(uint32_t budgetMs) {
    uint32_t start = millis();
    
    int testReading = digitalRead(_testPin);
    int runReading = digitalRead(_runPin);
    
    // Update state if changed (trigger debounce timer)
    if (testReading != _lastTestState || runReading != _lastRunState) {
      _lastDebounceTime = millis();
      _lastTestState = testReading;
      _lastRunState = runReading;
    }
    
    return (millis() - start) <= budgetMs;
  }

  // Debounce check (20ms budget)
  // Returns true if debouncing completed within budget
  bool debounce(uint32_t budgetMs) {
    uint32_t start = millis();
    
    // If debounce period elapsed, update button states
    if ((millis() - _lastDebounceTime) >= _debounceMs) {
      // Test button pressed (active LOW with pullup)
      if (_lastTestState == LOW && !_testPressed) {
        _testPressed = true;
        _currentMode = MODE_TEST;
      } else if (_lastTestState == HIGH) {
        _testPressed = false;
      }
      
      // Run button pressed (active LOW with pullup)
      if (_lastRunState == LOW && !_runPressed) {
        _runPressed = true;
        _currentMode = MODE_RUN;
      } else if (_lastRunState == HIGH) {
        _runPressed = false;
      }
    }
    
    return (millis() - start) <= budgetMs;
  }

  ButtonMode getMode() const { return _currentMode; }
  bool isTestMode() const { return _currentMode == MODE_TEST; }
  bool isRunMode() const { return _currentMode == MODE_RUN; }
  bool testButtonPressed() const { return _testPressed; }
  bool runButtonPressed() const { return _runPressed; }

private:
  uint8_t _testPin;
  uint8_t _runPin;
  uint32_t _debounceMs;
  int _lastTestState;
  int _lastRunState;
  bool _testPressed;
  bool _runPressed;
  uint32_t _lastDebounceTime;
  ButtonMode _currentMode;
};