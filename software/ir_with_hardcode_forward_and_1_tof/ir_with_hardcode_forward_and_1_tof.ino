//this is a car ir sensor control with button pressed on-start and hard code forward project

#include <Wire.h>
#include "Car.h"
#include "Ads1115Sampler.h"
#include "Adafruit_VL53L0X.h"

#define SPIN_SEARCH_TIME 2500
#define FOWARD_SERACH_TIME 3500

//--- button
constexpr uint8_t Start_Button_PIN = 28; 
bool is_car_started = false;

//--- tof
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool first_search = true;
uint8_t state = 1;
bool _is_found_by_ds = false;
uint8_t is_spining = 0;
uint8_t start_squence = 3;
bool is_attackinging = false;
bool danger_zone = 1;
unsigned long danger_timer = 0;
unsigned long spin_timer = 0;
unsigned long start_timer = 0;

//---
constexpr uint8_t LEFT_MOTOR_PWM_PIN = 11;
constexpr uint8_t LEFT_MOTOR_DIR_PIN = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 14;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 15;
constexpr uint32_t MOTOR_FREQ = 40000; // 40kHz (safe for motor drivers)
constexpr uint8_t IR_SENSOR_CHANNELS = 4;
constexpr uint8_t IR_SENSOR_PINS[IR_SENSOR_CHANNELS] = {0, 1, 2, 3}; // A0 to A3
constexpr uint8_t ADS1115_I2C_ADDRESS = 0x48;

// Dynamic threshold variables
float ir_threshold_front = 1.5F; // Voltage threshold for A1, A2 (front sensors)
float ir_threshold_back = 3.0F;  // Voltage threshold for A0, A3 (back sensors)
float threshold_min = 0.0F; // Calibration: min voltage seen
float threshold_max = 5.0F; // Calibration: max voltage seen
bool auto_threshold_enabled = false;
unsigned long calibration_start = 0;
const unsigned long CALIBRATION_DURATION_MS = 3000UL; // 3 seconds to calibrate

// Edge detection confirmation
uint8_t pattern = 0;
uint8_t last_pattern = 0b0000;
bool edge_verification_mode = false; // New: waiting to verify edge after stopping
unsigned long edge_stop_time = 0;
const unsigned long EDGE_VERIFY_DELAY_MS = 100UL; // 100ms to verify after stop

// Emergency escape mode
bool emergency_mode = false;
unsigned long emergency_start = 0;
const unsigned long EMERGENCY_DURATION_MS = 1500UL; // 1.5 seconds escape
const float ESCAPE_SPEED = 50.0F; // 50% speed is enough

// Back sensor emergency (drive forward until safe)
bool back_escape_mode = false;
const float BACK_ESCAPE_SPEED = 1.0F; // Max forward speed when rear is off the edge

// Auto-start configuration
const unsigned long AUTO_START_DELAY_MS = 3000UL; // 3 second delay before auto-start
bool auto_start_enabled = false; // Set to true for standalone operation
unsigned long startup_time = 0;

/*
A3 (Channel 3) - Bottom-Right IR Sensor
A2 (Channel 2) - Top-Right IR Sensor
A1 (Channel 1) - Top-Left IR Sensor
A0 (Channel 0) - Bottom-Left IR Sensor

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

Bit Pattern Encoding: [A3][A2][A1][A0]
Example: 0b1010 means A3=1, A2=0, A1=1, A0=0

Behavior Logic:
- Voltage < threshold → Sensor detects line (bit=1)
- Voltage > threshold → Sensor sees clear path (bit=0)

Actions:
0b0000 - All clear → Forward (70%)
0b1111/0b1110/0b1101/0b1011/0b0111 - 3+ sensors → STOP
0b0001 - A0 (bottom-left) → Turn Right (30%)
0b0010 - A1 (top-left) → Turn Right (30%)
0b0100 - A2 (top-right) → Turn Left (30%)
0b1000 - A3 (bottom-right) → Turn Left (30%)
0b0011 - A0+A1 (left side) → Turn Right (50%)
0b1100 - A2+A3 (right side) → Turn Left (50%)
0b0101 - A0+A2 (diagonal) → Slight Right (40%)
0b1010 - A1+A3 (diagonal) → Slight Left (40%)
0b1001 - A0+A3 (bottom) → Forward (70%)
0b0110 - A1+A2 (top) → Backward (70%)
Other patterns → Enter search mode
*/

Car car;
Ads1115Sampler adcSampler;

const unsigned long SEARCH_DURATION_MS = 500UL;


char work_with_distance_sensor(void);

void setup(){
  Serial.begin(115200);
  delay(1000);
  Serial.println("Car IR Sensor Control Project");
  
  // Record startup time
  startup_time = millis();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Start_Button_PIN, INPUT_PULLUP);// Set start button pin with pull-up resistor
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  if (!adcSampler.begin(0x48, &Wire1, GAIN_ONE,RATE_ADS1115_860SPS)) {
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

  if (!lox.begin(0x30,false,&Wire1)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }

  // Print help on startup
  Serial.println("\n=== Dynamic Threshold Control ===");
  Serial.println("Press Start Button to begin sequence.");
  Serial.println("Type 'help' or '?' for commands");
  Serial.printf("Front threshold (A1,A2): %.3fV\n", ir_threshold_front);
  Serial.printf("Back threshold (A0,A3): %.3fV\n", ir_threshold_back);
  Serial.println("=================================\n");
  while (digitalRead(Start_Button_PIN) == HIGH) {}

  car.forward(1.0F); 
  start_timer = millis();
}

void loop(){
  // Read IR sensor values with non-blocking call (proper timing)
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead >= 10) { // Read every 10ms for fast response
    lastSensorRead = millis();
    
    int16_t rawValues[IR_SENSOR_CHANNELS];
    float voltValues[IR_SENSOR_CHANNELS];
    adcSampler.readAll(rawValues, voltValues, IR_SENSOR_CHANNELS);
    
    // Update calibration if active
    
    // Don't process sensor logic during calibration
    if (auto_threshold_enabled) {
        car.stop();
        return;
    }
    
    // Determine which sensors detect EDGE (voltage > threshold = out of area)
    bool edge_detected[IR_SENSOR_CHANNELS];
    // A0, A3 use back threshold; A1, A2 use front threshold
    pattern = 0;
    if(voltValues[0] > ir_threshold_back){pattern = 1;}  // A0 - back-left
    if(voltValues[1] > ir_threshold_front){pattern |= 2;}  // A0 - back-left
    if(voltValues[2] > ir_threshold_front){pattern |= 4;}  // A0 - back-left
    if(voltValues[3] > ir_threshold_back){pattern |= 8;}  // A0 - back-left
  }
  switch (state){
    case 0://search 
      car.setMotors(30,-60);
      state = 1;
      Serial.printf("State:%d\n",state);
    break;

    case 1: //wait for input
      if (pattern != 0){
        start_squence = 0;
        //Serial.printf("pattern: %x", pattern);
        Serial.println("not = 0 entered");
        is_attackinging = false;
        is_spining = 0;
        last_pattern = (pattern&0x0f);
        digitalWrite(LED_BUILTIN,LOW);
        if ((pattern&0x0f) == 0b0110){//A1 A2 front > backward
          car.backward(1);
        }
        else if (((pattern&0x0f) == 0b1001)||((pattern&0x0f) == 0b0001)||((pattern&0x0f) == 0b1000)){//A0 A3 back > forward
          car.forward(1);
        }
        else if (((pattern&0x0f) == 0b0100) || ((pattern&0x0f)==0b1100)){//A2 right front > forward-rught
          car.setMotors(-20,20);
          danger_zone = 1;//right
        }
        else if (((pattern&0x0f) == 0b0010) || ((pattern&0x0f) == 0b0011)){ // A1 left-front > forward-left
          car.setMotors(-20,20);
          danger_zone =0;//left
        }
        else{
          state = 0;//unkown > search
        }
        state = 2;
        Serial.printf("State:%d\n",state);
      }
      else{
        if (start_squence != 0){
          if (start_squence == 3){
            if (millis() - start_timer >= 1500){
            car.turnRight(1);
            start_squence--;
            start_timer = millis();            
            }
          }
          else if (start_squence == 2){
            if (millis() - start_timer >= 300){
            car.forward(1);
            start_squence--;
            start_timer = millis();            
            }
          }
          else if (start_squence == 1){
            if (millis() - start_timer >= 3500){
              start_squence = 0;
              state = 0;
            }
          }
        }
        else{
          if (millis() - danger_timer >= 1000){
            if (work_with_distance_sensor() == 'Y'  ){
              if ( !is_attackinging){
                is_attackinging = true;
                is_spining = 0;

                digitalWrite(LED_BUILTIN,HIGH);//led on if tof see object
                state = 4;//attack
                Serial.printf("State:%d\n",state);
              }
            }
            else if(is_attackinging){
              is_attackinging = false;
              is_spining = 5;

              state = 5;
              spin_timer = millis();
            }
            else if (is_spining != 0){
              if (is_spining == 3){
                if ((millis() - spin_timer)   >= 3000){
                  state = 0;
                  spin_timer = millis();
                  is_spining = 0;
                }
              }
              else if(is_spining == 5){
                if ((millis() - spin_timer) >= 1000){
                  state = 3;
                  is_spining = 3;
                  spin_timer = millis();
                }
              }
              else if(is_spining == 6){
                if ((millis() - spin_timer) >= SPIN_SEARCH_TIME){
                  state = 0;
                  is_spining = 0;
                }
              }
              else if(is_spining == 7){
                digitalWrite(LED_BUILTIN,LOW);//
                spin_timer = millis();
                if (danger_zone == 1){
                  state = 3;//right 
                  is_spining = 6;
                }
                else{
                  state = 5;//left
                  is_spining = 6; 
                }          
              }
            }
          }
          else{
            if (millis() - spin_timer >= FOWARD_SERACH_TIME){
              state = 3;//right 
              is_spining = 6;
            }
          }
        }
      }  
    break;

    case 2:   //retreating
      if (((pattern&0x0f) == 0) || ((pattern&0x0f) != last_pattern) ){
        danger_timer = millis();
        is_attackinging = false;
        is_spining = 7;        
        state  = 1;
        Serial.printf("pattern:%d last_pattern:%d\n", pattern, last_pattern);
        
        Serial.printf("State:%d\n",state);
      }
    break;

    case 3:   //spining right
      car.setMotors(50,50);
      Serial.printf("State:%d\n",state);
      state = 1;
    break;

    case 4: //attacking
      car.forward(1);
      state = 1;
      Serial.printf("State:%d\n",state);
    break;

    case 5://spining left
      car.setMotors(-50,-50);
      state = 1;
    break;

    case 6:
      car.forward(1);
      start_timer = millis();
      state = 1;
    break;
  }

}

// tof sensor
char work_with_distance_sensor(void){
  VL53L0X_RangingMeasurementData_t measure;
    
//  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    if ((measure.RangeMilliMeter < 350)&&(measure.RangeMilliMeter !=0)){
      return 'Y';
    }
  }
  return 'N';
}

