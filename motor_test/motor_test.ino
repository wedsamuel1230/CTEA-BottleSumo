/*This is a example of motor testing with OLED display
Make sure there is a common ground between mcu and motor, otherwise you wouldn't be able to control the speed of the motor using pwm.
To power the motor, connect the VCC and the GND  to the power supply, and turn the voltage to ~7V.*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

//pins for motor and button
const uint8_t BluePin1 = 17; //pwm speed control for motor 1
const uint8_t BluePin2 = 18; //pwm speed control for motor 2
const uint8_t WhitePin = 16; //direction control
const uint8_t btn = 3; //for testing only

void move_straight(uint8_t speed_for_Motor1, uint8_t speed_for_motor2){
  analogWrite(BluePin1, speed_for_Motor1);
  analogWrite(BluePin2, speed_for_motor2);
}

void setup() {
  Serial.begin(9600);
  
  // Initialize I2C for OLED on pins 26 and 27
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();

  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show startup message on OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Motor Test Started"));
  display.println(F("Press button to"));
  display.println(F("control motors"));
  display.println(F(""));
  display.println(F("Motor 1: Pin 17"));
  display.println(F("Motor 2: Pin 18"));
  display.display();
  delay(2000);

  //make the pins outputs 
  pinMode(BluePin1, OUTPUT);
  pinMode(BluePin2, OUTPUT);
  move_straight(0,0);
}

void setup1(){
  Serial.begin(9600);
  pinMode(WhitePin, OUTPUT);
  pinMode(btn, INPUT_PULLUP);
}

void loop() {
  move_straight(0,0);//stop motors
  delay(2500);
  move_straight(127,127);//half speed
  delay(2500);
  move_straight(255,255);//full speed
  delay(2500);
}


void loop1(){ //using 2nd core in raspberry pi pico 
  static bool lastButtonState = HIGH;
  static bool motorsRunning = false;
  
  bool currentButtonState = digitalRead(btn);
  
  // Detect button press (transition from HIGH to LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    motorsRunning = !motorsRunning; // Toggle motor state
    
    if (motorsRunning) {
      // Start both motors at full speed
      move_straight(255, 255);
      digitalWrite(WhitePin, HIGH); // Set direction clockwise
      
      // Update OLED display
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("MOTORS RUNNING"));
      display.println(F(""));
      display.println(F("Motor 1: ON"));
      display.println(F("Motor 2: ON"));
      display.println(F("Direction: CW"));
      display.println(F(""));
      display.println(F("Press button to stop"));
      display.display();
      
      Serial.println(F("Motors started - Clockwise"));
    } else {
      // Stop both motors
      move_straight(0, 0);
      digitalWrite(WhitePin, LOW);
      
      // Update OLED display
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println(F("MOTORS STOPPED"));
      display.println(F(""));
      display.println(F("Motor 1: OFF"));
      display.println(F("Motor 2: OFF"));
      display.println(F(""));
      display.println(F("Press button to start"));
      display.display();
      
      Serial.println(F("Motors stopped"));
    }
    
    delay(50); // Debounce delay
  }
  
  lastButtonState = currentButtonState;
  delay(10); // Small delay for stability
}
