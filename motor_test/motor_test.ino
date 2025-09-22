//This is a example of motor testing
//Make sure there is a common ground between mcu and motor, otherwise you wouldn't be able to control the speed of the motor using pwm.
//To power the motor, connect the VCC and the GND  to the power supply, and turn the voltage to ~7V.

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

//pins for motor and button
#define WhitePin 16 // Clockwise/Anti-clockwise
#define BluePin 17 //pwm speed control
#define btn 3 //for testing only

// OLED Display settings
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_RESET -1

// I2C setup for OLED
TwoWire i2c(i2c0, 26, 27); // SDA=GP26, SCL=GP27
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &i2c, OLED_RESET);

void setup() {
  Serial.begin(115200);
  
  //make the pins outputs 
  pinMode(WhitePin, OUTPUT);
  pinMode(BluePin, OUTPUT);
  pinMode(btn, INPUT_PULLUP);
  
  // Initialize I2C for OLED
  i2c.begin(26, 27);
  
  // Initialize OLED
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED initialization failed!");
    while (1);
  }
  
  // Display startup message
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.println("Motor Test Program");
  oled.println("==================");
  oled.println("Status: READY");
  oled.println();
  oled.println("Pin Config:");
  oled.println("White Pin: 16");
  oled.println("Blue Pin: 17");
  oled.println("Button: 3");
  oled.display();
  
  Serial.println("Motor Test Program Started");
  Serial.println("OLED Display Ready!");
  delay(2000);
}

void loop() {
  // Update OLED with motor status
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.println("Motor Test Running");
  oled.println("==================");
  oled.println();
  
  // Full speed phase
  oled.println("Phase: FULL SPEED");
  oled.println("PWM Value: 255");
  oled.println("Duration: 1000ms");
  oled.display();
  
  analogWrite(BluePin, 255); //control the motor at full speed using pwm
  delay(1000); //wait for 1 second
  
  // Half speed phase
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Motor Test Running");
  oled.println("==================");
  oled.println();
  oled.println("Phase: HALF SPEED");
  oled.println("PWM Value: 127");
  oled.println("Duration: 1000ms");
  oled.display();
  
  analogWrite(BluePin, 127); //control the motor at half speed using pwm
  delay(1000); //wait for 1 second
}

void loop1(){ //using 2nd core in rasberry pi pico 
  static unsigned long lastUpdate = 0;
  static bool lastButtonState = HIGH;
  
  //look if the button pressed
  bool currentButtonState = digitalRead(btn);
  
  if (digitalRead(btn) == 1){
    digitalWrite(BluePin, LOW); //turn the motor anti-clockwise
    Serial.println(F("Anti-clockwise"));
  }
  else {
    digitalWrite(BluePin, HIGH);//turn the motor clockwise
  }
  
  // Update OLED every 500ms or when button state changes
  if (millis() - lastUpdate > 500 || currentButtonState != lastButtonState) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);
    oled.println("Button Control Mode");
    oled.println("===================");
    oled.println();
    oled.print("Button State: ");
    oled.println(currentButtonState == LOW ? "PRESSED" : "RELEASED");
    oled.print("Motor Direction: ");
    oled.println(currentButtonState == LOW ? "CLOCKWISE" : "ANTI-CLOCKWISE");
    oled.println();
    oled.println("Press button to");
    oled.println("change direction");
    oled.display();
    
    lastUpdate = millis();
    lastButtonState = currentButtonState;
  }
}

