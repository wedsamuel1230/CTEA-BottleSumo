//Make sure there is a common ground between mcu and motor, otherwise you wouldn't be able to control the speed of the motor using pwm.
//Also, connect VCC and GND of the motor using power supply, and set the voltage to ~7V.

#define WhitePin 16 // Clockwise/Anti-clockwise
#define BluePin 17 //pwm speed control
#define btn 3 //for testing only

void setup() {
  pinMode(WhitePin, OUTPUT);
  pinMode(BluePin, OUTPUT);
  pinMode(btn, INPUT_PULLUP);
}

void loop() {
  analogWrite(BluePin, 255); //full speed in 8bit analogWrite(default)
  delay(1000);
  analogWrite(BluePin, 127); //half speed in 8bit analogWrite(default)
  delay(1000);
}

void loop1(){ //using 2nd core in rasberry pi pico 
  if (digitalRead(btn) == 1){
    digitalWrite(BluePin, LOW);
    Serial.println(F("Anticlockwise"));
  }
  else {
    digitalWrite(BluePin, HIGH);
  }
}

