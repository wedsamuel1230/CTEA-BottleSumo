//Make sure there is a common ground between mcu and motor, otherwise you wouldn't be able to control the speed of the motor using pwm.
//To power the motor, connect the VCC and the GND  to the power supply, and turn the voltage to ~7V.

//pins for motor and button
#define WhitePin 16 // Clockwise/Anti-clockwise
#define BluePin 17 //pwm speed control
#define btn 3 //for testing only

void setup() {
  //make the pins outputs 
  pinMode(WhitePin, OUTPUT);
  pinMode(BluePin, OUTPUT);
  pinMode(btn, INPUT_PULLUP);
}

void loop() {
  analogWrite(BluePin, 255); //control the motor at full speed using pwm
  delay(1000); //wait for 1 second
  analogWrite(BluePin, 127); //control the motor at half speed using pwm
  delay(1000); //wait for 1 second
}

void loop1(){ //using 2nd core in rasberry pi pico 
  //look if the button pressed
  if (digitalRead(btn) == 1){
    digitalWrite(BluePin, LOW); //turn the motor anti-clockwise
    Serial.println(F("Anti-clockwise"));
  }
  else {
    digitalWrite(BluePin, HIGH);//turn the motor clockwise
  }
}

