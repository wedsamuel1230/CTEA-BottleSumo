//This is a example of motor testing
//Make sure there is a common ground between mcu and motor, otherwise you wouldn't be able to control the speed of the motor using pwm.
//To power the motor, connect the VCC and the GND  to the power supply, and turn the voltage to ~7V.

//pins for motor and button
#define WhitePin 15 // Clockwise/Anti-clockwise
#define BluePin 15 //pwm speed control
#define btn 3 //for testing only

void setup() {
  Serial.begin(9600);
  //make the pins outputs 
  pinMode(WhitePin, OUTPUT);
  pinMode(BluePin, OUTPUT);
  pinMode(btn, INPUT_PULLUP);
}

void loop() {
  analogWrite(BluePin, 255); //control the motor at full speed using pwm
  analogWrite(WhitePin, 255); //control the motor at full speed using pwm
  delay(5000); //wait for 1 second
}

/*
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
*/
