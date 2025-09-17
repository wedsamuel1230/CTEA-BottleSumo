/*This is a example of motor testing
Make sure there is a common ground between mcu and motor, otherwise you wouldn't be able to control the speed of the motor using pwm.
To power the motor, connect the VCC and the GND  to the power supply, and turn the voltage to ~7V.*/

//pins for motor and button
const uint8_t BluePin1 = 17; //pwm speed control
const uint8_t BluePin2 = 18; //pwm speed control
const uint8_t WhitePin = 16; //pwm speed control
const uint8_t btn = 3 //for testing only

void move_straight(uint8_t speed_for_Motor1,uint8_t speed_for_motor2){
	analogWrite(BluePin1,speed_for_Motor1);
	analogWrite(BluePin2,speed_for_motor2);
}

void setup() {
  Serial.begin(9600);
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
	move_straight(0,0);//full speed
	delay(2500);
	move_straight(127,127);//half speed
	delay(2500);
	move_straight(255,255);//stop
	delay(2500);
/*
  analogWrite(BluePin, 127); //control the motor at half speed using pwm
  delay(2500); //wait for 2.5 second
  analogWrite(BluePin, 255);//stop
  delay(2500); //wait for 2.5 second
  analogWrite(BluePin, 0); //full speed
  delay(2500); //wait for 2.5 second
  analogWrite(BluePin, 255);//stop
  delay(2500); //wait for 2.5 second
*/
}


void loop1(){ //using 2nd core in rasberry pi pico 
  //look if the button pressed
  if (digitalRead(btn) == LOW){z
    digitalWrite(WhitePin, LOW); //turn the motor anti-clockwise
    Serial.println(F("Anti-clockwise"));
  }
  else {
    digitalWrite(WhitePin, HIGH);//turn the motor clockwise
  }
  delay(50);
}
