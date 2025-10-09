/*This is a example of motor testing
Make sure there is a common ground between mcu and motor, otherwise you wouldn't be able to control the speed of the motor using pwm.
To power the motor, connect the VCC and the GND  to the power supply, and turn the voltage to ~7V.*/

//pins for motor and button
const uint8_t LeftMotorPWM = 0; //pwm speed control
const uint8_t RightMotorPWM = 1; //pwm speed control
const uint8_t WhitePin = 16; //pwm speed control
const uint8_t btn = 3; //for testing only



void setup() {
  Serial.begin(9600);
  //make the pins outputs 
  pinMode(LeftMotorPWM, OUTPUT);
	pinMode(RightMotorPWM, OUTPUT);
  pinMode(btn, INPUT_PULLUP);
}

void move_straight(uint8_t speed_for_Motor1 = 0,uint8_t speed_for_motor2 = 0){
	analogWrite(LeftMotorPWM,speed_for_Motor1);
	analogWrite(RightMotorPWM,speed_for_motor2);
}
void loop() {
  if (digitalRead(btn) == LOW){
    move_straight(255,255);
  }
  else {
    move_straight();
  }
  delay(50);
  /*
	move_straight(0,0);//full speed
	delay(2500);
	move_straight(127,127);//half speed
	delay(2500);
	move_straight(255,255);//stop
	delay(2500);
  */
}
