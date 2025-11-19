//-- my script
//#include "Wifi_server.h"
#include "HUSKYLENS.h"
#include "Motor.h"
#include "button.h"
#include "Adafruit_VL53L0X.h"
#include "light_sensor.h"
#include "io_light.h"



/*#ifndef STASSID
#define STASSID "Lourenco-pico"
#define STAPSK "1234"
#endif*/


/*#ifndef STASSID
#define STASSID "Rm604@WiFi"
#define STAPSK "Rm604@WiFi01"
#endif*/


//Wifi_server Wifi_server(STASSID, STAPSK);

//---distance sensor---
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//---huskeylens---
HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA(4); blue line >> SCL(5)
const int LOWER_CENTER_RANGE = 120;
const int UPPER_CENTER_RANGE = 200;


//----- motor
const int M1_PWM = 16;
const int M1_DIR = 17;
const int M1_ENC = 18;

const int M2_PWM = 22;
const int M2_DIR = 26;
const int M2_ENC = 27;

Motor* Motor_1;
Motor* Motor_2;

//----- button
const int Btn_G_Pin = 15;
const int Btn_B_Pin = 14;
Button* Btn_G;
Button* Btn_B;

//---light sensor---
const int light_sensor_L_pin = 21;
const int light_sensor_R_pin = 28;
light_sensor* light_sensor_L;
light_sensor* light_sensor_R;

io_light* io_light_class;
//variable
bool _is_inside_table = true;
bool _is_found_by_ds = false;
int search_count = 0;
bool frist_search = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  io_light_class = new io_light(LED_BUILTIN);
  io_light_class -> switch_light(1);

  //---motor
  Motor_1 = new Motor(M1_DIR, M1_PWM);
  Motor_2 = new Motor(M2_DIR, M2_PWM);

  //---button
  Btn_G = new Button(Btn_G_Pin);
  Btn_B = new Button(Btn_B_Pin);


  //---huskylens
  Wire.begin();
  while (!huskylens.begin(Wire))
  {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }

  huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION); //Switch the algorithm to object tracking.

  //---distance sensor
  
  Serial.println("Adafruit VL53L0X test");
  Wire.begin();
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  //---light sensor
  light_sensor_L = new light_sensor(light_sensor_L_pin);
  light_sensor_R = new light_sensor(light_sensor_R_pin);

  bool is_started = false;
  while (!is_started){
    if (Btn_G -> is_button_pressed()){
      is_started = true;
    }
  }

  //webserver
  //Wifi_server.begin();

}

void loop() {
  /*Wifi_server.handleClient();
  switch(Wifi_server.get_command()){
    case 'F':
      //motor control forward
      car_forward(255);
      break;

    case 'B':
      //motor control backward
      car_backward(255);
      break;

    case 'L':
      //motor control Left
      car_turn_left(255);
      break;

    case 'R':
      //motor control right
      car_turn_right(255);
      break;

    case 'O':
      //motor control stop
      Motor_1 -> stop_motor();
      Motor_2 -> stop_motor();
      break;

    default:
      break;
  };*/

  switch(work_with_light_sensor()){
    case 'B':
      //motor control forward
      _is_inside_table = false;
      car_backward(180);
      delay(200);
      car_turn_left(255);
      delay(200);
      
      break;

    case 'W':
      //motor control forward
      _is_inside_table = false;
      car_backward(180);
      delay(400);
      
      break;

    case 'N':
      _is_inside_table = true;
      io_light_class -> switch_light(1);
      /*if (!_is_found_by_ds){
        stop_car();
      }*/
      break;

    default:
      break;
  };

  if (_is_inside_table){
    Serial.print("is_inside");
    switch(work_with_distance_sensor()){
      case 'Y':
        //motor control forward
        if (frist_search){
          stop_car();
          frist_search = false;
        }else{

          car_forward(255);
        }
        _is_found_by_ds = true;
        break;

      case 'N':
        _is_found_by_ds = false;
        car_turn_left(180);
        break;

      default:
        break;
    };

  }

  /*if (!_is_found_by_ds){
    switch(work_with_huskeylens()){
      case 'F':
        //motor control forward
        car_forward(255);
        break;

      case 'L':
        car_turn_left(255);
        break;

      case 'R':
        car_turn_left(255);
        break;

      case 'N':
        break;

      default:
        break;
    };
  }*/
  

  //--Button--
  if (Btn_G -> is_button_pressed()){
    Serial.println("G pressed");
  }

  if (Btn_B -> is_button_pressed()){
    Serial.println("B pressed");
  }
  delay(10);
}


//--- other function ---//
void car_forward(int speed){
  Motor_1 -> set_speed(-speed);
  Motor_2 -> set_speed(speed);
}

void car_backward(int speed){
  Motor_1 -> set_speed(speed);
  Motor_2 -> set_speed(-speed);
}

void car_turn_left(int speed){
  Motor_1 -> set_speed(-speed);
  Motor_2 -> set_speed(-speed);
}

void car_turn_right(int speed){
  Motor_1 -> set_speed(speed);
  Motor_2 -> set_speed(speed);
}

void stop_car(){
  Motor_1 -> stop_motor();
  Motor_2 -> stop_motor();
}

char work_with_huskeylens(){
  if (!huskylens.request()) Serial.println(F("Fail to request objects from HUSKYLENS!"));
  else if(!huskylens.isLearned()) {Serial.println(F("Object not learned!"));}
  else if(!huskylens.available()) {
    Serial.println(F("Object disappeared!"));
  }
  else
  {
    HUSKYLENSResult result = huskylens.read();
    if (result.xCenter < LOWER_CENTER_RANGE){
      Serial.println("Left");
      car_turn_left(128);
      return 'L';
    }else if (result.xCenter > UPPER_CENTER_RANGE){
      Serial.println("Right");
      car_turn_right(128);
      return 'R';
    }else{
      Serial.println("Forward");
      car_forward(210);
      return 'F';
    }

    
    //printResult(result);
  }
  return 'N';
}

char work_with_distance_sensor(){
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    if (measure.RangeMilliMeter < 1000){
      return 'Y';
    }
  }
  return 'N';
}

char work_with_light_sensor(){
  bool L_state = light_sensor_L -> is_inside_table();
  bool R_state = light_sensor_R -> is_inside_table();
  if((!L_state) && (!R_state)){
    Serial.println("**");
    return 'W';
  } else if (!L_state){ //not inside
    Serial.println("*-");
    return 'B';
  } else if (!R_state){ //not inside
    Serial.println("-*");
    return 'B';
  }
  return 'N';
}
