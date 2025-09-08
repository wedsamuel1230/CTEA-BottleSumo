
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();


void setup() {
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  // Robojax.com I2C address update 20181206
  lox.begin(0x51,false,&Wire1);// put any address between 0x29 to 0x7F 
}

void loop(){
  
}