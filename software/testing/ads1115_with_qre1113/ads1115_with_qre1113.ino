#include <Adafruit_ADS1X15.h>
#include <Wire.h>


Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */


void setup(void)
{
  delay(100);
  Serial.begin(115200);
  while (!Serial) {
    delay(100); // 等待序列埠連接
  }


  // 設定 I2C1 的 SDA 和 SCL 腳位
  // 將 GP2 設為 SDA，GP3 設為 SCL
  Wire1.setSDA(2);
  Wire1.setSCL(3);

  // 初始化 I2C1
  Wire1.begin();

  Serial.println("I2C1 已在 GP2 (SDA) 和 GP3 (SCL) 上初始化");
  ads.setGain(GAIN_ONE);
  // 修正：傳遞 Wire1 的指標 (&Wire1)
  if (!ads.begin(0x48, &Wire1)) {
    Serial.println("Failed to initialize ADS.");
  }

}


void loop(void)
{
  int16_t adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;


  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);


  volts0 = ads.computeVolts(adc0);
  volts1 = ads.computeVolts(adc1);
  volts2 = ads.computeVolts(adc2);
  volts3 = ads.computeVolts(adc3);


  Serial.println("-----------------------------------------------------------");
  Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
  Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1); Serial.println("V");
  Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println("V");
  Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3); Serial.println("V");


  delay(5);
}
