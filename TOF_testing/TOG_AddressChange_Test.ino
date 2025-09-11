#include <Wire.h>
#include <VL53L1X.h>       // For VL53L1X sensors
#include <Adafruit_SSD1306.h> // For OLED
#include <Adafruit_GFX.h>

// I2C
TwoWire i2c(i2c0, 26, 27);//ic20, SDA=GP26 , SCL=GP27
//TwoWire i2c = TwoWire(0);

// XSHUT pins for sensors
const int xshutPins[] = {13, 12, 11};
const int numSensors = 3;
VL53L1X sensors[numSensors];
const uint8_t newAddresses[] = {0x30, 0x32, 0x34};

// OLED
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &i2c, -1);

// Motor pins (L298N)
const int leftDirFwd = 7;
const int leftDirRev = 8;
const int leftPWM = 9;
const int rightDirFwd = 10;
const int rightDirRev = 11;
const int rightPWM = 12;

void setup() {
  Serial.begin(115200);

  // I2C
  i2c.begin(26, 27);

  // OLED
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED failed");
    while (1);
  }
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.println("OLED Ready!");
  oled.display();
  delay(1000);

  // Motors
  /*
  pinMode(leftDirFwd, OUTPUT);
  pinMode(leftDirRev, OUTPUT);
  pinMode(rightDirFwd, OUTPUT);
  pinMode(rightDirRev, OUTPUT);
  */

  // Initialize sensors
  for (int i = 0; i < numSensors; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  for (int i = 0; i < numSensors; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(50);
    sensors[i].setBus(&i2c);
    sensors[i].setAddress(0x29);
    sensors[i].init();
    sensors[i].setDistanceMode(VL53L1X::Short);
    sensors[i].setTimingBudgetInMicroSeconds(50000); // 50ms in microseconds
    sensors[i].startContinuous(50);
    sensors[i].setAddress(newAddresses[i]);
    delay(10);
  }

  // Keep XSHUT high
  for (int i = 0; i < numSensors; i++) {
    digitalWrite(xshutPins[i], HIGH);
  }
}

void motorForward(int speed = 200) {
  digitalWrite(leftDirFwd, HIGH);
  digitalWrite(leftDirRev, LOW);
  digitalWrite(rightDirFwd, HIGH);
  digitalWrite(rightDirRev, LOW);
  analogWrite(leftPWM, speed);
  analogWrite(rightPWM, speed);
}

void motorStop() {
  analogWrite(leftPWM, 0);
  analogWrite(rightPWM, 0);
}

void motorLeftTurn(int speed = 150) {
  digitalWrite(leftDirFwd, LOW);
  digitalWrite(leftDirRev, HIGH);
  digitalWrite(rightDirFwd, HIGH);
  digitalWrite(rightDirRev, LOW);
  analogWrite(leftPWM, speed);
  analogWrite(rightPWM, speed + 50);
}

void motorRightTurn(int speed = 150) {
  digitalWrite(leftDirFwd, HIGH);
  digitalWrite(leftDirRev, LOW);
  digitalWrite(rightDirFwd, LOW);
  digitalWrite(rightDirRev, HIGH);
  analogWrite(leftPWM, speed + 50);
  analogWrite(rightPWM, speed);
}

void motorSearchSpin(int speed = 100) {
  digitalWrite(leftDirFwd, HIGH);
  digitalWrite(leftDirRev, LOW);
  digitalWrite(rightDirFwd, LOW);
  digitalWrite(rightDirRev, HIGH);
  analogWrite(leftPWM, speed);
  analogWrite(rightPWM, speed);
}

void loop() {
  int distances[numSensors];
  bool opponentDetected = false;
  int closestSensor = -1;
  int closestDistance = 1000;

  // Read distances
  for (int i = 0; i < numSensors; i++) {
    distances[i] = sensors[i].read();
    if (distances[i] > 0 && distances[i] < closestDistance) {
      closestDistance = distances[i];
      closestSensor = i;
      opponentDetected = true;
    }
  }

  // Print to Serial
  for (int i = 0; i < numSensors; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(distances[i]);
    Serial.println(" mm");
  }

  // Update OLED
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Distances:");
  for (int i = 0; i < numSensors; i++) {
    oled.print("S");
    oled.print(i + 1);
    oled.print(": ");
    oled.print(distances[i] > 0 ? String(distances[i]) + "mm" : "---");
    oled.println();
  }
  oled.print("Closest: ");
  oled.print(closestDistance);
  oled.println("mm");
  oled.display();

  // Sumo logic
  /*
  if (opponentDetected && closestDistance < 500) {
    if (closestSensor == 0 || closestSensor == 1) {
      motorForward(200);
    } else if (closestSensor == 2) {
      motorLeftTurn(150);
    } else {
      motorRightTurn(150);
    }
  } else {
    motorSearchSpin(100);
  }

  delay(50);
  */
}
