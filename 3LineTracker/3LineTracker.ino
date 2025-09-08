void setup() {
  Serial.begin(9600); // Initialize serial communication
  analogReadResolution(12);
}

// Sensor result mapping explanation:
// 0  → 0000 (冇出界)
// 1  → 0001 (左上)
// 2  → 0010 (右上)
// 3  → 0011 (左上 + 右上)
// 4  → 0100 (右下)
// 5  → 0101 (左上 + 右下)
// 6  → 0110 (右上 + 右下)
// 7  → 0111 (左上 + 右上 + 右下)
// 8  → 1000 (左下)
// 9  → 1001 (左上 + 左下)
// 10 → 1010 (右上 + 左下)
// 11 → 1011 (左上 + 右上 + 左下)
// 12 → 1100 (右下 + 左下)
// 13 → 1101 (左上 + 右下 + 左下)
// 14 → 1110 (右上 + 右下 + 左下)
// 15 → 1111 (左上 + 右上 + 右下 + 左下)

int check_line_tracker(float thershold){
  int result;
  int sensorValue1 = analogRead(A0); // Read analog value from ADC0 (GP26)
  int sensorValue2 = analogRead(A1); // Read analog value from ADC1 (GP27)
  int sensorValue4 = analogRead(A2); // Read analog value from ADC2 (GP28)

  float voltage1 = (sensorValue1 / 4095.0) * 5;
  float voltage2 = (sensorValue2 / 4095.0) * 5;
  float voltage4 = (sensorValue4 / 4095.0) * 5;

  if(voltage1 >= thershold){
    Serial.println("Out of bounds");
    result = result + 1;
  }
  if(voltage2 >= thershold){
    Serial.println("Out of bounds");
    result = result + 2;
  }
  if(voltage4 >= thershold){
    Serial.println("Out of bounds");
    result = result + 4;
  }
  Serial.print("Voltage(1): ");
  Serial.print(voltage1, 2); // Print voltage with 2 decimal places
  Serial.print("V");
  
  Serial.print(", Voltage(2): ");
  Serial.print(voltage2, 2); // Print voltage with 2 decimal places
  Serial.print("V");

  Serial.print(", Voltage(4): ");
  Serial.print(voltage4, 2); // Print voltage with 2 decimal places
  Serial.println("V");
  return result;
}

void loop() {
  Serial.print("Result(Binary): ");
  Serial.println(check_line_tracker(2.5), BIN);
  Serial.print("Result: ");
  Serial.println(check_line_tracker(2.5));
  delay(100); // Small delay for stable readings
}

