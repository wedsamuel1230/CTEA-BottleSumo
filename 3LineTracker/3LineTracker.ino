void setup() {
  Serial.begin(9600); // Initialize serial communication
  analogReadResolution(12);
}

void loop() {
  int sensorValue1 = analogRead(A0); // Read analog value from ADC0 (GP26)
  int sensorValue2 = analogRead(A1); // Read analog value from ADC0 (GP26)
  int sensorValue4 = analogRead(A2); // Read analog value from ADC0 (GP26)
  
  float voltage1 = (sensorValue1 / 4095.0) * 5;
  float voltage2 = (sensorValue2 / 4095.0) * 5; 
  float voltage4 = (sensorValue2 / 4095.0) * 5; 

  Serial.print("Voltage(1): ");
  Serial.print(voltage1, 2); // Print voltage with 2 decimal places
  Serial.print("V");
  
  Serial.print(", Voltage(2): ");
  Serial.print(voltage2, 2); // Print voltage with 2 decimal places
  Serial.print("V");

  Serial.print(", Voltage(4): ");
  Serial.print(voltage4, 2); // Print voltage with 2 decimal places
  Serial.println("V");

  delay(100); // Small delay for stable readings
}