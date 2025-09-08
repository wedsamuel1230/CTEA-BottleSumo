void setup() {
  Serial.begin(9600); // Initialize serial communication
  analogReadResolution(12);
}

void loop() {
  int sensorValue = analogRead(A0); // Read analog value from ADC0 (GP26)

  // Convert the analog value to voltage
  float voltage = (sensorValue / 4095.0) * 5; 

  Serial.print("Analog Value: ");
  Serial.print(sensorValue);
  Serial.print(", Voltage: ");
  Serial.print(voltage, 2); // Print voltage with 2 decimal places
  Serial.println("V");

  if(voltage >= 3.0){
    Serial.println("Out of bounds");
    value = 1;
  }
  else:{
    Serial.println("Okay");
    value = 0;
  }

  delay(100); // Small delay for stable readings
}