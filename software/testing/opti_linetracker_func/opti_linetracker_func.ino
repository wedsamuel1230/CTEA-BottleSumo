constexpr uint8_t NUM_SENS = 3U;
constexpr float VREF = 5.0F;
constexpr float ADC_MAX = 4095.0F;
const uint8_t sensPins[NUM_SENS] = {A0,A1,A2};
const uint8_t sensMask[NUM_SENS] = {0b001,0b010,0b100};

void setup() {
  Serial.begin(9600); // Initialize serial communication
  analogReadResolution(12);
}

int check_line_tracker(float thersholdV){
  uint8_t result = 0;
  for (uint8_t i = 0; i < NUM_SENS; ++i){
    uint16_t counts = analogRead(sensPins[i]);
    float volts = (counts /ADC_MAX) * VREF;
    if (volts >= thersholdV){
      result |= sensMask[i];
    }
    Serial.print(F("Voltage("));
    Serial.print(i + 1);
    Serial.print(F("): "));
    Serial.print(volts,2);
    Serial.print(F("V\n"));
  }
  Serial.print(F("Result (DEC): "));
  Serial.println(result);
  return result;
}
void loop() {
  check_line_tracker(3.0);
  delay(200); // Small delay for stable readings
}