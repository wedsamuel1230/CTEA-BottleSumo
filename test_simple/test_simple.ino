// 最小測試
enum TestAction {
  ACTION_A,
  ACTION_B
};

struct TestStruct {
  int value;
};

String testFunction(TestStruct ts) {
  return "test";
}

void setup() {
  Serial.begin(115200);
  TestStruct ts;
  ts.value = 42;
  String result = testFunction(ts);
  Serial.println(result);
}

void loop() {
  delay(1000);
}