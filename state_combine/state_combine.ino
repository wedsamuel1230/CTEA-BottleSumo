#define btn1 6
#define btn2 4
#define btn3 3
#define btn4 5

byte result;

// Sensor result mapping explanation:
// Decimal → Binary (btn4 btn3 btn2 btn1)
// 0  → 0000 (無按鈕)
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

void setup() {
  Serial.begin(115200);
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
}

void loop() {
  result = 0; // reset each loop

  if (digitalRead(btn1) == LOW) {
    result = result + 1; // 0b0001
  }

  if (digitalRead(btn2) == LOW) {
    result = result + 2; // 0b0010
  }

  if (digitalRead(btn3) == LOW) {
    result = result + 4; // 0b0100
  }

  if (digitalRead(btn4) == LOW) {
    result = result + 8; // 0b1000
  }

  Serial.print("Result(Binary): ");
  Serial.println(result, BIN);
  Serial.print("Result: ");
  Serial.println(result);

  delay(100);
}
