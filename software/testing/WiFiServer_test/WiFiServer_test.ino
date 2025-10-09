#include <WiFi.h>

const char* ssid = "...";
const char* password = "...";

int port = 4242;
WiFiServer server(port);

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== Pico W WiFi 診斷工具 ===");
  Serial.printf("SSID: '%s'\n", ssid);
  Serial.printf("密碼長度: %d 字元\n", strlen(password));
  
  // 設定 WiFi 模式
  WiFi.mode(WIFI_STA);
  Serial.println("WiFi 模式設定為 STA");
  
  // 掃描可用網路
  Serial.println("\n掃描附近的 WiFi 網路...");
  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    Serial.println("❌ 沒有找到任何 WiFi 網路！");
    Serial.println("請檢查:");
    Serial.println("1. 路由器是否開啟");
    Serial.println("2. 是否在信號範圍內");
    return;
  }
  
  Serial.printf("✅ 找到 %d 個 WiFi 網路:\n", n);
  bool found_target = false;
  
  for (int i = 0; i < n; ++i) {
    String current_ssid = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    
    Serial.printf("%d: '%s' (信號: %d dBm) ", i + 1, current_ssid.c_str(), rssi);
    
    if (current_ssid == ssid) {
      Serial.print("🎯 目標網路!");
      found_target = true;
    }
    
    Serial.println();
  }
  
  if (!found_target) {
    Serial.println("\n❌ 找不到目標 WiFi 網路!");
    Serial.printf("請確認 SSID '%s' 是否正確\n", ssid);
    return;
  }
  
  Serial.println("\n開始連接 WiFi...");
  WiFi.begin(ssid, password);
  
  int timeout = 30;  // 增加到 30 秒
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    delay(1000);
    Serial.print(".");
    timeout--;
    
    // 每 5 秒顯示詳細狀態
    if (timeout % 5 == 0) {
      Serial.printf(" [狀態: %s] ", getWiFiStatusString(WiFi.status()).c_str());
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n🎉 WiFi 連線成功!");
    Serial.printf("IP 地址: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("子網路遮罩: %s\n", WiFi.subnetMask().toString().c_str());
    Serial.printf("閘道器: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("信號強度: %d dBm\n", WiFi.RSSI());
    Serial.printf("MAC 地址: %s\n", WiFi.macAddress().c_str());
    
    // 啟動伺服器
    server.begin();
    Serial.printf("\n🌐 TCP 伺服器已啟動!\n");
    Serial.printf("連接地址: %s:%d\n", WiFi.localIP().toString().c_str(), port);
    Serial.println("可以使用 telnet 或網路工具連接到此地址進行測試");
    
  } else {
    Serial.println("\n❌ 連線失敗!");
    Serial.printf("最終狀態: %s\n", getWiFiStatusString(WiFi.status()).c_str());
    Serial.println("\n可能的原因:");
    Serial.println("1. 密碼錯誤");
    Serial.println("2. 信號太弱");
    Serial.println("3. 路由器設定問題");
    Serial.println("4. MAC 地址被封鎖");
    return; // 連線失敗則結束
  }
}

void loop() {
  // 檢查 WiFi 連線狀態
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️  WiFi 連線中斷！嘗試重新連接...");
    WiFi.begin(ssid, password);
    delay(5000);
    return;
  }
  
  // 檢查是否有客戶端連接
  WiFiClient client = server.accept();
  if (!client) {
    delay(100);  // 短暫延遲
    return;
  }
  
  Serial.printf("🔗 新客戶端連接: %s\n", client.remoteIP().toString().c_str());
  
  // 發送歡迎訊息
  client.println("=== Pico W TCP 伺服器 ===");
  client.println("歡迎連接！請輸入任何文字並按 Enter：");
  
  // 等待客戶端發送資料
  unsigned long timeout = millis();
  while (!client.available() && millis() - timeout < 10000) {  // 10秒超時
    delay(10);
  }
  
  if (client.available()) {
    String message = client.readStringUntil('\n');
    message.trim();  // 移除首尾空白
    
    Serial.printf("📨 收到訊息: '%s'\n", message.c_str());
    
    // 回應客戶端
    client.printf("✅ 伺服器收到你的訊息: '%s'\n", message.c_str());
    client.printf("🕒 時間: %lu ms\n", millis());
    client.printf("📍 你的 IP: %s\n", client.remoteIP().toString().c_str());
    client.printf("🌐 伺服器 IP: %s\n", WiFi.localIP().toString().c_str());
    client.println("👋 感謝使用 Pico W 伺服器！");
  } else {
    client.println("⏰ 超時：沒有收到資料");
    Serial.println("⏰ 客戶端超時");
  }
  
  // 關閉連接
  client.stop();
  Serial.println("🔌 客戶端連接已關閉");
}

String getWiFiStatusString(int status) {
  switch(status) {
    case WL_IDLE_STATUS: return "閒置";
    case WL_NO_SSID_AVAIL: return "找不到SSID";
    case WL_SCAN_COMPLETED: return "掃描完成";
    case WL_CONNECTED: return "已連接";
    case WL_CONNECT_FAILED: return "連接失敗";
    case WL_CONNECTION_LOST: return "連接中斷";
    case WL_DISCONNECTED: return "已斷線";
    default: return "未知狀態(" + String(status) + ")";
  }
}
