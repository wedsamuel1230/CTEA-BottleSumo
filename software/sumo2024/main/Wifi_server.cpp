#include "Wifi_server.h"

Wifi_server::Wifi_server(const char* ssid, const char* password)
    : ssid(ssid), password(password), server(80){}

void Wifi_server::begin(){
    
    Serial.begin(115200);
    //WiFi.mode(WIFI_AP); // Set WiFi mode to AP
    //WiFi.softAP(ssid, password); // Start the AP with the specified SSID and password
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
//    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
//        Serial.print(".");
    }
/*    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());*/
    /*Serial.println("");
    Serial.print("Access Point \"");
    Serial.print(ssid);
    Serial.println("\" started");
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP()); // Print the IP address of the AP*/

    if (MDNS.begin("picow")) {
//        Serial.println("MDNS responder started");
    }

    set_api_to_server();


    server.onNotFound([this]() { handleNotFound(); });

    server.begin();
//    Serial.println("HTTP server started");
}

void Wifi_server::handleRoot() {
    server.send(200, "text/plain", "hello from pico w!\r\n");
}

void Wifi_server::handleNotFound() {
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++) {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", message);
}

void Wifi_server::handleClient() {
    server.handleClient();
    MDNS.update();
}

void Wifi_server::set_api_to_server(){
  server.on("/", [this]() { handleRoot(); });
  server.on("/hello-pico", [this]() {
    server.send(200, "text/plain", "Sumo Robot");
  });

  //direction
  server.on("/dir-f", [this]() {
    command_state = 'F';
    server.send(200, "text/plain", "⇑");
  });
  server.on("/dir-b", [this]() {
    command_state = 'B';
    server.send(200, "text/plain", "⇓");
  });
  server.on("/dir-r", [this]() {
    command_state = 'R';
    server.send(200, "text/plain", "⇒");
  });
  server.on("/dir-l", [this]() {
    command_state = 'L';
    server.send(200, "text/plain", "⇐");
  });

  server.on("/stop-car", [this]() {
    command_state = 'O';
    server.send(200, "text/plain", "O");
  });
}

char Wifi_server::get_command(){
  return command_state;
}