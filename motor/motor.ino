#include <WiFi.h>
#include <ArduinoJson.h>
#include "Motor.h"

// WiFi AP credentials
const char* ssid = "BottleSumo_AP";
const char* password = "12345678";  // Must be at least 8 characters

// TCP Server
WiFiServer server(8080);
WiFiClient client;

Motor motor1(11, 12);  // GPIO 11 left motor
Motor motor2(20, 19);  // GPIO 20 right motor

void setup() {
  Serial.begin(115200);  // Faster baud rate for better debugging
  delay(1000);
  
  // Initialize motors
  motor1.begin(20000); // 20kHz
  motor2.begin(20000); // 20kHz
  
  // Setup WiFi AP
  Serial.println("Setting up WiFi AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Start TCP server
  server.begin();
  Serial.println("TCP Server started on port 8080");
  Serial.println("Waiting for clients...");
}

void loop() {
  // Check for new clients
  if (!client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("New client connected!");
    }
  }
  
  // Handle client data
  if (client && client.connected()) {
    if (client.available()) {
      String jsonString = client.readStringUntil('\n');
      Serial.print("Received: ");
      Serial.println(jsonString);
      
      // Parse JSON and control motors
      parseAndControl(jsonString);
      
      // Send acknowledgment
      client.println("{\"status\":\"ok\"}");
    }
  }
  
  delay(10); // Small delay to prevent overwhelming the system
}

void parseAndControl(String jsonString) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Check for command type
  if (doc.containsKey("command")) {
    String command = doc["command"].as<String>();
    int speed = doc["speed"] | 0;  // Default to 0 if not provided
    
    Serial.print("Command: ");
    Serial.print(command);
    Serial.print(", Speed: ");
    Serial.println(speed);
    
    // Execute command
    if (command == "forward") {
      car_forward(speed);
    } else if (command == "backward") {
      car_backward(speed);
    } else if (command == "stop") {
      car_stop();
    } else if (command == "rotate_left") {
      car_rotate_turn_left(speed);
    } else if (command == "rotate_right") {
      car_rotate_turn_right(speed);
    } else if (command == "revolve_left") {
      car_revolve_turn_left(speed);
    } else if (command == "revolve_right") {
      car_revolve_turn_right(speed);
    } else if (command == "ping") {
      // Do nothing, just acknowledge
      Serial.println("Ping received");
    } else {
      Serial.println("Unknown command");
    }
  }
  // Alternative: Direct motor control
  else if (doc.containsKey("motor1") && doc.containsKey("motor2")) {
    int motor1_speed = doc["motor1"];
    int motor2_speed = doc["motor2"];
    
    Serial.print("Direct control - Motor1: ");
    Serial.print(motor1_speed);
    Serial.print(", Motor2: ");
    Serial.println(motor2_speed);
    
    motor1.setDuty(motor1_speed);
    motor2.setDuty(motor2_speed);
  } else {
    Serial.println("Invalid JSON format");
  }
}

//--- Car controlling functions ---//
void car_stop(){
  motor1.stop();
  motor2.stop();
}

void car_forward(int speed){
  motor1.setDuty(speed);
  motor2.setDuty(-speed);
}

void car_backward(int speed){
  motor1.setDuty(-speed);
  motor2.setDuty(speed);
}

void car_rotate_turn_left(int speed){ // 自轉
  motor1.setDuty(-speed);
  motor2.setDuty(-speed);
}

void car_rotate_turn_right(int speed){ // 自轉
  motor1.setDuty(speed);
  motor2.setDuty(speed);
}

void car_revolve_turn_left(int speed){
  motor1.stop();
  motor2.setDuty(-speed);
}

void car_revolve_turn_right(int speed){
  motor1.setDuty(speed);
  motor2.stop();
}