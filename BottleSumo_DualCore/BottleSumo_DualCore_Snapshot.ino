// BottleSumo Dual Core - WiFi/TCP on Core 1, Motor Control on Core 0
// This code runs on Raspberry Pi Pico W with dual-core functionality
// Core 0: Motor control and sensor reading (main loop)
// Core 1: WiFi and TCP server operations (loop1)

#include <WiFi.h>

// WiFi credentials
const char* ssid = "YourNetworkSSID";
const char* password = "YourPassword";

// TCP Server
WiFiServer server(80);
WiFiClient client;

// Motor pins
#define LEFT_MOTOR_FWD 16
#define LEFT_MOTOR_REV 17
#define RIGHT_MOTOR_FWD 18
#define RIGHT_MOTOR_REV 19

// Sensor pins
#define LINE_SENSOR_1 26
#define LINE_SENSOR_2 27
#define LINE_SENSOR_3 28

// Shared variables between cores (with mutex protection in production)
volatile int motorSpeed = 0;
volatile int motorDirection = 0;
volatile bool emergencyStop = false;

// Core 0 Setup (Main)
void setup() {
  Serial.begin(115200);
  
  // Configure motor pins
  pinMode(LEFT_MOTOR_FWD, OUTPUT);
  pinMode(LEFT_MOTOR_REV, OUTPUT);
  pinMode(RIGHT_MOTOR_FWD, OUTPUT);
  pinMode(RIGHT_MOTOR_REV, OUTPUT);
  
  // Configure sensor pins
  pinMode(LINE_SENSOR_1, INPUT);
  pinMode(LINE_SENSOR_2, INPUT);
  pinMode(LINE_SENSOR_3, INPUT);
  
  Serial.println("Core 0: Motor control initialized");
}

// Core 0 Main Loop - Motor Control and Sensors
void loop() {
  // Read line sensors
  int sensor1 = digitalRead(LINE_SENSOR_1);
  int sensor2 = digitalRead(LINE_SENSOR_2);
  int sensor3 = digitalRead(LINE_SENSOR_3);
  
  // Emergency stop from WiFi command
  if (emergencyStop) {
    stopMotors();
    delay(100);
    return;
  }
  
  // Motor control based on sensor readings or WiFi commands
  if (motorDirection == 1) {
    moveForward(motorSpeed);
  } else if (motorDirection == 2) {
    moveBackward(motorSpeed);
  } else if (motorDirection == 3) {
    turnLeft(motorSpeed);
  } else if (motorDirection == 4) {
    turnRight(motorSpeed);
  } else {
    stopMotors();
  }
  
  delay(50);
}

// Core 1 Setup - WiFi and TCP
void setup1() {
  delay(1000); // Wait for Core 0 to initialize
  
  Serial.println("Core 1: Starting WiFi initialization");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nCore 1: WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Start TCP server
    server.begin();
    Serial.println("Core 1: TCP server started on port 80");
  } else {
    Serial.println("\nCore 1: WiFi connection failed");
  }
}

// Core 1 Main Loop - WiFi and TCP Server
void loop1() {
  // Check for WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Core 1: WiFi disconnected, attempting reconnect...");
    WiFi.reconnect();
    delay(5000);
    return;
  }
  
  // Handle TCP client connections
  if (!client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Core 1: New client connected");
    }
  }
  
  // Read commands from TCP client
  if (client && client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    
    Serial.print("Core 1: Received command: ");
    Serial.println(command);
    
    // Parse commands
    if (command == "FORWARD") {
      motorDirection = 1;
      motorSpeed = 200;
      client.println("OK: Moving forward");
    } else if (command == "BACKWARD") {
      motorDirection = 2;
      motorSpeed = 200;
      client.println("OK: Moving backward");
    } else if (command == "LEFT") {
      motorDirection = 3;
      motorSpeed = 150;
      client.println("OK: Turning left");
    } else if (command == "RIGHT") {
      motorDirection = 4;
      motorSpeed = 150;
      client.println("OK: Turning right");
    } else if (command == "STOP") {
      motorDirection = 0;
      motorSpeed = 0;
      client.println("OK: Stopped");
    } else if (command == "EMERGENCY") {
      emergencyStop = true;
      client.println("OK: Emergency stop activated");
    } else if (command == "RESET") {
      emergencyStop = false;
      motorDirection = 0;
      motorSpeed = 0;
      client.println("OK: Reset complete");
    } else if (command == "STATUS") {
      client.print("Direction: ");
      client.print(motorDirection);
      client.print(", Speed: ");
      client.print(motorSpeed);
      client.print(", Emergency: ");
      client.println(emergencyStop ? "YES" : "NO");
    } else {
      client.println("ERROR: Unknown command");
    }
  }
  
  delay(10);
}

// Motor control functions (Core 0)
void moveForward(int speed) {
  digitalWrite(LEFT_MOTOR_FWD, HIGH);
  digitalWrite(LEFT_MOTOR_REV, LOW);
  digitalWrite(RIGHT_MOTOR_FWD, HIGH);
  digitalWrite(RIGHT_MOTOR_REV, LOW);
  analogWrite(LEFT_MOTOR_FWD, speed);
  analogWrite(RIGHT_MOTOR_FWD, speed);
}

void moveBackward(int speed) {
  digitalWrite(LEFT_MOTOR_FWD, LOW);
  digitalWrite(LEFT_MOTOR_REV, HIGH);
  digitalWrite(RIGHT_MOTOR_FWD, LOW);
  digitalWrite(RIGHT_MOTOR_REV, HIGH);
  analogWrite(LEFT_MOTOR_REV, speed);
  analogWrite(RIGHT_MOTOR_REV, speed);
}

void turnLeft(int speed) {
  digitalWrite(LEFT_MOTOR_FWD, LOW);
  digitalWrite(LEFT_MOTOR_REV, HIGH);
  digitalWrite(RIGHT_MOTOR_FWD, HIGH);
  digitalWrite(RIGHT_MOTOR_REV, LOW);
  analogWrite(LEFT_MOTOR_REV, speed);
  analogWrite(RIGHT_MOTOR_FWD, speed);
}

void turnRight(int speed) {
  digitalWrite(LEFT_MOTOR_FWD, HIGH);
  digitalWrite(LEFT_MOTOR_REV, LOW);
  digitalWrite(RIGHT_MOTOR_FWD, LOW);
  digitalWrite(RIGHT_MOTOR_REV, HIGH);
  analogWrite(LEFT_MOTOR_FWD, speed);
  analogWrite(RIGHT_MOTOR_REV, speed);
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_FWD, LOW);
  digitalWrite(LEFT_MOTOR_REV, LOW);
  digitalWrite(RIGHT_MOTOR_FWD, LOW);
  digitalWrite(RIGHT_MOTOR_REV, LOW);
}
