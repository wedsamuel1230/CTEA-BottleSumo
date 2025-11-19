#include <WiFi.h>
#include "Motor.h"
#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <LEAmDNS.h>

class Wifi_server {
  public:
    Wifi_server(const char* ssid, const char* password);
    void begin();
    void handleClient();
    char get_command();

  private:
    const char* ssid;
    const char* password;
    WebServer server;

    void handleRoot();
    void handleNotFound();
    void set_api_to_server();
    char command_state;
};

#endif // WIFI_SERVER_H