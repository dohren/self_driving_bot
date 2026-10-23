#ifndef MICROROS_WIFI_RESOLVER_H
#define MICROROS_WIFI_RESOLVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <string.h>  // für strdup

class DNSResolver {
public:
  DNSResolver(const char* ssid, const char* password, const char* hostname, uint16_t port)
    : ssid(ssid), password(password), hostname(hostname), port(port) {}

  bool connect() {
    Serial.printf(" Verbinde mit WLAN \"%s\"...\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts++ < 30) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\n WLAN-Verbindung fehlgeschlagen!");
      return false;
    }

    Serial.println("\n WLAN verbunden!");
    Serial.print("Lokale IP: ");
    Serial.println(WiFi.localIP());

    IPAddress agent_ip;
    Serial.printf("Hostnamen \"%s\" auflösen...\n", hostname);

    if (!WiFi.hostByName(hostname, agent_ip)) {
      Serial.println("DNS-Auflösung fehlgeschlagen!");
      return false;
    }

    Serial.print("Aufgelöste Agent-IP: ");
    Serial.println(agent_ip);

    // Strings duplizieren (char*) für set_microros_wifi_transports
    char* ssid_dup = strdup(ssid);
    char* password_dup = strdup(password);
    // set_microros_wifi_transports(ssid_dup, password_dup, agent_ip, port);

    return true;
  }

private:
  const char* ssid;
  const char* password;
  const char* hostname;
  uint16_t port;
};

#endif
