#include "OTAHandler.h"

void OTAHandler::setupOTA(const char* hostname, const char* password) {
  // Falls du einen Hostnamen setzen willst
  if (hostname != nullptr) {
    ArduinoOTA.setHostname(hostname);
  }

  // Falls du ein OTA-Passwort setzen willst
  if (password != nullptr) {
    ArduinoOTA.setPassword(password);
  }

  // Callback-Funktionen
  ArduinoOTA
    .onStart([]() {
      Serial.println("OTA-Update gestartet");
    })
    .onEnd([]() {
      Serial.println("\nOTA-Update beendet");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Fortschritt: %u%%\r", (progress * 100) / total);
    })
    .onError([](ota_error_t error) {
      Serial.printf("OTA-Fehler [%u]: ", error);
      if      (error == OTA_AUTH_ERROR)   Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR)  Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR)Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR)Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR)    Serial.println("End Failed");
    });

  // OTA starten
  ArduinoOTA.begin();
  Serial.println("OTA bereit!");
}

void OTAHandler::handleOTA() {
  ArduinoOTA.handle();
}
