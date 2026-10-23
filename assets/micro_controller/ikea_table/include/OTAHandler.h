#ifndef OTA_HANDLER_H
#define OTA_HANDLER_H

#include <Arduino.h>
#include <ArduinoOTA.h>

class OTAHandler {
public:
  void setupOTA(const char* hostname = nullptr, const char* password = nullptr);
  void handleOTA();
};

#endif
