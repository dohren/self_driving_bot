#include <Arduino.h>

#include <micro_ros_platformio.h>

#include "OTAHandler.h"
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>

#include <std_msgs/msg/int32.h>
#include <ESP32Encoder.h>

#include "MotorControl.h"
#include <MicroRosHandler.h>
#include "EncoderController.h"
#include <DNSResolver.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// WLAN-Zugangsdaten
const char* ssid     = "xxx;
const char* password = "xxx;


// Falls du Hostname/Passwort für OTA setzen willst
const char* otaHostname = "esp32-ota";
const char* otaPassword = "deinOTAPass";

OTAHandler otaHandler;

// Controller
MotorControl motorCtrl;
EncoderController encoders(35, 34, 33, 32);

// Micro-ROS Handler (statt MQTT)
MicroRosHandler microRosHandler(motorCtrl, encoders); 


void error_loop(){
  while(1){
    delay(100);
  }
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

bool initialized = false;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Testausgabe läuft!");

  set_microros_serial_transports(Serial);
  /*
  DNSResolver resolver(ssid, password, "mycroft.local", 8888);
  if (!resolver.connect()) {
    Serial.println("Fehler beim micro-ROS Verbindungsaufbau.");
    while (true) {
      delay(1000);
    }
  }
    */

  delay(1000);

  state = WAITING_AGENT;
  
  delay(100);

  // WLAN verbinden
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Verbinde mit WLAN...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nVerbunden! IP-Adresse:");
  Serial.println(WiFi.localIP());

  // OTA einrichten
  otaHandler.setupOTA(otaHostname, otaPassword);

  encoders.begin();

 }

void loop() {
  otaHandler.handleOTA();
  microRosHandler.update(); 

 // delay(100); 
 switch (state) {
    case AGENT_AVAILABLE:
      if (!initialized) {
        initialized = microRosHandler.init();
        state = initialized ? AGENT_CONNECTED : WAITING_AGENT;
      } else {
        state = AGENT_CONNECTED;
      }
      break;

    case AGENT_CONNECTED:
      microRosHandler.update();  
      break;

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT);
      break;

    case AGENT_DISCONNECTED:
      microRosHandler.destroy();
      state = WAITING_AGENT;
      break;  
  }  

}

