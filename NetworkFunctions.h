#ifndef ESP32_SOIL_OTA_NETWORKFUNCTIONS_H
#define ESP32_SOIL_OTA_NETWORKFUNCTIONS_H

#include "ESP32_Soil_OTA.h"
#ifdef ESP8266
// These headers are installed when the ESP8266 is installed in board manager.
#include "ESP8266WiFi.h" // ESP8266 WiFi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <ESP8266mDNS.h> // OTA - mDNSResponder (Multicast DNS) for the ESP8266 family.
const unsigned int LED_ON = 0;
const unsigned int LED_OFF = 1;
#else
// These headers are installed when the ESP32 is installed in board manager.
#include "WiFi.h"		// ESP32 Wifi support.  https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFi.h
#include <ESPmDNS.h> // OTA - Multicast DNS for the ESP32.
const unsigned int LED_ON = 1;
const unsigned int LED_OFF = 0;
#endif

#include <WiFiUdp.h>		// OTA
#include <ArduinoOTA.h> // OTA - The Arduino OTA library.  Specific version of this are installed along with specific boards in board manager.

void onReceiveCallback( char *topic, byte *payload, unsigned int length );
void configureOTA();
int checkForSSID( const char *ssidName );
bool wifiConnect( const char *ssid, const char *password );
void wifiMultiConnect();
bool mqttMultiConnect( int maxAttempts );
void lookupWifiCode( int code, char *buffer );
void lookupMQTTCode( int code, char *buffer );

WiFiClient espClient;
PubSubClient mqttClient( espClient );

#endif //ESP32_SOIL_OTA_NETWORKFUNCTIONS_H
