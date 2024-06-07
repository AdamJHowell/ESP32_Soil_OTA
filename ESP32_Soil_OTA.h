#ifndef ESP32_SOIL_OTA_ESP32_SOIL_OTA_H
#define ESP32_SOIL_OTA_ESP32_SOIL_OTA_H


#include <Wire.h>            // This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include "PubSubClient.h"    // PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>     // A JSON manipulation library.  Author: Benoît Blanchon  https://github.com/bblanchon/ArduinoJson  https://arduinojson.org/
#include <Adafruit_seesaw.h> // Used to read the soil sensor. https://github.com/adafruit/Adafruit_Seesaw - Requires https://github.com/adafruit/Adafruit_BusIO
#include "privateInfo.h"     // I use this file to hide my network information from random people browsing my GitHub repo.
#include "NetworkFunctions.h"

#define LOCATION "backYard"
#define DEVICE "plantWatering"

#define MQTT_TOPIC LOCATION "/" DEVICE "/"
#define COMMAND_TOPIC MQTT_TOPIC "command"

/**
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 * If you do not want to create that file, set them here instead.
 */
//const char* wifiSsid = "your WiFi SSID";
//const char* wifiPassword = "your Wi-Fi password";
//const char* mqttBroker = "your broker address";
//const int mqttPort = 1883;
const char *HOSTNAME = "ESP32_Soil_OTA";                                           // The network hostname to set.
const char *NOTES = "HiLetgo ESP32 with Adafruit I2C soil sensor";                 // Notes about this program.
const char *SKETCH_TOPIC = "backYard/plantWatering/sketch";                        // The topic used to publish the sketch name.
const char *MAC_TOPIC = "backYard/plantWatering/mac";                              // The topic used to publish the MAC address.
const char *IP_TOPIC = "backYard/plantWatering/ip";                                // The topic used to publish the IP address.
const char *RSSI_TOPIC = "backYard/plantWatering/rssi";                            // The topic used to publish the Wi-Fi Received Signal Strength Indicator.
const char *PUBLISH_COUNT_TOPIC = "backYard/plantWatering/publishCount";           // The topic used to publish the loop count.
const char *NOTES_TOPIC = "backYard/plantWatering/NOTES";                          // The topic used to publish NOTES relevant to this project.
const char *TEMP_C_TOPIC = "backYard/plantWatering/soil/tempC";                    // The topic used to publish the soil temperature in Celsius.
const char *TEMP_F_TOPIC = "backYard/plantWatering/soil/tempF";                    // The topic used to publish the soil temperature in Fahrenheit.
const char *MOISTURE_TOPIC = "backYard/plantWatering/soil/moisture";               // The topic used to publish the soil moisture.
const char *MOISTURE_THRESHOLD_TOPIC = "backYard/plantWatering/moistureThreshold"; // The topic used to publish the soil moisture.
const char *PUMP_RUNNING_TOPIC = "backYard/plantWatering/pumpRunning";             // The topic used to publish the pump status.
const unsigned int MCU_LED = 2;                                                    // Use this LED for notifications.
//const unsigned int SCL_GPIO = 22;                                                  // The GPIO to use for SCL.
//const unsigned int SDA_GPIO = 21;                                                  // The GPIO to use for SDA.
const unsigned int RELAY_GPIO = 4;                                                 // The GPIO which controls the relay.
const unsigned int PUMP_ON = 0;                                                    // Change this to work with low-trigger relays.
const unsigned int PUMP_OFF = 1;                                                   // Change this to work with low-trigger relays.
const unsigned int JSON_DOC_SIZE = 512;                                            // The ArduinoJson document size.
const unsigned int MILLIS_IN_SEC = 1000;                                           // The number of milliseconds in a second.
const unsigned int ARRAY_SIZE = 3;                                                 // The size of the telemetry arrays.
unsigned int wifiConnectCount = 0;                                                 // A counter for how many times the wifiConnect() function has been called.
unsigned int wifiConnectionTimeout = 10000;                                        // Set the Wi-Fi connection timeout to 10 seconds.
unsigned int mqttReconnectInterval = 3000;                                         // When mqttMultiConnect is set to try multiple times, this is how long to delay between each attempt.
unsigned int mqttConnectCount = 0;                                                 // A counter for how many times the mqttConnect() function has been called.
//unsigned int mqttReconnectCooldown = 20000;                                        // Set the minimum time between calls to mqttMultiConnect() to 20 seconds.
unsigned int minMoisture = 700;                                                    // The moisture level which triggers the pump.
unsigned int callbackCount = 0;                                                    // A count for how many times a callback was received.
unsigned int invalidValueCount = 0;                                                // A count of how many sensor readings were out of bounds.
unsigned long lastWifiConnectTime = 0;                                             // The last time a Wi-Fi connection was attempted.
unsigned long publishCount = 0;                                                    // A count of how many publishes have taken place.
unsigned long publishInterval = 20000;                                             // The delay in milliseconds between MQTT publishes.  This prevents "flooding" the broker.
unsigned long sensorPollInterval = 5000;                                           // This is the delay between polls of the soil sensor.  This should be greater than 100 milliseconds.
unsigned long pumpRunTime = 20000;                                                 // Minimum time to run the pump.
unsigned long pumpMinOffDelay = 20000;                                             // The time to wait after stopping, before the pump will start again.  This allows water to flow through the soil.
unsigned long lastPublishTime = 0;                                                 // This is used to determine the time since last MQTT publish.
unsigned long wifiCoolDownInterval = 10000;                                        // How long to wait between Wi-Fi connection attempts.
unsigned long mqttCoolDownInterval = 10000;                                        // How long to wait between MQTT broker connection attempts.
//unsigned long lastMqttConnectionTime = 0;                                          // The last time a MQTT connection was attempted.
unsigned long lastPollTime = 0;                                                    // This is used to determine the time since last sensor poll.
unsigned long bootTime = 0;                                                        // The time since boot.  This value "rolls" at about 50 days.
unsigned long pumpStartTime = 0;                                                   // The most recent time that the pump started.
unsigned long pumpStopTime = 0;                                                    // The most recent time that the pump stopped.
char ipAddress[16];                                                                // The IP address.
char macAddress[18];                                                               // The MAC address to use as part of the MQTT client ID.
bool pumpRunning = false;                                                          // Flag to indicate when the pump is running or not.
bool sensorInitialized = false;                                                    // Flag to indicate that the sensor has been initialized.
float tempC = 21.12;                                                               // A global to hold the temperature in Celsius.
float tempF = 21.12;                                                               // A global to hold the temperature in Fahrenheit.
long rssi = -42;                                                                   // A global to hold the Received Signal Strength Indicator.
float tempCArray[] = { -21.12, 21.12, 42.42 };                          // An array to hold the 3 most recent Celsius values, initialized to reasonable levels.
float moistureArray[] = { 0.0, 300, 1000 };                       // An array to hold the 3 most recent moisture values, initialized to reasonable levels.


void pollTelemetry();
void printUptime();
void printTelemetry();
void publishTelemetry();
void runPump();


// Create class objects.
Adafruit_seesaw soilSensor;


#endif //ESP32_SOIL_OTA_ESP32_SOIL_OTA_H
