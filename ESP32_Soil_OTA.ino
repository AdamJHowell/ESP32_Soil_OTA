/**
 * @brief This sketch will use an Adafruit I2C soil sensor to show soil moisture and temperature levels.
 * The ESP32 I2C SCL is on GPIO22, and SDA is GPIO33.
 * The first relay pin is on GPIO04.
 *
 * My topic formats are:
 *    <location>/<device>/<device reading>
 *    <location>/<device>/<sensor type>/<sensor reading>
 *
 * @copyright   Copyright © 2022 Adam Howell
 * @license     The MIT License (MIT)
 *
 * Pseudocode:
 * 1. Read the soil moisture level (capacitance) every sensorPollDelay.
 * 2. Read the temperature from the soil moisture sensor.
 * 3. If moisture is less than minMoisture, run pump for pumpRunTime.
 * The pump is run in pumpRunTime millisecond increments instead of waiting for the reading to get above the minimum level.  Waiting could result in over watering.
 * If the moisture level is still low after pumpRunTime milliseconds, it will trigger again during the next loop() after pumpMinOffDelay milliseconds has passed.
 *
 * Publish:
 * 	every time the pump runs: soilMoisture and tempC
 * 	every publishInterval: all stats (soilMoisture, tempC, publishCount, etc.)
 */
#ifdef ESP8266
// These headers are installed when the ESP8266 is installed in board manager.
#include "ESP8266WiFi.h" // ESP8266 WiFi support.  https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi
#include <ESP8266mDNS.h> // OTA - mDNSResponder (Multicast DNS) for the ESP8266 family.
#elif ESP32
// These headers are installed when the ESP32 is installed in board manager.
#include "WiFi.h"		// ESP32 Wifi support.  https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFi.h
#include <ESPmDNS.h> // OTA - Multicast DNS for the ESP32.
#else
#include "WiFi.h" // Arduino Wi-Fi support.  This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#endif
#include <Wire.h>				  // This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>	  // PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>	  // A JSON manipulation library.  Author: Benoît Blanchon  https://github.com/bblanchon/ArduinoJson  https://arduinojson.org/
#include <WiFiUdp.h>			  // OTA
#include <ArduinoOTA.h>		  // OTA - The Arduino OTA library.  Specific version of this are installed along with specific boards in board manager.
#include <Adafruit_seesaw.h> // Used to read the soil sensor. https://github.com/adafruit/Adafruit_Seesaw - Requires https://github.com/adafruit/Adafruit_BusIO
#include "privateInfo.h"	  // I use this file to hide my network information from random people browsing my GitHub repo.

/**
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 * If you do not want to create that file, set them here instead.
 */
//const char* wifiSsid = "your WiFi SSID";
//const char* wifiPassword = "your Wi-Fi password";
//const char* mqttBroker = "your broker address";
//const int mqttPort = 1883;
const char *hostName = "ESP32_Soil_OTA";											// The network hostname to set.
const char *notes = "HiLetgo ESP32 with Adafruit I2C soil sensor";		// Notes about this program.
const char *commandTopic = "backYard/plantWatering/command";				// The topic used to subscribe to update commands.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char *sketchTopic = "backYard/plantWatering/sketch";					// The topic used to publish the sketch name.
const char *macTopic = "backYard/plantWatering/mac";							// The topic used to publish the MAC address.
const char *ipTopic = "backYard/plantWatering/ip";								// The topic used to publish the IP address.
const char *rssiTopic = "backYard/plantWatering/rssi";						// The topic used to publish the Wi-Fi Received Signal Strength Indicator.
const char *publishCountTopic = "backYard/plantWatering/publishCount";	// The topic used to publish the loop count.
const char *notesTopic = "backYard/plantWatering/notes";						// The topic used to publish notes relevant to this project.
const char *tempCTopic = "backYard/plantWatering/soil/tempC";				// The topic used to publish the soil temperature in Celsius.
const char *tempFTopic = "backYard/plantWatering/soil/tempF";				// The topic used to publish the soil temperature in Fahrenheit.
const char *moistureTopic = "backYard/plantWatering/soil/moisture";		// The topic used to publish the soil moisture.
const char *mqttStatsTopic = "espStats";											// The topic this device will publish to upon connection to the broker.
const char *mqttTopic = "espWeather";												// The topic used to publish a single JSON message containing all data.
const unsigned int LED_PIN = 2;														// Use this LED for notifications.
const unsigned int sdaGPIO = 33;														// The GPIO to use for SDA.
const unsigned int sclGPIO = 22;														// The GPIO to use for SCL.
const unsigned int relayGPIO = 4;													// The GPIO which controls the relay.
const int JSON_DOC_SIZE = 512;														// The ArduinoJson document size.
unsigned int mqttReconnectDelay = 5000;											// How long to wait (in milliseconds) between MQTT connection attempts.
unsigned long publishCount = 0;														// A count of how many publishes have taken place.
unsigned long publishInterval = 60000;												// The delay in milliseconds between MQTT publishes.  This prevents "flooding" the broker.
unsigned long sensorPollDelay = 10000;												// This is the delay between polls of the soil sensor.  This should be greater than 100 milliseconds.
unsigned long pumpRunTime = 20000;													// Minimum time to run the pump.
unsigned long pumpMinOffDelay = 20000;												// The time to wait after stopping, before the pump will start again.  This allows water to flow through the soil.
unsigned long lastPublishTime = 0;													// This is used to determine the time since last MQTT publish.
unsigned long lastMqttConnectionTime = 0;											// The last time a MQTT connection was attempted.
unsigned int mqttReconnectCooldown = 20000;										// Set the minimum time between calls to mqttMultiConnect() to 20 seconds.
unsigned long lastPollTime = 0;														// This is used to determine the time since last sensor poll.
unsigned long bootTime = 0;															// The time since boot.  This value "rolls" at about 50 days.
unsigned long pumpStartTime = 0;														// The most recent time that the pump started.
unsigned long pumpStopTime = 0;														// The most recent time that the pump stopped.
char ipAddress[16];																		// The IP address.
char macAddress[18];																		// The MAC address to use as part of the MQTT client ID.
bool pumpRunning = false;																// Flag to indicate when the pump is running or not.
unsigned int invalidTemp = 0;															// Holds the current number of consecutive invalid temperature readings.
unsigned int invalidMoisture = 0;													// Holds the current number of consecutive invalid humidity readings.
float tempC;																				// A global to hold the temperature in Celsius.
float tempF;																				// A global to hold the temperature in Fahrenheit.
long rssi;																					// A global to hold the Received Signal Strength Indicator.
uint16_t soilMoisture = 0;																// The soil moisture level (capacitance).
uint16_t minMoisture = 500;															// The moisture level which triggers the pump.


// Create class objects.
WiFiClient espClient;
PubSubClient mqttClient( espClient );
Adafruit_seesaw soilSensor;


/**
 * @brief onReceiveCallback() processes MQTT callbacks.
 * @param topic the topic that the message arrived on.
 * @param payload the message to process.
 * @param length the length of the payload in bytes.
 */
void onReceiveCallback( char *topic, byte *payload, unsigned int length )
{
	Serial.printf( "\nMessage arrived on Topic: '%s'\n", topic );

	StaticJsonDocument<JSON_DOC_SIZE> callbackJsonDoc;
	deserializeJson( callbackJsonDoc, payload, length );

	// The command can be: publishTelemetry, publishStatus, pumpOn, pumpOff, or changeTelemetryInterval.
	const char *command = callbackJsonDoc["command"];
	if( strcmp( command, "publishTelemetry" ) == 0 )
	{
		Serial.println( "Reading and publishing sensor values." );
		// Poll the sensor.
		readTelemetry();
		// Publish the sensor readings.
		publishTelemetry();
		Serial.println( "Readings have been published." );
	}
	else if( strcmp( command, "publishStatus" ) == 0 )
	{
		Serial.println( "publishStatus is not yet implemented." );
	}
	else if( strcmp( command, "pumpOn" ) == 0 )
	{
		unsigned long currentTime = millis();
		// Note the start time.
		pumpStartTime = currentTime;
		// Turn the relay (pump) on.  This relay board switches on when the pin is pulled low.
		digitalWrite( relayGPIO, LOW );
		Serial.println( "Turning the pump on." );
	}
	else if( strcmp( command, "pumpOff" ) == 0 )
	{
		unsigned long currentTime = millis();
		// Note the stop time.
		pumpStopTime = currentTime;
		// Turn the relay (pump) off.  This relay board switches off when the pin is pulled high.
		digitalWrite( relayGPIO, HIGH );
		Serial.println( "Turning the pump off." );
	}
	else if( strcmp( command, "changeTelemetryInterval" ) == 0 )
	{
		Serial.println( "Changing the publish interval." );
		unsigned long tempValue = callbackJsonDoc["value"];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds confusion.
		if( tempValue > 4000 )
			publishInterval = tempValue;
		Serial.print( "MQTT publish interval has been updated to " );
		Serial.println( publishInterval );
		lastPublishTime = 0;
	}
	else
		Serial.printf( "Unknown command '%s'\n", command );
} // End of onReceiveCallback() function.


/**
 * @brief readTelemetry() will:
 * 1. read from all available sensors
 * 2. store legitimate values in global variables
 * 3. set a flag if any value is invalid
 */
void readTelemetry()
{
	float temporarySoilTempC = soilSensor.getTemp();
	uint16_t temporarySoilMoisture = soilSensor.touchRead( 0 );

	// Define the valid temperature range (in Celsius) for this sensor.
	if( temporarySoilTempC > -20 || temporarySoilTempC < 70 )
	{
		tempC = temporarySoilTempC;
		tempF = ( tempC * 9 / 5 ) + 32;
		invalidTemp = 0;
	}
	else
		invalidTemp++;

	// Define the valid moisture range for this sensor.
	if( temporarySoilMoisture > 100 || temporarySoilMoisture < 2000 )
	{
		soilMoisture = temporarySoilMoisture;
		invalidMoisture = 0;
	}
	else
			invalidMoisture++;

	// If either invalid count is too high, reset the device.
	if( invalidTemp > 4 || invalidMoisture > 4 )
	{
		Serial.println( "\n\n\n\n" );
		Serial.printf( "%u consecutive bad temperature readings!\n", invalidTemp );
		Serial.printf( "%u consecutive bad humidity readings!\n", invalidMoisture );
		mqttClient.publish( mqttStatsTopic, "Resetting the device due to invalid sensor readings!", false );
		Serial.println( "Resetting the device!\n\n\n" );
		ESP.restart();
	}
} // End of readTelemetry() function.


/**
 * @brief wifiConnect() will attempt to connect to the defined Wi-Fi network up to maxAttempts times.
 */
void wifiConnect( int maxAttempts )
{
	// Announce Wi-Fi parameters.
	Serial.print( "WiFi connecting to SSID: " );
	Serial.println( wifiSsid );
	WiFi.config( INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE );
	//WiFi.setHostname( hostName );


	// Connect to the Wi-Fi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : " - Failed!" );
	WiFi.begin( wifiSsid, wifiPassword );

	int i = 1;
	/*
     WiFi.status() return values:
     0 : WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
     1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
     3 : WL_CONNECTED after successful connection is established
     4 : WL_CONNECT_FAILED if wifiPassword is incorrect
     6 : WL_DISCONNECTED if module is not configured in station mode
  */
	// Loop until Wi-Fi has connected.
	while( WiFi.status() != WL_CONNECTED && i < maxAttempts )
	{
		digitalWrite( LED_PIN, 1 ); // Turn the LED off.
		delay( 1000 );
		Serial.println( "Waiting for a connection..." );
		Serial.print( "WiFi status: " );
		Serial.println( WiFi.status() );
		Serial.print( i++ );
		Serial.println( " seconds" );
	}

	if( WiFi.status() == WL_CONNECTED )
	{
		WiFi.setAutoReconnect( true );
		WiFi.persistent( true );

		// Print that Wi-Fi has connected.
		Serial.println( '\n' );
		Serial.println( "WiFi connection established!" );
		Serial.print( "MAC address: " );
		Serial.println( macAddress );
		Serial.print( "IP address: " );
		snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
		Serial.println( ipAddress );

		digitalWrite( LED_PIN, 0 ); // Turn the Wi-Fi LED on.
	}
} // End of wifiConnect() function.


/**
 * @brief mqttConnect() will attempt to (re)connect the MQTT client.
 */
bool mqttConnect( int maxAttempts )
{
	// Reconnect to Wi-Fi if necessary.
	if( WiFi.status() != WL_CONNECTED )
		wifiConnect( 2 );

	if( WiFi.status() == WL_CONNECTED )
	{
		unsigned long time = millis();
		if( lastMqttConnectionTime == 0 || ( ( time > mqttReconnectCooldown ) && ( time - mqttReconnectCooldown ) > lastMqttConnectionTime ) )
		{
			digitalWrite( LED_PIN, 1 ); // Turn the LED off.
			Serial.print( "Attempting to connect to the MQTT broker up to " );
			Serial.print( maxAttempts );
			Serial.println( " times." );

			int i = 0;
			// Loop until MQTT has connected.
			while( !mqttClient.connected() && i < maxAttempts )
			{
				Serial.print( "Attempt # " );
				Serial.print( i + 1 );
				Serial.print( "..." );

				char clientId[22];
				// Put the macAddress and a random number into clientId.  The random number suffix prevents brokers from rejecting a clientID as already in use.
				snprintf( clientId, 22, "%s-%03d", macAddress, random( 999 ) );
				Serial.print( "Connecting with client ID '" );
				Serial.print( clientId );
				Serial.print( "' " );

				// Connect to the broker using the pseudo-random clientId.
				if( mqttClient.connect( clientId ) )
				{
					Serial.println( " connected!" );
					digitalWrite( LED_PIN, 0 ); // Turn the LED on.
				}
				else
				{
					Serial.print( " failed!  Return code: " );
					Serial.print( mqttClient.state() );
					Serial.print( ".  Trying again in " );
					Serial.print( mqttReconnectDelay / 1000 );
					Serial.println( " seconds." );
					digitalWrite( LED_PIN, 0 ); // Turn the LED on.
					delay( mqttReconnectDelay / 2 );
					digitalWrite( LED_PIN, 1 ); // Turn the LED off.
					delay( mqttReconnectDelay / 2 );
				}
				i++;
			}
			if( mqttClient.connected() )
			{
				// Subscribe to backYard/plantWatering/command, which will respond to publishTelemetry and publishStatus
				mqttClient.subscribe( commandTopic );
				mqttClient.setBufferSize( JSON_DOC_SIZE );
				char connectString[JSON_DOC_SIZE];
				snprintf( connectString, JSON_DOC_SIZE, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\"\n}", __FILE__, macAddress, ipAddress );
				mqttClient.publish( "espConnect", connectString, false );
				digitalWrite( LED_PIN, 0 ); // Turn the LED on.
			}
			else
			{
				Serial.println( "Unable to connect to the MQTT broker!" );
				return false;
			}

			Serial.println( "Function mqttConnect() has completed." );
			lastMqttConnectionTime = millis();
			return true;
		}
	}
} // End of mqttConnect() function.


/**
 * @brief setup() will initialize the device and connected components.
 */
void setup()
{
	delay( 1000 );						// A pause to give me time to open the serial monitor.
	pinMode( LED_PIN, OUTPUT );	// Initialize the GPIO which controls the LED as an output.
	digitalWrite( LED_PIN, 0 );	// Turn the LED on.
	Wire.begin();						// Initialize I2C communication.

	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );

	Serial.println( '\n' );
	Serial.print( __FILE__ );
	Serial.println( " is beginning its setup()." );
	Serial.println( __FILE__ );

	pinMode( relayGPIO, OUTPUT );	  // Set the replay (pump) GPIO as an output.
	digitalWrite( relayGPIO, LOW ); // Turn the relay (pump) off.

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );
	mqttClient.setCallback( onReceiveCallback ); // Assign the onReceiveCallback() function to handle MQTT callbacks.

	Serial.println( "Attempting to connect to the soil sensor..." );
	if( !soilSensor.begin( 0x36 ) )
		Serial.println( "\n\nERROR: soil sensor not found!\n\n" );
	else
	{
		Serial.print( "Seesaw started! version: " );
		Serial.println( soilSensor.getVersion(), HEX );
	}

	// Set the MAC address variable to its value.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );

	wifiConnect( 5 );

	configureOTA();

	snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
	Serial.printf( "IP address: %s\n", ipAddress );

	printUptime();
} // End of setup() function.


/**
 * @brief configureOTA() will configure and initiate Over The Air (OTA) updates for this device.
 */
void configureOTA()
{
	Serial.println( "Configuring OTA." );

#ifdef ESP8266
	// The ESP8266 port defaults to 8266
	// ArduinoOTA.setPort( 8266 );
	// The ESP8266 hostname defaults to esp8266-[ChipID]
	ArduinoOTA.setHostname( hostName );
	// Authentication is disabled by default.
	// ArduinoOTA.setPassword( ( const char * )"admin" );
#elif ESP32
	// The ESP32 port defaults to 3232
	// ArduinoOTA.setPort( 3232 );
	// The ESP32 hostname defaults to esp32-[MAC]
	ArduinoOTA.setHostname( hostName );
	// Authentication is disabled by default.
	// ArduinoOTA.setPassword( "admin" );
	// Password can be set with it's md5 value as well
	// MD5( admin ) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash( "21232f297a57a5a743894a0e4a801fc3" );
#else
	// ToDo: Verify how stock Arduino code is meant to handle the port, username, and password.
	ArduinoOTA.setHostname( hostName );
#endif

	Serial.printf( "Using hostname '%s'\n", hostName );

	String type = "filesystem";	// SPIFFS
	if( ArduinoOTA.getCommand() == U_FLASH )
		type = "sketch";

	// Configure the OTA callbacks.
	ArduinoOTA.onStart( []()
	{
		String type = "flash";	// U_FLASH
		if( ArduinoOTA.getCommand() == U_SPIFFS )
			type = "filesystem";
		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		Serial.print( "OTA is updating the " );
		Serial.println( type );
	} );
	ArduinoOTA.onEnd( []() { Serial.println( "\nTerminating OTA communication." ); } );
	ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total ){ Serial.printf( "OTA progress: %u%%\r", ( progress / ( total / 100 ) ) ); } );
	ArduinoOTA.onError( []( ota_error_t error ){
		Serial.printf( "Error[%u]: ", error );
		if( error == OTA_AUTH_ERROR ) Serial.println( "OTA authentication failed!" );
		else if( error == OTA_BEGIN_ERROR ) Serial.println( "OTA transmission failed to initiate properly!" );
		else if( error == OTA_CONNECT_ERROR ) Serial.println( "OTA connection failed!" );
		else if( error == OTA_RECEIVE_ERROR ) Serial.println( "OTA client was unable to properly receive data!" );
		else if( error == OTA_END_ERROR ) Serial.println( "OTA transmission failed to terminate properly!" ); } );

	// Start listening for OTA commands.
	ArduinoOTA.begin();

	Serial.println( "OTA is configured and ready." );
} // End of the configureOTA() function.


/**
 * @brief printUptime() will print the uptime to the serial port.
 */
void printUptime()
{
	Serial.print( "Uptime in " );
	long seconds = ( millis() - bootTime ) / 1000;
	long minutes = seconds / 60;
	long hours = minutes / 60;
	if( seconds < 601 )
	{
		Serial.print( "seconds: " );
		Serial.println( seconds );
	}
	else if( minutes < 121 )
	{
		Serial.print( "minutes: " );
		Serial.println( minutes );
	}
	else
	{
		Serial.print( "hours: " );
		Serial.println( hours );
	}
} // End of printUptime() function.


/**
 * @brief printTelemetry() will print the sensor and device data to the serial port.
 */
void printTelemetry()
{
	Serial.printf( "WiFi SSID: %s\n", wifiSsid );
	Serial.printf( "Broker: %s:%d\n", mqttBroker, mqttPort );
	Serial.printf( "Temperature: %.2f C\n", tempC );
	Serial.printf( "Temperature: %.2f F\n", tempF );
	Serial.printf( "Moisture: %.2f %%\n", soilMoisture );
	Serial.printf( "WiFi RSSI: %ld\n", rssi );
} // End of printTelemetry() function.


/**
 * @brief publishTelemetry() will publish the sensor and device data over MQTT.
 */
void publishTelemetry()
{
	// Create a JSON Document on the stack.
	StaticJsonDocument<JSON_DOC_SIZE> publishTelemetryJsonDoc;
	// Add data: __FILE__, macAddress, ipAddress, tempC, tempF, soilMoisture, rssi, publishCount, notes
	publishTelemetryJsonDoc["sketch"] = __FILE__;
	publishTelemetryJsonDoc["mac"] = macAddress;
	publishTelemetryJsonDoc["ip"] = ipAddress;
	publishTelemetryJsonDoc["tempC"] = tempC;
	publishTelemetryJsonDoc["tempF"] = tempF;
	publishTelemetryJsonDoc["soilMoisture"] = soilMoisture;
	publishTelemetryJsonDoc["rssi"] = rssi;
	publishTelemetryJsonDoc["publishCount"] = publishCount;
	publishTelemetryJsonDoc["notes"] = notes;

	// Prepare a String to hold the JSON.
	char mqttString[JSON_DOC_SIZE];
	// Serialize the JSON into mqttString, with indentation and line breaks.
	serializeJsonPretty( publishTelemetryJsonDoc, mqttString );
	// Publish the JSON to the MQTT broker.
	bool success = mqttClient.publish( mqttTopic, mqttString, false );
	if( success )
	{
		Serial.println( "Successfully published to:" );
		char buffer[20];
		// New format: <location>/<device>/<sensor>/<value>
		if( mqttClient.publish( sketchTopic, __FILE__, false ) )
			Serial.printf( "  %s\n", sketchTopic );
		if( mqttClient.publish( macTopic, macAddress, false ) )
			Serial.printf( "  %s\n", macTopic );
		if( mqttClient.publish( ipTopic, ipAddress, false ) )
			Serial.printf( "  %s\n", ipTopic );
		ltoa( rssi, buffer, 10 );
		if( mqttClient.publish( rssiTopic, buffer, false ) )
			Serial.printf( "  %s\n", rssiTopic );
		ltoa( publishCount, buffer, 10 );
		if( mqttClient.publish( publishCountTopic, buffer, false ) )
			Serial.printf( "  %s\n", publishCountTopic );
		if( mqttClient.publish( notesTopic, notes, false ) )
			Serial.printf( "  %s\n", notesTopic );
		dtostrf( tempC, 1, 3, buffer );
		if( mqttClient.publish( tempCTopic, buffer, false ) )
			Serial.printf( "  %s\n", tempCTopic );
		dtostrf( ( tempF ), 1, 3, buffer );
		if( mqttClient.publish( tempFTopic, buffer, false ) )
			Serial.printf( "  %s\n", tempFTopic );
		dtostrf( ( soilMoisture ), 1, 3, buffer );
		if( mqttClient.publish( moistureTopic, buffer, false ) )
			Serial.printf( "  %s\n", moistureTopic );

		Serial.printf( "Successfully published to '%s', this JSON:\n", mqttTopic );
	}
	else
		Serial.println( "MQTT publish failed!  Attempted to publish this JSON to the broker:" );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );
	lastPublishTime = millis();
} // End of publishTelemetry() function.


/**
 * @brief publishStats() is called by mqttMultiConnect() every time the device (re)connects to the broker, and every publishInterval milliseconds thereafter.
 * It is also called by the callback when the "publishStats" command is received.
 */
void publishStats()
{
	char mqttStatsString[JSON_DOC_SIZE];
	// Create a JSON Document on the stack.
	StaticJsonDocument<JSON_DOC_SIZE> doc;
	// Add data: __FILE__, macAddress, ipAddress, rssi, publishCount
	doc["sketch"] = __FILE__;
	doc["mac"] = macAddress;
	doc["ip"] = ipAddress;
	doc["rssi"] = rssi;
	doc["publishCount"] = publishCount;

	// Serialize the JSON into mqttStatsString, with indentation and line breaks.
	serializeJsonPretty( doc, mqttStatsString );

	if( mqttClient.connected() )
	{
		if( mqttClient.connected() && mqttClient.publish( mqttStatsTopic, mqttStatsString ) )
			Serial.printf( "Published to this broker and port: %s:%d, and to this topic '%s':\n%s\n", mqttBroker, mqttPort, mqttStatsTopic, mqttStatsString );
		else
			Serial.println( "\n\nPublish failed!\n\n" );
	}
} // End of publishStats() function.


/**
 * @brief runPump() will turn the pump on and off as needed.
 *
 * The pump will turn on when:
 * 	The pump is not currently running.
 * 	The moisture level is low.
 * 	It has been off for at least pumpMinOffDelay milliseconds.
 *
 * The pump will turn off when:
 * 	The pump is currently running.
 * 	The pump has been running for at least pumpRunTime milliseconds.
 * 	Note that it does not check the moisture level.  This is deliberate.
 */
void runPump()
{
	unsigned long currentTime = millis();

	// If the soil is dry and the pump is not running.
	if( !pumpRunning && ( soilMoisture < minMoisture ) )
	{
		// If enough time has passed since the pump was shut off (so to give time for water to flow to the sensor).
		if( currentTime - pumpStopTime > pumpMinOffDelay )
		{
			Serial.println( "Starting the pump." );
			// Note the start time.
			pumpStartTime = currentTime;
			// Turn the relay (pump) on.  This relay board switches on when the pin is pulled to ground.
			digitalWrite( relayGPIO, LOW );
			// Flag that the pump is now running.
			pumpRunning = true;
		}
	}

	// If the pump is currently running.
	if( pumpRunning )
	{
		// If enough time has passed since the pump was started.
		if( currentTime - pumpStartTime > pumpRunTime )
		{
			Serial.println( "Stopping the pump." );
			// Note the start time.
			pumpStopTime = currentTime;
			// Turn the relay (pump) off.  This relay board switches off when the pin is pulled high.
			digitalWrite( relayGPIO, HIGH );
			// Flag that the pump has stopped.
			pumpRunning = false;
		}
	}
} // End of runPump() function.


void loop()
{
	// Reconnect to Wi-Fi and the MQTT broker as needed.
	if( !mqttClient.connected() )
		mqttConnect( 2 );
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	ArduinoOTA.handle();

	// Check if the pump should be run.
	runPump();

	unsigned long time = millis();
	if( lastPollTime == 0 || ( ( time > sensorPollDelay ) && ( time - sensorPollDelay ) > lastPollTime ) )
	{
		readTelemetry();
		Serial.print( "Temperature: " );
		Serial.print( tempC );
		Serial.println( " C" );
		Serial.print( "Moisture: " );
		Serial.println( soilMoisture );
		Serial.println( "" );
		lastPollTime = millis();
	}

	time = millis();
	// if( lastPublishTime == 0 || ( ( time > publishInterval ) && ( time - publishInterval ) > lastPublishTime ) )
	if( ( time > publishInterval ) && ( time - publishInterval ) > lastPublishTime )
	{
		publishCount++;
		// These next 3 lines act as a "heartbeat", to give local users a visual indication that the system is working.
		digitalWrite( LED_PIN, 0 ); // Turn the LED off, to alert the user that a publish is about to take place.
		delay( 1000 );					 // Wait for one second.
		digitalWrite( LED_PIN, 1 ); // Turn the LED on.

		readTelemetry();
		printUptime();
		printTelemetry();
		publishTelemetry();
		publishStats();

		Serial.printf( "publishCount: %lu\n", publishCount );

		lastPublishTime = millis();
		Serial.printf( "Next MQTT publish in %lu seconds.\n\n", publishInterval / 1000 );
	}
} // End of loop() function.
