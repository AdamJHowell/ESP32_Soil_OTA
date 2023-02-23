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


#include "ESP32_Soil_OTA.h"


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
 * @brief setup() will initialize the device and connected components.
 */
void setup()
{
	delay( 1000 );					 // A pause to give me time to open the serial monitor.
	pinMode( MCU_LED, OUTPUT ); // Initialize the GPIO which controls the LED as an output.
	digitalWrite( MCU_LED, LED_ON ); // Turn the LED on.
	Wire.begin();					 // Initialize I2C communication.

	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );

	Serial.println( '\n' );
	Serial.print( __FILE__ );
	Serial.println( " is beginning its setup()." );
	Serial.println( __FILE__ );

	pinMode( relayGPIO, OUTPUT );	  // Set the replay (pump) GPIO as an output.
	digitalWrite( relayGPIO, LED_OFF ); // Turn the relay (pump) off.

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

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

	wifiMultiConnect();
	configureOTA();

	snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
	Serial.printf( "IP address: %s\n", ipAddress );

	printUptime();
} // End of setup() function.


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
	Serial.println();
	Serial.println( __FILE__ );
	Serial.println();

	Serial.println( "Network stats:" );
	Serial.printf( "  MAC address: %s\n", macAddress );
	int wifiStatusCode = WiFi.status();
	char buffer[29];
	lookupWifiCode( wifiStatusCode, buffer );
	if( wifiStatusCode == 3 )
	{
		Serial.printf( "  IP address: %s\n", ipAddress );
		Serial.printf( "  RSSI: %ld\n", rssi );
		Serial.print( "~~IP address: " );
		Serial.println( WiFi.localIP() );
	}
	Serial.printf( "  wifiConnectCount: %u\n", wifiConnectCount );
	Serial.printf( "  wifiCoolDownInterval: %lu\n", wifiCoolDownInterval );
	Serial.printf( "  Wi-Fi status text: %s\n", buffer );
	Serial.printf( "  Wi-Fi status code: %d\n", wifiStatusCode );
	Serial.println();

	Serial.printf( "Broker: %s:%d\n", mqttClient.getServerDomain(), mqttClient.getServerPort() );
	Serial.printf( "Temperature: %.2f C\n", tempC );
	Serial.printf( "Temperature: %.2f F\n", tempF );
	Serial.printf( "Moisture: %.2f %%\n", soilMoisture );
	Serial.printf( "WiFi RSSI: %ld\n", rssi );
	printUptime();
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
			Serial.printf( "Published to this broker and port: %s:%d, and to this topic '%s':\n%s\n", mqttClient.getServerDomain(), mqttClient.getServerPort(), mqttStatsTopic, mqttStatsString );
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
			digitalWrite( relayGPIO, LED_OFF );
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
			digitalWrite( relayGPIO, LED_ON );
			// Flag that the pump has stopped.
			pumpRunning = false;
		}
	}
} // End of runPump() function.


void loop()
{
	// Reconnect to Wi-Fi and the MQTT broker as needed.
	if( !mqttClient.connected() )
		mqttMultiConnect( 2 );
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	ArduinoOTA.handle();

	// Check if the pump should be run.
	runPump();

	unsigned long time = millis();
	// Poll the first time.  Avoid subtraction overflow.  Poll every interval.
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
		digitalWrite( MCU_LED, LED_OFF ); // Turn the LED off, to alert the user that a publish is about to take place.
		delay( 1000 );					 // Wait for one second.
		digitalWrite( MCU_LED, LED_ON ); // Turn the LED on.

		readTelemetry();
		printTelemetry();
		publishTelemetry();
		publishStats();

		Serial.printf( "publishCount: %lu\n", publishCount );

		lastPublishTime = millis();
		Serial.printf( "Next MQTT publish in %lu seconds.\n\n", publishInterval / 1000 );
	}
} // End of loop() function.
