/**
 * @brief The purpose of this file is to hold network-related functions which are device-agnostic.
 * This is not realistic because of the presence of mqttCallback.
 * Ideally, this file could be used by an ESP32, ESP8266, or similar boards.
 * Because memory capacity varies wildly from device to device, buffer sizes are declared as variables in the entry-point file.
 */


#include "ESP32_Soil_OTA.h"


/**
 * @brief onReceiveCallback() processes MQTT callbacks.
 * @param topic the topic that the message arrived on.
 * @param payload the message to process.
 * @param length the length of the payload in bytes.
 */
void onReceiveCallback( char *topic, byte *payload, unsigned int length )
{
   Serial.printf( "\nMessage arrived on Topic: '%s'\n", topic );

   JsonDocument callbackJsonDoc;
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
   else if( strcmp( command, "pumpOn" ) == 0 )
   {
      unsigned long currentTime = millis();
      // Note the start time.
      pumpStartTime = currentTime;
      // Turn the relay (pump) on.  This relay board switches on when the pin is pulled low.
      digitalWrite( RELAY_GPIO, LED_OFF );
      Serial.println( "Turning the pump on." );
   }
   else if( strcmp( command, "pumpOff" ) == 0 )
   {
      unsigned long currentTime = millis();
      // Note the stop time.
      pumpStopTime = currentTime;
      // Turn the relay (pump) off.  This relay board switches off when the pin is pulled high.
      digitalWrite( RELAY_GPIO, LED_ON );
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
 * @brief configureOTA() will configure and initiate Over The Air (OTA) updates for this device.
 */
void configureOTA()
{
#ifdef ESP8266
   Serial.println( "Configuring OTA for the ESP8266" );
   // Port defaults to 8266
   // ArduinoOTA.setPort(8266);

   // Hostname defaults to esp8266-[ChipID]
   ArduinoOTA.setHostname( HOSTNAME );

   // No authentication by default
   ArduinoOTA.setPassword( otaPass );

   // Password can be set with it's md5 value as well
   // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
   // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

   ArduinoOTA.onStart( []()
                       {
                          String type;
                          if( ArduinoOTA.getCommand() == U_FLASH )
                             type = "sketch";
                          else // U_SPIFFS
                             type = "filesystem";

                          // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                          Serial.println( "Start updating " + type ); } );
   ArduinoOTA.onEnd( []()
                     { Serial.println( "\nEnd" ); } );
   ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total )
                          { Serial.printf( "Progress: %u%%\r", ( progress / ( total / 100 ) ) ); } );
   ArduinoOTA.onError( []( ota_error_t error )
                       {
                          Serial.printf( "Error[%u]: ", error );
                          if( error == OTA_AUTH_ERROR ) Serial.println( "Auth Failed" );
                          else if( error == OTA_BEGIN_ERROR )
                             Serial.println( "Begin Failed" );
                          else if( error == OTA_CONNECT_ERROR )
                             Serial.println( "Connect Failed" );
                          else if( error == OTA_RECEIVE_ERROR )
                             Serial.println( "Receive Failed" );
                          else if( error == OTA_END_ERROR )
                             Serial.println( "End Failed" ); } );
#else
   Serial.println( "Configuring OTA for the ESP32" );
   // The ESP32 port defaults to 3232
   // ArduinoOTA.setPort( 3232 );
   // The ESP32 hostname defaults to esp32-[MAC]
   //	ArduinoOTA.setHostname( HOSTNAME );  // I'm deliberately using the default.
   // Authentication is disabled by default.
   ArduinoOTA.setPassword( otaPass );
   // Password can be set with it's md5 value as well
   // MD5( admin ) = 21232f297a57a5a743894a0e4a801fc3
   // ArduinoOTA.setPasswordHash( "21232f297a57a5a743894a0e4a801fc3" );
   //	Serial.printf( "Using hostname '%s'\n", HOSTNAME );

   String type = "filesystem"; // SPIFFS
   if( ArduinoOTA.getCommand() == U_FLASH )
      type = "sketch";

   // Configure the OTA callbacks.
   ArduinoOTA.onStart( []()
                       {
                          String type = "flash"; // U_FLASH
                          if( ArduinoOTA.getCommand() == U_SPIFFS )
                             type = "filesystem";
                          // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                          Serial.print( "OTA is updating the " );
                          Serial.println( type ); } );
   ArduinoOTA.onEnd( []()
                     { Serial.println( "\nTerminating OTA communication." ); } );
   ArduinoOTA.onProgress( []( unsigned int progress, unsigned int total )
                          { Serial.printf( "OTA progress: %u%%\r", ( progress / ( total / 100 ) ) ); } );
   ArduinoOTA.onError( []( ota_error_t error )
                       {
                          Serial.printf( "Error[%u]: ", error );
                          if( error == OTA_AUTH_ERROR ) Serial.println( "OTA authentication failed!" );
                          else if( error == OTA_BEGIN_ERROR ) Serial.println( "OTA transmission failed to initiate properly!" );
                          else if( error == OTA_CONNECT_ERROR ) Serial.println( "OTA connection failed!" );
                          else if( error == OTA_RECEIVE_ERROR ) Serial.println( "OTA client was unable to properly receive data!" );
                          else if( error == OTA_END_ERROR ) Serial.println( "OTA transmission failed to terminate properly!" ); } );
#endif

   // Start listening for OTA commands.
   ArduinoOTA.begin();

   Serial.println( "OTA is configured and ready." );
} // End of the configureOTA() function.


/**
 * @brief checkForSSID() will scan for all visible SSIDs, see if any match 'ssidName',
 * and return a count of how many matches were found.
 *
 * @param ssidName the SSID name to search for.
 * @return int the count of SSIDs which match the passed parameter.
 */
int checkForSSID( const char *ssidName )
{
   int ssidCount = 0;
   byte networkCount = WiFi.scanNetworks();
   if( networkCount == 0 )
      Serial.println( "No WiFi SSIDs are in range!" );
   else
   {
      //      Serial.printf( "WiFi SSIDs in range: %d\n", networkCount );
      for( int i = 0; i < networkCount; ++i )
      {
         // Check to see if this SSID matches the parameter.
         if( strcmp( ssidName, WiFi.SSID( i ).c_str() ) == 0 )
            ssidCount++;
      }
   }
   return ssidCount;
} // End of checkForSSID() function.


/**
 * @brief wifiConnect() will attempt to connect to a single SSID.
 */
bool wifiConnect( const char *ssid, const char *password )
{
   wifiConnectCount++;
   // Turn the LED off to show Wi-Fi is not connected.
   digitalWrite( MCU_LED, LED_OFF );

   Serial.printf( "Attempting to connect to Wi-Fi SSID '%s'", ssid );
   WiFi.mode( WIFI_STA );
   //   WiFi.setHostname( HOSTNAME );
   WiFi.begin( ssid, password );

   unsigned long wifiConnectionStartTime = millis();

   // Loop until connected, or until wifiConnectionTimeout.
   while( WiFi.status() != WL_CONNECTED && ( millis() - wifiConnectionStartTime < wifiConnectionTimeout ) )
   {
      Serial.print( "." );
      delay( 1000 );
   }
   Serial.println( "" );

   if( WiFi.status() == WL_CONNECTED )
   {
      // Print that Wi-Fi has connected.
      Serial.println( "Wi-Fi connection established!" );
      snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
      // Turn the LED on to show that Wi-Fi is connected.
      digitalWrite( MCU_LED, LED_ON );
      return true;
   }
   Serial.println( "Wi-Fi failed to connect in the timeout period.\n" );
   return false;
} // End of the wifiConnect() function.


/**
 * @brief wifiMultiConnect() will iterate through the SSIDs in 'wifiSsidList[]', and then use checkForSSID() determine which are in range.
 * When a SSID is in range, wifiConnect() will be called with that SSID and password.
 */
void wifiMultiConnect()
{
   long time = millis();
   if( lastWifiConnectTime == 0 || ( time > wifiCoolDownInterval && ( time - wifiCoolDownInterval ) > lastWifiConnectTime ) )
   {
      Serial.println( "\nEntering wifiMultiConnect()" );
      digitalWrite( MCU_LED, LED_OFF ); // Turn the LED off to show that Wi-Fi is not yet connected.
      for( size_t networkArrayIndex = 0; networkArrayIndex < sizeof( wifiSsidArray ); networkArrayIndex++ )
      {
         // Get the details for this connection attempt.
         const char *wifiSsid = wifiSsidArray[networkArrayIndex];
         const char *wifiPassword = wifiPassArray[networkArrayIndex];

         // Announce the Wi-Fi parameters for this connection attempt.
         Serial.print( "Attempting to connect to to SSID \"" );
         Serial.print( wifiSsid );
         Serial.println( "\"" );

         // Don't even try to connect if the SSID cannot be found.
         int ssidCount = checkForSSID( wifiSsid );
         if( ssidCount > 0 )
         {
            // This is useful for detecting multiples APs.
            if( ssidCount > 1 )
               Serial.printf( "Found %d SSIDs matching '%s'.\n", ssidCount, wifiSsid );

            // If the Wi-Fi connection is successful, set the mqttClient broker parameters.
            if( wifiConnect( wifiSsid, wifiPassword ) )
            {
               mqttClient.setServer( mqttBrokerArray[networkArrayIndex], mqttPortArray[networkArrayIndex] );
               return;
            }
         }
         else
            Serial.printf( "Network '%s' was not found!\n\n", wifiSsid );
      }
      Serial.println( "Exiting wifiMultiConnect()\n" );
      lastWifiConnectTime = millis();
   }
} // End of wifiMultiConnect() function.


/*
 * mqttMultiConnect() will:
 * 1. Check the WiFi connection, and reconnect WiFi as needed.
 * 2. Attempt to connect the MQTT client up to 'maxAttempts' number of times.
 * 3. Subscribe to the topic defined in 'COMMAND_TOPIC'.
 * If the broker connection cannot be made, an error will be printed to the serial port.
 */
bool mqttMultiConnect( int maxAttempts )
{
   callbackCount++;
   Serial.println( "\nFunction mqttMultiConnect() has initiated." );
   if( WiFi.status() != WL_CONNECTED )
      wifiMultiConnect();

   mqttClient.setCallback( onReceiveCallback );

   int attemptNumber = 0;
   // Loop until MQTT has connected.
   while( !mqttClient.connected() && attemptNumber < maxAttempts )
   {
      mqttConnectCount++;
      // Put the macAddress and random number into clientId.
      char clientId[22];
      snprintf( clientId, 22, "%s-%03ld", macAddress, random( 999 ) );
      // Connect to the broker using the MAC address for a clientID.  This guarantees that the clientID is unique.
      Serial.printf( "Connecting with client ID '%s'.\n", clientId );
      Serial.printf( "Attempt # %d....", ( attemptNumber + 1 ) );
      if( mqttClient.connect( clientId ) )
      {
         Serial.println( " connected." );
         if( !mqttClient.setBufferSize( JSON_DOC_SIZE ) )
         {
            Serial.printf( "Unable to create a buffer %d bytes long!\n", JSON_DOC_SIZE );
            delay( 7000 );
            Serial.println( "Restarting the device!" );
            ESP.restart();
         }
         // Subscribe to the command topic.
         if( mqttClient.subscribe( COMMAND_TOPIC ) )
            Serial.printf( "Successfully subscribed to topic '%s'.\n", COMMAND_TOPIC );
         else
            Serial.printf( "Failed to subscribe to topic '%s'!\n", COMMAND_TOPIC );
      }
      else
      {
         char buffer[29];
         lookupMQTTCode( mqttClient.state(), buffer );
         Serial.printf( "  MQTT state: %s\n", buffer );

         Serial.printf( "Trying again in %lu seconds.\n\n", mqttReconnectInterval / 1000 );
         delay( mqttReconnectInterval );
      }
      attemptNumber++;
   }

   if( !mqttClient.connected() )
   {
      Serial.println( "Unable to connect to the MQTT broker!" );
      return false;
   }

   Serial.println( "Function mqttMultiConnect() has completed.\n" );
   return true;
} // End of mqttMultiConnect() function.


/**
 * @brief lookupWifiCode() will return the string for an integer code.
 */
void lookupWifiCode( int code, char *buffer )
{
   switch( code )
   {
      case 0:
         snprintf( buffer, 26, "%s", "Idle" );
         break;
      case 1:
         snprintf( buffer, 26, "%s", "No SSID" );
         break;
      case 2:
         snprintf( buffer, 26, "%s", "Scan completed" );
         break;
      case 3:
         snprintf( buffer, 26, "%s", "Connected" );
         break;
      case 4:
         snprintf( buffer, 26, "%s", "Connection failed" );
         break;
      case 5:
         snprintf( buffer, 26, "%s", "Connection lost" );
         break;
      case 6:
         snprintf( buffer, 26, "%s", "Disconnected" );
         break;
      default:
         snprintf( buffer, 26, "%s", "Unknown Wi-Fi status code" );
   }
} // End of lookupWifiCode() function.


/**
 * @brief lookupMQTTCode() will return the string for an integer state code.
 */
void lookupMQTTCode( int code, char *buffer )
{
   switch( code )
   {
      case -4:
         snprintf( buffer, 29, "%s", "Connection timeout" );
         break;
      case -3:
         snprintf( buffer, 29, "%s", "Connection lost" );
         break;
      case -2:
         snprintf( buffer, 29, "%s", "Connect failed" );
         break;
      case -1:
         snprintf( buffer, 29, "%s", "Disconnected" );
         break;
      case 0:
         snprintf( buffer, 29, "%s", "Connected" );
         break;
      case 1:
         snprintf( buffer, 29, "%s", "Bad protocol" );
         break;
      case 2:
         snprintf( buffer, 29, "%s", "Bad client ID" );
         break;
      case 3:
         snprintf( buffer, 29, "%s", "Unavailable" );
         break;
      case 4:
         snprintf( buffer, 29, "%s", "Bad credentials" );
         break;
      case 5:
         snprintf( buffer, 29, "%s", "Unauthorized" );
         break;
      default:
         snprintf( buffer, 29, "%s", "Unknown MQTT state code" );
   }
} // End of lookupMQTTCode() function.
