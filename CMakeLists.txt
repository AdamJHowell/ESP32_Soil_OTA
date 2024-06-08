cmake_minimum_required( VERSION 3.23 )
project( ESP32_Soil_OTA )

set( CMAKE_CXX_STANDARD 14 )

include_directories( .
		"C:/Users/Adam/AppData/Local/Arduino15/packages/esp32/hardware/esp32/3.0/libraries/ESPmDNS/src/"
		"C:/Users/Adam/AppData/Local/Arduino15/packages/esp32/hardware/esp32/3.0/libraries/Wire/src"
		"C:/Users/Adam/Documents/Arduino/libraries/ArduinoJson"
		"C:/Users/Adam/Documents/Arduino/libraries/Adafruit_seesaw_Library"
)

add_executable( ESP32_Soil_OTA
		ESP32_Soil_OTA.ino
		privateInfo.h
		NetworkFunctions.ino ESP32_Soil_OTA.h NetworkFunctions.h )

set_target_properties( ESP32_Soil_OTA PROPERTIES LINKER_LANGUAGE CXX )
