cmake_minimum_required( VERSION 3.23 )
project( ESP32_Soil_OTA )

set( CMAKE_CXX_STANDARD 14 )

include_directories( . )

add_executable( ESP32_Soil_OTA
		ESP32_Soil_OTA.ino
		privateInfo.h )
