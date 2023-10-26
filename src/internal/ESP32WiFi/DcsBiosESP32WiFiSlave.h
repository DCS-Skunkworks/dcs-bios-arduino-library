#ifndef _DCSBIOS_ESP32_WIFI_H_
#define _DCSBIOS_ESP32_WIFI_H_
#ifdef DCSBIOS_ESP32_WIFI

#ifndef ARDUINO_ARCH_ESP32
	#error "This code is designed to run on ESP32! Please check your build settings."
#endif

#ifndef DCSBIOS_ESP32_WIFI_LOCAL_PORT
	#define DCSBIOS_ESP32_WIFI_LOCAL_PORT 7779
#endif

#ifndef DCSBIOS_ESP32_WIFI_MULTICAST_GROUP
	#define DCSBIOS_ESP32_WIFI_MULTICAST_GROUP IPAddress(232, 0, 1, 3)
#endif

#ifdef DCSBIOS_ESP32_WIFI_TCP
#define DCSBIOS_ESP32_WIFI_SERVICE "_dcs-bios", "_tcp"
#else
#define DCSBIOS_ESP32_WIFI_SERVICE "_dcs-bios", "_udp"
#endif

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUDP.h>

#include "Utils.h"
#include "Utils.cpp.inc"

namespace DcsBios {
	ProtocolParser parser;

	class ESP32WiFiSlave {
	public:
		void begin();
		void loop();
		void send(const char* type);
		void send(const char* type, String data);

		#ifdef DCSBIOS_ESP32_WIFI_TCP
		WiFiClient client;
		#else
		WiFiUDP client;
		#endif
	private:
		IPAddress master_ip = IPAddress(0, 0, 0, 0);
		unsigned int master_port = 0;

		unsigned long lastReceivedTime = 0;
		const unsigned long timeoutDuration = 3000;
		unsigned long lastKeepAliveTime = 0;
		const unsigned long keepAliveTimeout = 1000;

		const char* ssid = DCSBIOS_ESP32_WIFI_SSID;
		const char* password = DCSBIOS_ESP32_WIFI_PASSWORD;

		bool udp_ready;
		bool connected();

		#ifdef DCSBIOS_ESP32_WIFI_MULTICAST
		bool registered = false;
		#endif

		// Performance Instrumentation
		#ifdef DCSBIOS_ESP32_WIFI_INSTRUMENTATION
		unsigned long lastLoopTime = 0;
		unsigned long lastLoopDuration = 0;
		#endif
	};

	ESP32WiFiSlave wifiSlave;
}

#endif // DCSBIOS_ESP32_WIFI
#endif // _DCSBIOS_ESP32_WIFI_H_