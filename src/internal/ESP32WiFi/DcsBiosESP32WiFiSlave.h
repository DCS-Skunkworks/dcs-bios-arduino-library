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

#ifndef DCSBIOS_ESP32_WIFI_MAX_RETRIES
	#define DCSBIOS_ESP32_WIFI_MAX_RETRIES 3
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
#include <deque>
#include <pb_decode.h>

namespace DcsBios {
	ProtocolParser parser;

	class Message {
	public:
		unsigned int id;
		unsigned int retries = 0;
		const char* type;
		String data;
		unsigned long lastSentTime = 0;
	};

	struct DataBuffer {
		uint8_t buffer[256];
		size_t size;
	};

	class ESP32WiFiSlave {
	public:
		void begin();
		void loop();
		void enqueue(const char* type);
		void enqueue(const char* type, String data);
		
		#ifdef DCSBIOS_ESP32_WIFI_TCP
		WiFiClient client;
		#else
		WiFiUDP client;
		#endif
	private:
		ClientInterface* multicast_client;

		IPAddress master_ip = IPAddress(0, 0, 0, 0);
		unsigned int master_port = 0;

		void send(const char* type);
		void send(const char* type, String data, unsigned int seq);

		unsigned long lastReceivedTime = 0;
		const unsigned long timeoutDuration = 3000;
		unsigned long lastKeepAliveTime = 0;
		const unsigned long keepAliveTimeout = 1000;

		const char* ssid = DCSBIOS_ESP32_WIFI_SSID;
		const char* password = DCSBIOS_ESP32_WIFI_PASSWORD;

		static bool decode_bytes(pb_istream_t *stream, const pb_field_t *field, void **arg);

		unsigned int last_message_id = 0;
		std::deque<Message> messages;

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