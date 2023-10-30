#ifndef _DCSBIOS_ESP32_WIFI_H_
#define _DCSBIOS_ESP32_WIFI_H_
#ifdef DCSBIOS_ESP32_WIFI

#include <deque>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>

#include "Utils.h"
#include "Utils.cpp.inc"

namespace DcsBios {
	ProtocolParser parser;

	class ESP32WiFiSlave {
	public:
		void begin();
		void loop();
		void enqueue(const char* type);
		void enqueue(const char* type, String data);
	private:
		bool connect_wifi();
		bool disconnect_wifi();
		void set_client();

		// Swappable client depending on communication protocol
		ClientInterface* client;

		// Timeout and keepalive
		unsigned long lastReceivedTime = 0;
		unsigned long lastKeepAliveTime = 0;

		unsigned int last_message_id = 0;
		std::deque<Message> receive_queue;
		std::deque<Message> send_queue;

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