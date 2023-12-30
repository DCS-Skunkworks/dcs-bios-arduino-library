#ifndef _DCSBIOS_ESP32_H_
#define _DCSBIOS_ESP32_H_
#ifdef DCSBIOS_ESP32_ID

#include <Arduino.h>
#include <WiFi.h>

#include "Utils.h"
#include "Utils.cpp.inc"

namespace DcsBios {
	ProtocolParser parser;

	class ESP32NetworkedSlave {
	public:
		void begin();
		void loop();
		void enqueue(const char* type, bool force = false);
		void enqueue(const char* type, String data, bool force = false);
	private:
		bool connect_phy();
		bool disconnect_phy();
		void set_client();

		// Swappable client depending on communication protocol
		ClientInterface* client;

		// Timeout and keepalive
		unsigned long lastReceivedTime = 0;
		unsigned long lastKeepAliveTime = 0;

		unsigned int last_message_id = 0;
		ThreadSafeDeque<Message> receive_queue;
		ThreadSafeDeque<Message> send_queue;

		// Performance Instrumentation
		#ifdef DCSBIOS_ESP32_INSTRUMENTATION
		unsigned long lastLoopTime = 0;
		unsigned long lastLoopDuration = 0;
		#endif
	};

	ESP32NetworkedSlave wifiSlave;
}

#endif // DCSBIOS_ESP32_WIFI
#endif // _DCSBIOS_ESP32_H_