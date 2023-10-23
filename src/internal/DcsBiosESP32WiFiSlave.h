#ifndef _DCSBIOS_ESP32_WIFI_H_
#define DCSBIOS_ESP32_WIFI_H_
#ifdef DCSBIOS_ESP32_WIFI

#ifndef ARDUINO_ARCH_ESP32
  #error "This code is designed to run on ESP32 platform! Please check your build settings."
#endif

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#ifdef DCSBIOS_ESP32_WIFI_NEOPIXEL
#include <Adafruit_NeoPixel.h>
#endif

namespace DcsBios {
	ProtocolParser parser;

	class STM32WiFiSlave {
	public:
		unsigned long lastLoopTime = 0;
		unsigned long lastLoopDuration = 0;

		void begin();
		void loop();
		void send(const char* type);
		void send(const char* type, String data);

		// Neopixel status LED
		#ifdef DCSBIOS_ESP32_WIFI_NEOPIXEL
		Adafruit_NeoPixel led;
		#endif
	private:
		WiFiUDP Udp;
		IPAddress master_ip;
		unsigned int master_port;
		unsigned int localPort = 7779;

		bool connected = false;

		unsigned long lastReceivedTime = 0;
		const unsigned long timeoutDuration = 3000;
		unsigned long lastKeepAliveTime = 0;
		const unsigned long keepAliveTimeout = 1000;

		const char* ssid = DCSBIOS_ESP32_WIFI_SSID;
		const char* password = DCSBIOS_ESP32_WIFI_PASSWORD;
		const String slaveID = DCSBIOS_ESP32_WIFI_ID;

		enum State {
			OFFLINE,
			ASSOCIATED,
			CONNECTED,
			RECEIVED
		};

		void setStatus(State state);
		String base64_decode(String input);
		String base64_encode(String input);
	};
}

#endif // DCSBIOS_ESP32_WIFI
#endif // _DCSBIOS_ESP32_WIFI_H_