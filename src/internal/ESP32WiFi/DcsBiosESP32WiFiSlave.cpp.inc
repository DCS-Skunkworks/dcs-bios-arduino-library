#ifdef DCSBIOS_ESP32_WIFI

namespace DcsBios {
	void STM32WiFiSlave::begin() {
		beginNeopixel();
		Serial.begin(115200);
		WiFi.begin(ssid, password);
		setNeopixel(OFFLINE);
		
		while (WiFi.status() != WL_CONNECTED) {
			delay(100);
			Serial.println("Connecting to WiFi...");
		}

		Serial.println("Connected to " + WiFi.SSID() + " with IP " + WiFi.localIP().toString());

		#ifndef DCSBIOS_ESP32_WIFI_MULTICAST
		if (!MDNS.begin("esp32_consumer")) {
			Serial.println("Error starting mDNS");
			return;
		}

		Serial.println("mDNS started");
		#endif
		
		setNeopixel(ASSOCIATED);
	}

	bool STM32WiFiSlave::connected() {
		#ifdef DCSBIOS_ESP32_WIFI_TCP
		return client.connected();
		#else
		return udp_ready;
		#endif
	}

	void STM32WiFiSlave::loop() {
		#ifdef DCSBIOS_ESP32_WIFI_INSTRUMENTATION
		lastLoopDuration = micros() - lastLoopTime;
		lastLoopTime = micros();
		#endif

		if (!connected()) {
			#ifdef DCSBIOS_ESP32_WIFI_MULTICAST
			int n = 1; // No discovery required in multicast mode
            #else
            int n = MDNS.queryService(DCSBIOS_ESP32_WIFI_SERVICE);
            #endif

			if (n > 0) {
				#ifndef DCSBIOS_ESP32_WIFI_MULTICAST
				master_ip = MDNS.IP(0);
				master_port = MDNS.port(0);
				#endif

				#ifdef DCSBIOS_ESP32_WIFI_TCP
				int tries = 0;
				while (!connected() && tries < 5) {
					Serial.println("Connecting to " + master_ip.toString() + ":" + master_port);
					client.connect(master_ip, master_port, 100);
					tries++;
				}
				Serial.println("Socket is connected");
				send("register");
				#elif defined(DCSBIOS_ESP32_WIFI_UDP)
				client.begin(DCSBIOS_ESP32_WIFI_LOCAL_PORT);
				udp_ready = true;
				send("register");
				#else
				if (client.beginMulticast(DCSBIOS_ESP32_WIFI_MULTICAST_GROUP, DCSBIOS_ESP32_WIFI_LOCAL_PORT)) {
					Serial.println("UDP Multicast listener started on group " + DCSBIOS_ESP32_WIFI_MULTICAST_GROUP.toString() + " and port " + String(DCSBIOS_ESP32_WIFI_LOCAL_PORT));
					udp_ready = true;
				} else {
					Serial.println("UDP Multicast listener failed to start");
					udp_ready = false;
				}
				#endif

				lastReceivedTime = millis(); // Update last received time when connected
				lastKeepAliveTime = millis();
				setNeopixel(CONNECTED);
			} else {
				Serial.println("No services found");
				delay(100);
			}
		} else {
			#ifdef DCSBIOS_ESP32_WIFI_TCP
			// TCP mode, only check for incoming data
            int packetSize = client.available();
            #else
			// UDP mode, check for incoming data and set master IP and port if required
            int packetSize = client.parsePacket();

			if (master_port == 0 && packetSize > 0) {
				master_ip = client.remoteIP();
				master_port = client.remotePort();

				#ifdef DCSBIOS_ESP32_WIFI_MULTICAST
				// Delayed registration in multicast mode, discovery is based on the first incoming packet
				send("register");
				#endif
			}
            #endif

			if (packetSize > 0) {
				lastReceivedTime = millis();
				uint8_t incomingPacket[packetSize];
				int len = client.read(incomingPacket, packetSize);
				
				DynamicJsonDocument incomingJson(packetSize);
				deserializeJson(incomingJson, incomingPacket);

				String type = incomingJson["type"].as<String>();
				String data = incomingJson["data"].as<String>();

				if (type == "message") {
					String decodedMessage = base64_decode(data);

					for (unsigned int i = 0; i < decodedMessage.length(); ++i) {
						char byte = decodedMessage[i];
						parser.processChar(byte);
					}
				} else if (type == "check-in") {
					Serial.println("Received check-in");
				}
			} else {
				// Check for timeout
				if (millis() - lastReceivedTime > timeoutDuration) {
					#ifdef DCSBIOS_ESP32_WIFI_TCP
					client.stop();
					#else
					udp_ready = false;
					#endif
					setNeopixel(ASSOCIATED);
					Serial.println("Connection timed out");
				}
			}

			// Check if it's time to send a keep-alive message
			if (connected() && (millis() - lastKeepAliveTime >= keepAliveTimeout)) {
				send("check-in");

				lastKeepAliveTime = millis();
			}
		}
	}

	void STM32WiFiSlave::send(const char* type) {
		send(type, "");
	}

	void STM32WiFiSlave::send(const char* type, String data) {
		if (master_port == 0) {
			return;
		}

		DynamicJsonDocument jsonDoc(1024);
		jsonDoc["type"] = type;
		jsonDoc["data"] = base64_encode(data);

		JsonObject slave = jsonDoc.createNestedObject("slave");
		slave["id"] = DCSBIOS_ESP32_WIFI;
		slave["mac"] = WiFi.macAddress();
		slave["rssi"] = WiFi.RSSI();
		slave["free_heap"] = ESP.getFreeHeap();
		#ifdef DCSBIOS_ESP32_WIFI_INSTRUMENTATION
		slave["loop_duration"] = lastLoopDuration;
		#endif

		if (type == "register") {
			slave["cpu_freq"] = ESP.getCpuFreqMHz();
			slave["flash_size"] = ESP.getFlashChipSize();
		}
		
		String output;
		serializeJson(jsonDoc, output);

		#ifndef DCSBIOS_ESP32_WIFI_TCP
		client.beginPacket(master_ip, master_port);
		#endif

		client.write((uint8_t *)output.c_str(), output.length());
		
		#ifndef DCSBIOS_ESP32_WIFI_TCP
		client.endPacket();
		#endif
	}

	

	void setup() {
		wifiSlave.begin();
	}

	void loop() {
		wifiSlave.loop();
		PollingInput::pollInputs();
		ExportStreamListener::loopAll();
	}

	bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
		String data = msg + String(' ') + arg + String('\n');
		wifiSlave.send("message", data);
		
		DcsBios::PollingInput::setMessageSentOrQueued();
		return true;
	}

	void resetAllStates() {
		PollingInput::resetAllStates();
	}
}
#endif
