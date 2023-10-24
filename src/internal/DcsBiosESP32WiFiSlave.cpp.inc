#ifdef DCSBIOS_ESP32_WIFI

namespace DcsBios {
	void STM32WiFiSlave::setStatus(State state) {
		#ifdef DCSBIOS_ESP32_WIFI_NEOPIXEL
		switch(state) {
			case OFFLINE:
				led.setPixelColor(0, led.Color(255, 0, 0));
				break;
			case ASSOCIATED:
				led.setPixelColor(0, led.Color(255, 255, 0));
				break;
			case CONNECTED:
				led.setPixelColor(0, led.Color(0, 255, 0));
				break;
			case RECEIVED:
				led.setPixelColor(0, led.Color(0, 0, 255));
				break;
		}

		led.show();
		#endif
	}

	String STM32WiFiSlave::base64_decode(String input) {
		const char* chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
		String decoded = "";
		int pad = 0;

		for (int i = 0; i < input.length(); i += 4) {
			int n = 0;

			for (int j = i; j < i + 4; ++j) {
				if (input[j] == '=') {
					pad++;
					n = n << 6;
				} else {
					n = n << 6 | strchr(chars, input[j]) - chars;
				}
			}

			for (int j = 2; j >= 0; --j) {
				if (pad <= j) {
					decoded += (char)((n >> (j * 8)) & 0xFF);
				}
			}
		}

		return decoded;
	}

	String STM32WiFiSlave::base64_encode(String input) {
		const char base64Chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
		String result = "";
		uint8_t temp[3];
		uint8_t index;
		size_t length = input.length();

		for (size_t i = 0; i < length; i += 3) {
			size_t remaining = length - i;

			// Load up to three bytes of input into 'temp'
			for (size_t j = 0; j < 3; ++j) {
				if (i + j < length) {
					temp[j] = input[i + j];
				} else {
					temp[j] = 0;
				}
			}

			// Convert the three input bytes to four Base64 characters
			index = (temp[0] & 0xFC) >> 2;
			result += base64Chars[index];

			index = ((temp[0] & 0x03) << 4) + ((temp[1] & 0xF0) >> 4);
			result += base64Chars[index];

			if (remaining > 1) {
				index = ((temp[1] & 0x0F) << 2) + ((temp[2] & 0xC0) >> 6);
				result += base64Chars[index];
			} else {
				result += '=';
			}

			if (remaining > 2) {
				index = temp[2] & 0x3F;
				result += base64Chars[index];
			} else {
				result += '=';
			}
		}

		return result;
	}

	void STM32WiFiSlave::begin() {
		#ifdef DCSBIOS_ESP32_WIFI_NEOPIXEL
		led = Adafruit_NeoPixel(1, DCSBIOS_ESP32_WIFI_NEOPIXEL, NEO_GRB + NEO_KHZ800);
		led.begin();
		led.setBrightness(2);
		#endif

		Serial.begin(115200);
		
		WiFi.begin(ssid, password);
		setStatus(OFFLINE);
		
		while (WiFi.status() != WL_CONNECTED) {
			delay(3000);
			Serial.println("Connecting to WiFi...");
		}

		Serial.println("Connected to " + WiFi.SSID() + " with IP " + WiFi.localIP().toString());

		if (!MDNS.begin("esp32_consumer")) {
			Serial.println("Error starting mDNS");
			return;
		}

		Serial.println("mDNS responder started");

		setStatus(ASSOCIATED);
	}

	void STM32WiFiSlave::loop() {
		if (!client.connected()) {
			int n = MDNS.queryService("_dcs-bios", "_tcp");

			if (n > 0) {
				master_ip = MDNS.IP(0);
				master_port = MDNS.port(0);

				while (!client.connected()) {
					Serial.println("Connecting to " + master_ip.toString() + ":" + master_port);
					client.connect(master_ip, master_port, 1000);
				}

				Serial.println("Socket is connected");
				
				send("register");
				
				lastReceivedTime = millis(); // Update last received time when connected
				lastKeepAliveTime = millis();
				setStatus(CONNECTED);
			} else {
				Serial.println("No services found");
				delay(3000);
			}
		} else {
			int packetSize = client.available();

			if (packetSize > 0) {
				lastReceivedTime = millis(); // Update last received time
				uint8_t incomingPacket[packetSize];
				int len = client.read(incomingPacket, packetSize);
				incomingPacket[len] = '\0';
				
				DynamicJsonDocument incomingJson(1024);
				deserializeJson(incomingJson, incomingPacket);

				String type = incomingJson["type"].as<String>();
				String data = incomingJson["data"].as<String>();

				if (type == "message") {
					String decodedMessage = base64_decode(data); // Assuming base64_decode returns a String
					String printableMessage = "";

					for (unsigned int i = 0; i < decodedMessage.length(); ++i) {
						char c = decodedMessage[i];
						
						// Check if the character is a printable ASCII character
						if (c >= 32 && c <= 126) {
							printableMessage += c;
						} else {
							// Convert non-printable characters to hex representation
							printableMessage += "0x" + String((uint8_t)c, HEX);
						}
					}

					Serial.println("Received message: " + printableMessage);

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
					client.stop();
					setStatus(ASSOCIATED);
					Serial.println("Connection timed out");
				}
			}

			// Check if it's time to send a keep-alive message
			if (client.connected() && (millis() - lastKeepAliveTime >= keepAliveTimeout)) {
				send("check-in");

				lastKeepAliveTime = millis();
			}
		}
	}

	void STM32WiFiSlave::send(const char* type) {
		send(type, "");
	}

	void STM32WiFiSlave::send(const char* type, String data = "") {
		DynamicJsonDocument jsonDoc(1024);
		jsonDoc["type"] = type;
		jsonDoc["data"] = base64_encode(data);

		JsonObject slave = jsonDoc.createNestedObject("slave");
		slave["id"] = DCSBIOS_ESP32_WIFI_ID;
		slave["mac"] = WiFi.macAddress();
		slave["rssi"] = WiFi.RSSI();
		slave["free_heap"] = ESP.getFreeHeap();
		slave["loop_duration"] = lastLoopDuration;

		if (type == "register") {
			slave["cpu_freq"] = ESP.getCpuFreqMHz();
			slave["flash_size"] = ESP.getFlashChipSize();
		}
		
		String output;
		serializeJson(jsonDoc, output);

		client.write((uint8_t *)output.c_str(), output.length());		
	}

	STM32WiFiSlave wifiSlave;

	void setup() {
		wifiSlave.begin();
	}

	void loop() {
		wifiSlave.loop();
		PollingInput::pollInputs();
		ExportStreamListener::loopAll();

		wifiSlave.lastLoopDuration = micros() - wifiSlave.lastLoopTime;
		wifiSlave.lastLoopTime = micros();
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
