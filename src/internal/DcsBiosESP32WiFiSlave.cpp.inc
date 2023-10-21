#ifdef DCSBIOS_ESP32_WIFI

namespace DcsBios {
	void STM32WiFiSlave::setStatus(State state) {
		#ifdef DCSBIOS_ESP32_WIFI_NEOPIXEL
		switch(state) {
			case OFFLINE:
				led.setPixelColor(0, led.Color(4, 0, 0));
				break;
			case ASSOCIATED:
				led.setPixelColor(0, led.Color(4, 4, 0));
				break;
			case CONNECTED:
				led.setPixelColor(0, led.Color(0, 4, 0));
				break;
			case RECEIVED:
				led.setPixelColor(0, led.Color(0, 0, 4));
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
		#endif

		setStatus(OFFLINE);
		Serial.begin(115200);
		
		WiFi.begin(ssid, password);
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

		Udp.begin(localPort);

		setStatus(ASSOCIATED);
	}

	void STM32WiFiSlave::loop() {
		if (!connected) {
			int n = MDNS.queryService("_dcs-bios", "_udp");

			if (n > 0) {
				master_ip = MDNS.IP(0);
				master_port = MDNS.port(0);
				
				DynamicJsonDocument jsonDoc(1024);
				jsonDoc["id"] = slaveID;
				
				String output;
				serializeJson(jsonDoc, output);
				Udp.beginPacket(master_ip, master_port);
				Udp.write((uint8_t *)output.c_str(), output.length());
				Udp.endPacket();
				
				connected = true;
				lastReceivedTime = millis(); // Update last received time when connected
				lastKeepAliveTime = millis();
				setStatus(CONNECTED);
			} else {
				Serial.println("No services found");
				delay(3000);
			}
		} else {
			int packetSize = Udp.parsePacket();
			if (packetSize) {
				lastReceivedTime = millis(); // Update last received time
				char incomingPacket[255];
				int len = Udp.read(incomingPacket, 255);
				incomingPacket[len] = '\0';
				
				DynamicJsonDocument incomingJson(1024);
				deserializeJson(incomingJson, incomingPacket);

				String message = incomingJson["message"].as<String>();

				if (message == "Hello, World") {
					setStatus(RECEIVED);
				} else {
					String decodedMessage = base64_decode(message); // Assuming base64_decode returns a String
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
				}
			} else {
				// Check for timeout
				if (millis() - lastReceivedTime > timeoutDuration) {
					connected = false;
					setStatus(ASSOCIATED);
					Serial.println("Connection timed out");
				}
			}

			// Check if it's time to send a keep-alive message
			if (connected && (millis() - lastKeepAliveTime >= keepAliveTimeout)) {
				DynamicJsonDocument keepAliveJson(256);
				keepAliveJson["keep-alive"] = slaveID;

				String keepAliveOutput;
				serializeJson(keepAliveJson, keepAliveOutput);
				
				Udp.beginPacket(master_ip, master_port);
				Udp.write((uint8_t *)keepAliveOutput.c_str(), keepAliveOutput.length());
				Udp.endPacket();

				lastKeepAliveTime = millis();
			}
		}
	}

	void STM32WiFiSlave::send(String message) {
		DynamicJsonDocument jsonDoc(1024);
		jsonDoc["message"] = base64_encode(message);
		
		String output;
		serializeJson(jsonDoc, output);
		Udp.beginPacket(master_ip, master_port);
		Udp.write((uint8_t *)output.c_str(), output.length());
		Udp.endPacket();
	}

	STM32WiFiSlave wifiSlave;

	void setup() {
		wifiSlave.begin();
	}

	void loop() {
		wifiSlave.loop();
		PollingInput::pollInputs();
		ExportStreamListener::loopAll();			
	}

	bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
		wifiSlave.send(msg + String(' ') + arg + String('\n'));
		
		DcsBios::PollingInput::setMessageSentOrQueued();
		return true;
	}

	void resetAllStates() {
		PollingInput::resetAllStates();
	}
}
#endif
