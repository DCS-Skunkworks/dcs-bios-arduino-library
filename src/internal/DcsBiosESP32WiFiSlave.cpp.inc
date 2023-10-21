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
				IPAddress ip = MDNS.IP(0);
				int port = MDNS.port(0);
				
				DynamicJsonDocument jsonDoc(1024);
				jsonDoc["id"] = slaveID;
				
				String output;
				serializeJson(jsonDoc, output);
				Udp.beginPacket(ip, port);
				Udp.write((uint8_t *)output.c_str(), output.length());
				Udp.endPacket();
				
				connected = true;
				lastReceivedTime = millis(); // Update last received time when connected
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
		}
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
		// TODO: Implement sending
		//Serial.write(msg); Serial.write(' '); Serial.write(arg); Serial.write('\n');
		
		DcsBios::PollingInput::setMessageSentOrQueued();
		return true;
	}

	void resetAllStates() {
		PollingInput::resetAllStates();
	}
}
#endif
