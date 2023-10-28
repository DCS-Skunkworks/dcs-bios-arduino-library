#ifdef DCSBIOS_ESP32_WIFI

#include <pb_decode.h>
#include "SlaveMessage.pb.h"

namespace DcsBios {
	void ESP32WiFiSlave::begin() {
		beginNeopixel();
		Serial.begin(115200);
		WiFi.begin(ssid, password);
		setNeopixel(OFFLINE);
		
		while (WiFi.status() != WL_CONNECTED) {
			delay(500);
			Serial.println("Connecting to WiFi...");
		}

		Serial.println("Connected to " + WiFi.SSID() + " with IP " + WiFi.localIP().toString());

		multicast_client = new DcsBiosESP32MulticastClient();
		multicast_client->start();

		#ifndef DCSBIOS_ESP32_WIFI_MULTICAST
		if (!MDNS.begin("esp32_consumer")) {
			Serial.println("Error starting mDNS");
			return;
		}

		Serial.println("mDNS started");
		#endif
		
		setNeopixel(ASSOCIATED);
	}

	bool ESP32WiFiSlave::connected() {
		#ifdef DCSBIOS_ESP32_WIFI_TCP
		return client.connected();
		#else
		return udp_ready;
		#endif
	}

	void ESP32WiFiSlave::loop() {
		OutputMessage receivedMessage;

		// Attempt to dequeue a message
		if (multicast_client->dequeue(receivedMessage, 0)) {
			// Successfully dequeued a message, process it
			Serial.println("Successfully dequeued a message");

			// Assuming data is a String field in OutputMessage
			for (unsigned int i = 0; i < receivedMessage.length; ++i) {
				char byte = receivedMessage.data[i];
				parser.processChar(byte);
			}
		}

		return;

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
				size_t len = client.read(incomingPacket, packetSize);

				// Set up the Nanopb stream
				pb_istream_t stream = pb_istream_from_buffer(incomingPacket, len);

				// Allocate a buffer to hold the decoded data.
				DataBuffer localDataBuffer = { {0}, 0 }; // Initialize buffer and size
				
				// Prepare for decoding
				SlaveMessage slaveMessage = SlaveMessage_init_zero;
				slaveMessage.data.funcs.decode = &decode_bytes;
				slaveMessage.data.arg = &localDataBuffer;

				// Decode the message
				if (pb_decode(&stream, SlaveMessage_fields, &slaveMessage)) {
					// Successfully decoded
					Serial.println("Successfully decoded message");

					// Now the 'localDataBuffer' contains the data and its size
					for (size_t i = 0; i < localDataBuffer.size; ++i) {
						uint8_t byte = localDataBuffer.buffer[i];
						parser.processChar(byte);
					}
				} else {
					// Decode failed
					Serial.println("Decode failed");
				}
				
				/*DynamicJsonDocument incomingJson(packetSize*2);
				DeserializationError error = deserializeJson(incomingJson, incomingPacket);

				if (error) {
					Serial.print(F("deserializeJson() failed: "));
					Serial.println(error.c_str());
				}

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
				} else if (type == "ack") {
					String id = incomingJson["id"].as<String>();
					unsigned int seq = incomingJson["seq"].as<unsigned int>();

					if (id == DCSBIOS_ESP32_WIFI) {
						for (auto message = messages.begin(); message != messages.end();) {
							if (message->id == seq) {
								// Serial.println("Received ack for message " + String(seq) + " after " + String(millis() - message->lastSentTime) + "ms");
								message = messages.erase(message);
								//Serial.println("Removed message with seq: " + String(seq));
								break;  // Exit the loop as the message has been found and removed
							} else {
								++message;
							}
						}
					}
				} else if (type == "restart") {
					String id = incomingJson["id"].as<String>();
					
					if (id == DCSBIOS_ESP32_WIFI) {
						Serial.println("Kthxbai!");
						ESP.restart();
					}
				} else if (type == "restart-all") {
					Serial.println("Kthxbai!");
					ESP.restart();
				} else {
					Serial.println("Unknown message type: " + type);
				}*/
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

			// Send enqueued messages
			for (auto message = messages.begin(); message != messages.end(); /* empty */) {
				if (message->retries > DCSBIOS_ESP32_WIFI_MAX_RETRIES) {
					message = messages.erase(message);
					// Serial.println("Giving up on message " + String(message->id));
				} else if (millis() - message->lastSentTime > 250) {
					// Serial.println("Sending message with id " + String(message->id) + " and type " + String(message->type) + " retry: " + String(message->retries));
					send(message->type, message->data, message->id);
					message->retries++;
					message->lastSentTime = millis();
					++message;
				} else {
					++message;
				}
			}
		}
	}

	// Callback function to handle the 'data' field
	bool ESP32WiFiSlave::decode_bytes(pb_istream_t *stream, const pb_field_t *field, void **arg) {
		DataBuffer *dataBuffer = (DataBuffer *)(*arg);
		dataBuffer->size = stream->bytes_left; // Set the size
		
		if (dataBuffer->size > sizeof(dataBuffer->buffer)) {
			return false;
		}
		
		if (!pb_read(stream, dataBuffer->buffer, dataBuffer->size)) {
			return false;
		}
		
		return true;
	}

	void ESP32WiFiSlave::enqueue(const char* type) {
		enqueue(type, "");
	}

	void ESP32WiFiSlave::enqueue(const char* type, String data) {
		Message message;
		message.id = last_message_id++;
		message.type = type;
		message.data = data;
		messages.push_back(message);
		Serial.println("Enqueued " + String(type) + " with id " + String(message.id));
	}

	void ESP32WiFiSlave::send(const char* type) {
		send(type, "", 0);
	}

	void ESP32WiFiSlave::send(const char* type, String data, unsigned int seq) {
		if (master_port == 0) {
			return;
		}

		DynamicJsonDocument jsonDoc(1024);
		jsonDoc["type"] = type;
		jsonDoc["data"] = base64_encode(data);
		jsonDoc["seq"] = seq;

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
		wifiSlave.enqueue("message", data);
		
		DcsBios::PollingInput::setMessageSentOrQueued();
		return true;
	}

	void resetAllStates() {
		PollingInput::resetAllStates();
	}
}
#endif
