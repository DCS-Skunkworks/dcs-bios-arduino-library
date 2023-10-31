#ifdef DCSBIOS_ESP32_WIFI

#include <pb_decode.h>
#include "SlaveMessage.pb.h"

namespace DcsBios {
	void ESP32WiFiSlave::begin() {
		beginNeopixel();
		Serial.begin(115200);
		setNeopixel(OFFLINE);
		
		// Connect to WiFi
		if (!connect_wifi()) {
			Serial.println("Restarting.");
			ESP.restart();
		}

		// Load a protocol client based on #defines.
		set_client();
	}

	bool ESP32WiFiSlave::connect_wifi() {
		WiFi.begin(DCSBIOS_ESP32_WIFI_SSID, DCSBIOS_ESP32_WIFI_PASSWORD);

		unsigned long start_time = millis();
		
		while (WiFi.status() != WL_CONNECTED) {
			Serial.println("Connecting to WiFi...");
			
			// Check association timeout, sometimes restarting the ESP32 is faster than waiting.
			if (millis() - start_time > DCSBIOS_ESP32_WIFI_ASSOCIATION_TIMEOUT) {
				Serial.println("WiFi association timed out.");

				return false;
			}

			delay(500);
		}

		setNeopixel(ASSOCIATED);
		Serial.println("Connected to " + WiFi.SSID() + " with IP " + WiFi.localIP().toString());

		return true;
	}

	bool ESP32WiFiSlave::disconnect_wifi() {
		setNeopixel(OFFLINE);

		return WiFi.disconnect(true, true);
	}

	void ESP32WiFiSlave::set_client() {
		#ifdef DCSBIOS_ESP32_WIFI_MULTICAST
		client = new DcsBiosESP32MulticastClient(receive_queue);
		#elif defined(DCSBIOS_ESP32_WIFI_TCP)
		client = new DcsBiosESP32TCPClient(receive_queue);
		#elif defined(DCSBIOS_ESP32_WIFI_UDP)
		client = new DcsBiosESP32UnicastClient(receive_queue);
		#else
		#error "Communication protocol not selected. Please #define either DCSBIOS_ESP32_WIFI_MULTICAST, DCSBIOS_ESP32_WIFI_UDP or DCSBIOS_ESP32_WIFI_TCP."
		#endif
	}

	void ESP32WiFiSlave::loop() {
		#ifdef DCSBIOS_ESP32_WIFI_INSTRUMENTATION
		lastLoopDuration = micros() - lastLoopTime;
		lastLoopTime = micros();
		#endif

		if (!client->connected()) { // If not connected, ask the client to discover the Nexus.
			if (client->discover()) {
				if (client->start()) {
					// Reset timeout when connected
					lastReceivedTime = millis();
					lastKeepAliveTime = millis();

					setNeopixel(CONNECTED);
				}
			} else {
				Serial.println("No services found.");

				delay(250);
			}
		} else { // If connected
			// Check if there are messages to process
			for (auto message = receive_queue.begin(); message != receive_queue.end();) {
				lastReceivedTime = millis();

				if (message->type == "message") { // This is a DCS-BIOS message that needs to be forwarded to the parser
					for (unsigned char c: message->data) {
						parser.processChar(c);
					}
				} else if (message->type == "check-in") { // Nexus is checking in, nothing to do. lastReceivedTime is automatically updated at the top of this function.
					Serial.println("Received check-in");
				} else if (message->type == "ack") { // Nexus has acknowledged a message, remove it from the send_queue.
					for (auto queued_packet = send_queue.begin(); queued_packet != send_queue.end();) {
						if (message->slave_details.id == DCSBIOS_ESP32_WIFI && queued_packet->seq == message->seq) {
							queued_packet = send_queue.erase(queued_packet);
							break;
						} else {
							++queued_packet;
						}
					}
				} else if ((message->type == "restart" && message->slave_details.id == DCSBIOS_ESP32_WIFI) || message->type == "restart-all") { // Restart command.
					Serial.println("Restarting on Nexus request.");
					ESP.restart();
				}

				message = receive_queue.erase(message);
			}

			// Check for time-out.
			if (millis() - lastReceivedTime > DCSBIOS_ESP32_WIFI_RECEIVE_TIMEOUT) {
				client->stop();
				setNeopixel(ASSOCIATED);
				Serial.println("Connection timed out, restarting.");
			}

			// Send check-in
			if (client->can_send() && (millis() - lastKeepAliveTime >= DCSBIOS_ESP32_WIFI_KEEPALIVE_PERIOD)) {
				enqueue("check-in");

				lastKeepAliveTime = millis();
			}
			
			// Send enqueued data
			for (auto message = send_queue.begin(); message != send_queue.end();) {
				if (message->retries > DCSBIOS_ESP32_WIFI_SEND_RETRIES) {
					Serial.println("Giving up on message " + String(message->seq));
					message = send_queue.erase(message);
				} else if (millis() - message->lastSentTime > DCSBIOS_ESP32_WIFI_SEND_RETRY_DELAY) {
					client->send(static_cast<Message&>(*message));
					message->retries++;
					message->lastSentTime = millis();
					++message;
				} else {
					++message;
				}
			}
		}
	}

	void ESP32WiFiSlave::enqueue(const char* type) {
		enqueue(type, "");
	}

	void ESP32WiFiSlave::enqueue(const char* type, String data) {
		Message message(type, data, ++last_message_id);
		send_queue.push_back(message);

		Serial.println("Enqueued " + String(type) + " with id " + String(message.seq));
	}

	// Standard DCS-BIOS functions
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
