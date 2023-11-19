#ifdef DCSBIOS_ESP32_WIFI
#ifdef DCSBIOS_ESP32_WIFI_TCP

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>

namespace DcsBios {
    class DcsBiosESP32TCPClient : public ClientInterface {
    private:
        AsyncClient tcpClient;

        IPAddress master_ip;
        uint16_t master_port;

        ThreadSafeDeque<Message> &receive_queue;

        uint32_t expected_length = 0;
        std::vector<unsigned char> buffer;

        bool sendWithLengthPrefix(const std::vector<unsigned char> &data) {
            uint16_t length = data.size();
            tcpClient.write((char*)&length, sizeof(length));

            return tcpClient.write((const char*)data.data(), data.size()) == data.size();
        }

        size_t bytesToIgnore = 0;
        
        void onClientData(void* arg, AsyncClient* client, void* data, size_t len) {
            const size_t MAX_MESSAGE_SIZE = 1024;

            // Discard data for oversized messages
            if (bytesToIgnore > 0) {
                size_t discardAmount = std::min(len, bytesToIgnore);
                bytesToIgnore -= discardAmount;
                data = ((char*)data) + discardAmount;
                len -= discardAmount;

                if (bytesToIgnore > 0 || len == 0) {
                    // Continue ignoring if more data needs to be discarded
                    return;
                }
                
                // Once all data to ignore is discarded, reset for the next message
                expected_length = 0;
            }

            if (expected_length == 0 && len >= sizeof(uint16_t)) {
                // Read the length prefix
                expected_length = *((uint16_t*)data);

                // Check if message exceeds maximum size
                if (expected_length > MAX_MESSAGE_SIZE) {
                    Serial.println("Message size " + String(expected_length) + " exceeds maximum. Ignoring message.");
                    bytesToIgnore = expected_length - sizeof(uint16_t); // Set the remaining bytes to ignore
                    expected_length = 0; // Reset expected length to ignore this message
                    
                    return;
                }

                buffer.clear();
                try {
                    buffer.reserve(expected_length);
                } catch (std::bad_alloc& ba) {
                    Serial.println("Failed to allocate buffer of size " + String(expected_length) + ", free heap: " + String(ESP.getFreeHeap()));
                    expected_length = 0;
                    ESP.restart();
                }

                // Remove the 2 bytes that were used for the length prefix
                data = ((char*)data) + sizeof(uint16_t);
                len -= sizeof(uint16_t);
            }

            if (expected_length > 0) {
                buffer.insert(buffer.end(), (unsigned char*)data, (unsigned char*)data + len);

                if (buffer.size() >= expected_length) {
                    Message message(buffer);
                    receive_queue.push_back(message);

                    // Reset for the next message
                    expected_length = 0;
                    buffer.clear();
                }
            }
        }

    public:
        DcsBiosESP32TCPClient(ThreadSafeDeque<Message> &receive_queue) : receive_queue(receive_queue), master_port(0) {}

        bool start() {
            Serial.println("Starting TCP client.");

            tcpClient.onData([](void* arg, AsyncClient* client, void* data, size_t len) {
                ((DcsBiosESP32TCPClient*)arg)->onClientData(arg, client, data, len);
            }, this);

            tcpClient.onConnect([](void* arg, AsyncClient* client) {
                Serial.println("TCP client connected.");
            }, this);

            tcpClient.onError([](void* arg, AsyncClient* client, int error) {
                Serial.print("TCP client error: ");
                Serial.println(error);
            }, this);

            if (tcpClient.connect(master_ip, master_port)) {
                Serial.println("TCP client started.");

                return true;
            }

            Serial.println("TCP client failed to start.");

            return false;
        }

        void stop() {
            tcpClient.close();
        }

        void loop() {}

        bool connected() {
            return tcpClient.connected();
        }

        bool can_send() {
            return connected();
        }

        bool discover() {
            const long timeout = 3000;
            const long retryDelay = 500;
            long startTime = millis();

            if (!MDNS.begin("DCS-BIOS ESP32 Slave")) {
                Serial.println("Error starting mDNS");

                return false;
            }

            while (millis() - startTime < timeout) {
                int n = MDNS.queryService("_dcs-bios", "_tcp");

                if (n > 0) {
                    master_ip = MDNS.IP(0);
                    master_port = MDNS.port(0);
                    
                    Serial.println("Service discovered!");

                    return true;
                } else {
                    Serial.println("Service not found, retrying...");
                    delay(retryDelay);
                }
            }

            Serial.println("Service discovery timed out.");

            MDNS.end();

            return false;
        }

        bool send(Message &message, bool force = false) {
            if (!tcpClient.connected()) return false;

            std::vector<unsigned char> encoded(256);
            message.encodeToVector(encoded);

            return sendWithLengthPrefix(encoded);
        }
    };
}

#endif
#endif