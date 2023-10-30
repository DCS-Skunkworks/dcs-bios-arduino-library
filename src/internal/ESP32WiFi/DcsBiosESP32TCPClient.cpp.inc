#ifdef DCSBIOS_ESP32_WIFI

#include <deque>
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>

namespace DcsBios {
    class DcsBiosESP32TCPClient : public ClientInterface {
    private:
        AsyncClient tcpClient;

        IPAddress master_ip;
        uint16_t master_port;

        std::deque<Message> &receive_queue;
        bool listening;

        uint32_t expected_length = 0;
        std::vector<unsigned char> buffer;

        bool sendWithLengthPrefix(const std::vector<unsigned char> &data) {
            uint16_t length = data.size();
            tcpClient.write((char*)&length, sizeof(length));
            
            return tcpClient.write((const char*)data.data(), data.size()) == data.size();
        }
        
        void onClientData(void* arg, AsyncClient* client, void* data, size_t len) {
            if (expected_length == 0 && len >= sizeof(uint16_t)) {
                // Read the length prefix
                expected_length = *((uint16_t*)data);
                buffer.clear();
                buffer.reserve(expected_length);

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
        DcsBiosESP32TCPClient(std::deque<Message> &receive_queue) : receive_queue(receive_queue), listening(false), master_port(0) {}

        bool start() {
            tcpClient.onData([](void* arg, AsyncClient* client, void* data, size_t len) {
                ((DcsBiosESP32TCPClient*)arg)->onClientData(arg, client, data, len);
            }, this);

            if (tcpClient.connect(master_ip, master_port)) {
                listening = true;
                Serial.println("TCP client started.");
                return true;
            }

            Serial.println("TCP client failed to start.");
            return false;
        }

        void stop() {
            listening = false;
            tcpClient.close();
        }

        void loop() {}

        bool connected() {
            return listening;
        }

        bool discover() {
            int n = MDNS.queryService("_dcs-bios", "_tcp");

            if (n > 0) {
                master_ip = MDNS.IP(0);
                master_port = MDNS.port(0);
                return true;
            }

            return false;
        }

        bool send(Message &message) {
            if (!tcpClient.connected()) return false;

            std::vector<unsigned char> encoded;
            message.encodeToVector(encoded);

            return sendWithLengthPrefix(encoded);
        }
    };
}

#endif
