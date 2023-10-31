#ifdef DCSBIOS_ESP32_WIFI

#include <deque>
#include <Arduino.h>
#include <AsyncUDP.h>

namespace DcsBios {
    class DcsBiosESP32MulticastClient : public ClientInterface {
    private:
        AsyncUDP udp;

        IPAddress master_ip = IPAddress(0, 0, 0, 0);
		unsigned int master_port = 0;

        bool listening = false;
        std::deque<Message> &receive_queue;
    public:
        DcsBiosESP32MulticastClient(std::deque<Message> &receive_queue) : receive_queue(receive_queue) {}

        bool start() {
            if (udp.listenMulticast(DCSBIOS_ESP32_WIFI_MULTICAST_GROUP, DCSBIOS_ESP32_WIFI_LOCAL_PORT)) {
                Serial.println("Multicast listener started.");
                listening = true;

                udp.onPacket([this](AsyncUDPPacket packet) {
                    Message message = Message(packet);
                    receive_queue.push_back(message);

                    // Set master IP and port if not set
                    if (master_port == 0) {
                        master_ip = packet.remoteIP();
                        master_port = packet.remotePort();
                    }
                });

                return true;
            }

            Serial.println("Multicast listener failed to start.");

            return false;
        }

        void stop() {
            listening = false;
            udp.close();
            master_ip = IPAddress(0, 0, 0, 0);
            master_port = 0;
        }

        void loop() {}

        // The concept of a connection doesn't exist in UDP, so the listening status is used.
        bool connected() {
            return listening;
        }

        bool can_send() {
            return master_port != 0;
        }

        // Service discovery for multicast is not required. Always return true
        bool discover() {
            return true;
        }

        bool send(Message &message) {
            if (can_send()) { return false; }

            std::vector<unsigned char> encoded;
            message.encodeToVector(encoded);

            size_t sent = udp.writeTo((uint8_t *)encoded.data(), encoded.size(), master_ip, master_port);

            return sent == encoded.size();
        }
    };
}

#endif