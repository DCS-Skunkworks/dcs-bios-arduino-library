#ifdef DCSBIOS_ESP32_WIFI

#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>

namespace DcsBios {
    class DcsBiosESP32MulticastClient : public ClientInterface {
    private:
        AsyncUDP udp;

        IPAddress master_ip = IPAddress(0, 0, 0, 0);
		unsigned int master_port = DCSBIOS_ESP32_WIFI_LOCAL_PORT;

        bool listening = false;
        ThreadSafeDeque<Message> &receive_queue;
    public:
        DcsBiosESP32MulticastClient(ThreadSafeDeque<Message> &receive_queue) : receive_queue(receive_queue) {}

        bool start() {
            if (udp.listenMulticast(DCSBIOS_ESP32_WIFI_MULTICAST_GROUP, DCSBIOS_ESP32_WIFI_LOCAL_PORT)) {
                Serial.println("Multicast listener started.");
                master_ip = WiFi.broadcastIP();
                listening = true;

                udp.onPacket([this](AsyncUDPPacket packet) {
                    Message message = Message(packet);
                    receive_queue.push_back(message);

                    // Set master IP if not set
                    if (master_ip == WiFi.broadcastIP()) {
                        master_ip = packet.remoteIP();

                        Serial.println("Master IP: " + master_ip.toString() + " Port: " + String(master_port));
                    }
                });

                return true;
            }

            Serial.println("Multicast listener failed to start.");

            return false;
        }

        void stop() {
            listening = false;
            master_ip = WiFi.broadcastIP();
        }

        void loop() {}

        // The concept of a connection doesn't exist in UDP, so the listening status is used.
        bool connected() {
            return listening;
        }

        bool can_send() {
            return master_ip != WiFi.broadcastIP();
        }

        // Service discovery for multicast is not required. Always return true
        bool discover() {
            return true;
        }

        bool send(Message &message, bool force = false) {
            Serial.println("Can send: " + String(can_send()) + " Force: " + String(force) + " Message: " + message.type);

            if (!can_send() && !force ) { return false; }

            std::vector<unsigned char> encoded(256);
            message.encodeToVector(encoded);

            size_t sent = udp.writeTo((uint8_t *)encoded.data(), encoded.size(), master_ip, master_port);

            // Serial.println("Sent " + String(sent) + " of " + encoded.size() + " bytes to " + master_ip.toString() + ":" + String(master_port));

            return sent == encoded.size();
        }
    };
}

#endif