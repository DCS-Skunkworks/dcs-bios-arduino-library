#ifdef DCSBIOS_ESP32_WIFI

#include <Arduino.h>
#include <AsyncUDP.h>
#include <ESPmDNS.h>

namespace DcsBios {
    class DcsBiosESP32UnicastClient : public ClientInterface {
    private:
        AsyncUDP udp;

        IPAddress master_ip;
        uint16_t master_port;

        ThreadSafeDeque<Message> &receive_queue;
        bool listening;
    public:
        DcsBiosESP32UnicastClient(ThreadSafeDeque<Message> &receive_queue) : receive_queue(receive_queue), listening(false), master_port(0) {}

        bool start() {
            if (udp.listen(DCSBIOS_ESP32_WIFI_LOCAL_PORT)) {
                listening = true;

                Serial.println("Unicast listener started.");

                udp.onPacket([this](AsyncUDPPacket packet) {
                    Message message = Message(packet);
                    receive_queue.push_back(message);

                    if (master_port == 0) {
                        master_ip = packet.remoteIP();
                        master_port = packet.remotePort();
                    }
                });

                return true;
            }

            Serial.println("Unicast listener failed to start.");
            return false;
        }

        void stop() {
            listening = false;
            udp.close();
        }

        void loop() {}

        bool connected() {
            return listening;
        }

        bool discover() {
            #ifdef DCSBIOS_ESP32_WIFI_MASTER_IP
            master_ip = DCSBIOS_ESP32_WIFI_MASTER_IP;
            master_port = DCSBIOS_ESP32_WIFI_MASTER_PORT;

            return true;
            #endif

            int n = MDNS.queryService("_dcs-bios", "_udp");

            // TODO: Give up after x tries and restart
            while (n < 0) {
                master_ip = MDNS.IP(0);
                master_port = MDNS.port(0);

                return true;
            }

            return false;
        }

        bool can_send() {
            return connected();
        }

        bool send(Message &message, bool force = false) {
            if (master_port == 0) return false;

            std::vector<unsigned char> encoded;
            message.encodeToVector(encoded);

            size_t sent = udp.writeTo((uint8_t *)encoded.data(), encoded.size(), master_ip, master_port);

            return sent == encoded.size();
        }
    };
}

#endif
