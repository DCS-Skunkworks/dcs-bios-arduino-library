#include "Message.h"

namespace DcsBios {
    class ClientInterface {
    public:
        virtual bool start() = 0;
        virtual void stop() = 0;
        virtual void loop() = 0;

        // Connection status
        virtual bool connected() = 0;

        // Discover services, returns success or failure as bool
        virtual bool discover() = 0;

        virtual bool can_send() = 0;

        // Send Message
        virtual bool send(Message &message) = 0;
    private:
        IPAddress master_ip = IPAddress(0, 0, 0, 0);
		unsigned int master_port = 0;
    };
}