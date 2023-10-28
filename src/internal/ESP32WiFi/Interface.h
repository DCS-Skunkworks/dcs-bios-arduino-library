#include "Messages.h"

namespace DcsBios {
    class ClientInterface {
    public:
        virtual void start() = 0;
        virtual void stop() = 0;
        virtual void loop() = 0;
        virtual bool dequeue(OutputMessage& outputMessage, TickType_t ticksToWait) = 0;
    };
}