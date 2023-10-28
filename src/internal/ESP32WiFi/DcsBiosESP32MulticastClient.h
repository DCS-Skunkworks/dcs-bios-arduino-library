#include <Arduino.h>
#include <pb_decode.h>

namespace DcsBios {
    struct InputDataBuffer {
		uint8_t buffer[256];
		size_t size;
	};
    
    class DcsBiosESP32MulticastClient : public ClientInterface {
    private:
        TaskHandle_t taskHandle = NULL;
        QueueHandle_t messageQueue;
    public:
        void start();
        void stop();
        void loop();
        static void loopProxy(void* parameter);
        static bool decode_bytes(pb_istream_t *stream, const pb_field_t *field, void **arg);
        bool dequeue(OutputMessage& outputMessage, TickType_t ticksToWait);
    };
}