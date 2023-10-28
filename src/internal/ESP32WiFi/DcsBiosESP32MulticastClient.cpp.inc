#ifdef DCSBIOS_ESP32_WIFI

#include <Arduino.h>
#include <AsyncUDP.h>
#include <pb_decode.h>
#include "SlaveMessage.pb.h"

namespace DcsBios {
    void DcsBiosESP32MulticastClient::start() {
        messageQueue = xQueueCreate(10, sizeof(OutputMessage));

        xTaskCreatePinnedToCore(
            loopProxy,    /* Function to implement the task */
            "udpTask",    /* Name of the task */
            4096,         /* Stack size in words */
            this,         /* Task input parameter */
            1,            /* Priority of the task */
            &taskHandle,  /* Task handle. */
            1);           /* Core where the task should run */
    }

    void DcsBiosESP32MulticastClient::stop() {
        if (taskHandle != NULL) {
            vTaskDelete(taskHandle);
            taskHandle = NULL;
        }
        vQueueDelete(messageQueue);
    }

    bool DcsBiosESP32MulticastClient::dequeue(OutputMessage& outputMessage, TickType_t ticksToWait = portMAX_DELAY) {
        return xQueueReceive(messageQueue, &outputMessage, ticksToWait);
    }

    void DcsBiosESP32MulticastClient::loopProxy(void* parameter) {
        ((DcsBiosESP32MulticastClient*)parameter)->loop();
    }

    bool DcsBiosESP32MulticastClient::decode_bytes(pb_istream_t *stream, const pb_field_t *field, void **arg) {
		InputDataBuffer *dataBuffer = (InputDataBuffer *)(*arg);
		dataBuffer->size = stream->bytes_left; // Set the size
		
		if (dataBuffer->size > sizeof(dataBuffer->buffer)) {
			return false;
		}
		
		if (!pb_read(stream, dataBuffer->buffer, dataBuffer->size)) {
			return false;
		}
		
		return true;
	}

    void DcsBiosESP32MulticastClient::loop() {
        AsyncUDP udp;

        if (udp.listenMulticast(IPAddress(232, 0, 1, 3), 7779)) {
            udp.onPacket([this](AsyncUDPPacket packet) {
				// Set up the Nanopb stream
                
				pb_istream_t stream = pb_istream_from_buffer(packet.data(), packet.length());

				// Allocate a buffer to hold the decoded data.
				InputDataBuffer localDataBuffer = { {0}, 0 }; // Initialize buffer and size
				
				// Prepare for decoding
				SlaveMessage slaveMessage = SlaveMessage_init_zero;
				slaveMessage.data.funcs.decode = &this->decode_bytes;
				slaveMessage.data.arg = &localDataBuffer;

				// Decode the message
				if (pb_decode(&stream, SlaveMessage_fields, &slaveMessage)) {
					// Successfully decoded
					//Serial.println("Successfully decoded message");

					// Now the 'localDataBuffer' contains the data and its size
					/*for (size_t i = 0; i < localDataBuffer.size; ++i) {
						uint8_t byte = localDataBuffer.buffer[i];
						parser.processChar(byte);
					}*/

                    // Create an OutputMessage
                    OutputMessage outputMessage;
                    memcpy(outputMessage.data, localDataBuffer.buffer, localDataBuffer.size);
                    outputMessage.length = localDataBuffer.size;

                    // Send the OutputMessage to the queue
                    xQueueSend(messageQueue, &outputMessage, portMAX_DELAY);
				} else {
					// Decode failed
					//Serial.println("Decode failed");
				}
            });
        }

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Keep the task alive
        }
    }
}

#endif