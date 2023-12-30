#ifndef _MESSAGES_H_
#define _MESSAGES_H_

#include <vector>
#include <Arduino.h>
#include <AsyncUDP.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "SlaveMessage.pb.h"

namespace DcsBios {
    class Message {
    public:
        bool force = false;

    private:
        SlaveMessage slave_message;
        
        static bool encode_to_string(pb_ostream_t *stream, const pb_field_t *field, void * const *arg) {
            const String *str = reinterpret_cast<const String*>(*arg);
            if (!pb_encode_tag_for_field(stream, field)) {
                return false;
            }
            return pb_encode_string(stream, (const pb_byte_t*)str->c_str(), str->length());
        }

        static bool encode_to_vector(pb_ostream_t *stream, const pb_field_t *field, void * const *arg) {
            const std::vector<unsigned char> *vec = reinterpret_cast<const std::vector<unsigned char>*>(*arg);
            if (!pb_encode_tag_for_field(stream, field)) {
                return false;
            }
            return pb_encode_string(stream, vec->data(), vec->size());
        }
        
        static bool decode_to_string(pb_istream_t *stream, const pb_field_t *field, void **arg) {
            String *resultString = (String *)(*arg);

            size_t bytes_left = stream->bytes_left;

            if (bytes_left > 0) {
                char *tempBuffer = new char[bytes_left + 1];

                if (!pb_read(stream, (pb_byte_t *)tempBuffer, bytes_left)) {
                    delete[] tempBuffer;
                    return false;
                }

                tempBuffer[bytes_left] = '\0';

                // Append the bytes to the Arduino String
                *resultString += String(tempBuffer);

                delete[] tempBuffer;
            }

            return true;
        }

        static bool decode_to_vector(pb_istream_t *stream, const pb_field_t *field, void **arg) {
            std::vector<unsigned char> *resultVector = (std::vector<unsigned char> *)(*arg);

            size_t bytes_left = stream->bytes_left;

            if (bytes_left > 0) {
                resultVector->resize(bytes_left);

                if (!pb_read(stream, resultVector->data(), bytes_left)) {
                    return false;
                }
            }

            return true;
        }

        void decodeFromStream(pb_istream_t *stream) {
            slave_message = SlaveMessage_init_zero;
            
            slave_message.type.funcs.decode = &decode_to_string;
            slave_message.type.arg = &type;
            slave_message.data.funcs.decode = &decode_to_vector;
            slave_message.data.arg = &data;

            slave_message.id.funcs.decode = &decode_to_string;
            slave_message.id.arg = &slave_details.id;
            slave_message.mac.funcs.decode = &decode_to_string;
            slave_message.mac.arg = &slave_details.mac;

            isValid = pb_decode(stream, SlaveMessage_fields, &slave_message);

            if (isValid) {
                seq = slave_message.seq;
                slave_details.rssi = slave_message.rssi;
                slave_details.free_heap = slave_message.free_heap;
                slave_details.loop_duration = slave_message.loop_duration;
                slave_details.cpu_freq = slave_message.cpu_freq;
                slave_details.flash_size = slave_message.flash_size;
            }
        }
    public:
        class Slave {
        public:
            String id;
            String mac = "";
            int32_t rssi = 0;
            uint32_t free_heap = 0;
            uint32_t loop_duration = 0;
            uint32_t cpu_freq = 0;
            uint32_t flash_size = 0;
        };

        bool isValid;

        Slave slave_details;
        String type;
        std::vector<unsigned char> data;
        uint32_t seq = 0;

        // For retries
        uint8_t retries = 0;
        unsigned long lastSentTime = 0;

        Message(const char* type, const String& data, uint32_t seq) : isValid(true) {
            this->type = type;
            this->data.assign(data.begin(), data.end());
            this->seq = seq;
        }

        Message(pb_istream_t *stream) : isValid(false) {
            decodeFromStream(stream);
        }

        Message(const std::vector<unsigned char>& vec) : isValid(false) {
            pb_istream_t stream = pb_istream_from_buffer(vec.data(), vec.size());

            decodeFromStream(&stream);
        }

        Message(AsyncUDPPacket& packet) : isValid(false) {            
            pb_istream_t stream = pb_istream_from_buffer(packet.data(), packet.length());

            decodeFromStream(&stream);
        }

        bool encodeToVector(std::vector<unsigned char> &vec) {
            pb_ostream_t stream = pb_ostream_from_buffer(vec.data(), vec.size());

            slave_message = SlaveMessage_init_zero;

            slave_message.type.funcs.encode = &encode_to_string;
            slave_message.type.arg = &type;
            slave_message.data.funcs.encode = &encode_to_vector;
            slave_message.data.arg = &data;

            slave_message.id.funcs.encode = &encode_to_string;
            slave_message.id.arg = &slave_details.id;
            
            if (slave_details.mac.length() > 0) {
                slave_message.mac.funcs.encode = &encode_to_string;
                slave_message.mac.arg = &slave_details.mac;
            }

            slave_message.seq = seq;
            slave_message.rssi = slave_details.rssi;
            slave_message.free_heap = slave_details.free_heap;
            slave_message.loop_duration = slave_details.loop_duration;
            slave_message.cpu_freq = slave_details.cpu_freq;
            slave_message.flash_size = slave_details.flash_size;

            if (!pb_encode(&stream, SlaveMessage_fields, &slave_message)) {
                Serial.print("Failed to encode message: ");
                Serial.println(stream.errmsg);

                return false;
            } else {
                vec.resize(stream.bytes_written);

                return true;
            }
        }
    };
}

#endif