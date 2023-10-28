#ifndef _MESSAGES_H_
#define _MESSAGES_H_

#include <Arduino.h>

namespace DcsBios {
    struct OutputMessage
    {
        uint8_t data[256];
        size_t length;
    };
}

#endif