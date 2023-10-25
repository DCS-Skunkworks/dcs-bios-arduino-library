#ifndef _DCSBIOS_ESP32_UTILS_H_
#define _DCSBIOS_ESP32_UTILS_H_

#include <Arduino.h>

#ifdef DCSBIOS_ESP32_WIFI_NEOPIXEL
#ifndef DCSBIOS_ESP32_WIFI_NEOPIXEL_BRIGHTNESS
#define DCSBIOS_ESP32_WIFI_NEOPIXEL_BRIGHTNESS 128
#endif

#include <Adafruit_NeoPixel.h>
#endif

namespace DcsBios {
    // Status LED
    enum WiFiStatus {
        OFFLINE,
        ASSOCIATED,
        CONNECTED,
        RECEIVED
    };
    
    #ifdef DCSBIOS_ESP32_WIFI_NEOPIXEL
    Adafruit_NeoPixel neopixel;

    void beginNeopixel();
    inline void setNeopixel(WiFiStatus state);
    #else
    void beginNeopixel() {}
    inline void setNeopixel(WiFiStatus state) {}
    #endif
    
    const char base64Chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    inline String base64_decode(String input);
	inline String base64_encode(String input);
}

#endif