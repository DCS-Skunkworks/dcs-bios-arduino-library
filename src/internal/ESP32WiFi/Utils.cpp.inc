#include "Utils.h"

namespace DcsBios {
	#ifdef DCSBIOS_ESP32_NEOPIXEL
	void beginNeopixel() {
		neopixel = Adafruit_NeoPixel(1, DCSBIOS_ESP32_NEOPIXEL, NEO_GRB + NEO_KHZ800);
		neopixel.begin();
		neopixel.setBrightness(DCSBIOS_ESP32_NEOPIXEL_BRIGHTNESS);
	}

	void setNeopixel(WiFiStatus state) {
		switch(state) {
			case OFFLINE:
				neopixel.setPixelColor(0, neopixel.Color(255, 0, 0));
				break;
			case ASSOCIATED:
				neopixel.setPixelColor(0, neopixel.Color(255, 255, 0));
				break;
			case CONNECTED:
				neopixel.setPixelColor(0, neopixel.Color(0, 255, 0));
				break;
			case RECEIVED:
				neopixel.setPixelColor(0, neopixel.Color(0, 0, 255));
				break;
		}

		neopixel.show();
	}
	#endif
	
    String base64_decode(String input) {
        // Use the global variable instead of a local one
        String decoded = "";
        int pad = 0;

        for (int i = 0; i < input.length(); i += 4) {
            int n = 0;

            for (int j = i; j < i + 4; ++j) {
                if (input[j] == '=') {
                    pad++;
                    n = n << 6;
                } else {
                    n = n << 6 | strchr(base64Chars, input[j]) - base64Chars;
                }
            }

            for (int j = 2; j >= 0; --j) {
                if (pad <= j) {
                    decoded += (char)((n >> (j * 8)) & 0xFF);
                }
            }
        }

        return decoded;
    }

    String base64_encode(String input) {
        // Use the global variable instead of a local one
        String result = "";
        uint8_t temp[3];
        uint8_t index;
        size_t length = input.length();

        for (size_t i = 0; i < length; i += 3) {
            size_t remaining = length - i;

            for (size_t j = 0; j < 3; ++j) {
                if (i + j < length) {
                    temp[j] = input[i + j];
                } else {
                    temp[j] = 0;
                }
            }

            index = (temp[0] & 0xFC) >> 2;
            result += base64Chars[index];

            index = ((temp[0] & 0x03) << 4) + ((temp[1] & 0xF0) >> 4);
            result += base64Chars[index];

            if (remaining > 1) {
                index = ((temp[1] & 0x0F) << 2) + ((temp[2] & 0xC0) >> 6);
                result += base64Chars[index];
            } else {
                result += '=';
            }

            if (remaining > 2) {
                index = temp[2] & 0x3F;
                result += base64Chars[index];
            } else {
                result += '=';
            }
        }

        return result;
    }
}