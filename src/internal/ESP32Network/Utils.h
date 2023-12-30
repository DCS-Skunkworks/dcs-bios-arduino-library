#ifndef _DCSBIOS_ESP32_UTILS_H_
#define _DCSBIOS_ESP32_UTILS_H_

#include <Arduino.h>
#include <deque>
#include <mutex>
#include <functional>

#ifdef DCSBIOS_ESP32_NEOPIXEL
#ifndef DCSBIOS_ESP32_NEOPIXEL_BRIGHTNESS
#define DCSBIOS_ESP32_NEOPIXEL_BRIGHTNESS 128
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
    
    #ifdef DCSBIOS_ESP32_NEOPIXEL
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

    template <typename T>
    class ThreadSafeDeque {
    private:
        std::deque<T> deque_;
        mutable std::mutex mutex_;
        
    public:
        void push_back(const T& value) {
            std::lock_guard<std::mutex> lock(mutex_);
            deque_.push_back(value);
        }
        
        bool pop_front(T& value) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (deque_.empty()) {
                return false;
            }
            value = deque_.front();
            deque_.pop_front();

            return true;
        }

        size_t size() const {
            std::lock_guard<std::mutex> lock(mutex_);
            
            return deque_.size();
        }

        // Function returns true if the element should be removed
        void for_each_remove_if(std::function<bool(T&)> func) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto it = deque_.begin(); it != deque_.end(); ) {
                if (func(*it)) {
                    it = deque_.erase(it);
                } else {
                    ++it;
                }
            }
        }
    };

}

#endif