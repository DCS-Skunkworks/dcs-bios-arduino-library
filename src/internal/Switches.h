#ifndef __DCSBIOS_SWITCHES_H
#define __DCSBIOS_SWITCHES_H

#include <math.h>
#include "Arduino.h"

namespace DcsBios {
	class Switch2Pos : PollingInput {
		private:
			const char* msg_;
			char pin_;
			char lastState_;
			char switchState_;
			bool reverse_;
			unsigned long debounceDelay_;
			unsigned long lastDebounceTime = 0;

			void resetState()
			{
				lastState_ = (lastState_==0)?-1:0;
			}
			void pollInput() {
				char state = digitalRead(pin_);
				if (reverse_) state = !state;
				if (state != lastState_) {
					lastDebounceTime = millis();
				}
				
				if ((millis() - lastDebounceTime) > debounceDelay_) {
					if (state != switchState_) {
					if (tryToSendDcsBiosMessage(msg_, state == HIGH ? "0" : "1")) {
							switchState_ = state;
						}
					}
				}

				lastState_ = state;
			}
		public:
			Switch2Pos(const char* msg, char pin, bool reverse = false, unsigned long debounceDelay = 50, unsigned long pollIntervalMs = POLL_EVERY_TIME) :
				PollingInput(pollIntervalMs)
			{ 
				msg_ = msg;
				pin_ = pin;
				pinMode(pin_, INPUT_PULLUP);
				switchState_ = digitalRead(pin_);
				lastState_ = switchState_;
				reverse_ = reverse;
				debounceDelay_ = debounceDelay;
			}
					
			void SetControl( const char* msg )
			{
				msg_ = msg;
			}
		};
	
	class Switch3Pos : PollingInput {
		private:
			const char* msg_;
			char pinA_;
			char pinB_;
			char lastState_;
			char steadyState_;
			unsigned long debounceDelay_;
			unsigned long lastDebounceTime = 0;

			char readState() {
				if (digitalRead(pinA_) == LOW) return 0;
				if (digitalRead(pinB_) == LOW) return 2;
				return 1;
			}
			void resetState()
			{
				lastState_ = (lastState_==0)?-1:0;
				steadyState_ = lastState_;
			}
			void pollInput() {
				char state = readState();
				if (state != lastState_) {
					lastDebounceTime = millis();
				}

				if ((millis() - lastDebounceTime) > debounceDelay_)
				{
					if (state != steadyState_) {
						if (state == 0)
						{
							if (tryToSendDcsBiosMessage(msg_, "0"))
								steadyState_ = state;
						}
						else if (state == 1)
						{
							if (tryToSendDcsBiosMessage(msg_, "1"))
								steadyState_ = state;
						}
						else if (state == 2)
						{
							if(tryToSendDcsBiosMessage(msg_, "2"))
								steadyState_ = state;
						}
					}
				}

				lastState_ = state;
			}
		public:
			Switch3Pos(const char* msg, char pinA, char pinB, unsigned long debounceDelay = 50, unsigned long pollIntervalMs = POLL_EVERY_TIME) :
				PollingInput(pollIntervalMs)
			{
				msg_ = msg;
				pinA_ = pinA;
				pinB_ = pinB;
				pinMode(pinA_, INPUT_PULLUP);
				pinMode(pinB_, INPUT_PULLUP);
				lastState_ = readState();
				steadyState_ = lastState_;
				debounceDelay_ = debounceDelay;
			}
			
			void SetControl( const char* msg )
			{
				msg_ = msg;
			}
	};

	class SwitchMultiPos : PollingInput {
		private:
			const char* msg_;
			const byte* pins_;
			char numberOfPins_;
			char lastState_;
			bool reverse_;
			char readState() {
				unsigned char i;
				for (i=0; i<numberOfPins_; i++) {
					if (digitalRead(pins_[i]) == LOW && reverse_ == false) return i;
					else if (digitalRead(pins_[i]) == HIGH && reverse_ == true) return i;
				}
				return lastState_;
			}
			void resetState()
			{
				lastState_ = (lastState_==0)?-1:0;
			}
			void pollInput() {
				char state = readState();
				if (state != lastState_) {
					char buf[7];
					utoa(state, buf, 10);
					if (tryToSendDcsBiosMessage(msg_, buf))
						lastState_ = state;
				}
			}
		public:
			SwitchMultiPos(const char* msg, const byte* pins, char numberOfPins, bool reverse = false, unsigned long pollIntervalMs = POLL_EVERY_TIME) :
				PollingInput(pollIntervalMs),
				lastState_(0)
			{
				msg_ = msg;
				pins_ = pins;
				reverse_ = reverse;
				numberOfPins_ = numberOfPins;
				unsigned char i;
				for (i=0; i<numberOfPins; i++) {
					pinMode(pins[i], INPUT_PULLUP);
				}
				lastState_ = readState();
			}
			
			void SetControl( const char* msg )
			{
				msg_ = msg;
			}
	};	
}

#endif	