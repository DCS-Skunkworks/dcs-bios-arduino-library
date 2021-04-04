#ifndef __DCSBIOS_POTS_H
#define __DCSBIOS_POTS_H

#include <math.h>
#include "Arduino.h"
#include "PollingInput.h"

namespace DcsBios {

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, unsigned int hysteresis = 128, unsigned int ewma_divisor = 5, bool invert = false>
	class PotentiometerEWMA : PollingInput, public ResettableInput {
		private:
			void resetState()
			{
				lastState_ = (lastState_==0)?-1:0;
			}
			void pollInput() {
				unsigned int state; 
				if (reverse_)
					state = map(analogRead(pin_), 0, 1023, 65535, 0);
				else
					state = map(analogRead(pin_), 0, 1023, 0, 65535);
				accumulator += ((float)state - accumulator) / (float)ewma_divisor;
				state = (unsigned int)accumulator;
				
				if (((lastState_ > state && (lastState_ - state > hysteresis)))
				|| ((state > lastState_) && (state - lastState_ > hysteresis))
				|| ((state > (65535 - hysteresis) && state > lastState_))
				|| ((state < hysteresis && state < lastState_))
				) {
					char buf[6];
					utoa(state, buf, 10);
					if (tryToSendDcsBiosMessage(msg_, buf))
						lastState_ = state;
				}
			}

			inline unsigned int readState()
			{
				return map(analogRead(pin_), invert?1023:0, invert?0:1023, 0, 65535);
			}

			const char* msg_;
			char pin_;
			unsigned int lastState_;
			float accumulator;
			bool reverse_;
			
		public:
			PotentiometerEWMA(const char* msg, char pin, bool reverse = false) :
				PollingInput(pollIntervalMs) {
				msg_ = msg;
				pin_ = pin;
				reverse_ = reverse;
				pinMode(pin_, INPUT);
				if (reverse_)
					lastState_ = map(analogRead(pin_), 0, 1023, 65535, 0);
				else
					lastState_ = map(analogRead(pin_), 0, 1023, 0, 65535);
			}

			void SetControl( const char* msg )
			{
				msg_ = msg;
			}
        
			void resetThisState()
			{
				this->resetState();
			}
	};
	typedef PotentiometerEWMA<> Potentiometer;
	typedef PotentiometerEWMA<POLL_EVERY_TIME, 128, 5, true> InvertedPotentiometer;
}

#endif
