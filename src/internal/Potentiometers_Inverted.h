#ifndef __DCSBIOS_POTS_INV_H
#define __DCSBIOS_POTS_INV_H

#include <math.h>
#include "Arduino.h"
#include "PollingInput.h"

namespace DcsBios {
	template < unsigned int hysteresis = 128, unsigned int ewma_divisor = 5>
	class PotentiometerInvEWMA : PollingInput {
		private:
			void pollInput() {
				unsigned int state = map(analogRead(pin_), 1023, 0, 0, 65535);
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
			const char* msg_;
			char pin_;
			unsigned int lastState_;
			float accumulator;
			
		public:
			PotentiometerInvEWMA(const char* msg, char pin, unsigned long pollIntervalMs = POLL_EVERY_TIME) :
				PollingInput(pollIntervalMs) {
				msg_ = msg;
				pin_ = pin;
				pinMode(pin_, INPUT);
				lastState_ = (float)map(analogRead(pin_), 0, 1023, 0, 65535);
			}
	};

	typedef PotentiometerInvEWMA<> InvertedPotentiometer;	
}

#endif
