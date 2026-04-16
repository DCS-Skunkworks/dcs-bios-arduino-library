#ifndef __DCSBIOS_POTS_H
#define __DCSBIOS_POTS_H

#include <math.h>
#include "Arduino.h"
#include "PollingInput.h"

namespace DcsBios {

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, unsigned int hysteresis = 128, unsigned int ewma_divisor = 5>
	class PotentiometerEWMA : PollingInput, public ResettableInput {
		private:
			void resetState()
			{
				lastState_ = (lastState_==0)?-1:0;
			}
			void pollInput() {
				unsigned int state;
				unsigned int value = (unsigned int)analogRead(pin_);
				if (value < input_min_) value = input_min_;
				else if (value > input_max_) value = input_max_;
				if (reverse_)
					state = map(value, input_min_, input_max_, 65535, 0);
				else
					state = map(value, input_min_, input_max_, 0, 65535);

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
			bool reverse_;
			unsigned int input_min_;
			unsigned int input_max_;
			
		public:
			PotentiometerEWMA(const char* msg, char pin, bool reverse = false, unsigned int input_min = 0, unsigned int input_max = 1023) :
				PollingInput(pollIntervalMs) {
				msg_ = msg;
				pin_ = pin;
				reverse_ = reverse;
				input_min_ = input_min;
				input_max_ = input_max;

				pinMode(pin_, INPUT);
				unsigned int value = (unsigned int)analogRead(pin_);
				if (value < input_min_) value = input_min_;
				else if (value > input_max_) value = input_max_;
				if (reverse_)
					lastState_ = map(value, input_min_, input_max_, 65535, 0);
				else
					lastState_ = map(value, input_min_, input_max_, 0, 65535);
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
}

#endif
