#ifndef __DCSBIOS_ROTARYPOTS_H
#define __DCSBIOS_ROTARYPOTS_H

#include <math.h>
#include "Arduino.h"
#include "PollingInput.h"

namespace DcsBios {

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, unsigned int hysteresis = 128, unsigned int ewma_divisor = 5, bool invert = false>
	class RotarySyncingPotentiometerEWMA : PollingInput, Int16Buffer, public ResettableInput {
		private:
			void resetState()
			{
				lastState_ = (lastState_==0)?-1:0;
			}
			void pollInput() {
				unsigned int state = readState();
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

			unsigned int mask;
			unsigned char shift;
			
		public:
			RotarySyncingPotentiometerEWMA(const char* msg, char pin,
				unsigned int syncToAddress, unsigned int syncToMask, unsigned char syncToShift) :
				PollingInput(pollIntervalMs), Int16Buffer(syncToAddress) {
				msg_ = msg;
				pin_ = pin;
				pinMode(pin_, INPUT);
				lastState_ = (float)readState();

				this->mask = syncToMask;
				this->shift = syncToShift;
			}

			void SetControl( const char* msg )
			{
				msg_ = msg;
			}
        
			void resetThisState()
			{
				this->resetState();
			}

			unsigned int getData() {
				return ((this->Int16Buffer::getData()) & mask) >> shift;
			}
			// Reminder: If this doesn't work, I think I want to make a new SyncControls(IControl, IBuffer, converterCallback) which would be more flexible, but need everything to fit IControl or IBuffer... better
			virtual void loop() {
				if (hasUpdatedData()) {
					unsigned int dcsData = getData();
					int deltaDcsToPit = MapValue(dcsData - lastState_);
					
					if( deltaDcsToPit >= 10000)
						deltaDcsToPit = 9999;
					else if( deltaDcsToPit <= -10000)
						deltaDcsToPit = -9999;

					char buff[5];
					itoa(deltaDcsToPit, buff, 10);
					sprintf(buff, "%+d", deltaDcsToPit);

					tryToSendDcsBiosMessage(msg_, buff);
				}
			}

			// This will be a callback ingested later, but for now
			int MapValue(unsigned int controlPosition)
			{
				// controlPosition ranges from 0 to 65535
				// DCS Altimeter ranges from -something, 0=2992 to +something

				int result = map((int)controlPosition, 0, 65535, -524288, 524288);
				return result;
			}
	};
	typedef RotarySyncingPotentiometerEWMA<> RotarySyncingPotentiometer;
	typedef RotarySyncingPotentiometerEWMA<POLL_EVERY_TIME, 128, 5, true> InvertedRotarySyncingPotentiometer;
}

#endif
