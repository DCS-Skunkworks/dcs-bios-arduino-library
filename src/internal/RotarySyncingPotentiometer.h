#ifndef __DCSBIOS_ROTARYPOTS_H
#define __DCSBIOS_ROTARYPOTS_H

#include <math.h>
#include "Arduino.h"
#include "PollingInput.h"

namespace DcsBios {
	int debugSyncVal = 0;

	template <unsigned long pollIntervalMs = POLL_EVERY_TIME, bool invert = false>
	class RotarySyncingPotentiometerEWMA : PollingInput, Int16Buffer, public ResettableInput {
		private:
			void resetState()
			{
				lastState_ = (lastState_==0)?-1:0;
			}
			void pollInput() {
				lastState_ = readState();
			}

			inline unsigned int readState()
			{
				return map(analogRead(pin_), invert?1023:0, invert?0:1023, 0, 65535);
			}

			const char* msg_;
			char pin_;
			unsigned int lastState_;

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

			// Reminder: If this works, consider making it more general.  Either a control that implement IBuffer but takes a new IControl interface and a callback.  Or heck if I go that far, I'm back to simply making it a callback though right?
			virtual void loop() {
				// If this syncs at all, I think I'll still need something to slow it down
				//if (hasUpdatedData())
				 {
					Serial.write("Physical:");Serial.println(lastState_);

					//unsigned int dcsData = getData();
					//Serial.write("SyncDCS:");Serial.print(dcsData);
					//int deltaDcsToPit = MapValue(lastState_ - dcsData);
					
					//Serial.write("Delta:");Serial.print(deltaDcsToPit);
					
					// Send the adjustment to DCS
					// char buff[5];
					// itoa(deltaDcsToPit, buff, 10);
					// sprintf(buff, "%+d", deltaDcsToPit);
					// tryToSendDcsBiosMessage(msg_, buff);
				}
			}

			// This will be a callback ingested later, but for now
			int MapValue(unsigned int controlPosition)
			{
				// Initial testing here is for hornet min height, with the input being a 
				// +/- 3200 rotary and the output being in the range 0-65535, so I'm GUESSING
				// I need a big k value.  Tune this later
				unsigned int result = 50 * controlPosition;

				if( result >= 10000)
					result = 9999;
				else if( result <= -10000)
					result = -9999;

				return result;
			}
	};
	typedef RotarySyncingPotentiometerEWMA<> RotarySyncingPotentiometer;
	typedef RotarySyncingPotentiometerEWMA<POLL_EVERY_TIME, true> InvertedRotarySyncingPotentiometer;
}

#endif
