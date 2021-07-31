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
					//Serial.write("Physical:");Serial.println(lastState_);
					unsigned int dcsData = getData();

					// Fake data to test my alignment maths
					//lastState_ = 0;
					//dcsData = 32768;

					//Serial.write("SyncDCS:");Serial.println(dcsData);
					int requiredAdjustment = MapValue(lastState_, dcsData);
					
					//Serial.write("lastState_:");Serial.println(lastState_);
					//Serial.write("dcsData:");Serial.println(dcsData);
					//Serial.write("requiredAdjustment:");Serial.println(requiredAdjustment);
					
					// Send the adjustment to DCS
					if( requiredAdjustment != 0 )
					{
						char buff[6];
						sprintf(buff, "%+d", requiredAdjustment);
						tryToSendDcsBiosMessage(msg_, buff);
					}
				}
			}

			// This will be a callback ingested later, but for now
			int MapValue(unsigned int physicalPosition, unsigned int dcsPosition)
			{
				// Initial testing here is for hornet min height, with the input being a 
				// +/- 3200 rotary and the output being in the range 0-65535, so I'm GUESSING
				// I need a big k value.  Tune this later.

				// For the pilot project: Potentiometer lastState range is 0 to 65535.
				// dcs data is 0 to 64355

				int deltaPitToDcs = (int)physicalPosition - (int)map(dcsPosition,0,64355,0,65535);
				int result = 1 * deltaPitToDcs;

				// Limit the range of adjustment
				if( result > 9600)
					result = 9600;
				else if( result < -9600)
					result = -9600;

				result = (result/3200)*3200;	// Snap to a multiple of 3200 in case DCS cares

				return result;
			}
	};
	typedef RotarySyncingPotentiometerEWMA<> RotarySyncingPotentiometer;
	typedef RotarySyncingPotentiometerEWMA<POLL_EVERY_TIME, true> InvertedRotarySyncingPotentiometer;
}

#endif
