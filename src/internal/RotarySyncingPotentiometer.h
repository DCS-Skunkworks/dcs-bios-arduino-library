#ifndef __DCSBIOS_ROTARYPOTS_H
#define __DCSBIOS_ROTARYPOTS_H

#include <math.h>
#include "Arduino.h"
#include "PollingInput.h"

namespace DcsBios {
	unsigned int debugSyncVal = 0;

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
				//return 25000;
				return map(analogRead(pin_), invert?1023:0, invert?0:1023, 0, 65535);
			}

			const char* msg_;
			char pin_;
			unsigned int lastState_;

			unsigned int mask;
			unsigned char shift;

			unsigned long lastSendTime;
			
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
				lastSendTime = millis();
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
				//return debugSyncVal;
				return ((this->Int16Buffer::getData()) & mask) >> shift;
			}

			// Reminder: If this works, consider making it more general.  Either a control that implement IBuffer but takes a new IControl interface and a callback.  Or heck if I go that far, I'm back to simply making it a callback though right?
			virtual void loop() {
				// If this syncs at all, I think I'll still need something to slow it down
				// TODO: updated data or was a delta last time?
				//if (hasUpdatedData())
				 {
					//Serial.write("Physical:");Serial.println(lastState_);
					unsigned int dcsData = getData();

					//Serial.write("SyncDCS:");Serial.println(dcsData);
					int requiredAdjustment = MapValue(lastState_, dcsData);
					
					//Serial.println("**************");
					//Serial.write("lastState_:");Serial.println(lastState_);
					//Serial.write("dcsData:");Serial.println(dcsData);
					//Serial.write("requiredAdjustment:");Serial.println(requiredAdjustment);
					
// One of the two values is not what I think it is.  I watched it consistently moving to 1.2 on the guage at 1 turn per second (approx), and when he cranked down in DCS, it moved back to the same spot).
// However when he turned right, it correct back down but to around 1.8, then fell off the rails.

					// Send the adjustment to DCS
					if( requiredAdjustment != 0 )
					{
						if( millis() - lastSendTime > 200)
						{
							char buff[6];
							sprintf(buff, "%+d", requiredAdjustment);
							tryToSendDcsBiosMessage(msg_, buff);
							lastSendTime = millis();
						}
					}
				}
			}

			// This will be a callback ingested later, but for now
			static int MapValue(unsigned int physicalPosition, unsigned int dcsPosition)
			{
				// Initial testing here is for hornet min height, with the input being a 
				// +/- 3200 rotary and the output being in the range 0-65535, so I'm GUESSING
				// I need a big k value.  Tune this later.

				// For the pilot project: Potentiometer lastState range is 0 to 65535.
				// dcs data is 0 to 64355
				unsigned int a = physicalPosition;
				unsigned int b = map(dcsPosition,0,64355,0,65535);

				// TODO: Must be a smarter way to handle this 16 bit subtraction!?
				unsigned int delta;
				if( a >= b)
					delta = a - b;
				else
					delta = b - a;
				
				if( delta > 9999 )
					delta = 9999;

				if( a >= b )
					return (int)delta;
				else
					return -1*(int)delta;
			}
	};
	typedef RotarySyncingPotentiometerEWMA<> RotarySyncingPotentiometer;
	typedef RotarySyncingPotentiometerEWMA<POLL_EVERY_TIME, true> InvertedRotarySyncingPotentiometer;
}

#endif
