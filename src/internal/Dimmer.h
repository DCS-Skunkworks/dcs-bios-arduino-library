#ifndef __DCSBIOS_DIMMER_H
#define __DCSBIOS_DIMMER_H

#include "Arduino.h"
#include "ExportStreamListener.h"

namespace DcsBios {

	class Dimmer : public Int16Buffer {
		private:
			void onDcsBiosFrameSync();
			char pin_;
			const char* msg_;
			int minOutput_;
			int maxOutput_;
		public:
			Dimmer(unsigned int address, char pin, int minOutput=0, int maxOutput=255) : Int16Buffer(address){
				pin_ = pin;
				minOutput_ = minOutput;
				maxOutput_ = maxOutput;
			}
			virtual void loop() {
				if (hasUpdatedData()) {
				analogWrite(pin_, 
					map(getData(), 0, 65535, minOutput_, maxOutput_));
				} 
			}
			
			void SetControl( const char* msg )
			{
				msg_ = msg;
			}
		};

}

#endif