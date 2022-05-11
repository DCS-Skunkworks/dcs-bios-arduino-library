#ifndef __DCSBIOS_GRAYCODENCODERS_H
#define __DCSBIOS_GRAYCODENCODERS_H

#include "Arduino.h"
#include "PollingInput.h"

static char GrayCodeToBinary(char grayCode)
{
	switch(grayCode)
	{
		case 0: return 0;
		case 1: return 1;
		case 2: return 3;
		case 3: return 2;
		case 4: return 7;
		case 5: return 6;
		case 6: return 4;
		case 7: return 5;
		case 8: return 15;
		case 9: return 14;
		case 10: return 12;
		case 11: return 13;
		case 12: return 8;
		case 13: return 9;
		case 14: return 11;
		case 15: return 10;
		default: return 0;
	}
}
namespace DcsBios {
	template <unsigned long pollIntervalMs = POLL_EVERY_TIME>
	class GrayCodeEncoderT : PollingInput, public ResettableInput {
	private:
		const char* msg_;
		const char* decArg_;
		const char* incArg_;
		char pin0_;
		char pin1_;
		char pin2_;
		char pin3_;
		char lastState_;
		char readState() {
			return 
			(digitalRead(pin3_) << 3) | 
			(digitalRead(pin2_) << 2) | 
			(digitalRead(pin1_) << 1) | 
			digitalRead(pin0_);
		}
		void resetState()
		{
			lastState_ = (lastState_==0)?-1:0;
		}

		void pollInput() {
			char state = readState();
			
			if( state == lastState_ )
				return;

			char currentBinary = GrayCodeToBinary(state);
			char lastBinary = GrayCodeToBinary(lastState_);

			char delta;
			if( lastBinary == 0x0F && currentBinary == 0 )
				delta = -1;
			else if( lastBinary == 0 && currentBinary == 0x0F )
				delta = 1;
			else
				delta = currentBinary - lastBinary;

			lastState_ = state;

			if (delta > 0) {
				tryToSendDcsBiosMessage(msg_, incArg_);
			}
			if (delta < 0) {
				tryToSendDcsBiosMessage(msg_, decArg_);
			}
		}
	public:
		GrayCodeEncoderT(const char* msg, const char* decArg, const char* incArg, char pin0, char pin1, char pin2, char pin3) :
			PollingInput(pollIntervalMs) {
			msg_ = msg;
			decArg_ = decArg;
			incArg_ = incArg;
			pin0_ = pin0;
			pin1_ = pin1;
			pin2_ = pin2;
			pin3_ = pin3;
			pinMode(pin0_, INPUT_PULLUP);
			pinMode(pin1_, INPUT_PULLUP);
			pinMode(pin2_, INPUT_PULLUP);
			pinMode(pin3_, INPUT_PULLUP);
			lastState_ = readState();
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
	typedef GrayCodeEncoderT<> GrayCodeEncoder;
}

#endif
