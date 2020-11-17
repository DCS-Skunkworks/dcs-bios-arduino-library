#ifndef __DCSBIOS_ENCODERS_H
#define __DCSBIOS_ENCODERS_H

#include "Arduino.h"
#include "PollingInput.h"

namespace DcsBios {
	enum StepsPerDetent {
		ONE_STEP_PER_DETENT = 1,
		TWO_STEPS_PER_DETENT = 2,
		FOUR_STEPS_PER_DETENT = 4
	};
	class RotaryEncoder : PollingInput {
		private:
			const char* msg_;
			const char* decArg_;
			const char* incArg_;
			char pinA_;
			char pinB_;
			char lastState_;
			signed char delta_;
			char stepsPerDetent_;
			char readState() {
				return (digitalRead(pinA_) << 1) | digitalRead(pinB_);
			}
			void resetState()
			{
				lastState_ = (lastState_==0)?-1:0;
			}
			void pollInput() {
				char state = readState();
				switch(lastState_) {
					case 0:
						if (state == 2) delta_--;
						if (state == 1) delta_++;
						break;
					case 1:
						if (state == 0) delta_--;
						if (state == 3) delta_++;
						break;
					case 2:
						if (state == 3) delta_--;
						if (state == 0) delta_++;
						break;
					case 3:
						if (state == 1) delta_--;
						if (state == 2) delta_++;
						break;
				}
				lastState_ = state;
				
				if (delta_ >= stepsPerDetent_) {
					if (tryToSendDcsBiosMessage(msg_, incArg_))
						delta_ -= stepsPerDetent_;
				}
				if (delta_ <= -stepsPerDetent_) {
					if (tryToSendDcsBiosMessage(msg_, decArg_))
						delta_ += stepsPerDetent_;
				}
			}
		public:
			RotaryEncoder(const char* msg, const char* decArg, const char* incArg, char pinA, char pinB, StepsPerDetent stepsPerDetent = ONE_STEP_PER_DETENT) {
				msg_ = msg;
				decArg_ = decArg;
				incArg_ = incArg;
				pinA_ = pinA;
				pinB_ = pinB;
				stepsPerDetent_ = stepsPerDetent;
				pinMode(pinA_, INPUT_PULLUP);
				pinMode(pinB_, INPUT_PULLUP);
				delta_ = 0;
				lastState_ = readState();
			}

			void SetControl( const char* msg )
			{
				msg_ = msg;
			}
	};

	class RotaryAcceleratedEncoder : PollingInput {
    private:
		const char* msg_;
		const char* decArg_;
		const char* incArg_;
		const char* fastDecArg_;
		const char* fastIncArg_;
		char pinA_;
		char pinB_;
		char lastState_;
		signed char delta_;
		char stepsPerDetent_;

		unsigned long timeLastDetent_;
		
		char readState() {
			return (digitalRead(pinA_) << 1) | digitalRead(pinB_);
		}
		
		void resetState()
		{
			lastState_ = (lastState_==0)?-1:0;
		}

		void pollInput()
		{
			char state = readState();
			switch(lastState_) {
				case 0:
				if (state == 2) delta_--;
				if (state == 1) delta_++;
				break;
				case 1:
				if (state == 0) delta_--;
				if (state == 3) delta_++;
				break;
				case 2:
				if (state == 3) delta_--;
				if (state == 0) delta_++;
				break;
				case 3:
				if (state == 1) delta_--;
				if (state == 2) delta_++;
				break;
			}
			lastState_ = state;
			
			if (delta_ >= stepsPerDetent_) {
				const char *arg;
				if( (millis() - timeLastDetent_) < 750 )
				arg = fastIncArg_;
				else
				arg = incArg_;
				if (tryToSendDcsBiosMessage(msg_, arg))
				{
				delta_ -= stepsPerDetent_;
				timeLastDetent_ = millis();
				}
			}
			if (delta_ <= -stepsPerDetent_) {
				const char *arg;
				if( (millis() - timeLastDetent_) < 750 )
				arg = fastDecArg_;
				else
				arg = decArg_;
				if (tryToSendDcsBiosMessage(msg_, arg))
				{
				delta_ += stepsPerDetent_;
				timeLastDetent_ = millis();
				}
			}
		}
	public:
		RotaryAcceleratedEncoder(const char* msg, const char* decArg, const char* incArg, const char* fastDecArg, const char* fastIncArg, char pinA, char pinB, char stepsPerDetent) {
		msg_ = msg;
		decArg_ = decArg;
		incArg_ = incArg;
		fastDecArg_ = fastDecArg;
		fastIncArg_ = fastIncArg;
		pinA_ = pinA;
		pinB_ = pinB;
		stepsPerDetent_ = stepsPerDetent;
		pinMode(pinA_, INPUT_PULLUP);
		pinMode(pinB_, INPUT_PULLUP);
		delta_ = 0;
		lastState_ = readState();
		timeLastDetent_ = millis();
		}
  };
}

#endif