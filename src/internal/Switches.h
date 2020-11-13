#ifndef __DCSBIOS_SWITCHES_H
#define __DCSBIOS_SWITCHES_H

#include <math.h>
#include "Arduino.h"
#include "SwitchMatrix.h"

SwitchMatrix swPanel = SwitchMatrix();

namespace DcsBios {

	
	class Switch2Pos : PollingInput {
		private:
			const char* msg_;
			char pin_;
			char lastState_;
			char switchState_;
			bool reverse_;
			unsigned long debounceDelay_; 
			unsigned long lastDebounceTime = 0;
			void init_(const char* msg, char pin, bool reverse, unsigned long debounceDelay) {
				msg_ = msg;
				pin_ = pin;
				pinMode(pin_, INPUT_PULLUP);
				switchState_ = digitalRead(pin_);
				lastState_ = switchState_;
				reverse_ = reverse;
				debounceDelay_ = debounceDelay;
			}
			void pollInput() {
				char state = digitalRead(pin_);
				if (reverse_) state = !state;
				if (state != lastState_) {
					lastDebounceTime = millis();
				}
				
				if ((millis() - lastDebounceTime) > debounceDelay_) {
					if (state != switchState_) {
					if (tryToSendDcsBiosMessage(msg_, state == HIGH ? "0" : "1")) {
							switchState_ = state;
						}
					}
				}
						lastState_ = state;
			}
		public:
			Switch2Pos(const char* msg, char pin, bool reverse, unsigned long debounceDelay) { init_(msg, pin, reverse, debounceDelay); }
			Switch2Pos(const char* msg, char pin, bool reverse) { init_(msg, pin, reverse, 50); }
			Switch2Pos(const char* msg, char pin) { init_(msg, pin, false, 50); }
			
			///////////////////////////////////////////
			//	Inserted Code
			//
			
				void pollInputCurrent() 
				{
					char state = digitalRead(pin_);
					if (reverse_) state = !state;
					if (tryToSendDcsBiosMessage(msg_, state == HIGH ? "0" : "1")) {
							lastState_ = state;
							delay(4);
						}
						
				}	
			
				void SetControl( const char* msg )
				{
					msg_ = msg;
				}
			
		};
	
	class Switch3Pos : PollingInput {
		private:
			const char* msg_;
			char pinA_;
			char pinB_;
			char lastState_;
			char readState() {
				if (digitalRead(pinA_) == LOW) return 0;
				if (digitalRead(pinB_) == LOW) return 2;
				return 1;
			}
			void pollInput() {
				char state = readState();
				if (state != lastState_) {
					if (state == 0)
						if (tryToSendDcsBiosMessage(msg_, "0"))
							lastState_ = state;
					if (state == 1)
						if (tryToSendDcsBiosMessage(msg_, "1"))
							lastState_ = state;
					if (state == 2)
						if(tryToSendDcsBiosMessage(msg_, "2"))
							lastState_ = state;
				}
			}
		public:
			Switch3Pos(const char* msg, char pinA, char pinB) {
				msg_ = msg;
				pinA_ = pinA;
				pinB_ = pinB;
				pinMode(pinA_, INPUT_PULLUP);
				pinMode(pinB_, INPUT_PULLUP);
				lastState_ = readState();
			}
			
			//////////////////////////////////////////
			//	Inserted Code
			//
			
				void pollInputCurrent() 
				{
					char state = readState();
					if (state == 0)
						if (tryToSendDcsBiosMessage(msg_, "0"))
							lastState_ = state;
					if (state == 1)
						if (tryToSendDcsBiosMessage(msg_, "1"))
							lastState_ = state;
					if (state == 2)
						if(tryToSendDcsBiosMessage(msg_, "2"))
							lastState_ = state;
							
					delay(4);
					
				}
				
				void SetControl( const char* msg )
				{
					msg_ = msg;
				}
	};

	
	class SwitchMultiPos : PollingInput {
		private:
			const char* msg_;
			const byte* pins_;
			char numberOfPins_;
			char lastState_;
			bool reverse_;
			char readState() {
				unsigned char i;
				for (i=0; i<numberOfPins_; i++) {
					if (digitalRead(pins_[i]) == LOW && reverse_ == false) return i;
					else if (digitalRead(pins_[i]) == HIGH && reverse_ == true) return i;
				}
				return lastState_;
			}
			void pollInput() {
				char state = readState();
				if (state != lastState_) {
					char buf[7];
					utoa(state, buf, 10);
					if (tryToSendDcsBiosMessage(msg_, buf))
						lastState_ = state;
				}
			}
		public:
			SwitchMultiPos(const char* msg, const byte* pins, char numberOfPins, bool reverse = false) : lastState_(0) {
				msg_ = msg;
				pins_ = pins;
				reverse_ = reverse;
				numberOfPins_ = numberOfPins;
				unsigned char i;
				for (i=0; i<numberOfPins; i++) {
					pinMode(pins[i], INPUT_PULLUP);
				}
				lastState_ = readState();
			}
			
			//////////////////////////////////////////
			//	Inserted Code
			//
			
				void pollInputCurrent() 
				{
					char state = readState();
					char buf[7];
					utoa(state, buf, 10);
					if (tryToSendDcsBiosMessage(msg_, buf))
						lastState_ = state;
				
					delay(4);
				}
				
				void SetControl( const char* msg )
				{
					msg_ = msg;
				}
	};

	class Matrix2Pos : PollingInput {
		private:
			const char* msg_;
			char row_;
			char col_;
			char lastState_;
			bool reverse_;
			void init_(const char* msg, char row, char col, bool reverse) {
				msg_ = msg;
				row_ = row;
				col_ = col;
				lastState_ = swPanel.GetSwitchState(row_, col_);
				reverse_ = reverse;
			}
			void pollInput() {
				char state = swPanel.GetSwitchState(row_, col_);
				if (reverse_) state = !state;
				if (state != lastState_) {
					if (tryToSendDcsBiosMessage(msg_, state == false ? "0" : "1")) {
						lastState_ = state;
					}
				}
			}
		public:
			Matrix2Pos(const char* msg, char row, char col, bool reverse) { init_(msg, row, col, reverse); }
			Matrix2Pos(const char* msg, char row, char col) { init_(msg, row, col, false); }
	};
	
			class Matrix3Pos : PollingInput {
		private:
			const char* msg_;
			char rowA_;
			char colA_;
			char rowB_;
			char colB_;
			char lastState_;
			char readState() {
				if (swPanel.GetSwitchState(rowA_, colA_) == true) return 0;
				if (swPanel.GetSwitchState(rowB_, colB_) == true) return 2;
				return 1;
			}
			void pollInput() {
				char state = readState();
				if (state != lastState_) {
					if (state == 0)
						if (tryToSendDcsBiosMessage(msg_, "0"))
							lastState_ = state;
					if (state == 1)
						if (tryToSendDcsBiosMessage(msg_, "1"))
							lastState_ = state;
					if (state == 2)
						if(tryToSendDcsBiosMessage(msg_, "2"))
							lastState_ = state;
				}
			}
		public:
			Matrix3Pos(const char* msg, char rowA, char colA, char rowB, char colB) {
				msg_ = msg;
				colA_ = colA;
				rowA_ = rowA;
				colB_ = colB;
				rowB_ = rowB;
				lastState_ = readState();
			}
	};
	
			class AnalogMultiPos : PollingInput {
		private:
			const char* msg_;
			char pin_;
			char numOfSteps;
			int divisor;
			char lastState_;
			char readState() {
				if (round(analogRead(pin_)/divisor) == 0) return 0;
				if (round(analogRead(pin_)/divisor) == 1) return 1;
				if (analogRead(pin_)/divisor == 2) return 2;
				if (analogRead(pin_)/divisor == 3) return 3;
				if (analogRead(pin_)/divisor == 4) return 4;
				if (analogRead(pin_)/divisor == 5) return 5;
				if (analogRead(pin_)/divisor == 6) return 6;
				if (analogRead(pin_)/divisor == 7) return 7;
				if (analogRead(pin_)/divisor == 8) return 8;
				if (analogRead(pin_)/divisor == 9) return 9;
				if (analogRead(pin_)/divisor == 10) return 10;
			}

			void pollInput() {
				char state = readState();
				if (state != lastState_) {
					if (state == 0)
						if (tryToSendDcsBiosMessage(msg_, "0"))
							lastState_ = state;
					if (state == 1)
						if (tryToSendDcsBiosMessage(msg_, "1"))
							lastState_ = state;
					if (state == 2)
						if(tryToSendDcsBiosMessage(msg_, "2"))
							lastState_ = state;
					if (state == 3)
						if(tryToSendDcsBiosMessage(msg_, "3"))
							lastState_ = state;
					if (state == 4)
						if(tryToSendDcsBiosMessage(msg_, "4"))
							lastState_ = state;
					if (state == 5)
						if(tryToSendDcsBiosMessage(msg_, "5"))
							lastState_ = state;
					if (state == 6)
						if(tryToSendDcsBiosMessage(msg_, "6"))
							lastState_ = state;
					if (state == 7)
						if(tryToSendDcsBiosMessage(msg_, "7"))
							lastState_ = state;
					if (state == 8)
						if(tryToSendDcsBiosMessage(msg_, "8"))
							lastState_ = state;
					if (state == 9)
						if(tryToSendDcsBiosMessage(msg_, "9"))
							lastState_ = state;
					if (state == 10)
						if(tryToSendDcsBiosMessage(msg_, "10"))
							lastState_ = state;
				}
			}
		public:
			AnalogMultiPos(const char* msg, char pin, char numOfSteps, char divisor_) {
				msg_ = msg;
				pin_ = pin;
				divisor = divisor_;
				lastState_ = readState();
			}
	//////////////////////////////////////////
			//	Inserted Code
			//
			
				void pollInputCurrent() 
				{
					char state = readState();
					char buf[7];
					utoa(state, buf, 10);
					if (tryToSendDcsBiosMessage(msg_, buf))
						lastState_ = state;
				
					delay(4);
				}
				
				void SetControl( const char* msg )
				{
					msg_ = msg;
				}
			};	
}

#endif	