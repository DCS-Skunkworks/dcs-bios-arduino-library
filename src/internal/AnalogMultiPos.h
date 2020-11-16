#ifndef __DCSBIOS_ANALOGMULTIPOS_H
#define __DCSBIOS_ANALOGMULTIPOS_H

#include "Arduino.h"

namespace DcsBios {
	
class AnalogMultiPos : PollingInput
{
private:
    const char *msg_;
    char pin_;
    char numOfSteps;
    int divisor;
    char lastState_;
    int period = 750;
    unsigned long time_now = 0;

    char readState()
    {
        float fAnalogPct = analogRead(pin_) / 1024;

        char result = (char)((numOfSteps-1) * fAnalogPct);

        return result;
    }

    void resetState()
    {
        lastState_ = (lastState_==0)?-1:0;
    }

    void pollInput()
    {
        if (millis() > time_now + period)
        {
            char state = readState();

            time_now = millis();
            if (state != lastState_)
            {
                if (state == 0)
                {
                    if (tryToSendDcsBiosMessage(msg_, "0"))
                        lastState_ = state;
                }
                else if (state == 1)
                {
                    if (tryToSendDcsBiosMessage(msg_, "1"))
                        lastState_ = state;
                }
                else if (state == 2)
                {
                    if (tryToSendDcsBiosMessage(msg_, "2"))
                        lastState_ = state;
                }
                else if (state == 3)
                {
                    if (tryToSendDcsBiosMessage(msg_, "3"))
                        lastState_ = state;
                }
                else if (state == 4)
                {
                    if (tryToSendDcsBiosMessage(msg_, "4"))
                        lastState_ = state;
                }
                else if (state == 5) {
                    if (tryToSendDcsBiosMessage(msg_, "5"))
                        lastState_ = state;
                }
                else if (state == 6) {
                    if (tryToSendDcsBiosMessage(msg_, "6"))
                        lastState_ = state;
                }
                else if (state == 7)
                {
                    if (tryToSendDcsBiosMessage(msg_, "7"))
                        lastState_ = state;
                }
                else if (state == 8)
                {
                    if (tryToSendDcsBiosMessage(msg_, "8"))
                        lastState_ = state;
                }
                else if (state == 9)
                {
                    if (tryToSendDcsBiosMessage(msg_, "9"))
                        lastState_ = state;
                }
                else if (state == 10)
                {
                    if (tryToSendDcsBiosMessage(msg_, "10"))
                        lastState_ = state;
                }
            }
        }
    }

public:
    AnalogMultiPos(const char *msg, char pin, char numOfSteps_, int divisor_)
    {
        msg_ = msg;
        pin_ = pin;
        divisor = divisor_;
        lastState_ = readState();
        numOfSteps = numOfSteps_;
    }

    void SetControl(const char *msg)
    {
        msg_ = msg;
    }
}