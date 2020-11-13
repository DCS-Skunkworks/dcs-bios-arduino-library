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
        int which = 0;
        int nextval = -10;
        int readvalue = analogRead(pin_);
        for (int i = 0; i < (int)numOfSteps; i++)
        {
            if (readvalue >= nextval)
            {
                which = i;
                nextval = nextval + (1024 / numOfSteps);
            }
        }
        return which;
    }

    void pollInput()
    {
        char state = readState();

        if (millis() > time_now + period)
        {
            time_now = millis();
            if (state != lastState_)
            {
                if (state == 0)
                    if (tryToSendDcsBiosMessage(msg_, "0"))
                        lastState_ = state;
                if (state == 1)
                    if (tryToSendDcsBiosMessage(msg_, "1"))
                        lastState_ = state;
                if (state == 2)
                    if (tryToSendDcsBiosMessage(msg_, "2"))
                        lastState_ = state;
                if (state == 3)
                    if (tryToSendDcsBiosMessage(msg_, "3"))
                        lastState_ = state;
                if (state == 4)
                    if (tryToSendDcsBiosMessage(msg_, "4"))
                        lastState_ = state;
                if (state == 5)
                    if (tryToSendDcsBiosMessage(msg_, "5"))
                        lastState_ = state;
                if (state == 6)
                    if (tryToSendDcsBiosMessage(msg_, "6"))
                        lastState_ = state;
                if (state == 7)
                    if (tryToSendDcsBiosMessage(msg_, "7"))
                        lastState_ = state;
                if (state == 8)
                    if (tryToSendDcsBiosMessage(msg_, "8"))
                        lastState_ = state;
                if (state == 9)
                    if (tryToSendDcsBiosMessage(msg_, "9"))
                        lastState_ = state;
                if (state == 10)
                    if (tryToSendDcsBiosMessage(msg_, "10"))
                        lastState_ = state;
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


    void pollInputCurrent()
    {
        char state = readState();
        char buf[7];
        utoa(state, buf, 10);
        if (tryToSendDcsBiosMessage(msg_, buf))
        lastState_ = state;

        delay(100); // Thats a long time.  Why?
    }

    void SetControl(const char *msg)
    {
        msg_ = msg;
    }
}