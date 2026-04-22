#ifndef _DCSBIOS_RS485_SLAVE_H_
#define DCSBIOS_RS485_SLAVE_H_
#ifdef DCSBIOS_RS485_SLAVE

#include "Arduino.h"
#include "../RingBuffer.h"

namespace DcsBios {

	ProtocolParser parser;
#ifdef DCSBIOS_RS485_SLAVE_LARGE_BUFFER
	DcsBios::RingBuffer<64> messageBuffer;
#else
	DcsBios::RingBuffer<32> messageBuffer;
#endif

	bool tryToSendDcsBiosMessage(const char* msg, const char* arg);
	void setup();
	void loop();
	void resetAllStates();
}

#endif // DCSBIOS_RS485_SLAVE
#endif // _DCSBIOS_RS485_SLAVE_H_
