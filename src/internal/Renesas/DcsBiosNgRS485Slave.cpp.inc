#ifdef DCSBIOS_RS485_SLAVE

namespace DcsBios {

	enum State {
		UNINITIALIZED,
		SYNC,
		RX_WAIT_ADDRESS,
		RX_WAIT_MSGTYPE,
		RX_WAIT_DATALENGTH,
		RX_WAIT_DATA,
		RX_WAIT_CHECKSUM,
		RX_HOST_MESSAGE_COMPLETE,
		RX_WAIT_ANSWER_DATALENGTH,
		RX_WAIT_ANSWER_MSGTYPE,
		RX_WAIT_ANSWER_DATA,
		RX_WAIT_ANSWER_CHECKSUM
	};

	enum RxDataType {
		RXDATA_IGNORE,
		RXDATA_DCSBIOS_EXPORT
	};

	static State state = UNINITIALIZED;
	static unsigned long last_rx_time = 0;
	static uint8_t rx_slave_address = 0;
	static uint8_t rx_msgtype = 0;
	static uint8_t rxtx_len = 0;
	static RxDataType rx_datatype = RXDATA_IGNORE;

	static void transmitResponse() {
		digitalWrite(TXENABLE_PIN, HIGH);
		delayMicroseconds(10);

		if (!messageBuffer.complete) {
			Serial1.write((uint8_t)0);
		} else {
			uint8_t txLen = messageBuffer.getLength();
			Serial1.write(txLen);
			Serial1.write((uint8_t)0); // MSGTYPE
			while (txLen > 0) {
				txLen--;
				Serial1.write(messageBuffer.get());
			}
			Serial1.write((uint8_t)0x72); // TODO: transmit actual checksum
			messageBuffer.clear();
			messageBuffer.complete = false;
		}

		Serial1.flush();
		digitalWrite(TXENABLE_PIN, LOW);
	}

	bool tryToSendDcsBiosMessage(const char* msg, const char* arg) {
		if (messageBuffer.complete) return false;

		messageBuffer.clear();
		const char* c = msg;
		while (*c) {
			messageBuffer.put(*c++);
		}
		messageBuffer.put(' ');
		c = arg;
		while (*c) {
			messageBuffer.put(*c++);
		}
		messageBuffer.put('\n');

		messageBuffer.complete = true;
		DcsBios::PollingInput::setMessageSentOrQueued();
		return true;
	}

	static void processRxByte(uint8_t c) {
		switch (state) {
			case UNINITIALIZED:
				last_rx_time = micros();
				state = SYNC;
			break;

			case SYNC:
				if (micros() - last_rx_time < 500) {
					last_rx_time = micros();
					return;
				}
				state = RX_WAIT_ADDRESS;
				// fall through

			case RX_WAIT_ADDRESS:
				rx_slave_address = c;
				state = RX_WAIT_MSGTYPE;
			break;

			case RX_WAIT_MSGTYPE:
				rx_msgtype = c;
				state = RX_WAIT_DATALENGTH;
			break;

			case RX_WAIT_DATALENGTH:
				rxtx_len = c;
				if (rxtx_len == 0) {
					state = RX_HOST_MESSAGE_COMPLETE;
					break;
				}

				if (rx_slave_address == 0 && rx_msgtype == 0) {
					rx_datatype = RXDATA_DCSBIOS_EXPORT;
				} else {
					rx_datatype = RXDATA_IGNORE;
				}
				state = RX_WAIT_DATA;
			break;

			case RX_WAIT_DATA:
				rxtx_len--;
				if (rx_datatype == RXDATA_DCSBIOS_EXPORT) {
					if (rxtx_len <= parser.incomingDataBuffer.availableForWrite()) {
						parser.processCharISR(c);
					} else {
						last_rx_time = micros();
						state = SYNC;
						return;
					}
				}
				if (rxtx_len == 0) {
					state = RX_WAIT_CHECKSUM;
				}
			break;

			case RX_WAIT_CHECKSUM:
				state = RX_HOST_MESSAGE_COMPLETE;
			break;

			case RX_WAIT_ANSWER_DATALENGTH:
				rxtx_len = c;
				if (rxtx_len == 0) {
					state = RX_WAIT_ADDRESS;
				} else {
					state = RX_WAIT_ANSWER_MSGTYPE;
				}
			break;

			case RX_WAIT_ANSWER_MSGTYPE:
				rx_msgtype = c;
				state = RX_WAIT_ANSWER_DATA;
			break;

			case RX_WAIT_ANSWER_DATA:
				rxtx_len--;
				if (rxtx_len == 0) {
					state = RX_WAIT_ANSWER_CHECKSUM;
				}
			break;

			case RX_WAIT_ANSWER_CHECKSUM:
				state = RX_WAIT_ADDRESS;
			break;
		}

		if (state == RX_HOST_MESSAGE_COMPLETE) {
			if (rx_slave_address == 0) {
				state = RX_WAIT_ADDRESS;
			} else if (rx_slave_address == DCSBIOS_RS485_SLAVE) {
				if (rx_msgtype == 0 && rxtx_len == 0) {
					transmitResponse();
					state = RX_WAIT_ADDRESS;
				} else {
					last_rx_time = micros();
					state = SYNC;
				}
			} else {
				state = RX_WAIT_ANSWER_DATALENGTH;
			}
		}
	}

	void setup() {
		pinMode(TXENABLE_PIN, OUTPUT);
		digitalWrite(TXENABLE_PIN, LOW);
		Serial1.begin(250000);
		state = UNINITIALIZED;
	}

	void loop() {
		while (Serial1.available()) {
			processRxByte((uint8_t)Serial1.read());
		}

		PollingInput::pollInputs();
		ExportStreamListener::loopAll();

#ifdef DCSBIOS_DEFER_RS485_PROCESSING
		if (!parser.incomingDataBuffer.isEmpty()) {
			unsigned char nextByte = parser.incomingDataBuffer.get();
			parser.processChar(nextByte);
		}
#endif
	}

	void resetAllStates() {
		PollingInput::resetAllStates();
	}

}
#endif
