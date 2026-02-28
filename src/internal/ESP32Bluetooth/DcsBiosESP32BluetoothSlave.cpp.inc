#ifdef DCSBIOS_ESP32_BLUETOOTH
#include "DcsBiosESP32BluetoothSlave.h"

namespace DcsBios
{
	void ESP32BluetoothSlave::begin()
	{
		bluetoothSerial.begin(DCSBIOS_ESP32_BLUETOOTH);
	}

	void ESP32BluetoothSlave::loop()
	{
		if (bluetoothSerial.available())
		{
			parser.processChar(bluetoothSerial.read());
		}
	}

    void ESP32BluetoothSlave::trySend(String data)
    {
		if (bluetoothSerial.hasClient())
		{
			bluetoothSerial.print(data);
		}
    }

    void setup()
    {
		bluetoothSlave.begin();
	}

	void loop()
	{
		bluetoothSlave.loop();

		PollingInput::pollInputs();
		ExportStreamListener::loopAll();
	}

	bool tryToSendDcsBiosMessage(const char *msg, const char *arg)
	{
		String data = msg + String(' ') + arg + String('\n');

		bluetoothSlave.trySend(data);

		DcsBios::PollingInput::setMessageSentOrQueued();
		return true;
	}

	void resetAllStates()
	{
		PollingInput::resetAllStates();
	}
}
#endif
