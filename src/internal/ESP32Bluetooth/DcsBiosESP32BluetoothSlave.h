#ifdef DCSBIOS_ESP32_BLUETOOTH

#ifndef _DCSBIOS_ESP32_BLUETOOTH_H_
#define _DCSBIOS_ESP32_BLUETOOTH_H_

#ifndef ARDUINO_ARCH_ESP32
#error "This code is designed to run on ESP32 - Using the arduino-esp32 framework"
#endif

#include <Arduino.h>
#include <BluetoothSerial.h>

namespace DcsBios
{
    ProtocolParser parser;

    class ESP32BluetoothSlave
    {
    public:
        void begin();
        void loop();

        void trySend(String Data);

        BluetoothSerial bluetoothSerial;
    };

    ESP32BluetoothSlave bluetoothSlave;
}

#endif // DCSBIOS_ESP32_BLUETOOTH
#endif // _DCSBIOS_ESP32_BLUETOOTH_H_