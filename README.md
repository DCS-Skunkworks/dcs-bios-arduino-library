
# DCS-BIOS Arduino Library

## About this fork

The DCSBIOSKit fork contains various fixes to the dcs-bios-arduino-library that have not yet made their way to the official [DCS-Skunkworks](https://github.com/DCS-Skunkworks/dcs-bios-arduino-library) fork.

## ESP32 over WiFi

This fork supports ESP32 communication over WiFi with [DCS-Nexus](https://github.com/DCSBIOSKit/DCS-Nexus/releases).

### Requirements
1. **ESP32Servo** library, or `#define DCSBIOS_DISABLE_SERVO`
2. **Adafruit NeoPixel** for status display on neopixel.
3. **nanopb** library is included.
4. **AsyncTCP** if you want to use TCP mode. Multicast is recommended if properly supported by your router.

### Usage

1. Download and install the latest `DCS-Nexus` from the link above.
2. If you are using Arduino IDE, replace your `dcs-bios-arduino-library` in `Documents\Arduino\Libraries` with this library. If you are using VSCode and PlatformIO, add this dependency to your project `maciekish/DCS-BIOS DCSBIOSKit-Fork@^3.9.0`
3. Replace your old dcs-bios `#defines` with the following:
```
#define  DCSBIOS_ESP32_WIFI "slave-name" // Displayed in DCS-Nexus
#define  DCSBIOS_ESP32_WIFI_MULTICAST // TCP requires the AsyncTCP library.
#define  DCSBIOS_ESP32_WIFI_SSID "YOUR-WIFI-NAME"
#define  DCSBIOS_ESP32_WIFI_PASSWORD "YOUR-WIFI-PASSWORD"
#define  DCSBIOS_ESP32_WIFI_NEOPIXEL 48 // Optional Neopixel pin. If your ESP32 doesn't have a neopixel, remove this line. Requires Adafruit Neopixel library.
#define  DCSBIOS_ESP32_WIFI_NEOPIXEL_BRIGHTNESS 2
#define  DCSBIOS_ESP32_WIFI_INSTRUMENTATION // Calculates loop() execution time, optional.
#include  "DcsBios.h" // If using PlatformIO, use <DcsBios.h>
```
4. Compile, upload and run DCS-Nexus. The slave should appear in the list shortly, and just work as soon as you launch a mission in DCS World.

## What is DCS-BIOS?

This is an Arduino library that makes it easy to write sketches that talk to DCS-BIOS.

For more information and documentation, see the [DCS-BIOS FlightPanels Project](https://github.com/DCSFlightpanels).  The example "OneOfEverything" is a good place to start looking for controls to use.

## Origins

DCS-BIOS was originally developed here, [DCS-BIOS project.](https://github.com/dcs-bios/dcs-bios)  DCS-BIOS Flightpanels forked all of DCS-BIOS in order to initially provide support for Saitek Flight Panels.  As of Nov, 2020, the original project has received limited support (despite a major update for HUB), and FlightPanels is gaining in popularity due to it's support.  Late in 2020, this fork of the arduino-only portion was created to become the "official" branch associated with the FlightPanels DCS-BIOS side.

## Support & Documentation

This is a community maintained plugin.  Support is best found at the DCS-Flightpanels discord channel.  The wiki is very much a work in progress, and can be found here: https://github.com/DCSFlightpanels/dcs-bios-arduino-library/wiki

## Releasing

1. Bump version number in library.properties
2. Run make_release, providing the same version number when prompted.
3. Manually make a zip file of the folder created in /Releases, and upload to github.

### Acknowledgements
This version of dcs-bios-arduino-library uses nanopb licensed under the zlib license.
For more information, see https://github.com/nanopb/nanopb
