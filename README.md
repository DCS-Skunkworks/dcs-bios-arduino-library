# DCS-BIOS Arduino Library

This is an Arduino library that makes it easy to write sketches that talk to DCS-BIOS.

For more information and documentation, see the [DCS-BIOS FlightPanels Project](https://github.com/DCSFlightpanels).

## Origins

DCS-BIOS was originally developed here, [DCS-BIOS project.](https://github.com/dcs-bios/dcs-bios)  DCS-BIOS Flightpanels forked all of DCS-BIOS in order to initially provide support for Saitek Flight Panels.  As of Nov, 2020, the original project has received limited support (despite a major update for HUB), and FlightPanels is gaining in popularity due to it's support.

## Support

This is a community maintained plugin.  Support is best found at the DCS-Flightpanels discord channel.

## v0.3.0

First version released after forking from the dcs-bios repo.  Several changes are included and only vaguely documented as the primary developer becomes oriented within the project.

- Remove pollInputCurrent in favor of DcsBios::resetAllStates(), which can be used to re-sync the state of every input.  Intended to be called from the sketch using something like:

  ```c++
  void onAcftNameBufferChange(char* newValue) {
  
    // Change of Aircraft
    DcsBios::resetAllStates();
  }
  
  DcsBios::StringBuffer<16> AcftNameBuffer(0x0000, onAcftNameBufferChange);
  ```

- AnalogMultiPos extracted into it's own file, and optimized/fixed.

- Resolved "hanging else" issues throughout which could affect certain compilers (but not Arduino, so no-factor for most applications).

- Differentiated Arduino library name from original fork.

- Resolve redef warning for PRR0 for certain boards (Teensy++2.0 for certain, likely more).

- Added RotaryAcceleratedEncoder, an extended RotaryEncoder which aims to solve issues with noisy/faulty rotaries by tracking rotary momentum and using that as a filter to counter-momentum signalling.

- Removed several SwitchMatrix references which appeared to be incomplete in the original fork.

- Added debounce support for Switch3Pos