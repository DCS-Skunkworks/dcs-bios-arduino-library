/*
  The purpose of this example is to demonstrate how the Arudino Mega will drop incoming RS485 data if
  processing is done in the ISR.
  
  #define DCSBIOS_DEFER_RS485_PROCESSING is enabled in this sketch, comment
  it out to see the problem. You don't need all LED's to be physicallyconnected, just any one that is
  supposed to be blinking.

  Even though the sketch uses efficient direct register access, this sketch will still drop data and
  not blink the LEDs in sync with the sim unless DCSBIOS_DEFER_RS485_PROCESSING is defined.
*/

#define DCSBIOS_RS485_SLAVE 23 // The following #define tells DCS-BIOS that this is a RS-485 slave device. It also sets the address of this slave device. The slave address should be between 1 and 126 and must be unique among all devices on the same bus.
#define TXENABLE_PIN 2 // The Arduino pin that is connected to the /RE and DE pins on the RS-485 transceiver.
#define DCSBIOS_DEFER_RS485_PROCESSING // Defer RS485 processing from the ISR to loop().
#include "DcsBios.h"

void onClA1Change(unsigned int newValue) { if (newValue == 1) { PORTE |= (1 << 5); } else { PORTE &= ~(1 << 5); } } DcsBios::IntegerBuffer clA1Buffer(0x10d4, 0x0001, 0, onClA1Change);
void onClA2Change(unsigned int newValue) { if (newValue == 1) { PORTG |= (1 << 5); } else { PORTG &= ~(1 << 5); } } DcsBios::IntegerBuffer clA2Buffer(0x10d4, 0x0002, 1, onClA2Change);
void onClA3Change(unsigned int newValue) { if (newValue == 1) { PORTE |= (1 << 3); } else { PORTE &= ~(1 << 3); } } DcsBios::IntegerBuffer clA3Buffer(0x10d4, 0x0004, 2, onClA3Change);
void onClA4Change(unsigned int newValue) { if (newValue == 1) { PORTH |= (1 << 3); } else { PORTH &= ~(1 << 3); } } DcsBios::IntegerBuffer clA4Buffer(0x10d4, 0x0008, 3, onClA4Change);
void onClB1Change(unsigned int newValue) { if (newValue == 1) { PORTH |= (1 << 4); } else { PORTH &= ~(1 << 4); } } DcsBios::IntegerBuffer clB1Buffer(0x10d4, 0x0010, 4, onClB1Change);
void onClB2Change(unsigned int newValue) { if (newValue == 1) { PORTH |= (1 << 5); } else { PORTH &= ~(1 << 5); } } DcsBios::IntegerBuffer clB2Buffer(0x10d4, 0x0020, 5, onClB2Change);
void onClB3Change(unsigned int newValue) { if (newValue == 1) { PORTH |= (1 << 6); } else { PORTH &= ~(1 << 6); } } DcsBios::IntegerBuffer clB3Buffer(0x10d4, 0x0040, 6, onClB3Change);
void onClB4Change(unsigned int newValue) { if (newValue == 1) { PORTB |= (1 << 4); } else { PORTB &= ~(1 << 4); } } DcsBios::IntegerBuffer clB4Buffer(0x10d4, 0x0080, 7, onClB4Change);
void onClC1Change(unsigned int newValue) { if (newValue == 1) { PORTB |= (1 << 5); } else { PORTB &= ~(1 << 5); } } DcsBios::IntegerBuffer clC1Buffer(0x10d4, 0x0100, 8, onClC1Change);
void onClC2Change(unsigned int newValue) { if (newValue == 1) { PORTB |= (1 << 6); } else { PORTB &= ~(1 << 6); } } DcsBios::IntegerBuffer clC2Buffer(0x10d4, 0x0200, 9, onClC2Change);
void onClC3Change(unsigned int newValue) { if (newValue == 1) { PORTJ |= (1 << 1); } else { PORTJ &= ~(1 << 1); } } DcsBios::IntegerBuffer clC3Buffer(0x10d4, 0x0400, 10, onClC3Change);
void onClC4Change(unsigned int newValue) { if (newValue == 1) { PORTJ |= (1 << 0); } else { PORTJ &= ~(1 << 0); } } DcsBios::IntegerBuffer clC4Buffer(0x10d4, 0x0800, 11, onClC4Change);
void onClD1Change(unsigned int newValue) { if (newValue == 1) { PORTH |= (1 << 1); } else { PORTH &= ~(1 << 1); } } DcsBios::IntegerBuffer clD1Buffer(0x10d4, 0x1000, 12, onClD1Change);
void onClD2Change(unsigned int newValue) { if (newValue == 1) { PORTH |= (1 << 0); } else { PORTH &= ~(1 << 0); } } DcsBios::IntegerBuffer clD2Buffer(0x10d4, 0x2000, 13, onClD2Change);
void onClD3Change(unsigned int newValue) { if (newValue == 1) { PORTD |= (1 << 3); } else { PORTD &= ~(1 << 3); } } DcsBios::IntegerBuffer clD3Buffer(0x10d4, 0x4000, 14, onClD3Change);
void onClD4Change(unsigned int newValue) { if (newValue == 1) { PORTF |= (1 << 0); } else { PORTF &= ~(1 << 0); } } DcsBios::IntegerBuffer clD4Buffer(0x10d4, 0x8000, 15, onClD4Change);
void onClE1Change(unsigned int newValue) { if (newValue == 1) { PORTF |= (1 << 1); } else { PORTF &= ~(1 << 1); } } DcsBios::IntegerBuffer clE1Buffer(0x10d6, 0x0001, 0, onClE1Change);
void onClE2Change(unsigned int newValue) { if (newValue == 1) { PORTF |= (1 << 2); } else { PORTF &= ~(1 << 2); } } DcsBios::IntegerBuffer clE2Buffer(0x10d6, 0x0002, 1, onClE2Change);
void onClE3Change(unsigned int newValue) { if (newValue == 1) { PORTF |= (1 << 3); } else { PORTF &= ~(1 << 3); } } DcsBios::IntegerBuffer clE3Buffer(0x10d6, 0x0004, 2, onClE3Change);
void onClE4Change(unsigned int newValue) { if (newValue == 1) { PORTF |= (1 << 4); } else { PORTF &= ~(1 << 4); } } DcsBios::IntegerBuffer clE4Buffer(0x10d6, 0x0008, 3, onClE4Change);
void onClF1Change(unsigned int newValue) { if (newValue == 1) { PORTF |= (1 << 5); } else { PORTF &= ~(1 << 5); } } DcsBios::IntegerBuffer clF1Buffer(0x10d6, 0x0010, 4, onClF1Change);
void onClF2Change(unsigned int newValue) { if (newValue == 1) { PORTF |= (1 << 6); } else { PORTF &= ~(1 << 6); } } DcsBios::IntegerBuffer clF2Buffer(0x10d6, 0x0020, 5, onClF2Change);
void onClF3Change(unsigned int newValue) { if (newValue == 1) { PORTF |= (1 << 7); } else { PORTF &= ~(1 << 7); } } DcsBios::IntegerBuffer clF3Buffer(0x10d6, 0x0040, 6, onClF3Change);
void onClF4Change(unsigned int newValue) { if (newValue == 1) { PORTK |= (1 << 0); } else { PORTK &= ~(1 << 0); } } DcsBios::IntegerBuffer clF4Buffer(0x10d6, 0x0080, 7, onClF4Change);
void onClG1Change(unsigned int newValue) { if (newValue == 1) { PORTK |= (1 << 1); } else { PORTK &= ~(1 << 1); } } DcsBios::IntegerBuffer clG1Buffer(0x10d6, 0x0100, 8, onClG1Change);
void onClG2Change(unsigned int newValue) { if (newValue == 1) { PORTK |= (1 << 2); } else { PORTK &= ~(1 << 2); } } DcsBios::IntegerBuffer clG2Buffer(0x10d6, 0x0200, 9, onClG2Change);
void onClG3Change(unsigned int newValue) { if (newValue == 1) { PORTK |= (1 << 3); } else { PORTK &= ~(1 << 3); } } DcsBios::IntegerBuffer clG3Buffer(0x10d6, 0x0400, 10, onClG3Change);
void onClG4Change(unsigned int newValue) { if (newValue == 1) { PORTK |= (1 << 4); } else { PORTK &= ~(1 << 4); } } DcsBios::IntegerBuffer clG4Buffer(0x10d6, 0x0800, 11, onClG4Change);
void onClH1Change(unsigned int newValue) { if (newValue == 1) { PORTK |= (1 << 5); } else { PORTK &= ~(1 << 5); } } DcsBios::IntegerBuffer clH1Buffer(0x10d6, 0x1000, 12, onClH1Change);
void onClH2Change(unsigned int newValue) { if (newValue == 1) { PORTK |= (1 << 6); } else { PORTK &= ~(1 << 6); } } DcsBios::IntegerBuffer clH2Buffer(0x10d6, 0x2000, 13, onClH2Change);
void onClH3Change(unsigned int newValue) { if (newValue == 1) { PORTK |= (1 << 7); } else { PORTK &= ~(1 << 7); } } DcsBios::IntegerBuffer clH3Buffer(0x10d6, 0x4000, 14, onClH3Change);
void onClH4Change(unsigned int newValue) { if (newValue == 1) { PORTD |= (1 << 2); } else { PORTD &= ~(1 << 2); } } DcsBios::IntegerBuffer clH4Buffer(0x10d6, 0x8000, 15, onClH4Change);
void onClI1Change(unsigned int newValue) { if (newValue == 1) { PORTD |= (1 << 1); } else { PORTD &= ~(1 << 1); } } DcsBios::IntegerBuffer clI1Buffer(0x10d8, 0x0001, 0, onClI1Change);
void onClI2Change(unsigned int newValue) { if (newValue == 1) { PORTD |= (1 << 0); } else { PORTD &= ~(1 << 0); } } DcsBios::IntegerBuffer clI2Buffer(0x10d8, 0x0002, 1, onClI2Change);
void onClI3Change(unsigned int newValue) { if (newValue == 1) { PORTA |= (1 << 0); } else { PORTA &= ~(1 << 0); } } DcsBios::IntegerBuffer clI3Buffer(0x10d8, 0x0004, 2, onClI3Change);
void onClI4Change(unsigned int newValue) { if (newValue == 1) { PORTA |= (1 << 1); } else { PORTA &= ~(1 << 1); } } DcsBios::IntegerBuffer clI4Buffer(0x10d8, 0x0008, 3, onClI4Change);
void onClJ1Change(unsigned int newValue) { if (newValue == 1) { PORTA |= (1 << 2); } else { PORTA &= ~(1 << 2); } } DcsBios::IntegerBuffer clJ1Buffer(0x10d8, 0x0010, 4, onClJ1Change);
void onClJ2Change(unsigned int newValue) { if (newValue == 1) { PORTA |= (1 << 3); } else { PORTA &= ~(1 << 3); } } DcsBios::IntegerBuffer clJ2Buffer(0x10d8, 0x0020, 5, onClJ2Change);
void onClJ3Change(unsigned int newValue) { if (newValue == 1) { PORTA |= (1 << 4); } else { PORTA &= ~(1 << 4); } } DcsBios::IntegerBuffer clJ3Buffer(0x10d8, 0x0040, 6, onClJ3Change);
void onClJ4Change(unsigned int newValue) { if (newValue == 1) { PORTA |= (1 << 5); } else { PORTA &= ~(1 << 5); } } DcsBios::IntegerBuffer clJ4Buffer(0x10d8, 0x0080, 7, onClJ4Change);
void onClK1Change(unsigned int newValue) { if (newValue == 1) { PORTA |= (1 << 6); } else { PORTA &= ~(1 << 6); } } DcsBios::IntegerBuffer clK1Buffer(0x10d8, 0x0100, 8, onClK1Change);
void onClK2Change(unsigned int newValue) { if (newValue == 1) { PORTA |= (1 << 7); } else { PORTA &= ~(1 << 7); } } DcsBios::IntegerBuffer clK2Buffer(0x10d8, 0x0200, 9, onClK2Change);
void onClK3Change(unsigned int newValue) { if (newValue == 1) { PORTC |= (1 << 7); } else { PORTC &= ~(1 << 7); } } DcsBios::IntegerBuffer clK3Buffer(0x10d8, 0x0400, 10, onClK3Change);
void onClK4Change(unsigned int newValue) { if (newValue == 1) { PORTC |= (1 << 6); } else { PORTC &= ~(1 << 6); } } DcsBios::IntegerBuffer clK4Buffer(0x10d8, 0x0800, 11, onClK4Change);
void onClL1Change(unsigned int newValue) { if (newValue == 1) { PORTC |= (1 << 5); } else { PORTC &= ~(1 << 5); } } DcsBios::IntegerBuffer clL1Buffer(0x10d8, 0x1000, 12, onClL1Change);
void onClL2Change(unsigned int newValue) { if (newValue == 1) { PORTC |= (1 << 4); } else { PORTC &= ~(1 << 4); } } DcsBios::IntegerBuffer clL2Buffer(0x10d8, 0x2000, 13, onClL2Change);
void onClL3Change(unsigned int newValue) { if (newValue == 1) { PORTC |= (1 << 3); } else { PORTC &= ~(1 << 3); } } DcsBios::IntegerBuffer clL3Buffer(0x10d8, 0x4000, 14, onClL3Change);
void onClL4Change(unsigned int newValue) { if (newValue == 1) { PORTC |= (1 << 2); } else { PORTC &= ~(1 << 2); } } DcsBios::IntegerBuffer clL4Buffer(0x10d8, 0x8000, 15, onClL4Change);

void setup() {
  DcsBios::setup();
}

void loop() {
  DcsBios::loop();
}